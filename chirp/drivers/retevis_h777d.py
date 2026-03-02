# -*- coding: utf-8 -*-
# Copyright 2026 Piotr Kochanowski <tar4nis@gmail.com>
#
# CHIRP driver for the Retevis H777D.
#
# This is based on the vendor BF480 CPS protocol and memory layout. The
# channel map and sparse clone ranges come from the original programming
# software in retevis_h777d-reference/.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.

import logging
import time

from chirp import bitwise, chirp_common, directory, errors, memmap, util
from chirp.settings import (
    RadioSetting,
    RadioSettingGroup,
    RadioSettingValueBoolean,
    RadioSettingValueList,
)

LOG = logging.getLogger(__name__)


MEM_FORMAT = """
#seekto 0x0000;
struct {
    lbcd rxfreq[4];
    lbcd txfreq[4];
    ul16 rxtone;
    ul16 txtone;
    u8 unknown12;
    u8 unknown13;
    u8 flags14;
    u8 flags15;
} memory[16];

#seekto 0x1000;
struct {
    char name[12];
    u8 unused[4];
} names[16];
"""


CMD_ACK = b"\x06"
TX1 = bytes([0x50, 0xBB, 0xFF, 0x20, 0x12, 0x07, 0x25])

READ_BLOCK_SIZE = 0x40
WRITE_BLOCK_SIZE = 0x10

# The vendor CPS reads only these sparse regions.
DOWNLOAD_RANGES = [
    (0x0000, 0x0100),
    (0x0800, 0x1100),
]

# The vendor CPS writes these sparse regions plus one fixed 0xFF byte at 0x1800.
UPLOAD_RANGES = [
    (0x0000, 0x0100),
    (0x0800, 0x0810),
    (0x0A00, 0x0D00),
    (0x0E00, 0x0E90),
    (0x0F00, 0x0F70),
    (0x1000, 0x1110),
]
TAIL_BYTE_ADDR = 0x1800
IMAGE_SIZE = TAIL_BYTE_ADDR + 1

NAME_LENGTH = 12
PTTID_LIST = ["Off", "BOT", "EOT", "Both"]
VALID_BANDS = [(400000000, 520000000)]
VENDOR_DTCS_CODES = (
    23, 25, 26, 31, 32, 36, 43, 47, 51, 53,
    54, 65, 71, 72, 73, 74, 114, 115, 116, 122,
    125, 131, 132, 134, 143, 145, 152, 155, 156, 162,
    165, 172, 174, 205, 212, 223, 225, 226, 243, 244,
    245, 246, 251, 252, 255, 261, 263, 265, 266, 271,
    274, 306, 311, 315, 325, 331, 332, 343, 346, 351,
    356, 364, 365, 371, 411, 412, 413, 423, 431, 432,
    445, 446, 452, 454, 455, 462, 464, 465, 466, 503,
    506, 516, 523, 526, 532, 546, 565, 606, 612, 624,
    627, 631, 632, 645, 654, 662, 664, 703, 712, 723,
    731, 732, 734, 743, 754,
)
VENDOR_DTCS_COUNT = len(VENDOR_DTCS_CODES)
CTCSS_TONE_TO_RAW = {tone: int(round(tone * 10)) for tone in chirp_common.TONES}
CTCSS_RAW_TO_TONE = {raw: tone for tone, raw in CTCSS_TONE_TO_RAW.items()}

POWER_LOW_MASK = 0x01
HOP_OFF_MASK = 0x02
SCRAMBLE_MASK = 0x04

PTTID_MASK = 0x03
SCAN_ADD_MASK = 0x04
BCL_MASK = 0x08
WIDE_MASK = 0x40


def _set_rts_dtr(serial, rts=True, dtr=True):
    """Try to assert RTS/DTR across different serial backends."""
    try:
        serial.rts = rts
    except Exception:
        try:
            serial.setRTS(rts)
        except Exception:
            pass
    try:
        serial.dtr = dtr
    except Exception:
        try:
            serial.setDTR(dtr)
        except Exception:
            pass


def _discard_buffers(serial):
    """Best-effort flush of serial buffers across pyserial variants."""
    try:
        serial.reset_output_buffer()
    except Exception:
        try:
            serial.flushOutput()
        except Exception:
            pass
    try:
        serial.reset_input_buffer()
    except Exception:
        try:
            serial.flushInput()
        except Exception:
            pass


def _enter_programming_mode(serial):
    """Enter BF480 clone mode: TX1, ident request, then ACK the ident."""
    serial.timeout = 0.6
    _set_rts_dtr(serial, True, True)

    serial.write(TX1)
    time.sleep(0.08)
    ack = serial.read(1)
    if ack != CMD_ACK:
        raise errors.RadioError(f"Bad ACK after TX1: {ack!r}")

    serial.write(b"\x02")
    ident = serial.read(8)
    if len(ident) != 8:
        raise errors.RadioError(f"Short ident: {len(ident)} bytes")
    LOG.info("Ident:\n%s", util.hexprint(ident))

    serial.write(CMD_ACK)
    ack = serial.read(1)
    if ack != CMD_ACK:
        raise errors.RadioError(f"Bad ACK after ident ACK: {ack!r}")

    return ident


def _exit_programming_mode(serial):
    try:
        serial.write(CMD_ACK)
    except Exception:
        pass


def _iter_blocks(ranges, block_size):
    for start, end in ranges:
        for addr in range(start, end, block_size):
            yield addr


def _ranges_size(ranges):
    return sum(end - start for start, end in ranges)


def _read_block(radio, addr):
    """Read one 64-byte block using the BF480 sparse-read protocol."""
    ser = radio.pipe
    hi = (addr >> 8) & 0xFF
    lo = addr & 0xFF

    if addr == 0:
        cmd = bytes([0x53, 0x00, 0x00, READ_BLOCK_SIZE])
    else:
        cmd = bytes([0x06, 0x53, hi, lo, READ_BLOCK_SIZE])

    expected = bytes([0x57, hi, lo, READ_BLOCK_SIZE])
    LOG.debug("READ @%04X cmd=%s", addr, util.hexprint(cmd))
    ser.write(cmd)

    resp = ser.read(1 + 4 + READ_BLOCK_SIZE)
    if len(resp) < 4 + READ_BLOCK_SIZE:
        raise errors.RadioError(f"Short read @ {addr:04X}: {len(resp)} bytes")

    if resp[:1] == CMD_ACK and len(resp) >= 1 + 4 + READ_BLOCK_SIZE:
        resp = resp[1:]

    hdr = resp[:4]
    if hdr[0] not in (0x57, 0x58):
        raise errors.RadioError(f"Bad header type @ {addr:04X}: {hdr!r}")
    if hdr[1:] != expected[1:]:
        raise errors.RadioError(f"Bad response header @ {addr:04X}: {hdr!r}")

    data = resp[4:4 + READ_BLOCK_SIZE]

    ser.write(CMD_ACK)
    ack = ser.read(1)
    if ack not in (b"", CMD_ACK):
        raise errors.RadioError(f"Bad post-read ACK @ {addr:04X}: {ack!r}")

    return data


def _write_block(radio, addr, size):
    """Write one block. The vendor uses 16-byte writes plus one final byte."""
    ser = radio.pipe
    hi = (addr >> 8) & 0xFF
    lo = addr & 0xFF
    data = radio.get_mmap().get_byte_compatible()[addr:addr + size]

    if len(data) != size:
        raise errors.RadioError(f"Short data slice for write @ {addr:04X}")

    cmd = bytes([0x58, hi, lo, size]) + data
    LOG.debug("WRITE @%04X cmd=%s", addr, util.hexprint(cmd[:8]))

    _discard_buffers(ser)
    time.sleep(0.001)
    ser.write(cmd)

    # The OEM CPS waits before checking for the ACK. Without that pacing,
    # some radios accept the first write then stop acknowledging subsequent
    # blocks (commonly failing at 0x0010).
    time.sleep(0.035 if size > 1 else 0.02)
    ack = ser.read(1)
    if ack != CMD_ACK:
        raise errors.RadioError(f"No ACK after write @ {addr:04X}: {ack!r}")


def do_download(radio):
    _enter_programming_mode(radio.pipe)
    radio.pipe.timeout = 1.0

    status = chirp_common.Status()
    status.msg = "Cloning from radio"
    status.cur = 0
    status.max = _ranges_size(DOWNLOAD_RANGES)

    data = bytearray(b"\xFF" * radio._memsize)
    done = 0

    try:
        for addr in _iter_blocks(DOWNLOAD_RANGES, READ_BLOCK_SIZE):
            block = _read_block(radio, addr)
            data[addr:addr + READ_BLOCK_SIZE] = block

            done += READ_BLOCK_SIZE
            status.cur = done
            radio.status_fn(status)
    finally:
        _exit_programming_mode(radio.pipe)

    # The CPS always writes a literal 0xFF here.
    data[TAIL_BYTE_ADDR] = 0xFF
    return memmap.MemoryMapBytes(bytes(data))


def do_upload(radio):
    _enter_programming_mode(radio.pipe)
    radio.pipe.timeout = 1.0

    status = chirp_common.Status()
    status.msg = "Uploading to radio"
    status.cur = 0
    status.max = _ranges_size(UPLOAD_RANGES) + 1

    done = 0

    try:
        for addr in _iter_blocks(UPLOAD_RANGES, WRITE_BLOCK_SIZE):
            _write_block(radio, addr, WRITE_BLOCK_SIZE)

            done += WRITE_BLOCK_SIZE
            status.cur = done
            radio.status_fn(status)

        _write_block(radio, TAIL_BYTE_ADDR, 1)
        status.cur = status.max
        radio.status_fn(status)
    finally:
        _exit_programming_mode(radio.pipe)


@directory.register
class RetevisH777D(chirp_common.CloneModeRadio):
    """Retevis H777D (BF480 protocol)"""

    VENDOR = "Retevis"
    MODEL = "H777D"
    VARIANT = "PMR446"

    BAUD_RATE = 9600
    _memsize = IMAGE_SIZE

    POWER_LEVELS = [
        chirp_common.PowerLevel("Low", dBm=0),
        chirp_common.PowerLevel("High", dBm=1),
    ]

    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.has_settings = False
        rf.has_name = True
        rf.has_bank = False
        rf.has_tuning_step = False

        rf.valid_modes = ["NFM", "FM"]
        rf.valid_skips = ["", "S"]
        # Memory channels are validated against this list even when the
        # radio does not store a per-channel tuning step. PMR446 channels
        # are spaced at 12.5 kHz, but the 446.00625 MHz base requires
        # 6.25 kHz resolution for absolute-frequency validation.
        rf.valid_tuning_steps = [6.25, 12.5]
        rf.valid_tmodes = ["", "Tone", "TSQL", "DTCS", "Cross"]
        rf.valid_cross_modes = [
            "Tone->Tone",
            "DTCS->",
            "->DTCS",
            "Tone->DTCS",
            "DTCS->Tone",
            "->Tone",
            "DTCS->DTCS",
        ]
        # The OEM software exposes explicit RX and TX frequencies, not
        # repeater-style +/- offsets. Use simplex or exact split TX.
        rf.valid_duplexes = ["", "split"]
        rf.has_rx_dtcs = True
        rf.has_ctone = True
        rf.has_cross = True
        rf.can_odd_split = True

        rf.memory_bounds = (1, 16)
        rf.valid_bands = VALID_BANDS
        rf.valid_power_levels = self.POWER_LEVELS
        rf.valid_dtcs_codes = list(VENDOR_DTCS_CODES)
        rf.valid_name_length = NAME_LENGTH
        rf.valid_characters = chirp_common.CHARSET_ASCII
        return rf

    def process_mmap(self):
        self._memobj = bitwise.parse(MEM_FORMAT, self._mmap)

    def sync_in(self):
        try:
            self._mmap = do_download(self)
        except errors.RadioError:
            raise
        except Exception:
            LOG.exception("Unexpected error during download")
            raise errors.RadioError("Unexpected error communicating with radio")
        self.process_mmap()

    def sync_out(self):
        # The vendor CPS always writes 0xFF at 0x1800.
        self.get_mmap()[TAIL_BYTE_ADDR] = 0xFF

        try:
            do_upload(self)
        except errors.RadioError:
            raise
        except Exception:
            LOG.exception("Unexpected error during upload")
            raise errors.RadioError("Unexpected error communicating with radio")

    def get_raw_memory(self, number):
        return repr(self._memobj.memory[number - 1])

    def _decode_tone(self, memval):
        raw = int.from_bytes(memval.get_raw(), "little")

        if raw in (0, 0xFFFF):
            return "", None, None
        if 1 <= raw <= VENDOR_DTCS_COUNT:
            return "DTCS", VENDOR_DTCS_CODES[raw - 1], "N"
        if VENDOR_DTCS_COUNT < raw <= VENDOR_DTCS_COUNT * 2:
            return "DTCS", VENDOR_DTCS_CODES[raw - VENDOR_DTCS_COUNT - 1], "R"
        if raw in CTCSS_RAW_TO_TONE:
            return "Tone", CTCSS_RAW_TO_TONE[raw], None

        LOG.warning("Unknown tone encoding 0x%04X", raw)
        return "", None, None

    def _encode_tone(self, memval, mode, value, polarity):
        if mode == "":
            raw = 0xFFFF
        elif mode == "Tone":
            raw = int(round(float(value) * 10))
        elif mode == "DTCS":
            index = VENDOR_DTCS_CODES.index(value) + 1
            if polarity == "R":
                raw = index + VENDOR_DTCS_COUNT
            else:
                raw = index
        else:
            raise errors.RadioError(f"Unsupported tone mode: {mode}")

        memval.set_raw(raw.to_bytes(2, "little"))

    def _decode_name(self, name_entry):
        name = ""
        for char in name_entry.name:
            value = str(char)
            if value in ("\x00", "\xFF"):
                value = " "
            name += value
        return name.rstrip()

    def _apply_tone_fields(self, mem, txtone, rxtone):
        txmode, txval, txpol = txtone
        rxmode, rxval, rxpol = rxtone

        mem.dtcs_polarity = "%s%s" % (txpol or "N", rxpol or "N")

        if not txmode and not rxmode:
            return

        # Keep the simple CHIRP "Tone" mode for TX-tone-only channels.
        if txmode == "Tone" and not rxmode:
            mem.tmode = "Tone"
            mem.rtone = txval
            return

        # The OEM software exposes separate QT/DQT decode and encode fields.
        # Preserve that explicitly so CHIRP can show both sides, even when
        # they match, instead of collapsing them into a single TSQL/DTCS cell.
        mem.tmode = "Cross"
        mem.cross_mode = "%s->%s" % (txmode or "", rxmode or "")

        if txmode == "Tone":
            mem.rtone = txval
        elif txmode == "DTCS":
            mem.dtcs = txval

        if rxmode == "Tone":
            mem.ctone = rxval
        elif rxmode == "DTCS":
            mem.rx_dtcs = rxval

    def get_memory(self, number):
        _mem = self._memobj.memory[number - 1]
        _name = self._memobj.names[number - 1]

        mem = chirp_common.Memory()
        mem.number = number

        raw_rx = _mem.rxfreq.get_raw()
        if raw_rx in (b"\x00\x00\x00\x00", b"\xFF\xFF\xFF\xFF"):
            mem.empty = True
            return mem

        mem.freq = int(_mem.rxfreq) * 10
        if not mem.freq:
            mem.empty = True
            return mem

        raw_tx = _mem.txfreq.get_raw()
        if raw_tx in (b"\x00\x00\x00\x00", b"\xFF\xFF\xFF\xFF"):
            mem.duplex = ""
            mem.offset = 0
        else:
            txfreq = int(_mem.txfreq) * 10
            if txfreq == mem.freq:
                mem.duplex = ""
                mem.offset = 0
            else:
                mem.duplex = "split"
                mem.offset = txfreq

        flags14 = int(_mem.flags14)
        flags15 = int(_mem.flags15)

        mem.mode = "FM" if (flags15 & WIDE_MASK) else "NFM"
        # The radio does not store a per-channel tuning step, but CHIRP
        # still validates the hidden field on edit. Use the channel-plan
        # spacing so existing rows do not retain the default 5.0 kHz value.
        mem.tuning_step = 12.5
        mem.power = (self.POWER_LEVELS[0]
                     if (flags14 & POWER_LOW_MASK)
                     else self.POWER_LEVELS[1])
        mem.skip = "" if (flags15 & SCAN_ADD_MASK) else "S"
        mem.name = self._decode_name(_name)

        txtone = self._decode_tone(_mem.txtone)
        rxtone = self._decode_tone(_mem.rxtone)
        self._apply_tone_fields(mem, txtone, rxtone)

        mem.extra = RadioSettingGroup("extra", "Extra")
        mem.extra.append(
            RadioSetting(
                "pttid",
                "PTT-ID",
                RadioSettingValueList(PTTID_LIST,
                                      current_index=flags15 & PTTID_MASK),
            )
        )
        mem.extra.append(
            RadioSetting(
                "bcl",
                "Busy Channel Lockout",
                RadioSettingValueBoolean(bool(flags15 & BCL_MASK)),
            )
        )
        mem.extra.append(
            RadioSetting(
                "scramble",
                "Scramble",
                RadioSettingValueBoolean(bool(flags14 & SCRAMBLE_MASK)),
            )
        )
        mem.extra.append(
            RadioSetting(
                "hopping",
                "Frequency Hopping",
                RadioSettingValueBoolean(not bool(flags14 & HOP_OFF_MASK)),
            )
        )

        return mem

    def set_memory(self, mem):
        _mem = self._memobj.memory[mem.number - 1]
        _name = self._memobj.names[mem.number - 1]

        if mem.empty:
            _mem.set_raw(b"\xFF" * (_mem.size() // 8))
            _name.set_raw(b"\xFF" * (_name.size() // 8))
            return

        _mem.set_raw(b"\x00" * (_mem.size() // 8))
        _name.set_raw(b"\xFF" * (_name.size() // 8))

        _mem.rxfreq = mem.freq / 10

        if mem.duplex == "off":
            for i in range(4):
                _mem.txfreq[i].set_raw(b"\xFF")
        elif mem.duplex == "split":
            _mem.txfreq = mem.offset / 10
        elif mem.duplex == "+":
            _mem.txfreq = (mem.freq + mem.offset) / 10
        elif mem.duplex == "-":
            _mem.txfreq = (mem.freq - mem.offset) / 10
        else:
            _mem.txfreq = mem.freq / 10

        txtone, rxtone = chirp_common.split_tone_encode(mem)
        self._encode_tone(_mem.txtone, *txtone)
        self._encode_tone(_mem.rxtone, *rxtone)

        _mem.flags14 = 0
        _mem.flags15 = 0

        if mem.power == self.POWER_LEVELS[0]:
            _mem.flags14 |= POWER_LOW_MASK
        if mem.mode == "FM":
            _mem.flags15 |= WIDE_MASK
        if mem.skip != "S":
            _mem.flags15 |= SCAN_ADD_MASK

        mem_name = mem.name or ""
        for i in range(NAME_LENGTH):
            try:
                _name.name[i] = mem_name[i]
            except IndexError:
                _name.name[i] = "\xFF"

        for setting in mem.extra or []:
            name = setting.get_name()
            if name == "pttid":
                _mem.flags15 &= ~PTTID_MASK
                _mem.flags15 |= PTTID_LIST.index(str(setting.value))
            elif name == "bcl":
                if int(setting.value):
                    _mem.flags15 |= BCL_MASK
                else:
                    _mem.flags15 &= ~BCL_MASK
            elif name == "scramble":
                if int(setting.value):
                    _mem.flags14 |= SCRAMBLE_MASK
                else:
                    _mem.flags14 &= ~SCRAMBLE_MASK
            elif name == "hopping":
                if int(setting.value):
                    _mem.flags14 &= ~HOP_OFF_MASK
                else:
                    _mem.flags14 |= HOP_OFF_MASK

    @classmethod
    def match_model(cls, filedata, filename):
        # This model is matched via metadata, not by old-style image probing.
        return False
