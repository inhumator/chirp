# -*- coding: utf-8 -*-
# Copyright 2026 Piotr Kochanowski <tar4nis@gmail.com>
#
# CHIRP driver (first pass) for Retevis H777D (BF480-family CPS protocol)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.

import time
import logging

from chirp import chirp_common, directory, memmap, bitwise, errors, util
from chirp.settings import (
    RadioSetting, RadioSettingGroup,
    RadioSettingValueInteger, RadioSettingValueList,
    RadioSettingValueBoolean, RadioSettings
)

LOG = logging.getLogger(__name__)

# --------------------------------------------------------------------
# Memory map formats (copied from the classic H777-family; refine later)
# --------------------------------------------------------------------
MEM_FORMAT = """
#seekto 0x0010;
struct {
    lbcd rxfreq[4];
    lbcd txfreq[4];
    lbcd rxtone[2];
    lbcd txtone[2];
    u8 unknown3:1,
       unknown2:1,
       unknown1:1,
       skip:1,
       highpower:1,
       narrow:1,
       beatshift:1,
       bcl:1;
    u8 unknown4[3];
} memory[16];
#seekto 0x02B0;
struct {
    u8 voiceprompt;
    u8 voicelanguage;
    u8 scan;
    u8 vox;
    u8 voxlevel;
    u8 voxinhibitonrx;
    u8 lowvolinhibittx;
    u8 highvolinhibittx;
    u8 alarm;
    u8 fmradio;
} settings;
"""

H777_SETTINGS2 = """
#seekto 0x03C0;
struct {
    u8 unused:6,
       batterysaver:1,
       beep:1;
    u8 squelchlevel;
    u8 sidekeyfunction;
    u8 timeouttimer;
    u8 unused2[3];
    u8 unused3:7,
       scanmode:1;
} settings2;
"""

# --------------------------------------------------------------------
# BF480-family wire protocol constants (from official CPS source)
# --------------------------------------------------------------------
CMD_ACK = b"\x06"
TX1 = bytes([0x50, 0xBB, 0xFF, 0x20, 0x12, 0x07, 0x25])  # SC.TX1 in CPS

READ_BLOCK_SIZE = 0x40   # 64 bytes
WRITE_BLOCK_SIZE = 0x10  # 16 bytes
DEFAULT_MEMSIZE = 0x1100 # 

VOICE_LIST = ["English", "Chinese"]
TIMEOUTTIMER_LIST = ["Off", "30 seconds", "60 seconds", "90 seconds",
                     "120 seconds", "150 seconds", "180 seconds",
                     "210 seconds", "240 seconds", "270 seconds",
                     "300 seconds"]

DTCS_FLAG = 0x80
DTCS_REV_FLAG = 0x40


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


def _enter_programming_mode(serial):
    """
    BF480-family handshake:
      1) TX1 (7 bytes) -> expect 0x06
      2) 0x02 -> read 8-byte ident
      3) 0x06 -> expect 0x06
    """
    serial.timeout = 0.6
    _set_rts_dtr(serial, True, True)

    # Step 1: TX1 -> ACK
    serial.write(TX1)
    time.sleep(0.08)
    ack = serial.read(1)
    if ack != CMD_ACK:
        raise errors.RadioError(f"Bad ACK after TX1: {ack!r}")

    # Step 2: 0x02 -> ident(8)
    serial.write(b"\x02")
    ident = serial.read(8)
    if len(ident) != 8:
        raise errors.RadioError(f"Short ident: {len(ident)} bytes")
    LOG.info("Ident:\n%s", util.hexprint(ident))

    # Step 3: 0x06 -> ACK
    serial.write(CMD_ACK)
    ack2 = serial.read(1)
    if ack2 != CMD_ACK:
        raise errors.RadioError(f"Bad ACK after ident ACK: {ack2!r}")

    return ident


def _exit_programming_mode(serial):
    # Many radios exit clone mode when the port closes; keep harmless.
    try:
        serial.write(CMD_ACK)
    except Exception:
        pass


def _read_block(radio, addr):
    """
    Read 64 bytes at addr.
    CPS does:
      first block:  [0x53, 0x00, 0x00, 0x40]
      next blocks:  [0x06, 0x53, hi, lo, 0x40]
    Response header is: [0x57, hi, lo, 0x40] then 64 bytes payload.
    """
    ser = radio.pipe
    hi = (addr >> 8) & 0xFF
    lo = addr & 0xFF

    if addr == 0:
        cmd = bytes([0x53, 0x00, 0x00, 0x40])
        expected = bytes([0x57, 0x00, 0x00, 0x40])
    else:
        cmd = bytes([0x06, 0x53, hi, lo, 0x40])
        expected = bytes([0x57, hi, lo, 0x40])

    LOG.debug("READ @%04X cmd=%s", addr, util.hexprint(cmd))
    ser.write(cmd)

    resp = ser.read(4 + READ_BLOCK_SIZE + 2)  # allow 1 extra prefix byte

    LOG.debug("RAW RESP @%04X: %s", addr, util.hexprint(resp[:12]))
    
    if len(resp) < 4 + READ_BLOCK_SIZE:
        raise errors.RadioError(f"Short read @ {addr:04X}: {len(resp)} bytes")

    # Some variants prefix the response with 0x06
    if resp[0:1] == CMD_ACK and len(resp) >= 1 + 4 + READ_BLOCK_SIZE:
        resp = resp[1:]

    hdr = resp[:4]

    # Length must match
    if hdr[3] != 0x40:
        raise errors.RadioError(f"Bad length byte @ {addr:04X}: hdr={hdr!r}")

    # Header type can be 'W' or 'X'
    if hdr[0] not in (0x57, 0x58):
        raise errors.RadioError(f"Bad header type @ {addr:04X}: hdr={hdr!r}")

    data = resp[4:4 + READ_BLOCK_SIZE]

    # ACK after each block
    
    try:
        LOG.debug("About to post-ACK @%04X", addr)
        ser.write(CMD_ACK)
        ack = ser.read(1)
        LOG.debug("Post-ACK read @%04X got: %r", addr, ack)
        if ack and ack != CMD_ACK:
            LOG.debug("Unexpected post-read byte @%04X: %r", addr, ack)
    except Exception:
        pass

    return data


def _write_block(radio, addr):
    """
    Write 16 bytes at addr:
      [0x58, hi, lo, 0x10, <16 bytes>] -> expect 0x06
    """
    ser = radio.pipe
    hi = (addr >> 8) & 0xFF
    lo = addr & 0xFF

    data = radio.get_mmap().get_byte_compatible()[addr:addr + WRITE_BLOCK_SIZE]
    if len(data) != WRITE_BLOCK_SIZE:
        raise errors.RadioError(f"Short data slice for write @ {addr:04X}")

    cmd = bytes([0x58, hi, lo, 0x10]) + data
    LOG.debug("WRITE @%04X hdr=%s ...", addr, util.hexprint(cmd[:8]))
    ser.write(cmd)

    ack = ser.read(1)
    if ack != CMD_ACK:
        raise errors.RadioError(f"No ACK after write @ {addr:04X}")


def do_download(radio):
    _enter_programming_mode(radio.pipe)

    status = chirp_common.Status()
    status.msg = "Cloning from radio"
    status.cur = 0
    status.max = radio._memsize

    data = bytearray()

    radio.pipe.timeout = 1.0

    for addr in range(0, radio._memsize, READ_BLOCK_SIZE):
        status.cur = addr + READ_BLOCK_SIZE
        radio.status_fn(status)
        data.extend(_read_block(radio, addr))

    _exit_programming_mode(radio.pipe)
    return memmap.MemoryMapBytes(bytes(data))


def do_upload(radio):
    _enter_programming_mode(radio.pipe)

    status = chirp_common.Status()
    status.msg = "Uploading to radio"
    status.cur = 0
    status.max = radio._memsize

    for start, end in radio._ranges:
        for addr in range(start, end, WRITE_BLOCK_SIZE):
            status.cur = addr + WRITE_BLOCK_SIZE
            radio.status_fn(status)
            _write_block(radio, addr)

    _exit_programming_mode(radio.pipe)


@directory.register
class RetevisH777D(chirp_common.CloneModeRadio):
    """Retevis H777D (BF480-family protocol) - first pass"""
    VENDOR = "Retevis"
    MODEL = "H777D"
    VARIANT = "PMR446"

    BAUD_RATE = 9600

    # First pass: assume full 0x1800 image; refine once confirmed.
    _memsize = DEFAULT_MEMSIZE
    _ranges = [(0x0000, DEFAULT_MEMSIZE)]

    POWER_LEVELS = [chirp_common.PowerLevel("Low", watts=1.00),
                    chirp_common.PowerLevel("High", watts=5.00)]
    VALID_BANDS = (400000000, 490000000)

    SIDEKEYFUNCTION_LIST = ["Off", "Monitor", "Transmit Power", "Alarm"]
    SCANMODE_LIST = ["Carrier", "Time"]
    MAX_VOXLEVEL = 5

    # Feature toggles (first pass)
    _has_fm = True
    _has_sidekey = True
    _has_scanmodes = True
    _has_scramble = True

    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.has_settings = True
        rf.has_name = False
        rf.has_bank = False
        rf.has_tuning_step = False

        rf.valid_modes = ["NFM", "FM"]
        rf.valid_skips = ["", "S"]
        rf.valid_tmodes = ["", "Tone", "TSQL", "DTCS", "Cross"]
        rf.has_rx_dtcs = True
        rf.has_ctone = True
        rf.has_cross = True
        rf.valid_cross_modes = [
            "Tone->Tone",
            "DTCS->",
            "->DTCS",
            "Tone->DTCS",
            "DTCS->Tone",
            "->Tone",
            "DTCS->DTCS",
        ]
        rf.valid_duplexes = ["", "-", "+", "split", "off"]
        rf.can_odd_split = True

        rf.memory_bounds = (1, 16)
        rf.valid_bands = [self.VALID_BANDS]
        rf.valid_power_levels = self.POWER_LEVELS
        return rf

    def process_mmap(self):
        # NOTE: This format is assumed from H777-family; adjust if needed.
        self._memobj = bitwise.parse(MEM_FORMAT + H777_SETTINGS2, self._mmap)

    def sync_in(self):
        self._mmap = do_download(self)
        self.process_mmap()

    def sync_out(self):
        do_upload(self)

    def get_raw_memory(self, number):
        return repr(self._memobj.memory[number - 1])

    # --- Tone helpers (same as your working code) ---
    def _decode_tone(self, memval):
        memval[1].ignore_bits(DTCS_FLAG | DTCS_REV_FLAG)
        is_dtcs = memval[1].get_bits(DTCS_FLAG)
        is_rev = memval[1].get_bits(DTCS_REV_FLAG)
        if memval.get_raw() == b"\xFF\xFF":
            return '', None, None
        elif is_dtcs:
            return 'DTCS', int(memval), 'R' if is_rev else 'N'
        else:
            return 'Tone', int(memval) / 10.0, None

    def _encode_tone(self, memval, mode, value, pol):
        memval[1].ignore_bits(DTCS_FLAG | DTCS_REV_FLAG)
        if mode == '':
            memval.fill_raw(b'\xFF')
        elif mode == 'Tone':
            memval[1].clr_bits(DTCS_FLAG | DTCS_REV_FLAG)
            memval.set_value(int(value * 10))
        elif mode == 'DTCS':
            memval[1].set_bits(DTCS_FLAG)
            if pol == 'R':
                memval[1].set_bits(DTCS_REV_FLAG)
            else:
                memval[1].clr_bits(DTCS_REV_FLAG)
            memval.set_value(value)
        else:
            raise Exception(f"Internal error: invalid mode `{mode}`")

    def get_memory(self, number):
        _mem = self._memobj.memory[number - 1]
        mem = chirp_common.Memory()
        mem.number = number
        mem.freq = int(_mem.rxfreq) * 10

        if mem.freq == 0 or _mem.rxfreq.get_raw() == b"\xFF\xFF\xFF\xFF":
            mem.empty = True
            mem.freq = 0
            return mem

        if _mem.txfreq.get_raw() == b"\xFF\xFF\xFF\xFF":
            mem.duplex = "off"
            mem.offset = 0
        elif int(_mem.rxfreq) == int(_mem.txfreq):
            mem.duplex = ""
            mem.offset = 0
        else:
            mem.duplex = "-" if int(_mem.rxfreq) > int(_mem.txfreq) else "+"
            mem.offset = abs(int(_mem.rxfreq) - int(_mem.txfreq)) * 10

        mem.mode = "FM" if not _mem.narrow else "NFM"
        mem.power = self.POWER_LEVELS[_mem.highpower]
        mem.skip = "S" if _mem.skip else ""

        txtone = self._decode_tone(_mem.txtone)
        rxtone = self._decode_tone(_mem.rxtone)
        chirp_common.split_tone_decode(mem, txtone, rxtone)

        mem.extra = RadioSettingGroup("Extra", "extra")
        mem.extra.append(
            RadioSetting("bcl", "Busy Channel Lockout",
                         RadioSettingValueBoolean(not _mem.bcl))
        )
        if self._has_scramble:
            mem.extra.append(
                RadioSetting("beatshift", "Beat Shift(scramble)",
                             RadioSettingValueBoolean(not _mem.beatshift))
            )
        return mem

    def set_memory(self, mem):
        _mem = self._memobj.memory[mem.number - 1]

        if mem.empty:
            _mem.set_raw("\xFF" * (_mem.size() // 8))
            return

        _mem.rxfreq = mem.freq / 10

        if mem.duplex == "off":
            for i in range(0, 4):
                _mem.txfreq[i].set_raw("\xFF")
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

        _mem.narrow = 'N' in mem.mode
        _mem.highpower = (mem.power == self.POWER_LEVELS[1])
        _mem.skip = (mem.skip == "S")

        for setting in mem.extra:
            setattr(_mem, setting.get_name(), not int(setting.value))

        # Keep compatibility with older CPS expectations
        _mem.unknown1 = 0
        _mem.unknown2 = 0
        _mem.unknown3 = 0

    def get_settings(self):
        _settings = self._memobj.settings
        basic = RadioSettingGroup("basic", "Basic Settings")
        top = RadioSettings(basic)

        basic.append(RadioSetting(
            "voiceprompt", "Voice prompt",
            RadioSettingValueBoolean(_settings.voiceprompt)))

        basic.append(RadioSetting(
            "voicelanguage", "Voice language",
            RadioSettingValueList(VOICE_LIST,
                                  current_index=_settings.voicelanguage)))

        basic.append(RadioSetting(
            "scan", "Scan",
            RadioSettingValueBoolean(_settings.scan)))

        if self._has_scanmodes:
            basic.append(RadioSetting(
                "settings2.scanmode", "Scan mode",
                RadioSettingValueList(self.SCANMODE_LIST,
                                      current_index=self._memobj.settings2.scanmode)))

        basic.append(RadioSetting(
            "vox", "VOX",
            RadioSettingValueBoolean(_settings.vox)))

        basic.append(RadioSetting(
            "voxlevel", "VOX level",
            RadioSettingValueInteger(1, self.MAX_VOXLEVEL, _settings.voxlevel + 1)))

        basic.append(RadioSetting(
            "voxinhibitonrx", "Inhibit VOX on receive",
            RadioSettingValueBoolean(_settings.voxinhibitonrx)))

        basic.append(RadioSetting(
            "lowvolinhibittx", "Low voltage inhibit transmit",
            RadioSettingValueBoolean(_settings.lowvolinhibittx)))

        basic.append(RadioSetting(
            "highvolinhibittx", "High voltage inhibit transmit",
            RadioSettingValueBoolean(_settings.highvolinhibittx)))

        if self._has_fm:
            basic.append(RadioSetting(
                "fmradio", "FM function",
                RadioSettingValueBoolean(_settings.fmradio)))

        basic.append(RadioSetting(
            "settings2.beep", "Beep",
            RadioSettingValueBoolean(self._memobj.settings2.beep)))

        basic.append(RadioSetting(
            "settings2.batterysaver", "Battery saver",
            RadioSettingValueBoolean(self._memobj.settings2.batterysaver)))

        basic.append(RadioSetting(
            "settings2.squelchlevel", "Squelch level",
            RadioSettingValueInteger(0, 9, self._memobj.settings2.squelchlevel)))

        if self._has_sidekey:
            basic.append(RadioSetting(
                "settings2.sidekeyfunction", "Side key function",
                RadioSettingValueList(self.SIDEKEYFUNCTION_LIST,
                                      current_index=self._memobj.settings2.sidekeyfunction)))

        basic.append(RadioSetting(
            "settings2.timeouttimer", "Timeout timer",
            RadioSettingValueList(TIMEOUTTIMER_LIST,
                                  current_index=self._memobj.settings2.timeouttimer)))

        return top

    def set_settings(self, settings):
        for element in settings:
            if not isinstance(element, RadioSetting):
                self.set_settings(element)
                continue

            if "." in element.get_name():
                bits = element.get_name().split(".")
                obj = self._memobj
                for bit in bits[:-1]:
                    obj = getattr(obj, bit)
                setting = bits[-1]
            else:
                obj = self._memobj.settings
                setting = element.get_name()

            if element.has_apply_callback():
                element.run_apply_callback()
            elif setting == "voxlevel":
                setattr(obj, setting, int(element.value) - 1)
            else:
                setattr(obj, setting, element.value)

    @classmethod
    def match_model(cls, filedata, filename):
        # Don't do old-style detection
        return False