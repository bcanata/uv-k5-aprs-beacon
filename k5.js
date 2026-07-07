// k5.js — shared UV-K5 serial layer for the APRS site (installer + beacon).
// Web Serial (desktop) and WebUSB/CH340 (Android) transports with a read path,
// the AB CD..DC BA envelope, normal-mode commands and the bootloader protocol.
// Ported from utils/k5flash.py, libuvk5.py, eeprom_tool.py, aprs_pc.py.
"use strict";

var K5 = (function () {

  // ---- envelope (XOR + CRC-16/XMODEM) ----
  var XOR = [0x16,0x6C,0x14,0xE6,0x2E,0x91,0x0D,0x40,0x21,0x35,0xD5,0x40,0x13,0x03,0xE9,0x80];
  var TS  = [0x46,0x9C,0x6F,0x64];  // fixed session timestamp used by K5TOOL/libuvk5

  function crc16(bytes) {
    var crc = 0;
    for (var i = 0; i < bytes.length; i++) {
      crc ^= bytes[i] << 8;
      for (var b = 0; b < 8; b++)
        crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
    }
    return crc;
  }
  function xorApply(bytes) {
    var out = new Uint8Array(bytes.length);
    for (var i = 0; i < bytes.length; i++) out[i] = bytes[i] ^ XOR[i % 16];
    return out;
  }
  function concat(a, b) {
    var out = new Uint8Array(a.length + b.length);
    out.set(a, 0); out.set(b, a.length);
    return out;
  }
  // frame a fully-formed payload (ID + size + body): AB CD | len | XOR(payload+crc) | DC BA
  function frameRaw(payload) {
    var crc = crc16(payload);
    var body = new Uint8Array(payload.length + 2);
    body.set(payload, 0);
    body[payload.length] = crc & 0xFF; body[payload.length + 1] = (crc >> 8) & 0xFF;
    var enc = xorApply(body);
    var out = new Uint8Array(4 + enc.length + 2);
    out[0] = 0xAB; out[1] = 0xCD; out[2] = payload.length & 0xFF; out[3] = (payload.length >> 8) & 0xFF;
    out.set(enc, 4);
    out[out.length - 2] = 0xDC; out[out.length - 1] = 0xBA;
    return out;
  }
  // build ID(2) + len(2) + data, then frame it (normal-mode command shape)
  function frameCommand(id, data) {
    data = data || new Uint8Array(0);
    var payload = new Uint8Array(4 + data.length);
    payload[0] = id & 0xFF; payload[1] = (id >> 8) & 0xFF;
    payload[2] = data.length & 0xFF; payload[3] = (data.length >> 8) & 0xFF;
    payload.set(data, 4);
    return frameRaw(payload);
  }
  // extract one framed reply from a buffer; returns {payload, rest} or null.
  function extractFrame(buf) {
    for (var s = 0; s + 8 <= buf.length; s++) {
      if (buf[s] !== 0xAB || buf[s + 1] !== 0xCD) continue;
      var len = buf[s + 2] | (buf[s + 3] << 8);
      var total = 4 + (len + 2) + 2;               // hdr + xor(payload+crc) + DC BA
      if (len > 2048) continue;
      if (s + total > buf.length) return null;      // wait for more bytes
      if (buf[s + total - 2] !== 0xDC || buf[s + total - 1] !== 0xBA) continue;
      var dec = xorApply(buf.slice(s + 4, s + 4 + len + 2));
      var payload = dec.slice(0, len);
      // Do NOT validate the CRC on RX: the radio's replies carry a placeholder
      // CRC (0xFFFF), not a real CRC-16 over the payload, so a strict check
      // rejects every valid reply. The AB CD / DC BA framing + length field are
      // the integrity signals here (matches aprs_pc.py and joaquimorg's
      // uv-kx-tools, which also ignore the reply CRC). We still emit a correct
      // CRC on TX in frameRaw(), which the radio accepts.
      return { payload: payload, rest: buf.slice(s + total) };
    }
    return null;
  }

  // ---- CH341 baud divisor (modern kernel encoding; 38400 -> 0x6403) ----
  function ch341Divisor(baud) {
    var CLK = 48000000, fact = 1, ps, clkDiv, div;
    var minRate = [CLK / (4096 * 512), CLK / (512 * 512), CLK / (64 * 512), CLK / (8 * 512)];
    for (ps = 3; ps >= 0; ps--) if (baud > minRate[ps]) break;
    if (ps < 0) ps = 0;
    clkDiv = 1 << (12 - 3 * ps - fact);
    div = Math.floor(CLK / (clkDiv * baud));
    if (div < 9 || div > 255) { fact = 0; clkDiv = 1 << (12 - 3 * ps); div = Math.floor(CLK / (clkDiv * baud)); }
    return ((((0x100 - div) & 0xff) << 8) | (fact << 2) | ps) & 0xffff;
  }

  // ---- transports: each exposes connect(preDevice), send(bytes), recv(max,timeoutMs), name ----
  function makeWebSerial() {
    var port = null, reader = null, writer = null, pending = null;
    var TIMED_OUT = {};
    return {
      name: "Web Serial",
      isSerial: true,
      connect: async function () {
        port = await navigator.serial.requestPort();
        await port.open({ baudRate: 38400 });
        writer = port.writable.getWriter();
        reader = port.readable.getReader();
        pending = null;
      },
      send: async function (bytes) { await writer.write(bytes); },
      // Keep ONE outstanding read() across calls. Web Streams deliver each
      // chunk to the oldest pending read(); if the timeout won the race and we
      // let read() go, the radio's reply would resolve that orphaned promise
      // and be lost (that was the "connected but no answer" bug). So on
      // timeout we return empty but preserve `pending` for the next recv.
      recv: async function (max, timeoutMs) {
        if (!pending) pending = reader.read();
        var timer, to = new Promise(function (r) { timer = setTimeout(function () { r(TIMED_OUT); }, timeoutMs); });
        try {
          var res = await Promise.race([pending, to]);
          if (res === TIMED_OUT) return new Uint8Array(0);   // keep `pending`
          pending = null;
          if (!res || res.done || !res.value) return new Uint8Array(0);
          return res.value;
        } finally { clearTimeout(timer); }
      },
      disconnect: async function () {
        try { if (reader) { await reader.cancel(); reader.releaseLock(); } } catch (e) {}
        try { if (writer) writer.releaseLock(); } catch (e) {}
        try { if (port) await port.close(); } catch (e) {}
        pending = null;
      }
    };
  }

  function makeWebUsb(showAll) {
    var dev = null, epOut = 0, epIn = 0, step = "", pendingIn = null;
    var TIMED_OUT = {};
    async function ctrl(tag, req, val, idx) {
      step = tag;
      var r = await dev.controlTransferOut(
        { requestType: "vendor", recipient: "device", request: req, value: val, index: idx });
      if (r && r.status && r.status !== "ok") throw new Error("ctrl " + tag + " -> " + r.status);
    }
    async function ch340Init() {
      try {
        await ctrl("init", 0xA1, 0x0000, 0x0000);
        await ctrl("baud", 0x9A, 0x1312, ch341Divisor(38400));
        await ctrl("lcr",  0x9A, 0x2518, 0x00C3);   // 8N1 + enable RX + enable TX
      } catch (e) {
        throw new Error("CH340 setup failed at '" + step + "': " + (e && e.message ? e.message : e));
      }
    }
    var filters = showAll ? [] : [
      { vendorId: 0x1A86 }, { vendorId: 0x10C4 }, { vendorId: 0x0403 }, { vendorId: 0x067B }
    ];
    return {
      name: "WebUSB",
      isSerial: false,
      connect: async function (preDevice) {
        dev = preDevice || await navigator.usb.requestDevice({ filters: filters });
        await dev.open();
        if (dev.configuration === null) await dev.selectConfiguration(1);
        var iface = dev.configuration.interfaces[0];
        try { await dev.claimInterface(iface.interfaceNumber); }
        catch (e) { throw new Error("adapter busy — Android's serial driver may have claimed it; this phone/Android version may not allow WebUSB serial"); }
        iface.alternates[0].endpoints.forEach(function (e) {
          if (e.direction === "out" && e.type === "bulk") epOut = e.endpointNumber;
          if (e.direction === "in"  && e.type === "bulk") epIn  = e.endpointNumber;
        });
        if (!epOut || !epIn) throw new Error("no bulk endpoints on the adapter");
        await ch340Init();
      },
      send: async function (bytes) { await dev.transferOut(epOut, bytes); },
      // Same rule as Web Serial: never orphan a pending transferIn on timeout,
      // or the reply bytes it later receives are lost. Preserve it for reuse.
      recv: async function (max, timeoutMs) {
        if (!pendingIn) pendingIn = dev.transferIn(epIn, max || 64);
        var timer, to = new Promise(function (r) { timer = setTimeout(function () { r(TIMED_OUT); }, timeoutMs); });
        try {
          var res = await Promise.race([pendingIn, to]);
          if (res === TIMED_OUT) return new Uint8Array(0);   // keep `pendingIn`
          pendingIn = null;
          if (!res || !res.data || !res.data.byteLength) return new Uint8Array(0);
          return new Uint8Array(res.data.buffer, res.data.byteOffset, res.data.byteLength);
        } catch (e) { pendingIn = null; return new Uint8Array(0); }
        finally { clearTimeout(timer); }
      },
      disconnect: async function () { try { if (dev) await dev.close(); } catch (e) {} pendingIn = null; }
    };
  }

  // ---- radio: wraps a transport, buffers RX, speaks the command layers ----
  function Radio(transport) { this.t = transport; this.rx = new Uint8Array(0); }

  Radio.prototype.send = function (bytes) { return this.t.send(bytes); };

  // read one framed reply within timeoutMs (null on timeout)
  Radio.prototype.readFrame = async function (timeoutMs) {
    var deadline = Date.now() + timeoutMs;
    for (;;) {
      var f = extractFrame(this.rx);
      if (f) { this.rx = f.rest; return f.payload; }
      if (Date.now() >= deadline) return null;
      var chunk = await this.t.recv(256, Math.min(250, deadline - Date.now()));
      if (chunk && chunk.length) this.rx = concat(this.rx, chunk);
    }
  };
  Radio.prototype._exchange = async function (frame, timeoutMs) {
    this.rx = new Uint8Array(0);
    await this.t.send(frame);
    return this.readFrame(timeoutMs || 2000);
  };

  // --- normal-mode commands ---
  // hello -> firmware version string, or null if the radio isn't in normal mode
  Radio.prototype.hello = async function () {
    var p = await this._exchange(frameCommand(0x0514, new Uint8Array(TS)), 1500);
    if (!p || (p[0] | (p[1] << 8)) !== 0x0515) return null;
    var s = "";
    for (var i = 4; i < p.length && p[i]; i++) s += String.fromCharCode(p[i]);
    return s;
  };
  // read `len` (<=128) bytes of config EEPROM at addr
  Radio.prototype.readCfg = async function (addr, len) {
    var d = new Uint8Array(4 + 4);
    d[0] = addr & 0xFF; d[1] = (addr >> 8) & 0xFF; d[2] = len & 0xFF; d[3] = (len >> 8) & 0xFF;
    d.set(TS, 4);
    var p = await this._exchange(frameCommand(0x051B, d), 2000);
    if (!p || (p[0] | (p[1] << 8)) !== 0x051C) throw new Error("read " + addr.toString(16) + " no reply");
    return p.slice(8, 8 + len);   // payload = ID(2)+size(2)+offset(2)+size(1)+pad(1)+data
  };
  // write payload (length must be a multiple of 8) to config EEPROM at addr
  Radio.prototype.writeCfg = async function (addr, payload) {
    if (payload.length % 8 !== 0) throw new Error("writeCfg length must be a multiple of 8");
    var body = new Uint8Array(4 + 4 + payload.length);
    body[0] = addr & 0xFF; body[1] = (addr >> 8) & 0xFF;
    body[2] = payload.length & 0xFF; body[3] = (payload.length >> 8) & 0xFF;
    body.set(TS, 4); body.set(payload, 8);
    var p = await this._exchange(frameCommand(0x051D, body), 2000);
    if (!p || (p[0] | (p[1] << 8)) !== 0x051E) throw new Error("write " + addr.toString(16) + " no ack");
    return true;
  };
  Radio.prototype.reboot = async function () {
    await this.t.send(frameCommand(0x05DD, new Uint8Array(0)));  // no reply
  };

  // Serialize all radio operations on the single serial link so a screen poll
  // and a key-press can never interleave their bytes. Every RC method funnels
  // through _run(); failures don't break the chain.
  Radio.prototype._run = function (fn) {
    var next = (this._q || Promise.resolve()).then(fn, fn);
    // keep the chain alive even if fn rejects
    this._q = next.catch(function () {});
    return next;
  };

  // write a full EEPROM image back, 128 bytes at a time, up to `end` (default
  // 0x1E00 so calibration 0x1E00+ is never touched). bytes.length must cover [0,end).
  Radio.prototype.restore = function (bytes, end, onProgress) {
    var self = this; end = end || 0x1E00;
    return this._run(async function () {
      for (var a = 0; a < end; a += 128) {
        var n = Math.min(128, end - a);
        if (n % 8 !== 0) n -= (n % 8);
        await self.writeCfg(a, bytes.slice(a, a + n));
        if (onProgress) onProgress(a + n, end);
      }
      return true;
    });
  };

  // --- radio remote control (firmware built with ENABLE_UART_RC) ---
  // KEY_Code_t values (match driver/keyboard.h): 0-9 digits, 10 MENU, 11 UP,
  // 12 DOWN, 13 EXIT, 14 STAR(*), 15 F(#), 16 PTT, 17 SIDE2, 18 SIDE1.
  Radio.prototype.injectKey = function (key, flags) {
    var self = this;
    return this._run(async function () {
      var p = await self._exchange(frameCommand(0x0B01, new Uint8Array([key & 0xFF, flags & 0xFF])), 1200);
      return !!p && (p[0] | (p[1] << 8)) === 0x0B81 && p[6] === 1;
    });
  };
  // tap = press then release; held=true sends a long-press first
  Radio.prototype.tapKey = function (key, held) {
    var self = this;
    return this._run(async function () {
      await self._exchange(frameCommand(0x0B01, new Uint8Array([key & 0xFF, held ? 0x03 : 0x01])), 1200);
      await self._exchange(frameCommand(0x0B01, new Uint8Array([key & 0xFF, 0x00])), 1200);
      return true;
    });
  };
  Radio.prototype.getState = function () {
    var self = this;
    return this._run(async function () {
      var p = await self._exchange(frameCommand(0x0B02, new Uint8Array(0)), 1200);
      if (!p || (p[0] | (p[1] << 8)) !== 0x0B82) return null;
      var dv = new DataView(p.buffer, p.byteOffset, p.length);
      return {
        txVfo:      p[4],
        screen:     p[5],
        func:       p[6],
        isTx:       p[7] === 1,
        rxFreq:     dv.getUint32(8, true),   // 10 Hz units
        txFreq:     dv.getUint32(12, true),
        modulation: p[16],                   // 0 FM, 1 AM, 2 USB
        bandwidth:  p[17],                   // 0 wide, 1 narrow
        power:      p[18],                   // OUTPUT_POWER_*
        channel:    p[19],
        squelch:    p[20],
        rssi:       dv.getUint16(22, true),
        batteryMv:  dv.getUint16(24, true)
      };
    });
  };
  function rcSet(id) {
    return function (value) {
      var self = this;
      return self._run(async function () {
        var p = await self._exchange(frameCommand(id, new Uint8Array([value & 0xFF])), 1200);
        return !!p && (p[0] | (p[1] << 8)) === 0x0B81 && p[6] === 1;
      });
    };
  }
  Radio.prototype.setPower      = rcSet(0x0B03);
  Radio.prototype.setBandwidth  = rcSet(0x0B04);
  Radio.prototype.setModulation = rcSet(0x0B05);

  // poll one display frame: send 0x0A03, read the raw push 0xAB 0xED + 1024
  // bytes (paged 128x64 framebuffer; NOT the AB CD/CRC envelope). Returns the
  // 1024-byte buffer or null.
  Radio.prototype.pollScreen = function (timeoutMs) {
    var self = this;
    return this._run(async function () {
      self.rx = new Uint8Array(0);
      await self.t.send(frameCommand(0x0A03, new Uint8Array(0)));
      var deadline = Date.now() + (timeoutMs || 1500);
      for (;;) {
        for (var i = 0; i + 1 < self.rx.length; i++) {
          if (self.rx[i] === 0xAB && self.rx[i + 1] === 0xED) {
            if (self.rx.length >= i + 2 + 1024) return self.rx.slice(i + 2, i + 2 + 1024);
            break;
          }
        }
        if (Date.now() >= deadline) return null;
        var chunk = await self.t.recv(1200, Math.min(400, deadline - Date.now()));
        if (chunk && chunk.length) self.rx = concat(self.rx, chunk);
      }
    });
  };

  // read a whole EEPROM range into one Uint8Array (default 0x0000..0x2000)
  Radio.prototype.dumpEeprom = async function (start, end, onProgress) {
    start = start || 0; end = end || 0x2000;
    var out = new Uint8Array(end - start);
    for (var a = start; a < end; a += 128) {
      var n = Math.min(128, end - a);
      var chunk = await this.readCfg(a, n);
      out.set(chunk.slice(0, n), a - start);
      if (onProgress) onProgress(a + n - start, end - start);
    }
    return out;
  };
  // fill a range with 0xFF, 128 bytes at a time (settings wipe; keep 0x1E00+)
  Radio.prototype.wipe = async function (start, end, onProgress) {
    var blank = new Uint8Array(128); blank.fill(0xFF);
    for (var a = start; a < end; a += 128) {
      var n = Math.min(128, end - a);
      await this.writeCfg(a, blank.slice(0, n));
      if (onProgress) onProgress(a + n - start, end - start);
    }
  };

  // --- bootloader (flash mode) ---
  Radio.prototype.waitBeacon = async function (timeoutMs) {
    var deadline = Date.now() + (timeoutMs || 5000);
    this.rx = new Uint8Array(0);
    while (Date.now() < deadline) {
      var p = await this.readFrame(Math.min(1500, deadline - Date.now()));
      if (p && (p[0] | (p[1] << 8)) === 0x0518) {
        var s = "";
        for (var i = 4; i < p.length && p[i]; i++) s += String.fromCharCode(p[i] >= 32 && p[i] < 127 ? p[i] : 46);
        return s;
      }
    }
    return null;
  };
  // flash raw firmware bytes; version must start with '*'. onProgress(done,total).
  Radio.prototype.flash = async function (fw, version, onProgress) {
    // 1) version handshake (0x0530), expect a beacon back
    var vbuf = new Uint8Array(16);
    for (var i = 0; i < version.length && i < 16; i++) vbuf[i] = version.charCodeAt(i);
    var vp = new Uint8Array(4 + 16);
    vp[0] = 0x30; vp[1] = 0x05; vp[2] = 0x10; vp[3] = 0x00; vp.set(vbuf, 4);
    this.rx = new Uint8Array(0);
    await this.t.send(frameRaw(vp));
    var r = await this.readFrame(3000);
    if (!r || (r[0] | (r[1] << 8)) !== 0x0518) throw new Error("version not accepted");

    // 2) 256-byte chunks (0x0519), acked by 0x051A
    var chunkCount = Math.ceil(fw.length / 256);
    if (chunkCount * 256 > 0xF000) throw new Error("firmware too large");
    var seq = 0x1D9F8D8A;
    for (var n = 0; n < chunkCount; n++) {
      var data = new Uint8Array(256); data.fill(0xFF);
      var dlen = Math.min(256, fw.length - n * 256);
      data.set(fw.slice(n * 256, n * 256 + dlen), 0);
      var pl = new Uint8Array(16 + 256);
      pl[0] = 0x19; pl[1] = 0x05; pl[2] = 0x0C; pl[3] = 0x01;
      pl[4] = seq & 0xFF; pl[5] = (seq >> 8) & 0xFF; pl[6] = (seq >> 16) & 0xFF; pl[7] = (seq >> 24) & 0xFF;
      pl[8] = n & 0xFF; pl[9] = (n >> 8) & 0xFF;
      pl[10] = chunkCount & 0xFF; pl[11] = (chunkCount >> 8) & 0xFF;
      pl[12] = dlen & 0xFF; pl[13] = (dlen >> 8) & 0xFF;
      pl.set(data, 16);
      this.rx = new Uint8Array(0);
      await this.t.send(frameRaw(pl));
      // wait for ack 0x051A (skip interleaved beacons)
      var ack = null, tries = 0;
      while (tries++ < 15) {
        var p = await this.readFrame(2000);
        if (!p) break;
        var id = p[0] | (p[1] << 8);
        if (id === 0x0518) continue;         // beacon, ignore
        ack = p; break;
      }
      if (!ack) throw new Error("no ack for chunk " + n);
      if ((ack[0] | (ack[1] << 8)) !== 0x051A || ack[10] !== 0) throw new Error("chunk " + n + " failed");
      var got = ack[8] | (ack[9] << 8);
      if (got !== n) throw new Error("chunk mismatch " + got + " != " + n);
      if (onProgress) onProgress(n + 1, chunkCount);
    }
    return true;
  };

  // ---- channel editor: EEPROM codec (egzumer/F4HWN layout) ----
  // CTCSS in 0.1 Hz; DCS values are octal-of-value (0x0013 -> "023").
  var CTCSS = [670,693,719,744,770,797,825,854,885,915,948,974,1000,1035,1072,1109,1148,1188,1230,1273,
    1318,1365,1413,1462,1514,1567,1598,1622,1655,1679,1713,1738,1773,1799,1835,1862,1899,1928,1966,1995,
    2035,2065,2107,2181,2257,2291,2336,2418,2503,2541];
  var DCS = [0x0013,0x0015,0x0016,0x0019,0x001A,0x001E,0x0023,0x0027,0x0029,0x002B,0x002C,0x0035,0x0039,0x003A,0x003B,0x003C,
    0x004C,0x004D,0x004E,0x0052,0x0055,0x0059,0x005A,0x005C,0x0063,0x0065,0x006A,0x006D,0x006E,0x0072,0x0075,0x007A,
    0x007C,0x0085,0x008A,0x0093,0x0095,0x0096,0x00A3,0x00A4,0x00A5,0x00A6,0x00A9,0x00AA,0x00AD,0x00B1,0x00B3,0x00B5,
    0x00B6,0x00B9,0x00BC,0x00C6,0x00C9,0x00CD,0x00D5,0x00D9,0x00DA,0x00E3,0x00E6,0x00E9,0x00EE,0x00F4,0x00F5,0x00F9,
    0x0109,0x010A,0x010B,0x0113,0x0119,0x011A,0x0125,0x0126,0x012A,0x012C,0x012D,0x0132,0x0134,0x0135,0x0136,0x0143,
    0x0146,0x014E,0x0153,0x0156,0x015A,0x0165,0x0166,0x0169,0x016C,0x0175,0x0186,0x018A,0x0194,0x0197,0x0199,0x019A,
    0x01AC,0x01B2,0x01B5,0x01B9,0x01BC,0x01C3,0x01CA,0x01D3]; // 104
  var STEP_HZ = [250,500,625,1000,1250,2500,833,1,5,10,25,50,100,125,900,1500,2000,3000,5000,10000,12500,20000,25000,50000]; // *10 Hz
  var POWER = ["USER","LOW1","LOW2","LOW3","LOW4","LOW5","MID","HIGH"];
  var MODU  = ["FM","AM","USB"];
  // CodeType: 0 off, 1 CTCSS, 2 DCS-N, 3 DCS-I
  function toneLabel(codeType, code){
    if (codeType === 1) return "CT " + (CTCSS[code] / 10).toFixed(1);
    if (codeType === 2) return "D" + ("00" + DCS[code].toString(8)).slice(-3) + "N";
    if (codeType === 3) return "D" + ("00" + DCS[code].toString(8)).slice(-3) + "I";
    return "off";
  }
  function decodeChannel(rec, attr, nameBytes){
    var dv = new DataView(rec.buffer, rec.byteOffset, rec.length);
    var empty = (attr === 0xFF);
    var name = "";
    for (var i = 0; nameBytes && i < 10 && nameBytes[i] >= 32 && nameBytes[i] < 127; i++) name += String.fromCharCode(nameBytes[i]);
    var d12 = rec[12];
    return {
      empty: empty,
      rxFreq: dv.getUint32(0, true),                 // 10 Hz units
      txOffset: dv.getUint32(4, true),
      rxCode: rec[8], txCode: rec[9],
      rxCodeType: rec[10] & 0x0F, txCodeType: (rec[10] >> 4) & 0x0F,
      offsetDir: rec[11] & 0x0F,                      // 0 off, 1 +, 2 -
      modulation: (rec[11] >> 4) & 0x0F,
      reverse: d12 & 1, bandwidth: (d12 >> 1) & 1, power: (d12 >> 2) & 7,
      busyLock: (d12 >> 5) & 1, txLock: (d12 >> 6) & 1,
      step: rec[14],
      band: attr & 7, compander: (attr >> 3) & 3,
      scan1: (attr >> 5) & 1, scan2: (attr >> 6) & 1, scan3: (attr >> 7) & 1,
      name: name.trim()
    };
  }
  // returns { rec:Uint8Array(16), attr:byte, name:Uint8Array(16) }
  function encodeChannel(c){
    var rec = new Uint8Array(16), dv = new DataView(rec.buffer);
    dv.setUint32(0, c.rxFreq >>> 0, true);
    dv.setUint32(4, (c.txOffset || 0) >>> 0, true);
    rec[8]  = c.rxCode & 0xFF; rec[9] = c.txCode & 0xFF;
    rec[10] = (c.rxCodeType & 0x0F) | ((c.txCodeType & 0x0F) << 4);
    rec[11] = (c.offsetDir & 0x0F) | ((c.modulation & 0x0F) << 4);
    rec[12] = (c.reverse & 1) | ((c.bandwidth & 1) << 1) | ((c.power & 7) << 2) | ((c.busyLock & 1) << 5) | ((c.txLock & 1) << 6);
    rec[13] = 0; rec[14] = c.step & 0xFF; rec[15] = 0;
    var attr = (c.band & 7) | ((c.compander & 3) << 3) | ((c.scan1 & 1) << 5) | ((c.scan2 & 1) << 6) | ((c.scan3 & 1) << 7);
    var name = new Uint8Array(16); name.fill(0);
    var s = (c.name || "").toUpperCase();
    for (var i = 0; i < 10 && i < s.length; i++) name[i] = s.charCodeAt(i) & 0x7F;
    return { rec: rec, attr: attr, name: name };
  }
  // read MR channels [first,last] -> array of decoded channel objects (with .index)
  Radio.prototype.readChannels = function (first, last, onProgress){
    var self = this; first = first || 0; last = (last == null) ? 199 : last;
    return this._run(async function(){
      var out = [];
      // attributes page-aligned block covering [first,last]
      var attrStart = 0x0D60 + first, attrLen = (last - first + 1);
      var attrs = await self._readRange(0x0D60 + (first & ~7), (((last)|7) + 1) - (first & ~7));
      for (var ch = first; ch <= last; ch++){
        var rec  = await self.readCfg(ch * 16, 16);
        var name = await self.readCfg(0x0F50 + ch * 16, 16);
        var attr = attrs[(ch) - (first & ~7)];
        var c = decodeChannel(rec, attr, name); c.index = ch;
        out.push(c);
        if (onProgress) onProgress(ch - first + 1, last - first + 1);
      }
      return out;
    });
  };
  // read a >128 range by chunking (used for the attribute block)
  Radio.prototype._readRange = async function (addr, len){
    var out = new Uint8Array(len);
    for (var a = 0; a < len; a += 128){ var n = Math.min(128, len - a); out.set(await this.readCfg(addr + a, n), a); }
    return out;
  };
  // write one channel (record + name + attribute), read-modify-write the attr page
  Radio.prototype.writeChannel = function (ch, c){
    var self = this;
    return this._run(async function(){
      var e = encodeChannel(c);
      await self.writeCfg(ch * 16, e.rec);                    // 16 B record (2 pages)
      await self.writeCfg(0x0F50 + ch * 16, e.name);          // 16 B name (2 pages)
      var pageAddr = 0x0D60 + (ch & ~7);                      // 8-channel attr page
      var page = await self.readCfg(pageAddr, 8);
      page = new Uint8Array(page); page[ch & 7] = e.attr;
      await self.writeCfg(pageAddr, page);
      return true;
    });
  };
  // mark a channel empty (attr 0xFF) in its page
  Radio.prototype.deleteChannel = function (ch){
    var self = this;
    return this._run(async function(){
      var pageAddr = 0x0D60 + (ch & ~7);
      var page = new Uint8Array(await self.readCfg(pageAddr, 8));
      page[ch & 7] = 0xFF;
      await self.writeCfg(pageAddr, page);
      return true;
    });
  };

  // ---- transport picker + auto-reconnect helper ----
  var UA = (typeof navigator !== "undefined" && navigator.userAgent) || "";
  var isMobile = /Android|iPhone|iPad|iPod|Mobile/i.test(UA);
  var isIOS    = /iPhone|iPad|iPod/i.test(UA);
  function supported() { return typeof navigator !== "undefined" && (("serial" in navigator) || ("usb" in navigator)); }
  // pick the right transport for a user-gesture connect (serial on desktop, usb on mobile)
  function pickTransport() {
    if (!isMobile && ("serial" in navigator)) return makeWebSerial();
    if ("usb" in navigator) return makeWebUsb(false);
    if ("serial" in navigator) return makeWebSerial();
    return null;
  }

  return {
    crc16: crc16, xorApply: xorApply, frameRaw: frameRaw, frameCommand: frameCommand,
    extractFrame: extractFrame, ch341Divisor: ch341Divisor,
    makeWebSerial: makeWebSerial, makeWebUsb: makeWebUsb, Radio: Radio,
    pickTransport: pickTransport, supported: supported,
    isMobile: isMobile, isIOS: isIOS, TS: TS,
    // channel editor codec + tables
    CTCSS: CTCSS, DCS: DCS, STEP_HZ: STEP_HZ, POWER: POWER, MODU: MODU,
    toneLabel: toneLabel, decodeChannel: decodeChannel, encodeChannel: encodeChannel
  };
})();

if (typeof module !== "undefined" && module.exports) module.exports = K5;  // for Node tests
