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
    isMobile: isMobile, isIOS: isIOS, TS: TS
  };
})();

if (typeof module !== "undefined" && module.exports) module.exports = K5;  // for Node tests
