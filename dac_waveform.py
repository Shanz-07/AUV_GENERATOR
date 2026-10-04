#!/usr/bin/env python3
"""
sonar_live.py  -  live "oscilloscope + spectrum analyser" fed by the REAL DAC
samples that main.c streams over the ST-Link virtual COM port.

main.c (DAC_STREAM_ENABLE 1) sends, every DAC_STREAM_EVERY pings, the exact
uint16 buffer that the DMA just clocked into the DAC, framed with a header and
a CRC.  This script finds those frames in the serial stream (the normal text
telemetry keeps working and is echoed to the console), and for every frame it
draws, from the actual samples:

  full pulse + envelope, DAC steps (+ optional analog LPF), instantaneous
  frequency, spectrogram, FFT spectrum, matched-filter (pulse compression)
  response, start/end edges, and an info panel (what the firmware decided vs
  what the samples really contain).

USAGE
  pip install pyserial numpy scipy matplotlib
  python sonar_live.py --list                       # find the COM port
  python sonar_live.py --port COM5                  # live window (Windows)
  python sonar_live.py --port /dev/ttyACM0          # Linux  (macOS: /dev/cu.usbmodem*)
  python sonar_live.py --port COM5 --once           # grab ONE frame -> PNG + CSV, exit
  python sonar_live.py --port COM5 --log run.bin    # also record raw bytes
  python sonar_live.py --replay run.bin             # replay a recording offline

IMPORTANT: close PuTTY / Tera Term / the CubeIDE console first - only one
program can own the COM port.  Baud rate is 921600 (same as the firmware).
"""
import argparse
import binascii
import math
import os
import queue
import struct
import sys
import threading
import time

import numpy as np

import matplotlib
if ("--once" in sys.argv or "--no-gui" in sys.argv) and "--show" not in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec, GridSpecFromSubplotSpec
from scipy.signal import butter, hilbert, sosfilt, sosfilt_zi, spectrogram

# ----------------------------------------------------------------------------
#  Frame format (must match DacStream() in main.c)
# ----------------------------------------------------------------------------
MAGIC = b"\xA5\x5A\xA5\x5A"
HDR_FMT = "<BBBBIHIIIIIIHHHHHH"
HDR_LEN = struct.calcsize(HDR_FMT)
assert HDR_LEN == 46
MAX_SAMPLES = 20000
TAIL_PAD = 2
BAND_MIN, BAND_MAX = 20_000.0, 50_000.0         # hardware band (PROFILE_MVP)
MODEL_MIN, MODEL_MAX = 100_000.0, 500_000.0     # payload band it stands in for
WAVE_NAMES = ["LFM", "GeoFM", "HFM", "Barker13"]
WIN_NAMES = ["Tukey", "Hann", "Hamming", "Blackman"]


def crc16(data, init=0xFFFF):
    return binascii.crc_hqx(data, init)          # CRC-16/CCITT-FALSE, same as firmware


class FrameParser:
    """Splits a mixed byte stream into ASCII telemetry lines and DAC frames."""

    def __init__(self):
        self.buf = bytearray()
        self.textbuf = ""
        self.frames_ok = 0
        self.frames_bad = 0

    def _text(self, chunk, out):
        self.textbuf += chunk.decode("ascii", errors="ignore")
        while "\n" in self.textbuf:
            line, self.textbuf = self.textbuf.split("\n", 1)
            out.append(("text", line.rstrip("\r")))

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(MAGIC)
            if i < 0:
                if len(self.buf) > 3:                       # keep a possible partial magic
                    chunk = bytes(self.buf[:-3])
                    del self.buf[:-3]
                    self._text(chunk, out)
                break
            if i > 0:
                self._text(bytes(self.buf[:i]), out)
                del self.buf[:i]
            if len(self.buf) < 4 + HDR_LEN:
                break
            n = struct.unpack_from("<H", self.buf, 4 + 8)[0]
            if n == 0 or n > MAX_SAMPLES:
                del self.buf[:1]
                self.frames_bad += 1
                continue
            total = 4 + HDR_LEN + 2 * n + 2
            if len(self.buf) < total:
                break
            payload = bytes(self.buf[4:total - 2])
            crc_rx = struct.unpack_from("<H", self.buf, total - 2)[0]
            if crc16(payload) != crc_rx:
                self.frames_bad += 1
                del self.buf[:1]
                continue
            out.append(("frame", decode_frame(payload)))
            self.frames_ok += 1
            del self.buf[:total]
        return out


def decode_frame(payload):
    (ver, wtype, win, amp, seq, n, fs_nom, fs_meas, f0, f1, fc, bw, dur, pri,
     range_dm, c_dms, alpha, res_mm) = struct.unpack_from(HDR_FMT, payload, 0)
    codes = np.frombuffer(payload, dtype="<u2", count=n, offset=HDR_LEN).astype(np.int32)
    return dict(ver=ver, type=wtype, window=win, amp_pct=amp, seq=seq, n=n,
                fs=fs_nom, fs_meas=fs_meas, f0=f0, f1=f1, fc_model=fc, bw_model=bw,
                dur_us=dur * 10, pri_ms=pri, range_m=range_dm / 10.0,
                c=c_dms / 10.0, alpha=alpha, res_mm=res_mm, codes=codes)


# ----------------------------------------------------------------------------
#  Analysis of the real samples
# ----------------------------------------------------------------------------
def pslr_txt(v):
    return "<-80" if v <= -79.9 else f"{v:.1f}"


def analyse(fr, vref):
    fs = float(fr["fs"])
    npulse = fr["n"] - TAIL_PAD
    codes = fr["codes"]
    x = (codes[:npulse] - 2048).astype(float)

    a = hilbert(x)
    env = np.abs(a)
    ph = np.unwrap(np.angle(a))
    finst = np.diff(ph) * fs / (2 * np.pi)
    k = int(max(5, npulse // 200))
    finst = np.convolve(finst, np.ones(k) / k, mode="same")
    finst[env[:-1] < 0.1 * env.max()] = np.nan

    N = 1 << 18
    X = np.fft.rfft(x, N)
    f = np.fft.rfftfreq(N, 1.0 / fs)
    mag = np.abs(X)
    db = 20 * np.log10(mag / mag.max() + 1e-12)

    def span(th):
        idx = np.where(db >= th)[0]
        return f[idx[0]], f[idx[-1]]

    f3, f10 = span(-3.0), span(-10.0)
    oob = ((f < fr["f0"] - 10e3) | (f > fr["f1"] + 10e3)) & (f > 1e3)
    worst_oob = db[oob].max() if oob.any() else -120.0

    M = 1 << int(math.ceil(math.log2(2 * len(a))))
    S = np.fft.fft(a, M)
    ac = np.fft.fftshift(np.fft.ifft(np.abs(S) ** 2))
    cenv = np.abs(ac)
    cenv /= cenv.max()
    lag = (np.arange(M) - M // 2) / fs
    pos = cenv[M // 2:]
    i1 = int(np.argmax(pos < 0.7071)) if (pos < 0.7071).any() else len(pos) - 1
    lag3 = (i1 - 1 + (pos[i1 - 1] - 0.7071) / max(pos[i1 - 1] - pos[i1], 1e-12)) / fs
    rises = np.where(np.diff(pos) > 0)[0]
    pslr = max(20 * np.log10(pos[rises[0]:].max() + 1e-12), -80.0) if len(rises) else -80.0

    fin = finst[np.isfinite(finst)]
    q = max(1, len(fin) // 7)
    sweep0, sweep1 = (float(np.median(fin[:q])), float(np.median(fin[-q:]))) if len(fin) else (0.0, 0.0)

    c = fr["c"] if fr["c"] > 0 else 1500.0
    bw3 = f3[1] - f3[0]
    return dict(x=x, env=env, finst=finst, f=f, db=db, peak_hz=float(f[int(np.argmax(mag))]),
                f3=f3, bw3=bw3, bw10=f10[1] - f10[0], tbp=bw3 * npulse / fs,
                lag=lag, cenv=cenv, lag3=lag3, res_hw_mm=c * lag3 * 1000.0, pslr=pslr,
                worst_oob=worst_oob, sweep0=sweep0, sweep1=sweep1,
                cmin=int(codes[:npulse].min()), cmax=int(codes[:npulse].max()),
                dc_lsb=float(x.mean()), clips=int(((codes <= 0) | (codes >= 4095)).sum()),
                vpp=(codes[:npulse].max() - codes[:npulse].min()) * vref / 4095.0,
                npulse=npulse)


def info_text(fr, m, stats):
    wname = WAVE_NAMES[fr["type"]] if fr["type"] < 4 else "?"
    wnn = WIN_NAMES[fr["window"]] if fr["window"] < 4 else "?"
    fsm = fr["fs_meas"] / 1e6
    c1 = (f"FIRMWARE DECISION (from frame header)\n"
          f"  Waveform : {wname} / {wnn}\n"
          f"  HW sweep : {fr['f0']/1e3:.1f} -> {fr['f1']/1e3:.1f} kHz\n"
          f"  Model    : fc {fr['fc_model']/1e3:.1f}, B {fr['bw_model']/1e3:.1f} kHz (payload)\n"
          f"  Pulse    : tau {fr['dur_us']} us, {fr['n']} samples\n"
          f"  Amplitude: {fr['amp_pct']} %    PRI {fr['pri_ms']} ms\n"
          f"  Link     : R {fr['range_m']:.1f} m, c {fr['c']:.1f} m/s, "
          f"alpha {fr['alpha']} dB/km")
    c2 = (f"MEASURED FROM THE REAL DAC SAMPLES\n"
          f"  Peak freq    : {m['peak_hz']/1e3:6.2f} kHz\n"
          f"  Sweep (meas.): {m['sweep0']/1e3:5.1f} -> {m['sweep1']/1e3:5.1f} kHz\n"
          f"  -3 dB BW     : {m['bw3']/1e3:6.2f} kHz   -10 dB: {m['bw10']/1e3:.2f}\n"
          f"  Time-BW prod.: {m['tbp']:6.1f}\n"
          f"  Range res.   : {m['res_hw_mm']:5.0f} mm (HW) vs {fr['res_mm']} mm (payload)\n"
          f"  PSLR         : {pslr_txt(m['pslr']):>6} dB   out-of-band {m['worst_oob']:.0f} dB")
    c3 = (f"DAC / LINK\n"
          f"  Swing     : codes {m['cmin']}..{m['cmax']}  ({m['vpp']:.2f} Vpp)\n"
          f"  DC offset : {m['dc_lsb']:+.1f} LSB   clipped samples: {m['clips']}\n"
          f"  Edges     : first {fr['codes'][0]}, last pulse "
          f"{fr['codes'][m['npulse']-1]}, parked {fr['codes'][-1]}\n"
          f"  fs        : {fr['fs']/1e6:.3f} MS/s nominal, {fsm:.3f} measured (incl. DMA)\n"
          f"  Frame     : seq {fr['seq']}   ok {stats['ok']}   CRC errors {stats['bad']}")
    return c1, c2, c3


def lpf_trace(volts, lpf_hz, fs, up=10):
    sos = butter(2, lpf_hz, fs=fs * up, output="sos")
    sig = np.repeat(volts, up)
    y, _ = sosfilt(sos, sig, zi=sosfilt_zi(sos) * sig[0])
    return np.arange(len(sig)) / (fs * up) * 1e6, y


# ----------------------------------------------------------------------------
#  Figure
# ----------------------------------------------------------------------------
def make_figure():
    fig = plt.figure(figsize=(15, 15))
    gs = GridSpec(5, 2, figure=fig, height_ratios=[0.6, 1, 1, 1, 1], hspace=0.5,
                  wspace=0.2, left=0.06, right=0.97, top=0.93, bottom=0.04)
    ax = dict(info=fig.add_subplot(gs[0, :]), full=fig.add_subplot(gs[1, 0]),
              zoom=fig.add_subplot(gs[1, 1]), freq=fig.add_subplot(gs[2, 0]),
              spec=fig.add_subplot(gs[2, 1]), fft=fig.add_subplot(gs[3, 0]),
              comp=fig.add_subplot(gs[3, 1]))
    ax["edge0"] = fig.add_subplot(gs[4, 0])
    ax["edge1"] = fig.add_subplot(gs[4, 1])
    ax["info"].axis("off")
    ax["info"].text(0.5, 0.5, "waiting for a DAC frame from the board ...", ha="center",
                    va="center", fontsize=14, transform=ax["info"].transAxes)
    return fig, ax


def draw(fig, ax, fr, vref, lpf_khz, stats):
    for a in ax.values():
        a.cla()
    m = analyse(fr, vref)
    fs = float(fr["fs"])
    codes = fr["codes"]
    n, npulse = fr["n"], m["npulse"]
    t_us = np.arange(n) / fs * 1e6
    tau_us = npulse / fs * 1e6
    volts = codes / 4095.0 * vref
    mid = vref * 2048 / 4095.0
    f0, f1 = fr["f0"] / 1e3, fr["f1"] / 1e3
    wname = WAVE_NAMES[fr["type"]] if fr["type"] < 4 else "?"
    wnn = WIN_NAMES[fr["window"]] if fr["window"] < 4 else "?"

    fig.suptitle(f"LIVE DAC OUTPUT  |  {wname} / {wnn}  |  {f0:.1f}-{f1:.1f} kHz  |  "
                 f"tau {fr['dur_us']} us  |  amp {fr['amp_pct']} %  |  frame #{fr['seq']}",
                 fontsize=14, fontweight="bold")

    ax["info"].axis("off")
    for x0, txt in zip((0.0, 0.33, 0.69), info_text(fr, m, stats)):
        ax["info"].text(x0, 0.98, txt, family="monospace", fontsize=8.8, va="top",
                        transform=ax["info"].transAxes,
                        bbox=dict(boxstyle="round", fc="#f2f2f2", ec="#bbbbbb"))

    a = ax["full"]
    a.step(t_us, volts, where="post", lw=0.4, color="#1f4e9c", label="DAC output")
    a.plot(t_us[:npulse], mid + m["env"] / 4095.0 * vref, "--", color="#d62728", lw=1.1,
           label="envelope (from samples)")
    a.plot(t_us[:npulse], mid - m["env"] / 4095.0 * vref, "--", color="#d62728", lw=1.1)
    a.axhline(mid, color="gray", lw=0.6)
    a.set(title="DAC output - full pulse (the scope view)", xlabel="time (us)",
          ylabel="volts", ylim=(0, vref))
    a.legend(loc="upper right", fontsize=8)
    a.grid(alpha=0.3)

    a = ax["zoom"]
    span = min(80.0, tau_us)
    t0 = max(0.0, tau_us / 2 - span / 2)
    sel = (t_us >= t0) & (t_us <= t0 + span)
    a.step(t_us[sel], volts[sel], where="post", lw=1.0, color="#1f4e9c", label="DAC steps (ZOH)")
    a.plot(t_us[sel], volts[sel], ".", ms=3, color="#1f4e9c")
    if lpf_khz > 0:
        tu, yu = lpf_trace(volts, lpf_khz * 1e3, fs)
        s2 = (tu >= t0) & (tu <= t0 + span)
        a.plot(tu[s2], yu[s2], color="#2ca02c", lw=1.8, label=f"after 2nd-order LPF {lpf_khz:.0f} kHz")
    a.set(title=f"Zoom ({span:.0f} us at pulse centre, 1 sample = {1e6/fs:.1f} us)",
          xlabel="time (us)", ylabel="volts")
    a.legend(loc="upper right", fontsize=8)
    a.grid(alpha=0.3)

    a = ax["freq"]
    a.plot(t_us[:npulse - 1], m["finst"] / 1e3, color="#ff7f0e", lw=1.5, label="measured (from samples)")
    a.axhline(f0, color="gray", ls=":", lw=0.9)
    a.axhline(f1, color="gray", ls=":", lw=0.9, label="firmware start / end")
    pad = max(2.0, 0.4 * (f1 - f0))
    a.set(title="Instantaneous frequency of the real samples", xlabel="time (us)",
          ylabel="kHz (hardware)", ylim=(max(0, f0 - pad), f1 + pad))
    a.legend(loc="upper right", fontsize=8)
    a.grid(alpha=0.3)

    a = ax["spec"]
    nper = int(max(32, min(256, npulse // 6)))
    fr_, tt, Sxx = spectrogram(m["x"], fs=fs, window="hann", nperseg=nper,
                               noverlap=nper * 7 // 8, nfft=1024, mode="magnitude")
    Sdb = 20 * np.log10(Sxx / Sxx.max() + 1e-6)
    a.pcolormesh(tt * 1e6, fr_ / 1e3, Sdb, vmin=-70, vmax=0, shading="auto", cmap="magma")
    a.set(title="Spectrogram (dB, -70..0)", xlabel="time (us)", ylabel="kHz",
          ylim=(0, max(80.0, f1 * 1.6)))

    a = ax["fft"]
    a.plot(m["f"] / 1e3, m["db"], color="#1f4e9c", lw=0.9)
    a.axvspan(f0, f1, color="#ff7f0e", alpha=0.18, label=f"firmware sweep {f0:.1f}-{f1:.1f} kHz")
    a.axhline(-3, color="gray", ls=":", lw=0.8)
    a.axhline(-10, color="gray", ls=":", lw=0.8)
    a.set(title=f"FFT of the real DAC samples (peak {m['peak_hz']/1e3:.1f} kHz, "
                f"-3 dB BW {m['bw3']/1e3:.1f} kHz)", xlabel="kHz", ylabel="dB rel. peak",
          xlim=(0, fs / 2e3), ylim=(-100, 3))
    a.legend(loc="upper right", fontsize=8)
    a.grid(alpha=0.3)

    a = ax["comp"]
    halfspan = min(tau_us * 1e-6, max(12 * m["lag3"], 30e-6))
    sel = np.abs(m["lag"]) <= halfspan
    c = fr["c"] if fr["c"] > 0 else 1500.0
    a.plot(c * m["lag"][sel] / 2 * 1000, 20 * np.log10(m["cenv"][sel] + 1e-6), color="#d62728", lw=1.4)
    a.axhline(-3, color="gray", ls=":", lw=0.8)
    a.set(title=f"Pulse compression: res {m['res_hw_mm']:.0f} mm, PSLR {pslr_txt(m['pslr'])} dB",
          xlabel="range offset (mm)", ylabel="dB", ylim=(-60, 3))
    a.grid(alpha=0.3)

    edge = min(120.0, tau_us / 2)
    for key, (lo, hi, ttl) in {"edge0": (0, edge, "start edge"),
                               "edge1": (tau_us - edge, tau_us + 2, "end edge")}.items():
        a = ax[key]
        s = (t_us >= lo) & (t_us <= hi)
        a.step(t_us[s], volts[s], where="post", lw=0.9, color="#1f4e9c")
        a.axhline(mid, color="gray", lw=0.6)
        a.set(title=f"{ttl} (parked at code 2048)", xlabel="time (us)", ylabel="volts", ylim=(0, vref))
        a.grid(alpha=0.3)


def save_outputs(fr, fig, out_dir, vref, csv=True):
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.join(out_dir, f"frame_{fr['seq']:06d}")
    fig.savefig(base + ".png", dpi=100)
    paths = [base + ".png"]
    if csv:
        with open(base + ".csv", "w") as fh:
            fh.write("sample,time_us,dac_code,volts\n")
            for i, c in enumerate(fr["codes"]):
                fh.write(f"{i},{i / fr['fs'] * 1e6:.3f},{int(c)},{c / 4095.0 * vref:.5f}\n")
        paths.append(base + ".csv")
    return paths


# ----------------------------------------------------------------------------
#  Serial reader thread
# ----------------------------------------------------------------------------
def reader_thread(port, baud, q, stop, logfile, parser):
    import serial
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except Exception as e:                                   # noqa: BLE001
        q.put(("error", f"cannot open {port}: {e}"))
        return
    while not stop.is_set():
        try:
            data = ser.read(4096)
        except Exception as e:                               # noqa: BLE001
            q.put(("error", f"serial error: {e}"))
            break
        if data:
            if logfile:
                logfile.write(data)
                logfile.flush()
            for ev in parser.feed(data):
                q.put(ev)
    ser.close()


def list_ports():
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        print("pyserial missing: pip install pyserial")
        return
    ports = list(lp.comports())
    if not ports:
        print("no serial ports found")
    for p in ports:
        print(f"{p.device:<14} {p.description}")


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Live plot of the real DAC samples from the board")
    ap.add_argument("--port", help="COM5, /dev/ttyACM0, ...")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--replay", help="raw capture (.bin) recorded with --log")
    ap.add_argument("--delay", type=float, default=1.5, help="seconds per frame in --replay")
    ap.add_argument("--log", help="record the raw serial bytes to this file")
    ap.add_argument("--once", action="store_true", help="save PNG+CSV of the first frame and exit")
    ap.add_argument("--no-gui", action="store_true", help="no window (saves PNG every frame)")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--save", action="store_true", help="save PNG+CSV of every frame")
    ap.add_argument("--out", default="live_frames")
    ap.add_argument("--lpf", type=float, default=150.0, help="analog LPF cutoff kHz (0 = off)")
    ap.add_argument("--vref", type=float, default=3.3)
    ap.add_argument("--quiet", action="store_true", help="do not echo the text telemetry")
    a = ap.parse_args()

    if a.list:
        list_ports()
        return
    if not a.port and not a.replay:
        ap.error("give --port COMx (see --list) or --replay file.bin")

    parser = FrameParser()
    fig, ax = make_figure()
    gui = not (a.once or a.no_gui)
    if gui:
        plt.ion()
        plt.show(block=False)

    def stats():
        return dict(ok=parser.frames_ok, bad=parser.frames_bad)

    def show_frame(fr):
        draw(fig, ax, fr, a.vref, a.lpf, stats())
        if a.save or a.once or a.no_gui:
            for p in save_outputs(fr, fig, a.out, a.vref, csv=True):
                print("saved", p)
        if gui:
            fig.canvas.draw_idle()
            fig.canvas.flush_events()

    # ---------------- offline replay ----------------
    if a.replay:
        with open(a.replay, "rb") as fh:
            events = parser.feed(fh.read())
        frames = [v for k, v in events if k == "frame"]
        if not a.quiet:
            for k, v in events:
                if k == "text":
                    print(v)
        print(f"{len(frames)} frame(s) found, {parser.frames_bad} CRC/format error(s)")
        for fr in frames:
            show_frame(fr)
            if a.once:
                return
            if gui:
                plt.pause(a.delay)
        if gui and frames:
            plt.show(block=True)
        return

    # ---------------- live serial ----------------
    q = queue.Queue()
    stop = threading.Event()
    logfile = open(a.log, "wb") if a.log else None
    th = threading.Thread(target=reader_thread, args=(a.port, a.baud, q, stop, logfile, parser),
                          daemon=True)
    th.start()
    print(f"listening on {a.port} @ {a.baud} - waiting for DAC frames "
          f"(one every DAC_STREAM_EVERY pings) ...")
    try:
        while True:
            newest = None
            try:
                while True:
                    kind, val = q.get_nowait()
                    if kind == "text" and not a.quiet:
                        print(val)
                    elif kind == "frame":
                        newest = val
                    elif kind == "error":
                        print(val)
                        return
            except queue.Empty:
                pass
            if newest is not None:
                show_frame(newest)
                if a.once:
                    return
            if gui:
                if not plt.fignum_exists(fig.number):
                    break
                plt.pause(0.05)
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if logfile:
            logfile.close()


if __name__ == "__main__":
    main()