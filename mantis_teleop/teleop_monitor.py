#!/usr/bin/env python3
"""Teleop session monitor: run it on the HOST around a recording session, read the report after."""
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

DIR = Path(__file__).resolve().parent
LOGS = DIR / "logs"
CSV = LOGS / "monitor.csv"
PIDF = LOGS / "monitor.pid"
CONTAINER = "mantis"
DATA_DIR = DIR / "lerobot_data"
ROBOT_IPS = {"left": "192.168.1.21", "right": "192.168.1.20"}
COLS = ["t", "udp_in", "udp_drop", "cpu_busy", "cpu_mhz", "cpu_temp", "mem_avail_mb", "dirty_mb",
        "psi_cpu", "psi_io", "psi_mem", "disk_w_mbs", "disk_free_gb", "teleop_rss_mb", "teleop_cpu",
        "teleop_thr", "rc_rss_mb", "rc_cpu", "gpu_temp", "quest_status", "quest_cpu_temp",
        "quest_batt_temp", "quest_batt_level"]
NAN = float("nan")


def read_udp():
    hdr = val = None
    for ln in Path("/proc/net/snmp").read_text().splitlines():
        if ln.startswith("Udp:"):
            if hdr is None:
                hdr = ln.split()[1:]
            else:
                val = ln.split()[1:]
    d = dict(zip(hdr, map(int, val)))
    return d["InDatagrams"], d["RcvbufErrors"]


def read_cpu_ticks():
    f = Path("/proc/stat").read_text().splitlines()[0].split()[1:]
    v = list(map(int, f))
    idle = v[3] + v[4]
    return sum(v) - idle, sum(v)


def read_mhz():
    xs = [float(l.split(":")[1]) for l in Path("/proc/cpuinfo").read_text().splitlines() if l.startswith("cpu MHz")]
    return sum(xs) / len(xs) if xs else NAN


_k10 = None
def read_cpu_temp():
    global _k10
    if _k10 is None:
        _k10 = ""
        for h in Path("/sys/class/hwmon").glob("hwmon*"):
            try:
                if (h / "name").read_text().strip() == "k10temp":
                    _k10 = str(h / "temp1_input")
            except OSError:
                pass
    try:
        return int(Path(_k10).read_text()) / 1000.0 if _k10 else NAN
    except (OSError, ValueError):
        return NAN


def read_mem():
    d = {}
    for ln in Path("/proc/meminfo").read_text().splitlines():
        k, v = ln.split(":")
        d[k] = int(v.split()[0])
    return d["MemAvailable"] / 1024.0, d["Dirty"] / 1024.0


def read_psi():
    out = []
    for k in ("cpu", "io", "memory"):
        try:
            m = re.search(r"some avg10=([\d.]+)", Path(f"/proc/pressure/{k}").read_text())
            out.append(float(m[1]) if m else NAN)
        except OSError:
            out.append(NAN)
    return out


_devkey = None
def read_disk_written():
    """Sectors written on the block device holding lerobot_data."""
    global _devkey
    if _devkey is None:
        st = os.stat(DATA_DIR if DATA_DIR.exists() else DIR)
        _devkey = (os.major(st.st_dev), os.minor(st.st_dev))
    for ln in Path("/proc/diskstats").read_text().splitlines():
        f = ln.split()
        if (int(f[0]), int(f[1])) == _devkey:
            return int(f[9])
    return 0


def disk_free_gb():
    try:
        s = os.statvfs(DATA_DIR if DATA_DIR.exists() else DIR)
        return s.f_bavail * s.f_frsize / 1e9
    except OSError:
        return NAN


def find_pid(match):
    """PID of the first process whose argv satisfies `match(argv)`."""
    for p in Path("/proc").glob("[0-9]*"):
        try:
            argv = (p / "cmdline").read_bytes().split(b"\0")
            argv = [a.decode(errors="replace") for a in argv if a]
            if argv and match(argv):
                return int(p.name)
        except OSError:
            continue
    return None


is_teleop = lambda a: len(a) >= 2 and a[1].endswith("teleop_mantis.py") and "python" in os.path.basename(a[0])
is_rc = lambda a: os.path.basename(a[0]) == "ros2_control_node"


def proc_stats(pid):
    """(rss_mb, threads, cpu_ticks) or None if the process is gone."""
    try:
        st = Path(f"/proc/{pid}/status").read_text()
        rss = int(re.search(r"VmRSS:\s+(\d+)", st)[1]) / 1024.0
        thr = int(re.search(r"Threads:\s+(\d+)", st)[1])
        f = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return rss, thr, int(f[11]) + int(f[12])
    except (OSError, TypeError, IndexError):
        return None


def slow_sampler(shared, stop, no_adb):
    """Every 30 s: GPU temperature and Quest thermals (adb)."""
    while not stop.is_set():
        try:
            out = subprocess.run(["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader,nounits"],
                                 capture_output=True, text=True, timeout=8).stdout.strip().split("\n")[0]
            shared["gpu_temp"] = float(out)
        except Exception:
            pass
        if not no_adb:
            try:
                th = subprocess.run(["adb", "shell", "dumpsys", "thermalservice"], capture_output=True, text=True, timeout=8).stdout
                m = re.search(r"Thermal Status:\s*(\d+)", th)
                shared["quest_status"] = float(m[1]) if m else NAN
                m = re.search(r"mValue=([\d.]+),\s*mType=0,\s*mName=cpu", th)
                shared["quest_cpu_temp"] = float(m[1]) if m else NAN
                bt = subprocess.run(["adb", "shell", "dumpsys", "battery"], capture_output=True, text=True, timeout=8).stdout
                m = re.search(r"temperature:\s*(\d+)", bt)
                shared["quest_batt_temp"] = float(m[1]) / 10.0 if m else NAN
                m = re.search(r"level:\s*(\d+)", bt)
                shared["quest_batt_level"] = float(m[1]) if m else NAN
            except Exception:
                pass
        stop.wait(30.0)


def sample_loop(no_adb):
    LOGS.mkdir(parents=True, exist_ok=True)
    pings = []
    for side, ip in ROBOT_IPS.items():
        try:
            f = open(LOGS / f"monitor_ping_{side}.log", "a")
            f.write(f"# start {time.time():.3f}\n"); f.flush()
            pings.append(subprocess.Popen(["ping", "-D", "-n", "-i", "1", "-W", "1", ip], stdout=f, stderr=subprocess.STDOUT))
        except Exception:
            pass
    stop = threading.Event()
    shared = {k: NAN for k in ("gpu_temp", "quest_status", "quest_cpu_temp", "quest_batt_temp", "quest_batt_level")}
    threading.Thread(target=slow_sampler, args=(shared, stop, no_adb), daemon=True).start()

    def bye(*_):
        stop.set()
        for p in pings:
            try:
                p.terminate()
            except Exception:
                pass
        PIDF.unlink(missing_ok=True)
        os._exit(0)
    signal.signal(signal.SIGTERM, bye); signal.signal(signal.SIGINT, bye)

    clk = os.sysconf("SC_CLK_TCK")
    busy0, tot0 = read_cpu_ticks(); wr0 = read_disk_written()
    pids = {"teleop": None, "rc": None}; pid_t = 0.0
    prev = {}
    with open(CSV, "a") as f:
        f.write(f"# start {time.time():.3f}\n# cols {','.join(COLS)}\n"); f.flush()
        while True:
            t = time.time()
            row = {"t": t}
            try:
                row["udp_in"], row["udp_drop"] = read_udp()
                busy, tot = read_cpu_ticks()
                row["cpu_busy"] = 100.0 * (busy - busy0) / max(tot - tot0, 1); busy0, tot0 = busy, tot
                row["cpu_mhz"] = read_mhz(); row["cpu_temp"] = read_cpu_temp()
                row["mem_avail_mb"], row["dirty_mb"] = read_mem()
                row["psi_cpu"], row["psi_io"], row["psi_mem"] = read_psi()
                wr = read_disk_written(); row["disk_w_mbs"] = (wr - wr0) * 512 / 1e6; wr0 = wr
                row["disk_free_gb"] = disk_free_gb()
                if t - pid_t > 10.0 or not all(pids.values()):
                    pid_t = t
                    pids["teleop"] = pids["teleop"] if pids["teleop"] and proc_stats(pids["teleop"]) else find_pid(is_teleop)
                    pids["rc"] = pids["rc"] if pids["rc"] and proc_stats(pids["rc"]) else find_pid(is_rc)
                for key, col in (("teleop", "teleop"), ("rc", "rc")):
                    ps = proc_stats(pids[key]) if pids[key] else None
                    if ps is None:
                        row[f"{col}_rss_mb"] = NAN; row[f"{col}_cpu"] = NAN
                        if col == "teleop": row["teleop_thr"] = NAN
                        prev.pop(key, None); pids[key] = None
                        continue
                    rss, thr, ticks = ps
                    pt, pticks = prev.get(key, (t - 1.0, ticks))
                    row[f"{col}_rss_mb"] = rss; row[f"{col}_cpu"] = 100.0 * (ticks - pticks) / clk / max(t - pt, 1e-3)
                    if col == "teleop": row["teleop_thr"] = thr
                    prev[key] = (t, ticks)
                row.update(shared)
            except Exception as exc:
                f.write(f"# sampler error {exc}\n")
            f.write(",".join(f"{row.get(c, NAN):.3f}" if isinstance(row.get(c, NAN), float) else str(row.get(c, "")) for c in COLS) + "\n")
            f.flush()
            time.sleep(max(0.0, 1.0 - (time.time() - t)))


def alive():
    try:
        pid = int(PIDF.read_text()); os.kill(pid, 0); return pid
    except (FileNotFoundError, ValueError, ProcessLookupError, PermissionError):
        return None


def start():
    if alive():
        print(f"already running (pid {alive()}), logging to {CSV}"); return
    LOGS.mkdir(parents=True, exist_ok=True)
    no_adb = "--no-adb" in sys.argv
    if os.fork():
        time.sleep(0.5)
        print(f"teleop_monitor running in the background (pid {alive()}) -> {CSV}"
              f"{' (Quest adb polling OFF)' if no_adb else ' (Quest thermals via adb every 30 s)'}\n"
              f"record now; afterwards: {sys.argv[0]} report")
        return
    os.setsid()
    if os.fork():
        os._exit(0)
    PIDF.write_text(str(os.getpid()))
    sample_loop(no_adb)


def stop():
    pid = alive()
    if pid:
        os.kill(pid, signal.SIGTERM); print(f"stopped pid {pid}")
    else:
        print("not running")
    PIDF.unlink(missing_ok=True)


def load_rows():
    rows, cols = [], COLS
    for ln in CSV.read_text().splitlines():
        if ln.startswith("# start"):
            rows = []
        elif ln.startswith("# cols"):
            cols = ln.split()[2].split(",")
        elif ln and not ln.startswith("#"):
            vals = ln.split(",")
            rows.append({c: (float(v) if v not in ("", "nan") else NAN) for c, v in zip(cols, vals)})
    return rows


def col(rows, k):
    return [r[k] for r in rows if r.get(k) == r.get(k)]


def pct(xs, q):
    if not xs:
        return NAN
    s = sorted(xs); i = min(len(s) - 1, max(0, int(round(q / 100.0 * (len(s) - 1)))))
    return s[i]


def med(xs):
    return pct(xs, 50)


def docker_grep(pattern, files):
    if not files:
        return ""
    cmd = f"grep -hE {json.dumps(pattern)} {' '.join(files)} 2>/dev/null"
    return subprocess.run(["docker", "exec", CONTAINER, "bash", "-c", cmd], capture_output=True, text=True, timeout=60).stdout


def docker_files(glob_pat, since):
    cmd = f"find /home/ros/.ros/log -maxdepth 1 -name {json.dumps(glob_pat)} -newermt @{int(since)} 2>/dev/null"
    return subprocess.run(["docker", "exec", CONTAINER, "bash", "-c", cmd], capture_output=True, text=True, timeout=30).stdout.split()


TS = re.compile(r"^\[(\w+)\] \[(\d+\.\d+)\] \[([^\]]+)\]: (.*)$")
TELEOP_PAT = ("TICKTIME|OUTSTREAM|POSESTREAM n=|EPISODE|recorder:|\\]: ENGAGED$|\\]: FROZEN$|HOME:|stale|DEGRADED|SNAPS|"
              "frozen|re-anchored|barrier holding|singularity|auto-freeze|not following|inside safety gate|cam sync|"
              "cameras delivered|measured frame rate|frames dropped|frame skipped|SAVE FAILED|DISCARDED|LOST|Teleop ready")

FLICK_SCAN = r'''
import glob, sys, json, warnings; warnings.filterwarnings("ignore")
import numpy as np, pandas as pd
root, ep_min = sys.argv[1], int(sys.argv[2])
files = sorted(glob.glob(f"{root}/data/**/*.parquet", recursive=True))
out = []
if files:
    df = pd.concat([pd.read_parquet(f) for f in files], ignore_index=True)
    df = df[df.episode_index >= ep_min].reset_index(drop=True)
    A = np.stack(df["action"].to_numpy()) if len(df) else np.zeros((0, 7))
    S = np.stack(df["observation.state"].to_numpy()) if len(df) else np.zeros((0, 7))
    for ep, g in df.groupby("episode_index"):
        idx = g.index.to_numpy(); a = A[idx][:, :6]; s = S[idx][:, :6]
        v = np.linalg.norm(np.diff(a, axis=0), axis=1) * 15.0
        moving = v > 0.05
        stall = sum(1 for i in range(2, len(v) - 1) if v[i-1] > 0.08 and v[i] < 0.3 * v[i-1] and v[i+1] > 2.0 * v[i] and v[i+1] > 0.08)
        d = np.diff(a, axis=0); rev = 0
        for j in range(6):
            sg = np.sign(d[:, j]) * (np.abs(d[:, j]) > 0.002); run = 0
            for i in range(1, len(sg)):
                if sg[i] != 0 and sg[i] == -sg[i-1]: run += 1
                else:
                    if run >= 3: rev += 1
                    run = 0
        lag = np.abs(a - s).max(axis=1)
        jerk = np.diff(v); mv = moving[1:] & moving[:-1]
        smooth = float(np.sqrt(np.mean(jerk[mv] ** 2)) / max(v[moving].mean(), 1e-6)) if mv.sum() > 5 else None
        out.append(dict(ep=int(ep), secs=len(g) / 15.0, moving=float(moving.mean() * 100), stall=int(stall), rev=int(rev),
                        smooth=smooth, lag95=float(np.percentile(lag, 95)), vmax=float(v.max()) if len(v) else 0.0))
print(json.dumps(out))
'''


def teleop_context(t0, t1):
    """Everything the report needs from the container's teleop log(s) within [t0, t1]."""
    ctx = dict(events=[], episodes=[], eng=[], home=[], wins=[], dataset=None, logs=[])
    files = [f for f in docker_files("python3_*.log", t0 - 60) if re.search(r"_(\d+)\.log$", f) and int(re.search(r"_(\d+)\.log$", f)[1]) / 1000.0 <= t1]
    files = [f for f in files if docker_grep("vive_mantis_pink_bridge", [f]).strip()]
    ctx["logs"] = files
    ev = []
    for ln in docker_grep(TELEOP_PAT, files).splitlines():
        m = TS.match(ln.rstrip())
        if m and t0 - 60 <= float(m[2]) <= t1 + 60:
            ev.append((float(m[2]), m[4]))
    ev.sort()
    ctx["events"] = ev
    cur_r = cur_e = cur_h = None
    for t, x in ev:
        m = re.match(r"EPISODE (\d+) RECORDING", x)
        if m:
            cur_r = dict(ep=int(m[1]), start=t, stop=None, frames=None, dropped=0, hz=None, discarded=False, poor_sync=False)
        elif re.match(r"EPISODE \d+ (STOPPED|DISCARDED)", x) and cur_r:
            cur_r["stop"] = t; cur_r["discarded"] = "DISCARDED" in x
            m = re.search(r"\((\d+) frames", x); cur_r["frames"] = int(m[1]) if m else None
            ctx["episodes"].append(cur_r); cur_r = None
        elif cur_r and (x.startswith("recorder:") or x.startswith("cam sync")):
            m = re.search(r"(\d+) frames dropped this episode", x)
            if m: cur_r["dropped"] = int(m[1])
            m = re.search(r"(?:cameras delivered|measured frame rate) ([\d.]+) Hz", x)
            if m: cur_r["hz"] = float(m[1])
            if "POOR SYNC" in x: cur_r["poor_sync"] = True
        elif x == "ENGAGED":
            cur_e = t
        elif x == "FROZEN" and cur_e is not None:
            ctx["eng"].append((cur_e, t)); cur_e = None
        elif x.startswith("HOME: moving"):
            cur_h = t
        elif x.startswith("HOME: reached") and cur_h is not None:
            ctx["home"].append((cur_h, t)); cur_h = None
        m = re.search(r"recorder: dataset \S+ at (\S+)", x)
        if m: ctx["dataset"] = m[1].rstrip(",;")
    if cur_r and cur_r["stop"] is None:
        cur_r["stop"] = t1; cur_r["open"] = True; ctx["episodes"].append(cur_r)
    if cur_e is not None:
        ctx["eng"].append((cur_e, t1))
    for t, x in ev:
        if x.startswith("TICKTIME"):
            m = re.search(r"p95=([\d.]+) max=([\d.]+)", x)
            ctx["wins"].append(dict(t=t, tick_p95=float(m[1]), tick_max=float(m[2])))
        elif x.startswith("OUTSTREAM") and ctx["wins"]:
            m = re.search(r"p95=([\d.]+) max=([\d.]+)", x); s = re.search(r"SNAPS (\d+)", x)
            ctx["wins"][-1].update(out_p95=float(m[1]), out_max=float(m[2]), snaps=int(s[1]) if s else 0)
        elif x.startswith("POSESTREAM n=") and ctx["wins"]:
            m = re.search(r"n=(\d+) .*p50=([\d.]+) p95=([\d.]+) max=([\d.]+) ms \| dropouts\(>\d+ms\) \+(\d+).*glitches \+(\d+).*holds \+(\d+)", x)
            if m:
                ctx["wins"][-1].update(pn=int(m[1]), p50=float(m[2]), p95=float(m[3]), pmax=float(m[4]), drops=int(m[5]), gl=int(m[6]), holds=int(m[7]))
    return ctx


def inside(iv, t):
    return any(a <= t <= b for a, b in iv)


def report():
    if not CSV.exists():
        print(f"no samples at {CSV} - run `start` first"); return
    rows = load_rows()
    if len(rows) < 3:
        print("fewer than 3 samples yet"); return
    t0, t1 = rows[0]["t"], rows[-1]["t"]
    hhmm = lambda t: time.strftime("%H:%M:%S", time.localtime(t))
    problems, clean, notes = [], [], []
    print(f"=== teleop session monitor: {hhmm(t0)} - {hhmm(t1)} ({(t1 - t0) / 60:.1f} min, {len(rows)} samples)"
          + ("  [monitor still running]" if alive() else ""))

    try:
        ctx = teleop_context(t0, t1)
    except Exception as exc:
        ctx = None; print(f"(teleop log correlation unavailable: {exc})")
    eps = ctx["episodes"] if ctx else []
    rec_iv = [(e["start"], e["stop"]) for e in eps]
    eng_iv = ctx["eng"] if ctx else []
    home_iv = ctx["home"] if ctx else []
    state_of = lambda t: "recording" if inside(rec_iv, t) else ("homing" if inside(home_iv, t) else ("engaged, not recording" if inside(eng_iv, t) else "parked"))
    if ctx:
        print(f"teleop log(s): {', '.join(os.path.basename(f) for f in ctx['logs']) or 'none found for this window'}"
              f" | episodes: {len(eps)} ({sum(e['stop'] - e['start'] for e in eps) / 60:.1f} min recorded)"
              + (f" | dataset: {ctx['dataset']}" if ctx["dataset"] else ""))

    if ctx and ctx["wins"]:
        w_in = [w for w in ctx["wins"] if inside(rec_iv, w["t"]) and inside(rec_iv, w["t"] - 9) and "pn" in w]
        w_eng = [w for w in ctx["wins"] if not inside(rec_iv, w["t"]) and not inside(rec_iv, w["t"] - 9) and inside(eng_iv, w["t"]) and inside(eng_iv, w["t"] - 9) and not inside(home_iv, w["t"]) and "pn" in w]
        def show(label, ws):
            if not ws:
                return
            c = lambda k: med([w[k] for w in ws if k in w])
            print(f"  {label:34s} {len(ws):3d} win | tick p95 {c('tick_p95'):4.2f} max {c('tick_max'):5.1f} ms | shaper max {c('out_max'):5.1f} ms"
                  f" (>=16 ms in {sum(1 for w in ws if w.get('out_max', 0) >= 16)}) | pose {c('pn') / 10:4.1f} Hz p95 {c('p95'):4.1f} max {c('pmax'):4.0f} ms"
                  f" dropouts {sum(w.get('drops', 0) for w in ws)} glitches {sum(w.get('gl', 0) for w in ws)} holds {sum(w.get('holds', 0) for w in ws)}")
        print("\n-- A. control loop (10-s windows from the teleop log)")
        show("recording", w_in); show("engaged, not recording", w_eng)
        if w_in and w_eng:
            dif = [(k, med([w[k] for w in w_in]), med([w[k] for w in w_eng])) for k in ("tick_max", "out_max", "p95", "pmax")]
            worse = [f"{k} {b:.1f}->{a:.1f}" for k, a, b in dif if a > 1.3 * b + 1.0]
            (problems.append(("MEDIUM", "control loop is worse while recording than engaged-only", ", ".join(worse))) if worse
             else clean.append("recording vs engaged-only: same tick / shaper / pose statistics"))
        stalls = sum(1 for w in w_in if w.get("out_max", 0) >= 16)
        if stalls:
            problems.append(("HIGH", f"teleop process stalled >=16 ms inside episodes ({stalls} of {len(w_in)} windows)",
                             "the shaper's publish loop missed >=4 periods: a GIL hold / GC / blocking call inside teleop_mantis.py; "
                             "needs in-process instrumentation (OUTSTREAM clamps at 16 ms so the true length is unknown)"))
        else:
            clean.append("no >=16 ms stalls of the 250 Hz command stream inside episodes")
        snaps = sum(w.get("snaps", 0) for w in w_in)
        if snaps:
            problems.append(("HIGH", f"shaper SNAPS inside episodes: {snaps}", "velocity limiter bypassed (command jumped > max_joint_lead)"))
        gl = sum(w.get("gl", 0) for w in w_in); dr = sum(w.get("drops", 0) for w in w_in); ho = sum(w.get("holds", 0) for w in w_in)
        secs_in = 10.0 * len(w_in)
        if w_in and (dr or ho or gl / max(secs_in, 1) > 0.3):
            problems.append(("HIGH", f"Quest pose stream problems inside episodes: {dr} dropouts (>200 ms), {ho} degraded-tracking holds, {gl} glitch rejects in {secs_in / 60:.0f} min",
                             "each dropout freezes the target and re-anchors; holds freeze it 0.3 s; the cause is on the headset / adb side, see section E"))
        elif w_in:
            clean.append(f"Quest pose stream inside episodes: {med([w['pn'] for w in w_in]) / 10:.0f} Hz, {dr} dropouts, {gl} glitches")
        if len(w_in) >= 6:
            n3 = max(2, len(w_in) // 3); early, late = w_in[:n3], w_in[-n3:]
            trend = []
            for k, lab, thr in (("tick_max", "tick max ms", 3.0), ("out_max", "shaper max ms", 3.0), ("p95", "pose p95 ms", 5.0), ("pmax", "pose max ms", 30.0), ("gl", "glitches/10s", 1.0), ("pn", "pose Hz*10", -30.0)):
                a, b = med([w[k] for w in early if k in w]), med([w[k] for w in late if k in w])
                trend.append((lab, a, b, (b - a > thr) if thr > 0 else (b - a < thr)))
            print("  early vs late third of the episodes: " + " | ".join(f"{lab} {a:.1f}->{b:.1f}{' !!' if bad else ''}" for lab, a, b, bad in trend))
            bad = [lab for lab, a, b, bad in trend if bad]
            (problems.append(("MEDIUM", "control-loop metrics degrade over the session: " + ", ".join(bad), "time-dependent -> heating / accumulating state / leak candidates; compare with sections D and E"))
             if bad else clean.append("no degradation of the control loop over the session (heating of the PC / headset unlikely to affect it)"))
    elif ctx:
        notes.append("no TICKTIME/OUTSTREAM/POSESTREAM windows in this window (teleop not running, or not yet 10 s)")

    if eps:
        print("\n-- B. episodes (recorder)")
        print("  ep   start     len  frames  dropped   cam Hz  flags")
        lossy = 0
        for e in eps:
            L = e["stop"] - e["start"]
            fl = ("DISCARDED " if e["discarded"] else "") + ("POOR-SYNC " if e["poor_sync"] else "") + ("still-open " if e.get("open") else "")
            dropped_pct = 100.0 * e["dropped"] / max((e["frames"] or 0) + e["dropped"], 1)
            if dropped_pct > 10 or (e["hz"] and e["hz"] < 13.5):
                lossy += 1; fl += "LOSSY"
            print(f"  {e['ep']:3d}  {hhmm(e['start'])} {L:6.1f}s {str(e['frames'] or '-'):>6}  {e['dropped']:4d} ({dropped_pct:3.0f}%)  {e['hz'] if e['hz'] else '-':>5}  {fl}")
        disc = sum(1 for e in eps if e["discarded"])
        if disc:
            problems.append(("LOW", f"{disc} episode(s) DISCARDED (< min_frames)", "double MENU press or an episode stopped immediately"))
        if lossy:
            problems.append(("MEDIUM", f"camera frames lost in {lossy}/{len(eps)} episodes (>10 % dropped or <13.5 Hz delivered)",
                             "NOT the UDP socket (see D) unless drops are shown there -> DDS shared-memory / camera driver / camera side; dataset fps declared 15 but real rate lower -> replay speed off"))
        else:
            clean.append("camera stream complete in every episode (>=13.5 Hz, <10 % dropped)")
        if any(e["poor_sync"] for e in eps):
            problems.append(("MEDIUM", "POOR SYNC between cameras reported", "check the femto-mega trigger cable / sync mode"))

    if ctx and ctx["events"]:
        keys = [("pose stale", "pose stale -> target frozen"), ("TRACKING DEGRADED", "TRACKING DEGRADED hold"), ("joint_states stale", "joint_states stale -> IK held"),
                ("joint_states recovered", "joint_states recovered (re-sync)"), ("re-anchored", "pose recovered -> re-anchored"), ("barrier holding", "barrier holding the arm back"),
                ("singularity", "near wrist singularity (IK damped)"), ("inside safety gate", "MEASURED pose inside safety gate -> auto-freeze"), ("not following", "not-following watchdog -> auto-freeze"),
                ("frame skipped", "recorder: frame skipped"), ("SAVE FAILED", "EPISODE SAVE FAILED"), ("LOST", "EPISODE LOST")]
        cnt = {}
        for t, x in ctx["events"]:
            for k, lab in keys:
                if k in x:
                    st = "in episode" if inside(rec_iv, t) else state_of(t)
                    cnt.setdefault(lab, {}).setdefault(st, 0); cnt[lab][st] += 1
        if cnt:
            print("\n-- C. teleop warnings (count by state)")
            for lab, d in cnt.items():
                print(f"  {lab:48s} " + ", ".join(f"{k}: {v}" for k, v in d.items()))
        ie = lambda lab: cnt.get(lab, {}).get("in episode", 0)
        if ie("joint_states stale -> IK held"):
            problems.append(("HIGH", f"robot state stream stalled inside episodes ({ie('joint_states stale -> IK held')} warnings)", "the UR driver / ros2_control stopped publishing /joint_states -> IK held; see section F (UR driver log)"))
        if ie("MEASURED pose inside safety gate -> auto-freeze") or ie("not-following watchdog -> auto-freeze"):
            problems.append(("MEDIUM", "auto-freezes inside episodes", f"safety gate {ie('MEASURED pose inside safety gate -> auto-freeze')}, not-following {ie('not-following watchdog -> auto-freeze')}: the arm was blocked (collision floor / obstacle)"))
        if ie("barrier holding the arm back") > 5:
            notes.append(f"barrier held the arm back {ie('barrier holding the arm back')} times inside episodes (collision floors, not a timing bug)")
        if ie("near wrist singularity (IK damped)") > 5:
            notes.append(f"wrist singularity damping active {ie('near wrist singularity (IK damped)')} times inside episodes (pose-dependent, feels sluggish there)")
        if ie("EPISODE SAVE FAILED") or ie("EPISODE LOST") or cnt.get("EPISODE SAVE FAILED") or cnt.get("EPISODE LOST"):
            problems.append(("HIGH", "episode save failures", "see the teleop console / log"))

    print("\n-- D. host")
    sec = {}
    for i in range(1, len(rows)):
        r, p = rows[i], rows[i - 1]
        st = state_of(r["t"])
        d = sec.setdefault(st, dict(n=0, udp=0, cpu=[], psi_cpu=[], psi_io=[], psi_mem=[]))
        d["n"] += 1
        if r["udp_drop"] == r["udp_drop"] and p["udp_drop"] == p["udp_drop"]:
            d["udp"] += int(r["udp_drop"] - p["udp_drop"])
        for k in ("cpu", "psi_cpu", "psi_io", "psi_mem"):
            v = r.get("cpu_busy" if k == "cpu" else k, NAN)
            if v == v: d[k].append(v)
    print("  state                        secs  UDP drops  cpu busy% med/max   PSI some avg10 max cpu/io/mem")
    for st, d in sec.items():
        print(f"  {st:26s} {d['n']:6d} {d['udp']:9d}   {med(d['cpu']):5.1f} / {max(d['cpu'] or [NAN]):5.1f}      {max(d['psi_cpu'] or [NAN]):5.1f} / {max(d['psi_io'] or [NAN]):5.1f} / {max(d['psi_mem'] or [NAN]):5.1f}")
    udp_rec = sec.get("recording", {}).get("udp", 0); udp_all = sum(d["udp"] for d in sec.values())
    if udp_all:
        problems.append(("HIGH" if udp_rec else "LOW", f"UDP receive-buffer drops: {udp_all} total, {udp_rec} inside episodes",
                         "a DDS socket overflowed (net.core.rmem_max=212992): fix rmem_max/rmem_default + FastDDS receiveBufferSize" if udp_rec else "drops happened only outside episodes"))
    else:
        clean.append("no UDP receive-buffer drops (the problem tested earlier did not occur)")
    cpu = col(rows, "cpu_busy"); temp = col(rows, "cpu_temp"); mhz = col(rows, "cpu_mhz"); mem = col(rows, "mem_avail_mb"); dirty = col(rows, "dirty_mb")
    gpu = col(rows, "gpu_temp"); wr = col(rows, "disk_w_mbs"); free = col(rows, "disk_free_gb")
    print(f"  cpu busy med {med(cpu):.0f}% max {max(cpu or [NAN]):.0f}% | cpu {min(mhz or [NAN]):.0f}-{max(mhz or [NAN]):.0f} MHz | cpu temp {min(temp or [NAN]):.0f}->{max(temp or [NAN]):.0f} C"
          f" | gpu temp max {max(gpu or [NAN]):.0f} C | mem avail min {min(mem or [NAN]) / 1024:.1f} GB, dirty max {max(dirty or [NAN]) / 1024:.2f} GB"
          f" | disk write med {med(wr):.0f} MB/s max {max(wr or [NAN]):.0f}, free {free[0] if free else NAN:.0f}->{free[-1] if free else NAN:.0f} GB")
    if max(cpu or [0]) > 85 or max(col(rows, "psi_cpu") or [0]) > 20:
        problems.append(("MEDIUM", "CPU contention on the host", f"cpu busy max {max(cpu or [0]):.0f}%, PSI cpu max {max(col(rows, 'psi_cpu') or [0]):.1f}"))
    if max(col(rows, "psi_io") or [0]) > 20:
        problems.append(("MEDIUM", "I/O stalls on the host (PSI io some avg10 > 20)", "disk writes are blocking tasks; check where lerobot_data is written"))
    if mem and min(mem) < 4096:
        problems.append(("HIGH", f"host memory pressure (MemAvailable min {min(mem) / 1024:.1f} GB)", "swap / reclaim stalls everything"))
    if temp and max(temp) >= 90:
        problems.append(("MEDIUM", f"host CPU hot (Tctl max {max(temp):.0f} C)", "possible thermal throttling"))
    if gpu and max(gpu) >= 85:
        problems.append(("LOW", f"GPU hot ({max(gpu):.0f} C)", ""))
    if free and free[-1] < 50:
        problems.append(("HIGH", f"disk nearly full ({free[-1]:.0f} GB free)", "temporary PNGs stay until teleop exit (batch_encoding_size)"))
    for name, lab in (("teleop", "teleop_mantis.py"), ("rc", "ros2_control_node")):
        rss = col(rows, f"{name}_rss_mb"); cpu_p = col(rows, f"{name}_cpu")
        if rss:
            grow = rss[-1] - rss[0]
            print(f"  {lab:20s} RSS {rss[0]:.0f} -> {rss[-1]:.0f} MB ({grow:+.0f}), cpu med {med(cpu_p):.0f}% max {max(cpu_p):.0f}%"
                  + (f", threads {col(rows, 'teleop_thr')[-1]:.0f}" if name == "teleop" and col(rows, "teleop_thr") else ""))
            if grow > 1500 and grow > 0.5 * rss[0]:
                problems.append(("MEDIUM", f"{lab} memory grows {grow:+.0f} MB over the session", "leak / unbounded buffers; long sessions get worse"))
        else:
            notes.append(f"{lab} was not seen running on this host during the window (process metrics unavailable)")
    if temp and len(temp) > 20 and med(temp[-len(temp) // 3:]) - med(temp[:len(temp) // 3]) > 10:
        notes.append(f"host CPU temperature rose {med(temp[:len(temp) // 3]):.0f} -> {med(temp[-len(temp) // 3:]):.0f} C over the session")

    qs = col(rows, "quest_status"); qc = col(rows, "quest_cpu_temp"); qb = col(rows, "quest_batt_temp"); ql = col(rows, "quest_batt_level")
    print("\n-- E. Quest headset")
    if qs or qc:
        print(f"  thermal status max {max(qs or [NAN]):.0f} (0 none, 1 light, 2 moderate, 3+ severe throttling) | cpu {min(qc or [NAN]):.0f}->{max(qc or [NAN]):.0f} C"
              f" | battery {min(qb or [NAN]):.0f}->{max(qb or [NAN]):.0f} C, level {ql[0] if ql else NAN:.0f}->{ql[-1] if ql else NAN:.0f}%")
        if qs and max(qs) >= 2:
            problems.append(("HIGH", f"Quest thermal throttling (status {max(qs):.0f})", "the headset throttles tracking/app when hot -> pose stream jitter and dropouts"))
        elif qs:
            clean.append(f"Quest not throttling (thermal status max {max(qs):.0f}, cpu max {max(qc or [0]):.0f} C)")
        if ql and ql[-1] <= 15:
            problems.append(("MEDIUM", f"Quest battery low ({ql[-1]:.0f}%)", "low battery triggers power saving"))
    else:
        notes.append("Quest thermals not sampled (adb unavailable or --no-adb)")
    try:
        qlogs = [f for f in Path.home().joinpath(".ros/log").glob("python*_*.log") if f.stat().st_mtime >= t0 - 60 and "[quest_pub]" in f.read_text(errors="replace")[:20000]]
        held = 0; other = {}
        for f in qlogs:
            for ln in f.read_text(errors="replace").splitlines():
                m = TS.match(ln.rstrip())
                if not m or not (t0 <= float(m[2]) <= t1) or m[1] != "WARN":
                    continue
                if "no data from the headset" in m[4]:
                    held += 1
                else:
                    k = re.sub(r"[-+]?\d+(\.\d+)?", "#", m[4])[:80]; other[k] = other.get(k, 0) + 1
        held_in = 0
        for f in qlogs:
            for ln in f.read_text(errors="replace").splitlines():
                m = TS.match(ln.rstrip())
                if m and "no data from the headset" in m[4] and inside(rec_iv, float(m[2])):
                    held_in += 1
        print(f"  quest_pub: 'no data from the headset' warnings {held} (inside episodes: {held_in}); other warnings: {other or 'none'}")
        if held_in:
            problems.append(("HIGH", f"headset stopped delivering poses during episodes ({held_in} warnings, ~{2 * held_in} s held)", "USB/adb hiccup, headset proximity sensor / sleep, or tracking loss"))
    except Exception as exc:
        notes.append(f"quest_pub log not checked ({exc})")

    try:
        rcf = docker_files("ros2_control_node_*.log", t0 - 60)
        cnt = {}
        for ln in docker_grep(r"\[(WARN|ERROR|FATAL)\]", rcf).splitlines():
            m = TS.match(ln.rstrip())
            if m and t0 <= float(m[2]) <= t1:
                k = re.sub(r"[-+]?\d+(\.\d+)?", "#", m[4])[:90]; cnt[k] = cnt.get(k, 0) + 1
        print("\n-- F. UR driver / ros2_control warnings in the window")
        for k, v in sorted(cnt.items(), key=lambda kv: -kv[1])[:8]:
            print(f"  {v:4d}  {k}")
        if not cnt:
            print("  none")
        conn = {k: v for k, v in cnt.items() if re.search(r"Failed to read|reconnect|timeout|Lost|Could not keep", k, re.I)}
        over = sum(v for k, v in cnt.items() if "Overrun" in k)
        if conn:
            problems.append(("HIGH", "UR driver lost / re-established the robot connection", "; ".join(f"{v}x {k}" for k, v in conn.items())))
        if over:
            problems.append(("HIGH" if over >= 5 else "LOW", f"ros2_control missed its 500 Hz rate {over}x (overrun warnings)",
                             "the control loop runs without RT priority ('Could not enable FIFO RT scheduling policy'): add --ulimit rtprio=99 --cap-add=SYS_NICE to the container"))
        if not conn and not over and rcf:
            clean.append("no UR driver connection loss / overrun warnings")
    except Exception as exc:
        notes.append(f"ros2_control log not checked ({exc})")

    print("\n-- G. host -> robot controller network (ping)")
    for side in ROBOT_IPS:
        f = LOGS / f"monitor_ping_{side}.log"
        if not f.exists():
            continue
        txt = f.read_text().split("# start")[-1]
        rtt, seqs = [], []
        for ln in txt.splitlines():
            m = re.match(r"\[(\d+\.\d+)\].*icmp_seq=(\d+).*time=([\d.]+) ms", ln)
            if m and t0 <= float(m[1]) <= t1:
                rtt.append(float(m[3])); seqs.append(int(m[2]))
        if not rtt:
            print(f"  {side} {ROBOT_IPS[side]}: no replies (not reachable from the host -> skipped)"); continue
        lost = (max(seqs) - min(seqs) + 1) - len(rtt)
        print(f"  {side} {ROBOT_IPS[side]}: {len(rtt)} replies, {lost} missed | rtt p50 {med(rtt):.2f} p95 {pct(rtt, 95):.2f} max {max(rtt):.2f} ms")
        if (lost >= 3 and lost > 0.02 * len(rtt)) or pct(rtt, 95) > 5.0:
            problems.append(("HIGH", f"robot network to {side} arm jittery/lossy (rtt p95 {pct(rtt, 95):.1f} ms, ~{lost} lost)", "the UR needs its 2 ms servoj stream on time; cameras share this link (192.168.1.x)"))
        else:
            clean.append(f"network to the {side} UR controller clean (rtt p95 {pct(rtt, 95):.2f} ms)")

    if ctx and ctx["dataset"] and eps:
        try:
            ep_min = min(e["ep"] for e in eps)
            res = subprocess.run(["docker", "exec", "-i", CONTAINER, "python3", "-", ctx["dataset"], str(ep_min)],
                                 input=FLICK_SCAN, capture_output=True, text=True, timeout=600)
            out = res.stdout.strip().splitlines()
            if res.returncode != 0:
                raise RuntimeError(res.stderr.strip().splitlines()[-1] if res.stderr.strip() else f"exit {res.returncode}")
            scan = json.loads(out[-1]) if out else []
            if not scan:
                notes.append(f"dataset scan found no saved episodes >= {ep_min} under {ctx['dataset']} yet (episodes are written at save time)")
            if scan:
                print("\n-- H. recorded commands (15 Hz action in the dataset): flick signatures per episode")
                print("  ep    len  moving  stall->jump  reversal-runs  jerk/speed  lag95[rad]  vmax[rad/s]")
                for s in scan:
                    print(f"  {s['ep']:3d} {s['secs']:5.1f}s {s['moving']:5.0f}%  {s['stall']:8d}  {s['rev']:10d}       {s['smooth'] if s['smooth'] is None else round(s['smooth'], 2)!s:>5}      {s['lag95']:.3f}     {s['vmax']:5.2f}")
                mins = sum(s["secs"] for s in scan) / 60.0; st = sum(s["stall"] for s in scan); rv = sum(s["rev"] for s in scan)
                print(f"  total {mins:.1f} min: stall->jump {st} ({st / max(mins, 1e-6):.1f}/min), reversal runs {rv} ({rv / max(mins, 1e-6):.1f}/min)")
                if st / max(mins, 1e-6) > 3 or rv / max(mins, 1e-6) > 3:
                    problems.append(("MEDIUM", f"flick signatures in the recorded commands: {st / max(mins, 1e-6):.1f} stall->jump/min, {rv / max(mins, 1e-6):.1f} hunting runs/min",
                                     "the commanded joints themselves are not smooth: pose input glitches or IK/clamp effects; compare episodes with the events above"))
                else:
                    clean.append(f"recorded commands smooth: {st / max(mins, 1e-6):.1f} stall->jump/min, {rv / max(mins, 1e-6):.1f} hunting runs/min")
                if len(scan) >= 6:
                    n3 = len(scan) // 3
                    e_s = sum(s["stall"] + s["rev"] for s in scan[:n3]) / max(sum(s["secs"] for s in scan[:n3]) / 60, 1e-6)
                    l_s = sum(s["stall"] + s["rev"] for s in scan[-n3:]) / max(sum(s["secs"] for s in scan[-n3:]) / 60, 1e-6)
                    if l_s > 2 * e_s + 1:
                        problems.append(("MEDIUM", f"flicks increase over the session ({e_s:.1f} -> {l_s:.1f} per min, early vs late)", "time-dependent: heating / accumulating state"))
        except Exception as exc:
            notes.append(f"dataset scan skipped ({exc})")

    print("\n=== PROBLEMS FOUND" + (" (none)" if not problems else ""))
    order = {"HIGH": 0, "MEDIUM": 1, "LOW": 2}
    for sev, title, why in sorted(problems, key=lambda p: order[p[0]]):
        print(f"  [{sev}] {title}" + (f"\n         -> {why}" if why else ""))
    print("=== CHECKED AND CLEAN")
    for c in clean:
        print(f"  ok  {c}")
    if notes:
        print("=== NOTES")
        for n in notes:
            print(f"  - {n}")


if __name__ == "__main__":
    cmd = next((a for a in sys.argv[1:] if not a.startswith("--")), "report")
    {"start": start, "stop": stop, "report": report}.get(cmd, lambda: print(__doc__))()
