#!/usr/bin/env python3
"""Background UDP-drop watcher for the teleop test."""
import os
import re
import subprocess
import sys
import time
from pathlib import Path

DIR = Path(__file__).resolve().parent
LOG = DIR / "logs" / "udp_watch.csv"
PID = DIR / "logs" / "udp_watch.pid"
CONTAINER = "mantis"


def read_udp():
    lines = Path("/proc/net/snmp").read_text().splitlines()
    hdr = val = None
    for ln in lines:
        if ln.startswith("Udp:"):
            if hdr is None:
                hdr = ln.split()[1:]
            else:
                val = ln.split()[1:]
    d = dict(zip(hdr, map(int, val)))
    return d["InDatagrams"], d["RcvbufErrors"], d["InErrors"]


def alive():
    try:
        pid = int(PID.read_text())
        os.kill(pid, 0)
        return pid
    except (FileNotFoundError, ValueError, ProcessLookupError, PermissionError):
        return None


def start():
    if alive():
        print(f"already running (pid {alive()}), logging to {LOG}")
        return
    LOG.parent.mkdir(parents=True, exist_ok=True)
    if os.fork():
        time.sleep(0.3)
        print(f"udp_watch running in the background (pid {alive()}), 1 Hz -> {LOG}\n"
              f"teleop now; afterwards: {sys.argv[0]} report")
        return
    os.setsid()
    if os.fork():
        os._exit(0)
    PID.write_text(str(os.getpid()))
    with open(LOG, "a") as f:
        f.write(f"# start {time.time():.3f}\n")
        while True:
            t = time.time()
            i, r, e = read_udp()
            f.write(f"{t:.3f},{i},{r},{e}\n")
            f.flush()
            time.sleep(max(0.0, 1.0 - (time.time() - t)))


def stop():
    pid = alive()
    if pid:
        os.kill(pid, 15)
        print(f"stopped pid {pid}")
    else:
        print("not running")
    PID.unlink(missing_ok=True)


def load_samples(since=None):
    rows = []
    for ln in LOG.read_text().splitlines():
        if ln.startswith("#") or not ln.strip():
            continue
        t, i, r, e = ln.split(",")
        rows.append((float(t), int(i), int(r)))
    if since is not None:
        rows = [x for x in rows if x[0] >= since]
    return rows


def teleop_intervals(t0, t1):
    """(recording, engaged, episodes) intervals from the container's teleop logs overlapping [t0, t1]."""
    try:
        names = subprocess.run(
            ["docker", "exec", CONTAINER, "bash", "-c",
             "grep -l vive_mantis_pink_bridge /home/ros/.ros/log/python3_*.log 2>/dev/null"],
            capture_output=True, text=True, timeout=20).stdout.split()
    except Exception as exc:
        print(f"(no teleop log correlation: {exc})")
        return [], [], []
    rec, eng, eps = [], [], []
    for name in names:
        m = re.search(r"_(\d+)\.log$", name)
        if not m or int(m[1]) / 1000.0 > t1:
            continue
        txt = subprocess.run(
            ["docker", "exec", CONTAINER, "bash", "-c",
             f"grep -hE 'EPISODE [0-9]+ (RECORDING|STOPPED|DISCARDED)|\\]: (ENGAGED|FROZEN)$' {name}"],
            capture_output=True, text=True, timeout=20).stdout
        cur_r = cur_e = None
        for ln in txt.splitlines():
            mm = re.match(r"^\[\w+\] \[(\d+\.\d+)\] \[[^\]]+\]: (.*)$", ln)
            if not mm:
                continue
            t, msg = float(mm[1]), mm[2]
            me = re.match(r"EPISODE (\d+) RECORDING", msg)
            if me:
                cur_r = (int(me[1]), t)
            elif re.match(r"EPISODE \d+ (STOPPED|DISCARDED)", msg) and cur_r:
                rec.append((cur_r[1], t)); eps.append((cur_r[0], cur_r[1], t)); cur_r = None
            elif msg == "ENGAGED":
                cur_e = t
            elif msg == "FROZEN" and cur_e is not None:
                eng.append((cur_e, t)); cur_e = None
    overlap = lambda a, b: b >= t0 and a <= t1
    return ([iv for iv in rec if overlap(*iv)], [iv for iv in eng if overlap(*iv)],
            [e for e in eps if overlap(e[1], e[2])])


def report():
    if not LOG.exists():
        print(f"no samples at {LOG} - run `start` first"); return
    rows = load_samples()
    starts = [float(l.split()[2]) for l in LOG.read_text().splitlines() if l.startswith("# start")]
    if starts:
        rows = [x for x in rows if x[0] >= starts[-1]]
    if len(rows) < 2:
        print("fewer than 2 samples yet"); return
    t0, t1 = rows[0][0], rows[-1][0]
    secs = [(rows[k][0], rows[k][1] - rows[k - 1][1], rows[k][2] - rows[k - 1][2]) for k in range(1, len(rows))]
    tot_in = sum(s[1] for s in secs); tot_drop = sum(s[2] for s in secs)
    print(f"window {time.strftime('%H:%M:%S', time.localtime(t0))} - {time.strftime('%H:%M:%S', time.localtime(t1))} "
          f"({(t1 - t0) / 60:.1f} min): {tot_in} UDP datagrams in, {tot_drop} dropped for a full socket buffer "
          f"({100.0 * tot_drop / max(tot_in, 1):.2f}%)")
    rec, eng, eps = teleop_intervals(t0, t1)
    inside = lambda iv, t: any(a <= t <= b for a, b in iv)
    if rec or eng:
        buckets = {"recording": [0, 0], "engaged, not recording": [0, 0], "parked / homing / other": [0, 0]}
        for t, _, d in secs:
            k = "recording" if inside(rec, t) else ("engaged, not recording" if inside(eng, t) else "parked / homing / other")
            buckets[k][0] += 1; buckets[k][1] += d
        print("\nstate                        seconds   drops   drops/s")
        for k, (n, d) in buckets.items():
            print(f"  {k:26s} {n:7d} {d:7d} {d / max(n, 1):9.1f}")
        if eps:
            print("\nper episode:  ep   length   drops   drops/s")
            for e, a, b in eps:
                d = sum(s[2] for s in secs if a <= s[0] <= b)
                print(f"             {e:3d}  {b - a:6.1f}s {d:6d} {d / max(b - a, 1e-9):8.1f}")
    else:
        print("(no teleop episodes/engages found in the container logs for this window)")
    worst = sorted(secs, key=lambda s: -s[2])[:5]
    if worst and worst[0][2] > 0:
        print("\nworst seconds:", ", ".join(f"{time.strftime('%H:%M:%S', time.localtime(t))} +{d}" for t, _, d in worst if d))
    print("\nverdict: " + (
        "drops happen while you teleop -> the DDS socket buffer overflows (camera bursts); fix rmem_max + FastDDS buffer sizes"
        if tot_drop > 0 else
        "no receive-buffer drops in this window -> the socket buffer is NOT the cause of what you felt"))


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "report"
    {"start": start, "stop": stop, "report": report}.get(cmd, lambda: print(__doc__))()
