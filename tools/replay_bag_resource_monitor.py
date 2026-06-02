#!/usr/bin/env python3
import argparse
import csv
import json
import os
import subprocess
import time
from pathlib import Path


CLK_TCK = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
PAGE_SIZE = os.sysconf(os.sysconf_names["SC_PAGE_SIZE"])


def read_total_jiffies():
    with open("/proc/stat", "r", encoding="utf-8") as f:
        parts = f.readline().split()[1:]
    return sum(int(x) for x in parts)


def read_proc(pid):
    base = Path("/proc") / str(pid)
    try:
        stat = (base / "stat").read_text(encoding="utf-8")
        cmdline = (base / "cmdline").read_bytes().replace(b"\x00", b" ").decode("utf-8", "replace").strip()
    except Exception:
        return None
    rpar = stat.rfind(")")
    fields = stat[rpar + 2 :].split()
    ppid = int(fields[1])
    utime = int(fields[11])
    stime = int(fields[12])
    rss_pages = int(fields[21])
    return {
        "pid": int(pid),
        "ppid": ppid,
        "jiffies": utime + stime,
        "rss_mb": rss_pages * PAGE_SIZE / 1024.0 / 1024.0,
        "cmd": cmdline or stat[stat.find("(") + 1 : rpar],
    }


def proc_tree(root_pid):
    procs = {}
    for p in Path("/proc").iterdir():
        if not p.name.isdigit():
            continue
        info = read_proc(p.name)
        if info:
            procs[info["pid"]] = info
    children = {}
    for pid, info in procs.items():
        children.setdefault(info["ppid"], []).append(pid)
    stack = [root_pid]
    out = []
    seen = set()
    while stack:
        pid = stack.pop()
        if pid in seen or pid not in procs:
            continue
        seen.add(pid)
        out.append(procs[pid])
        stack.extend(children.get(pid, []))
    return out


def short_name(cmd):
    if "ros2 bag play" in cmd:
        return "rosbag2_player"
    if "ros2" in cmd:
        return "ros2"
    if "python" in cmd:
        return "python"
    return Path(cmd.split()[0]).name if cmd.split() else "unknown"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--rate", type=float, default=3.0)
    ap.add_argument("--domain-id", type=int, default=91)
    ap.add_argument("--playback-duration", type=float, default=0.0, help="bag-time seconds to play; 0 means full bag")
    args = ap.parse_args()

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.domain_id)
    play_duration = ""
    if args.playback_duration > 0:
        play_duration = f" --playback-duration {args.playback_duration}"
    cmd = [
        "/bin/bash",
        "-lc",
        (
            "source /home/ubuntu/humanoid_ws/install/setup.bash && "
            f"ros2 bag play {args.bag} --clock --rate {args.rate} "
            f"--read-ahead-queue-size 100{play_duration} --disable-keyboard-controls"
        ),
    ]
    proc = subprocess.Popen(cmd, cwd="/home/ubuntu/humanoid_ws", env=env, stdout=(out / "rosbag_play.log").open("w"), stderr=subprocess.STDOUT)
    samples = []
    prev_proc = {}
    prev_total = read_total_jiffies()
    start_wall = time.time()
    try:
        while proc.poll() is None:
            time.sleep(1.0)
            total = read_total_jiffies()
            dt_total = max(1, total - prev_total)
            rows = proc_tree(proc.pid)
            stamp = time.time() - start_wall
            for r in rows:
                prev = prev_proc.get(r["pid"])
                cpu = 0.0
                if prev:
                    cpu = (r["jiffies"] - prev["jiffies"]) / dt_total * os.cpu_count() * 100.0
                samples.append({
                    "elapsed": f"{stamp:.3f}",
                    "pid": r["pid"],
                    "name": short_name(r["cmd"]),
                    "cpu_percent": f"{cpu:.2f}",
                    "rss_mb": f"{r['rss_mb']:.2f}",
                    "cmd": r["cmd"][:240],
                })
            prev_proc = {r["pid"]: r for r in rows}
            prev_total = total
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        rc = proc.wait()

    csv_path = out / "resource_samples.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        fields = ["elapsed", "pid", "name", "cpu_percent", "rss_mb", "cmd"]
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(samples)

    by_cmd = {}
    for s in samples:
        key = (s["name"], s["cmd"])
        item = by_cmd.setdefault(key, {"name": s["name"], "cmd": s["cmd"], "max_cpu_percent": 0.0, "avg_cpu_percent": 0.0, "max_rss_mb": 0.0, "samples": 0})
        cpu = float(s["cpu_percent"])
        rss = float(s["rss_mb"])
        item["max_cpu_percent"] = max(item["max_cpu_percent"], cpu)
        item["avg_cpu_percent"] += cpu
        item["max_rss_mb"] = max(item["max_rss_mb"], rss)
        item["samples"] += 1
    rows = []
    for item in by_cmd.values():
        if item["samples"]:
            item["avg_cpu_percent"] /= item["samples"]
        rows.append(item)
    rows.sort(key=lambda x: (x["max_cpu_percent"], x["max_rss_mb"]), reverse=True)
    summary = {
        "bag": args.bag,
        "name": args.name,
        "rate": args.rate,
        "playback_duration": args.playback_duration,
        "return_code": rc,
        "sample_count": len(samples),
        "elapsed_wall_sec": time.time() - start_wall,
        "processes_sorted": rows,
    }
    (out / "resource_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    lines = [
        f"# {args.name} bag 回放资源占用",
        "",
        f"- bag: `{args.bag}`",
        f"- rate: {args.rate}x",
        f"- wall time: {summary['elapsed_wall_sec']:.1f}s",
        f"- return code: {rc}",
        "",
        "| 排名 | 进程/节点 | 最大CPU% | 平均CPU% | 最大RSS MB | 命令 |",
        "|---:|---|---:|---:|---:|---|",
    ]
    for i, r in enumerate(rows[:20], 1):
        lines.append(
            f"| {i} | {r['name']} | {r['max_cpu_percent']:.2f} | {r['avg_cpu_percent']:.2f} | "
            f"{r['max_rss_mb']:.1f} | `{r['cmd']}` |"
        )
    (out / "resource_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(out / "resource_report.md")


if __name__ == "__main__":
    main()
