#!/usr/bin/env python3
"""Terminal progress for train_robot_offline.sh.

    <command> 2>&1 | progress.py --mode train --label "[flat 1/3]" --log train_1.log --total 1000

Everything the command prints is appended to --log unchanged.  The terminal shows only:
  train   a live bar: iteration, mean reward, episode length, task metric, speed, ETA
  eval    one line per test scenario / seed as it finishes
  gazebo  hold / release / fall events and the final report
While Isaac starts (1-2 min) an elapsed-time line shows that it is alive.
Without a terminal (nohup, redirect) it prints a plain line every 5 % instead.
Standard library only.
"""
import argparse
import json
import re
import signal
import sys
import threading
import time

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
NUMBER = r"([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?|nan|inf)"
FIELDS = {
    "iteration": re.compile(r"Learning iteration (\d+)/(\d+)"),
    "reward": re.compile(r"^\s*Mean reward:\s*" + NUMBER),
    "length": re.compile(r"^\s*Mean episode length:\s*" + NUMBER),
    "fps": re.compile(r"^\s*Steps per second:\s*" + NUMBER),
    "eta": re.compile(r"^\s*ETA:\s*(\S+)"),
    "vel_err": re.compile(r"Metrics/base_velocity/error_vel_xy:\s*" + NUMBER),
    "yaw_err": re.compile(r"Metrics/base_velocity/error_vel_yaw:\s*" + NUMBER),
    "pos_err": re.compile(r"Metrics/ee_pose/position_error:\s*" + NUMBER),
    "success": re.compile(r"Metrics/success_rate:\s*" + NUMBER),
    "timeout": re.compile(r"Episode_Termination/time_out:\s*" + NUMBER),
}


def clock(seconds):
    seconds = int(seconds)
    return f"{seconds // 3600}:{seconds // 60 % 60:02d}:{seconds % 60:02d}"


class Display:
    def __init__(self, label, tty):
        self.label, self.tty = label, tty
        self.live = self.closed = False
        self.lock = threading.Lock()

    def status(self, text):
        """Replace the live line (terminal) - ignored without a terminal."""
        if self.tty:
            with self.lock:
                if self.closed:
                    return
                width = 160
                sys.stdout.write("\r\x1b[2K" + f"{self.label} {text}"[:width])
                sys.stdout.flush()
                self.live = True

    def line(self, text, final=False):
        """Print a permanent line."""
        with self.lock:
            self.closed = self.closed or final
            if self.live:
                sys.stdout.write("\r\x1b[2K")
                self.live = False
            sys.stdout.write(f"{self.label} {text}\n")
            sys.stdout.flush()


def bar(fraction, width=24):
    filled = int(round(max(0.0, min(1.0, fraction)) * width))
    return "█" * filled + "░" * (width - filled)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["train", "eval", "gazebo", "plain"], required=True)
    parser.add_argument("--label", default="")
    parser.add_argument("--log", required=True)
    parser.add_argument("--total", type=int, default=0, help="training iterations of this run")
    args = parser.parse_args()
    # Ctrl+C reaches the whole pipeline: keep reading so the simulator's shutdown is logged.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    display = Display(args.label, sys.stdout.isatty())
    start = time.monotonic()
    state = {"first": None, "iteration": None, "last_plain": -1.0, "started": False}
    values = {}

    def heartbeat():
        while not state["started"]:
            display.status(f"starting {'Isaac' if args.mode != 'gazebo' else 'Gazebo'}… {clock(time.monotonic() - start)}")
            time.sleep(1.0)

    if args.mode in ("train", "eval", "gazebo"):
        threading.Thread(target=heartbeat, daemon=True).start()

    def render_training():
        it, total_label = state["iteration"]
        first = state["first"]
        total = args.total or max(1, total_label - first)
        done = it - first + 1
        fraction = done / total
        parts = [f"{bar(fraction)} {done}/{total} {fraction * 100:3.0f}%"]
        if "reward" in values:
            parts.append(f"reward {float(values['reward']):.2f}")
        if "length" in values:
            parts.append(f"ep {float(values['length']):.0f}")
        walking = "vel_err" in values
        for key, name, scale, unit in (("vel_err", "vel err", 1, " m/s"), ("yaw_err", "yaw err", 1, " rad/s"),
                                       ("pos_err", "hand err", 100, " cm"), ("success", "success", 100, "%"),
                                       ("timeout", "survived", 100, "%")):
            # walking logs also carry an unrelated success_rate; time_out = episodes survived
            if key in values and not (walking and key == "success") and (walking or key != "timeout"):
                parts.append(f"{name} {float(values[key]) * scale:.1f}{unit}" if scale != 1
                             else f"{name} {float(values[key]):.2f}{unit}")
        if "fps" in values:
            parts.append(f"{float(values['fps']):.0f} steps/s")
        parts.append(f"ETA {values.get('eta', '?')}")
        text = " | ".join(parts)
        if display.tty:
            display.status(text)
        elif fraction - state["last_plain"] >= 0.05 or done == total:
            state["last_plain"] = fraction
            display.line(text)

    def handle(line):
        """Update the display for one output line (never raises into the read loop)."""
        if args.mode == "train":
            match = FIELDS["iteration"].search(line)
            if match:
                state["started"] = True
                it = int(match[1])
                if state["first"] is None:
                    state["first"] = it
                state["iteration"] = (it, int(match[2]))
                return
            for key, regex in FIELDS.items():
                if key != "iteration" and (m := regex.search(line)):
                    values[key] = m[1]
                    if key == "eta" and state["iteration"]:
                        render_training()   # ETA is the last line of each iteration block
                    break
        elif args.mode == "eval":
            if line.startswith("{") and line.endswith("}"):
                try:
                    row = json.loads(line)
                except ValueError:
                    return
                state["started"] = True
                if "scenario" in row:
                    mae = row.get("velocity_mae_while_alive_vx_vy_wz") or [float("nan")] * 3
                    display.line(f"{row['scenario']:>14}: survived {row['survival_fraction'] * 100:5.1f}%  "
                                 f"error vx {mae[0]:.2f} vy {mae[1]:.2f} m/s, yaw {mae[2]:.2f} rad/s")
                elif "seed" in row and "successes" in row:
                    display.line(f"  seed {row['seed']}: {row['successes']}/{row['trials']} successes "
                                 f"({row['success_fraction'] * 100:.0f}%)")
            elif "Learning iteration" in line or "Evaluating" in line:
                state["started"] = True
        elif args.mode == "gazebo":
            if "HRUH_POLICY_READY" in line:
                state["started"] = True
                display.line("policy loaded; holding the stand pose")
            elif "released, policy in control" in line:
                display.line("released: policy in control")
            elif "Fall detected" in line:
                display.line("FALL detected")
            elif '"simulator": "Gazebo' in line:
                body = line[line.index("{"):]
                try:
                    r = json.loads(body)
                    display.line(f"{r.get('elapsed_sim_s', 0):.1f} s simulated, {r.get('policy_steps')} policy "
                                 f"steps at {r.get('policy_rate_hz', '?')} Hz, falls {r.get('falls')}, "
                                 f"completed {r.get('completed')}")
                except ValueError:
                    pass

    # The simulator's output goes through this pipe: if the display failed, the training
    # would get SIGPIPE. So every display error is swallowed and the log copy always continues.
    with open(args.log, "ab", buffering=0) as log:
        for raw in sys.stdin.buffer:
            log.write(raw)
            try:
                handle(ANSI.sub("", raw.decode("utf-8", "replace")).rstrip())
            except Exception:
                pass
    state["started"] = True
    display.line(f"finished in {clock(time.monotonic() - start)}  (full output: {args.log})", final=True)


if __name__ == "__main__":
    main()
