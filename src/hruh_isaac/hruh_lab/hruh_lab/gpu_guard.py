"""Keep Isaac / training inside a GPU-memory budget so the desktop survives.

GeForce drivers cannot hard-limit a process' VRAM, and on this laptop Xorg,
GNOME, the browser and VS Code render on the same NVIDIA GPU; when Isaac
takes all of it, those apps (VS Code first) crash.  This guard

  * caps PyTorch's CUDA allocator to the budget (if torch is in use), and
  * watches this process tree's real GPU memory (graphics + compute, from
    `nvidia-smi pmon`) and stops the program cleanly (SIGINT, then SIGTERM)
    when it stays above the budget.

Budget: HRUH_GPU_MEM_GB (default 8).  No Isaac / ROS imports.
"""
import os
import signal
import subprocess
import threading
import time


def budget_gb():
    return float(os.environ.get("HRUH_GPU_MEM_GB", "8"))


def _descendants(pid):
    children = {}
    for entry in os.listdir("/proc"):
        if entry.isdigit():
            try:
                with open(f"/proc/{entry}/stat") as f:
                    ppid = int(f.read().rsplit(")", 1)[1].split()[1])
                children.setdefault(ppid, []).append(int(entry))
            except (OSError, ValueError, IndexError):
                pass
    out, todo = {pid}, [pid]
    while todo:
        for c in children.get(todo.pop(), []):
            if c not in out:
                out.add(c)
                todo.append(c)
    return out


def used_mib(pid=None):
    """GPU framebuffer memory (MiB) used by pid and its children, or None if unknown."""
    pid = pid or os.getpid()
    try:
        text = subprocess.run(["nvidia-smi", "pmon", "-s", "m", "-c", "1"], capture_output=True,
                              text=True, timeout=10).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    pids, total = _descendants(pid), 0
    for line in text.splitlines():
        parts = line.split()
        if line.startswith("#") or len(parts) < 4 or not parts[1].isdigit():
            continue
        if int(parts[1]) in pids and parts[3].isdigit():
            total += int(parts[3])
    return total


def cap_torch(limit_gb=None):
    """Limit PyTorch's CUDA caching allocator to the budget (no-op without CUDA)."""
    try:
        import torch
        if torch.cuda.is_available():
            total = torch.cuda.get_device_properties(0).total_memory / 2**30
            torch.cuda.set_per_process_memory_fraction(min(1.0, (limit_gb or budget_gb()) / total), 0)
    except Exception:
        pass


def start(limit_gb=None, interval=3.0, grace=3, name="isaac"):
    """Start the watchdog thread; returns it.  Exceeding the budget for `grace`
    consecutive checks sends SIGINT (programs save / close), then SIGTERM."""
    limit = (limit_gb or budget_gb()) * 1024
    pid = os.getpid()

    def watch():
        over, signalled = 0, 0
        while True:
            time.sleep(interval)
            used = used_mib(pid)
            if used is None:
                continue
            over = over + 1 if used > limit else 0
            if over >= grace:
                sig = signal.SIGINT if signalled == 0 else signal.SIGTERM
                print(f"[gpu_guard] {name} uses {used} MiB GPU memory > budget {limit:.0f} MiB "
                      f"(HRUH_GPU_MEM_GB); stopping with {sig.name} to protect the desktop", flush=True)
                os.kill(pid, sig)
                signalled += 1
                over = 0

    thread = threading.Thread(target=watch, name="gpu_guard", daemon=True)
    thread.start()
    print(f"[gpu_guard] {name}: GPU memory budget {limit / 1024:.1f} GB", flush=True)
    return thread
