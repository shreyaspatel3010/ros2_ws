#!/usr/bin/env bash
# Run a command inside hard resource limits so Isaac / training can never take
# the whole laptop down (desktop freeze, VS Code crash).
#
#   limit.sh <command> [args...]
#
# The command runs in its own systemd user scope (a cgroup):
#   RAM   MemoryMax               OOM-kill *only this command* when it is exceeded
#   swap  MemorySwapMax            no swap thrashing (the usual cause of a frozen desktop)
#   CPU   CPUQuota + CPUWeight     at most N cores, and the desktop wins any contention
#   I/O   IOWeight, nice           lower priority than interactive programs
#   GPU   HRUH_GPU_MEM_GB          enforced inside the process by hruh_lab/gpu_guard.py
#
# Defaults scale with the machine (this laptop: 20 threads, 30 GB -> 12 cores, 18 GB):
#   HRUH_CPU_CORES   cores the command may use  (default: 60% of nproc, at least 2)
#   HRUH_MEM_GB      RAM ceiling                (default: 60% of RAM)
#   HRUH_GPU_MEM_GB  GPU memory budget          (default: 8)
#   HRUH_LIMITS=0    disable (run the command unchanged)
# If systemd user scopes are unavailable it falls back to nice/ionice only and says so.
set -euo pipefail
(($#)) || { echo "usage: limit.sh <command> [args...]" >&2; exit 2; }

export HRUH_GPU_MEM_GB="${HRUH_GPU_MEM_GB:-8}"
if [[ "${HRUH_LIMITS:-1}" == 0 ]]; then
  exec "$@"
fi

threads="$(nproc)"
mem_total_kb="$(awk '/^MemTotal:/ {print $2}' /proc/meminfo)"
cores="${HRUH_CPU_CORES:-$(( threads * 6 / 10 ))}"
(( cores >= 2 )) || cores=2
(( cores <= threads )) || cores="$threads"
mem_gb="${HRUH_MEM_GB:-$(( mem_total_kb * 6 / 10 / 1024 / 1024 ))}"
(( mem_gb >= 4 )) || mem_gb=4

# Libraries that size thread pools from nproc should see the quota, not 20 cores.
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-$cores}" MKL_NUM_THREADS="${MKL_NUM_THREADS:-$cores}"
export HRUH_CPU_CORES="$cores"

echo "[limit] cpu ${cores}/${threads} cores, ram ${mem_gb} GB (no swap), gpu ${HRUH_GPU_MEM_GB} GB: $(basename -- "$1")" >&2
state="$(systemctl --user is-system-running 2>/dev/null || true)"
if command -v systemd-run >/dev/null && [[ "$state" == running || "$state" == degraded ]]; then
  # --scope keeps the same PID, stdout and signals (Ctrl+C, ros2 launch shutdown).
  exec systemd-run --user --scope --quiet --collect \
    -p MemoryMax="${mem_gb}G" -p MemorySwapMax=0 \
    -p CPUQuota="$(( cores * 100 ))%" -p CPUWeight=20 -p IOWeight=20 \
    -p TasksMax=8192 \
    nice -n 10 "$@"
fi
echo "[limit] systemd user scopes unavailable: only nice/ionice applied (no hard RAM/CPU cap)" >&2
exec nice -n 10 ionice -c 2 -n 7 "$@"
