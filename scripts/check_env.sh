#!/usr/bin/env bash
set -u

status=0

report() {
  local level="$1"
  local name="$2"
  local detail="$3"
  printf '%-8s %-12s %s\n' "$level" "$name" "$detail"
}

if command -v python3 >/dev/null 2>&1; then
  report OK python "$(python3 --version 2>&1)"
else
  report ERROR python "python3 not found"
  status=1
fi

if python3 -m pytest --version >/dev/null 2>&1; then
  report OK pytest "$(python3 -m pytest --version | head -n 1)"
else
  report ERROR pytest "pytest not available for python3"
  status=1
fi

if command -v ffmpeg >/dev/null 2>&1; then
  report OK ffmpeg "$(ffmpeg -version 2>/dev/null | head -n 1)"
else
  report OPTIONAL ffmpeg "missing; install with sudo apt install -y ffmpeg for video export"
fi

if [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
  report OK WSLg "DISPLAY=${DISPLAY:-unset} WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-unset}"
else
  report OPTIONAL WSLg "DISPLAY and WAYLAND_DISPLAY are unset"
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  report OPTIONAL ROS2 "Humble setup found"
else
  report OPTIONAL ROS2 "not installed; not required for 2D mock mode"
fi

if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
  report OPTIONAL NVIDIA "nvidia-smi available"
elif [[ -x /usr/lib/wsl/lib/nvidia-smi ]] && /usr/lib/wsl/lib/nvidia-smi >/dev/null 2>&1; then
  report OPTIONAL NVIDIA "/usr/lib/wsl/lib/nvidia-smi available"
else
  report OPTIONAL NVIDIA "GPU unavailable; not required for CPU 2D simulation"
fi

exit "$status"
