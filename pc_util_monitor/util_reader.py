# SPDX-FileCopyrightText: 2025 Toshiaki Kou
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import math
import re
import subprocess
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class UtilSample:
    """
    A single utilization sample.
    cpu_util: 0.0-1.0
    gpu_util: 0.0-1.0, or NaN if unavailable
    status:   OK_CPU_GPU / NO_GPU / OK_CPU_ONLY
    """
    cpu_util: float
    gpu_util: float
    status: str


def _clamp01(x: float) -> float:
    if x < 0.0:
        return 0.0
    if x > 1.0:
        return 1.0
    return x


def _is_wsl() -> bool:
    """
    Detect WSL by checking /proc/version for 'microsoft'.
    """
    try:
        with open("/proc/version", "r", encoding="utf-8") as f:
            v = f.read().lower()
        return "microsoft" in v
    except OSError:
        return False


def _read_windows_cpu_percent_via_powershell() -> Optional[float]:
    """
    Read Windows host CPU utilization (%) via PowerShell Get-Counter.

    Returns:
        float in [0.0, 100.0] on success, or None on failure.

    Notes:
        - Works when running under WSL and powershell.exe is accessible.
        - Output may contain locale-specific decimal separators, so we parse robustly.
    """
    try:
        # InvariantCulture to avoid comma decimal; still parse defensively.
        cmd = [
            "powershell.exe",
            "-NoProfile",
            "-Command",
            "[CultureInfo]::InvariantCulture | Out-Null; "
            "(Get-Counter '\\Processor(_Total)\\% Processor Time').CounterSamples.CookedValue",
        ]
        out = subprocess.check_output(cmd, text=True, stderr=subprocess.DEVNULL).strip()
        # normalize decimal separator (just in case)
        out = out.replace(",", ".")
        m = re.search(r"[-+]?\d+(?:\.\d+)?", out)
        if not m:
            return None
        v = float(m.group(0))
        if math.isnan(v) or math.isinf(v):
            return None
        if v < 0.0:
            v = 0.0
        if v > 100.0:
            v = 100.0
        return v
    except (OSError, subprocess.CalledProcessError, ValueError):
        return None


def read_proc_stat_cpu_times() -> Tuple[int, int]:
    """
    Read the first 'cpu' line from /proc/stat and return:
      (idle_all, total_all)
    where idle_all = idle + iowait, total_all = sum(all cpu time fields).
    """
    with open("/proc/stat", "r", encoding="utf-8") as f:
        line = f.readline().strip()

    parts = line.split()
    if len(parts) < 5 or parts[0] != "cpu":
        raise RuntimeError("Unexpected /proc/stat format")

    nums = [int(x) for x in parts[1:]]
    idle = nums[3]
    iowait = nums[4] if len(nums) > 4 else 0

    idle_all = idle + iowait
    total_all = sum(nums)
    return idle_all, total_all


class CpuUtilReader:
    """
    CPU utilization reader.

    - On native Linux: computes CPU utilization from /proc/stat deltas (0.0-1.0).
    - On WSL: tries to read Windows host CPU utilization via PowerShell (0.0-1.0).
      If PowerShell read fails, falls back to /proc/stat deltas.

    Rationale:
        When heavy load is on Windows side (e.g., games), /proc/stat inside WSL may
        not reflect host usage well. PowerShell provides a host-wide metric closer
        to Windows Task Manager.
    """

    def __init__(self) -> None:
        self._wsl = _is_wsl()
        self._prev = read_proc_stat_cpu_times()

    def read_util(self) -> float:
        # Prefer Windows host CPU% on WSL
        if self._wsl:
            p = _read_windows_cpu_percent_via_powershell()
            if p is not None:
                return _clamp01(p / 100.0)
            # else: fall back to /proc/stat

        cur_idle, cur_total = read_proc_stat_cpu_times()
        prev_idle, prev_total = self._prev
        self._prev = (cur_idle, cur_total)

        dt_total = cur_total - prev_total
        dt_idle = cur_idle - prev_idle
        if dt_total <= 0:
            return 0.0

        util = 1.0 - (dt_idle / dt_total)
        return _clamp01(float(util))


class NvidiaSmiUtilReader:
    """
    Reads NVIDIA GPU utilization via nvidia-smi.
    If nvidia-smi is not available or fails, returns None.
    """
    def __init__(self, gpu_index: int = 0) -> None:
        self._gpu_index = int(gpu_index)

    def read_util(self) -> Optional[float]:
        try:
            out = subprocess.check_output(
                [
                    "nvidia-smi",
                    f"--id={self._gpu_index}",
                    "--query-gpu=utilization.gpu",
                    "--format=csv,noheader,nounits",
                ],
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
            if not out:
                return None
            v = float(out) / 100.0
            return _clamp01(v)
        except (OSError, subprocess.CalledProcessError, ValueError):
            return None


class UtilMonitor:
    """
    High-level reader that returns UtilSample.
    """
    def __init__(self, include_gpu: bool = True, gpu_index: int = 0) -> None:
        self._cpu = CpuUtilReader()
        self._include_gpu = bool(include_gpu)
        self._gpu = NvidiaSmiUtilReader(gpu_index)

    def read(self) -> UtilSample:
        cpu = self._cpu.read_util()

        if not self._include_gpu:
            return UtilSample(cpu_util=cpu, gpu_util=math.nan, status="OK_CPU_ONLY")

        gpu = self._gpu.read_util()
        if gpu is None:
            return UtilSample(cpu_util=cpu, gpu_util=math.nan, status="NO_GPU")

        return UtilSample(cpu_util=cpu, gpu_util=gpu, status="OK_CPU_GPU")
