"""Host environment diagnostics (does NOT require Isaac Sim).

Run this on any machine (laptop, workstation, AWS instance) before installing
or launching Isaac Sim. It reports the information needed to judge whether the
machine can run the simulation and prints actionable warnings.

Usage::

    python scripts/check_env.py
"""

from __future__ import annotations

import platform
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

#: Python version bundled with Isaac Sim 5.x.
ISAAC_PYTHON = (3, 11)
#: Minimum free disk space recommended for the container image plus caches [GB].
MIN_FREE_DISK_GB = 60.0
#: Minimum recommended VRAM [MiB].
MIN_VRAM_MIB = 8 * 1024
#: GPU name fragments without RT cores (not supported for RTX rendering).
NON_RTX_GPUS = ("A100", "H100", "H200", "V100", "P100", "T4", "K80")


@dataclass
class Check:
    """Result of a single diagnostic check."""

    name: str
    ok: bool
    detail: str


def run(cmd: list[str]) -> str | None:
    """Run a command and return its stdout, or ``None`` if it fails."""
    if shutil.which(cmd[0]) is None:
        return None
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=20, check=True)
    except (subprocess.SubprocessError, OSError):
        return None
    return out.stdout.strip()


def check_os() -> Check:
    """Check that the host is Linux (Ubuntu 22.04/24.04 recommended)."""
    distro = "unknown"
    os_release = Path("/etc/os-release")
    if os_release.exists():
        for line in os_release.read_text(encoding="utf-8").splitlines():
            if line.startswith("PRETTY_NAME="):
                distro = line.split("=", 1)[1].strip('"')
    ok = platform.system() == "Linux"
    return Check("Operating system", ok, f"{platform.system()} {platform.release()} ({distro})")


def check_python() -> Check:
    """Check the host Python version (3.11 recommended to match Isaac Sim)."""
    ver = sys.version_info
    ok = (ver.major, ver.minor) == ISAAC_PYTHON
    detail = f"{ver.major}.{ver.minor}.{ver.micro}"
    if not ok:
        detail += " (use the 'dronenav' conda env with Python 3.11 to match Isaac Sim)"
    return Check("Host Python", ok, detail)


def check_gpu() -> list[Check]:
    """Check the NVIDIA driver, GPU model and VRAM through ``nvidia-smi``."""
    out = run(["nvidia-smi", "--query-gpu=name,memory.total,driver_version", "--format=csv,noheader,nounits"])
    if out is None:
        return [Check("NVIDIA GPU", False, "nvidia-smi not found or failed: no usable NVIDIA driver")]
    checks = []
    for idx, line in enumerate(out.splitlines()):
        name, mem, driver = (field.strip() for field in line.split(","))
        rtx = not any(tag in name for tag in NON_RTX_GPUS)
        checks.append(Check(f"GPU {idx}", rtx, f"{name}, driver {driver}" + ("" if rtx else " (no RT cores!)")))
        checks.append(Check(f"GPU {idx} VRAM", float(mem) >= MIN_VRAM_MIB, f"{float(mem) / 1024:.1f} GiB"))
    return checks


def check_docker() -> list[Check]:
    """Check Docker and the NVIDIA Container Toolkit."""
    docker = run(["docker", "--version"])
    toolkit = run(["nvidia-ctk", "--version"])
    return [
        Check("Docker", docker is not None, docker or "not installed"),
        Check(
            "NVIDIA Container Toolkit",
            toolkit is not None,
            toolkit.splitlines()[0] if toolkit else "nvidia-ctk not found",
        ),
    ]


def check_resources() -> list[Check]:
    """Check RAM and free disk space."""
    checks = []
    meminfo = Path("/proc/meminfo")
    if meminfo.exists():
        total_kib = int(meminfo.read_text(encoding="utf-8").split("\n")[0].split()[1])
        gib = total_kib / 1024**2
        checks.append(Check("RAM", gib >= 31.0, f"{gib:.1f} GiB (32 GiB minimum recommended)"))
    free_gb = shutil.disk_usage(Path.home()).free / 1e9
    checks.append(Check("Free disk (home)", free_gb >= MIN_FREE_DISK_GB, f"{free_gb:.0f} GB"))
    return checks


def main() -> int:
    """Run every check and print a report. Returns 0 if all checks pass."""
    checks = [check_os(), check_python(), *check_gpu(), *check_docker(), *check_resources()]
    width = max(len(c.name) for c in checks)
    print("isaac-drone-nav environment check\n")
    for c in checks:
        print(f"  [{'OK' if c.ok else '!!'}] {c.name:<{width}}  {c.detail}")
    failed = [c for c in checks if not c.ok]
    print(f"\n{len(checks) - len(failed)}/{len(checks)} checks passed.")
    if failed:
        print("See docs/setup.md for how to fix the flagged items.")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
