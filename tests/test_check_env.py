"""Tests for the host diagnostics script (runs on any machine)."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

ROOT = Path(__file__).resolve().parents[1]


def _load_module() -> ModuleType:
    spec = importlib.util.spec_from_file_location("check_env", ROOT / "scripts" / "check_env.py")
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules["check_env"] = module
    spec.loader.exec_module(module)
    return module


def test_checks_return_well_formed_results() -> None:
    mod = _load_module()
    checks = [mod.check_os(), mod.check_python(), *mod.check_gpu(), *mod.check_docker(), *mod.check_resources()]
    assert checks
    for c in checks:
        assert isinstance(c.name, str) and isinstance(c.ok, bool) and isinstance(c.detail, str)


def test_non_rtx_gpu_is_flagged(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    mod = _load_module()
    monkeypatch.setattr(mod, "run", lambda cmd: "NVIDIA A100-SXM4-40GB, 40960, 550.54")
    gpu, vram = mod.check_gpu()
    assert not gpu.ok and vram.ok
