#!/usr/bin/env python3
"""Audit HAL register-access patterns that should stay behind the PAC.

This is intentionally small and dependency-free so it can run in both the HAL's
standalone CI and the parent workspace CI. It is not a Rust parser; it is a guard
against the concrete regressions this crate has seen:

* raw volatile MMIO in production HAL code,
* numeric MMIO addresses cast to pointers,
* read/modify on registers modeled as write-only,
* whole-register read/modify/write via `w.bits(r.bits() ...)` without an explicit
  allowlist entry.

When this trips, prefer fixing the SVD/PAC model first. Add an allowlist entry
only when the PAC cannot reasonably model the dynamic bit position (for example
GPIO pin masks).
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# Registers whose PAC access should remain write-only. If code needs one of
# these as read-write, the SVD access type is probably wrong and must be reviewed.
WRITE_ONLY_ACCESSORS = {
    "fifo_ctl",
    "reset_status_clear",
    "sys_diag_clr_1",
}

# Whole-register RMW is permitted only for dynamic bit-mask registers where the
# active bit is a runtime pin/channel number and svd2rust cannot generate a field
# accessor for every legal bit.
ALLOW_DYNAMIC_RMW = {
    ("src/gpio.rs", "gpio_sw_oen"),
    ("src/gpio.rs", "gpio_int_en"),
    ("src/ulp_gpio.rs", "gpio_sw_oen"),
    ("src/ulp_gpio.rs", "gpio_int_en"),
}

RAW_VOLATILE = re.compile(r"\b(?:read_volatile|write_volatile)\b")
NUMERIC_PTR_CAST = re.compile(r"0x[0-9A-Fa-f_]+\s*(?:as\s+usize\s*)?as\s+\*(?:const|mut)\b")
WRITE_ONLY_READ_MODIFY = re.compile(
    r"\b(" + "|".join(re.escape(name) for name in sorted(WRITE_ONLY_ACCESSORS)) + r")\s*\(\)\s*\.\s*(read|modify)\s*\("
)
DYNAMIC_RMW = re.compile(r"\b(\w+)\s*\(\)\s*\.\s*modify\s*\([^;]*w\s*\.\s*bits\s*\(\s*r\s*\.\s*bits\s*\(")


def rel(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def check_file(path: Path) -> list[str]:
    problems: list[str] = []
    relative = rel(path)
    for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if RAW_VOLATILE.search(line):
            problems.append(f"{relative}:{lineno}: raw volatile access in HAL production code")
        if NUMERIC_PTR_CAST.search(line):
            problems.append(f"{relative}:{lineno}: numeric MMIO pointer cast; model it in SVD/PAC")
        if m := WRITE_ONLY_READ_MODIFY.search(line):
            problems.append(
                f"{relative}:{lineno}: {m.group(1)} is write-only; use a full write, not {m.group(2)}()"
            )
        if m := DYNAMIC_RMW.search(line):
            accessor = m.group(1)
            if (relative, accessor) not in ALLOW_DYNAMIC_RMW:
                problems.append(
                    f"{relative}:{lineno}: whole-register dynamic RMW on {accessor} must be modeled "
                    "as fields or added to ALLOW_DYNAMIC_RMW with a rationale"
                )
    return problems


def main() -> int:
    problems: list[str] = []
    for path in sorted((ROOT / "src").rglob("*.rs")):
        problems.extend(check_file(path))

    if problems:
        print("register-access policy violations:", file=sys.stderr)
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        return 1

    print("register-access policy: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
