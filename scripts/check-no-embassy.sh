#!/usr/bin/env bash
# Run in an independent checkout. Optional cargo global args select the explicit
# development overlay; without them this is the registry-only release check.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p target/boundary-check
for chip in chip-ws53 chip-ws63 chip-bs21; do
    for mode in '' ',async'; do
        cargo "$@" tree --locked --target riscv32imfc-unknown-none-elf \
            --edges normal --prefix none --no-default-features \
            --features "$chip,unstable$mode" > target/boundary-check/dependencies.txt
        if grep -Eq '^(embassy-|hisi-embassy |hisi-rtos )' target/boundary-check/dependencies.txt; then
            echo 'HAL must not depend on an Embassy component' >&2
            exit 1
        fi
    done
done
