#!/usr/bin/env bash
# WS53 pre-release integration only: pinned remote crates stand in for packages
# that are not published yet. This is not crates.io release qualification.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_rev=eb3e2d55a41c7655c2be83e465e5dc524eb1d7cb
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
# Copy tracked project files, including staged additions, without parent workspace
# patches or local Cargo.lock state. Tests and source must be tracked in Git.
git -C "$root" ls-files -z | tar -C "$root" --null -T - -cf - | tar -C "$work" -xf -
mkdir -p "$work/.cargo"
cat > "$work/.cargo/config.toml" <<CONFIG
[patch.crates-io]
hisi-riscv-rt = { git = "https://github.com/hispark-rs/hisi-riscv-rt.git", rev = "$runtime_rev" }
CONFIG
cd "$work"
printf '%s\n' 'WS53 pre-release integration: remote overrides; not registry release validation.'
cargo generate-lockfile
features=chip-ws53,rt,unstable,async,defmt
cargo test --locked --target x86_64-unknown-linux-gnu --no-default-features --features "$features"
cargo clippy --locked -Zbuild-std=core,alloc --target riscv32imfc-unknown-none-elf --no-default-features --features "$features" -- -D warnings
# Cargo resolves optional dependencies even when these old-chip features do not
# enable WS53. Until WS53 runtime support is published, preserve independent host and
# target regression coverage with the same pinned remote integration overrides.
# The registry-only release jobs remain separate and must still pass for release.
for features in chip-ws63,rt,async,unstable chip-bs21,rt,unstable; do
    cargo test --locked --target x86_64-unknown-linux-gnu --no-default-features --features "$features"
    cargo clippy --locked -Zbuild-std=core,alloc --target riscv32imfc-unknown-none-elf --no-default-features --features "$features" -- -D warnings
done
for features in chip-ws53 chip-bs21 chip-ws53,chip-ws63,unstable chip-ws53,chip-bs21,unstable; do
    if cargo check --locked --target x86_64-unknown-linux-gnu --no-default-features --features "$features" > "$work/negative.log" 2>&1; then
        printf 'invalid feature combination succeeded: %s\n' "$features" >&2
        exit 1
    fi
    if [[ "$features" == chip-ws53 ]]; then
        grep -q 'chip-ws53 requires unstable' "$work/negative.log"
    elif [[ "$features" == chip-bs21 ]]; then
        grep -q 'BS2X support is experimental' "$work/negative.log"
    else
        grep -q 'select exactly ONE chip feature' "$work/negative.log"
    fi
done
