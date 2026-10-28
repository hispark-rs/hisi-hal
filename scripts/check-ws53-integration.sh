#!/usr/bin/env bash
# WS53 pre-release integration only: pinned remote crates stand in for packages
# that are not published yet. This is not crates.io release qualification.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
pac_rev=8dc9a3d2582a29e90338c71d26b4e27d88d3d503
runtime_rev=03fbef84017a00dfef6f67b102096a20318731c4
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
# Copy tracked project files, including staged additions, without parent workspace
# patches or local Cargo.lock state. Tests and source must be tracked in Git.
git -C "$root" ls-files -z | tar -C "$root" --null -T - -cf - | tar -C "$work" -xf -
mkdir -p "$work/.cargo"
cat > "$work/.cargo/config.toml" <<CONFIG
[patch.crates-io]
ws53-pac = { git = "https://github.com/hispark-rs/ws53-pac.git", rev = "$pac_rev" }
hisi-riscv-rt = { git = "https://github.com/hispark-rs/hisi-riscv-rt.git", rev = "$runtime_rev" }
CONFIG
cd "$work"
printf '%s\n' 'WS53 pre-release integration: remote overrides; not registry release validation.'
cargo generate-lockfile
features=chip-ws53,rt,unstable,async,embassy,defmt
cargo test --locked --target x86_64-unknown-linux-gnu --no-default-features --features "$features"
cargo clippy --locked -Zbuild-std=core,alloc --target riscv32imfc-unknown-none-elf --no-default-features --features "$features" -- -D warnings
for features in chip-ws53 chip-ws53,chip-ws63,unstable chip-ws53,chip-bs21,unstable; do
    if cargo check --locked --target x86_64-unknown-linux-gnu --no-default-features --features "$features" > "$work/negative.log" 2>&1; then
        printf 'invalid feature combination succeeded: %s\n' "$features" >&2
        exit 1
    fi
    if [[ "$features" == chip-ws53 ]]; then
        grep -q 'chip-ws53 requires unstable' "$work/negative.log"
    else
        grep -q 'select exactly ONE chip feature' "$work/negative.log"
    fi
done
