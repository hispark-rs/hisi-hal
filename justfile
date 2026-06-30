# Local dev recipes for hisi-riscv-hal — mirror the CI checks so a contributor can
# reproduce them before pushing. `just` = https://github.com/casey/just.
#
# The host target builds the lib + unit tests (the on-target HIL tests in
# tests/hil.rs are riscv-only and run via probe-rs — see ../../hil/README.md).

HOST := "x86_64-unknown-linux-gnu"

_default:
    @just --list

# Run everything CI runs (host): format, lint, test, SemVer.
ci: fmt clippy test semver

# Rustfmt check.
fmt:
    cargo fmt --all --check

# Clippy for both chips (chip-ws63 + chip-bs21 are mutually exclusive).
clippy:
    cargo clippy --no-default-features --features chip-ws63,rt,async,embassy -- -D warnings
    cargo clippy --no-default-features --features chip-bs21,rt -- -D warnings

# Host unit + property tests (chip-ws63).
test:
    cargo test --no-default-features --features chip-ws63 --target {{HOST}}

# SemVer check vs the last crates.io release — catches an accidental breaking API
# change that the version bump does not reflect (the same gate CI enforces).
# Only the explicit `chip-ws63` feature: the HAL has no default chip and `default`
# pulls host-unbuildable `rt`, so the default group would fail to build rustdoc.
# One-time install: `cargo install cargo-semver-checks`.
semver:
    cargo semver-checks --only-explicit-features --features chip-ws63

# Build + run the on-target HIL test suite on a real WS63 over probe-rs.
# Needs the patched probe-rs fork + hisi-fwpkg (see ../../hil/README.md).
# `unstable` is on so the peripheral-DMA HIL tests (gated behind it) compile + run.
# Add `,async` for the async DMA capstone (spi_dma_write_async / spi_dma_irq59).
hil:
    CARGO_TARGET_RISCV32IMFC_UNKNOWN_NONE_ELF_RUNNER=../../hil/embedded-test-runner.sh \
        cargo test -p hisi-riscv-hal --no-default-features --features chip-ws63,unstable \
        --target riscv32imfc-unknown-none-elf --test hil
