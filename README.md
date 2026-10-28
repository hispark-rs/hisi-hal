# hisi-hal

`hisi-hal` is the no-std hardware abstraction layer for supported HiSilicon
embedded chips. It provides typed, lifetime-owned peripheral drivers built on
`embedded-hal` traits, with WS63 as the current real-silicon HIL target.

Select exactly one chip feature:

```toml
[dependencies]
hisi-hal = { version = "0.7.0-alpha.1", features = ["chip-ws63"] }
```

```rust
use hisi_hal::{gpio::OutputConfig, peripherals::Peripherals};
```

The `unstable` feature exposes APIs whose safety, ownership, or silicon evidence
is not yet sufficient for the default stable surface. BS2X currently requires
`unstable` because no connected BS2X HIL board exists.

The previous package name remains maintained on the `release/0.6` branch for
critical fixes. See [MIGRATION.md](MIGRATION.md) for dependency-key and import
migration options.

WS53 support is experimental and requires `chip-ws53,unstable`. Resource
handoffs and ownership preconditions are documented on the corresponding APIs.
The [integration notes](docs/ws53-integration-history.md) preserve prior design
and test context; current cross-repository qualification belongs to the parent
project's evidence and status records. WS53 crates.io release qualification is
blocked until the required PAC and runtime versions are published.
