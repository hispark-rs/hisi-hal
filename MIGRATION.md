## Unreleased 0.7.0-alpha.10: hardware time capability preparation

TIMER gains experimental `is_enabled` and (legacy IP) `set_interrupt_mask`.
TCXO gains executor-independent retained `poll_counter`; WS53 also exposes
`snapshot_ready`. These do not configure frequencies or prove cross-core ownership.
The legacy `read_counter`/`read_counter32` APIs retain their signatures; they
return `None` while a new nonblocking snapshot is outstanding instead of issuing
a second refresh over it. Complete the retained poll before using other methods.

# Migration From `hisi-riscv-hal`

`hisi-riscv-hal 0.6.x` is the final release line under the original package
name. It remains available on crates.io and the `release/0.6` branch accepts
critical correctness and security fixes.

New development continues as `hisi-hal`, beginning with `0.7.0-alpha.1`. The
rename does not imply a new hardware abstraction, feature policy, or stable API
surface. It gives the multi-chip HAL a name that is not tied to one ISA spelling.

The normal migration is:

```toml
[dependencies]
hisi-hal = { version = "0.7.0-alpha.1", features = ["chip-ws63"] }
```

and Rust imports change from `hisi_riscv_hal` to `hisi_hal`.

For a staged manifest-only migration, retain the old dependency key and source
imports while resolving the new package:

```toml
[dependencies]
hisi-riscv-hal = { package = "hisi-hal", version = "0.7.0-alpha.1", features = ["chip-ws63"] }
```

Do not depend on the GitHub repository redirect as a build contract. Released
applications should use the crates.io package and an explicit version range.
