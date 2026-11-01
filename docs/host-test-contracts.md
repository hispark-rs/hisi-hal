# Host test contracts

The independent `scripts/check-ws53-integration.sh` gate runs host tests,
host-test Clippy (`--tests -D warnings`), doctests and target Clippy for WS53,
WS63 and BS21. It uses explicit remote development dependencies while release
dependencies remain unpublished; passing this gate is not registry-only release
qualification. Parent integration owns QEMU and hardware acceptance.

DMA control/configuration properties call the production `DmaChannelConfig`
encoders. Field positions and reserved-bit assertions are independent expected
register contracts; a second encoder inside tests could silently diverge while
the tests remained green. `DmaTransferSize` construction is checked for the whole
u16 input domain sampled by proptest, including rejection above 4095. The existing
low-level constructor accepts zero; buffer APIs reject empty transfers separately.
These tests do not change that contract or qualify DMA stopping/cache safety.

The PWM channel-boundary test calls `PwmChannelId::from_index` for every u8 value
and checks valid round-trips as well as rejection. Compile-time chip/range
invariants use const assertions. Deliberately ending a DMA error object's scope
checks that retained buffers are not dropped, including if its Drop behavior
changes later; it is not replaced by forgetting that object in the test.

An isolated mutation experiment moved the production DMA source-burst field
from bit 12 to bit 13 and bus-lock from bit 14 to bit 15. The previous 12 properties
all passed because they tested copied encoders. The revised properties detected
three failures (burst decoding, lock decoding, reserved bits). The mutations were
never applied to published production code. Detailed inputs, versions and logs
belong to the parent integration evidence. This experiment qualifies these
properties' sensitivity, not every test in the repository or silicon behavior.


Stable WS63 (`chip-ws63`, with/without `rt,async`) is checked separately from
experimental WS63/BS21/WS53. Running only `unstable` configurations can miss
public-example access failures. DMA ownership and unsafe-call examples are
included only when their experimental API is available; negative examples
require the intended diagnostic (escaping borrow E0521, unsafe start/resume/resource recovery E0133). They must not pass merely because the module is
private (E0603). The positive owned-token example still compiles with `unstable`.
No example is ignored to make stable CI pass, and this does not expose DMA in
the stable API or change DMA/cache/lifetime behavior.
