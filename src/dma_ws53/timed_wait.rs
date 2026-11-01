//! Timer-driven waits shared by DMA peripherals; no implicit IRQ or timeout.
use super::{Completion, Error};
use core::num::NonZeroU32;

pub(super) trait TimedTransfer {
    fn running(&self) -> bool;
    fn poll_step(&mut self) -> Result<Option<Completion>, Error>;
    fn stop(&mut self);
}

struct StopOnDrop<'a, T: TimedTransfer>(&'a mut T);
impl<T: TimedTransfer> Drop for StopOnDrop<'_, T> {
    fn drop(&mut self) {
        self.0.stop();
    }
}

pub(super) async fn wait<T: TimedTransfer, D: embedded_hal_async::delay::DelayNs>(
    transfer: &mut T,
    delay: &mut D,
    interval_us: NonZeroU32,
) -> Result<Completion, Error> {
    if !transfer.running() {
        return Err(Error::NotReady);
    }
    let guard = StopOnDrop(transfer);
    loop {
        if let Some(done) = guard.0.poll_step()? {
            return Ok(done);
        }
        delay.delay_us(interval_us.get()).await;
    }
}
