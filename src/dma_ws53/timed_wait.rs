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

#[cfg(test)]
mod tests {
    use super::*;
    use core::{
        future::Future,
        task::{Context, Poll, Waker},
    };

    struct Finished {
        running: bool,
        failed: bool,
        polls: u32,
        stops: u32,
    }
    impl TimedTransfer for Finished {
        fn running(&self) -> bool {
            self.running
        }
        fn poll_step(&mut self) -> Result<Option<Completion>, Error> {
            self.polls += 1;
            if self.failed { Err(Error::Bus) } else { Ok(Some(Completion { elements: 17 })) }
        }
        fn stop(&mut self) {
            self.stops += 1;
        }
    }
    struct NoTimer;
    impl embedded_hal_async::delay::DelayNs for NoTimer {
        async fn delay_ns(&mut self, _: u32) {
            panic!("immediate result must not arm or wait for a timer");
        }
    }

    #[test]
    fn immediate_completion_and_error_need_no_timer() {
        for failed in [false, true] {
            let mut transfer = Finished { running: true, failed, polls: 0, stops: 0 };
            let mut timer = NoTimer;
            {
                let mut future = core::pin::pin!(wait(&mut transfer, &mut timer, NonZeroU32::new(1).unwrap()));
                let expected = if failed { Err(Error::Bus) } else { Ok(Completion { elements: 17 }) };
                assert_eq!(future.as_mut().poll(&mut Context::from_waker(Waker::noop())), Poll::Ready(expected));
            }
            assert_eq!(transfer.polls, 1);
            assert_eq!(transfer.stops, 1);
        }
    }

    #[test]
    fn inactive_transfer_neither_polls_nor_arms_timer() {
        let mut transfer = Finished { running: false, failed: false, polls: 0, stops: 0 };
        let mut timer = NoTimer;
        {
            let mut future = core::pin::pin!(wait(&mut transfer, &mut timer, NonZeroU32::new(1).unwrap()));
            assert_eq!(
                future.as_mut().poll(&mut Context::from_waker(Waker::noop())),
                Poll::Ready(Err(Error::NotReady))
            );
        }
        assert_eq!((transfer.polls, transfer.stops), (0, 0));
    }
}
