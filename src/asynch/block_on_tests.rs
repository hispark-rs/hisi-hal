//! Wake/lifetime tests only. The injected idle hook is not a WFI hardware model.
use super::*;
use core::future::poll_fn;
use std::sync::Mutex as HostMutex;

static SERIAL: HostMutex<()> = HostMutex::new(());

#[test]
fn ready_never_idles() {
    let _serial = SERIAL.lock().unwrap();
    assert_eq!(block_on_with_idle(core::future::ready(42), || panic!("unexpected idle")), 42);
}

#[test]
#[allow(clippy::waker_clone_wake, reason = "exercise both RawWaker wake callbacks, not only wake_by_ref")]
fn repeated_software_wakes_repoll_without_sleep() {
    let _serial = SERIAL.lock().unwrap();
    let mut polls = 0;
    block_on_with_idle(
        poll_fn(|cx| {
            polls += 1;
            if polls == 4 {
                return Poll::Ready(());
            }
            cx.waker().wake_by_ref();
            cx.waker().clone().wake();
            Poll::Pending
        }),
        || panic!("recorded software wake must not enter idle"),
    );
    assert_eq!(polls, 4);
}

#[test]
fn pending_without_wake_still_uses_idle_not_busy_polling() {
    let _serial = SERIAL.lock().unwrap();
    let mut polls = 0;
    let mut idles = 0;
    block_on_with_idle(
        poll_fn(|_| {
            polls += 1;
            if polls == 2 { Poll::Ready(()) } else { Poll::Pending }
        }),
        || idles += 1,
    );
    assert_eq!(idles, 1);
}

#[test]
fn retained_waker_after_return_is_safe_and_can_only_spuriously_wake() {
    let _serial = SERIAL.lock().unwrap();
    let old = block_on(poll_fn(|cx| Poll::Ready(cx.waker().clone())));
    old.wake_by_ref(); // No live call; cannot reference its old stack.
    let mut polls = 0;
    block_on_with_idle(
        poll_fn(|_| {
            polls += 1;
            if polls == 2 {
                Poll::Ready(())
            } else {
                old.wake_by_ref(); // Spurious notification for this invocation.
                Poll::Pending
            }
        }),
        || panic!("retained wake should be observed"),
    );
    drop(old);
    assert_eq!(block_on(core::future::ready(7)), 7);
}

#[test]
fn nested_call_rejected_and_unwind_releases_claim() {
    let _serial = SERIAL.lock().unwrap();
    let result = std::panic::catch_unwind(|| {
        block_on(poll_fn(|_| {
            block_on(core::future::ready(()));
            Poll::Ready(())
        }));
    });
    assert!(result.is_err());
    assert_eq!(block_on(core::future::ready(9)), 9);
}

#[test]
fn concurrent_call_rejected_without_polling_and_owner_can_finish() {
    let _serial = SERIAL.lock().unwrap();
    block_on(poll_fn(|cx| {
        let waker = cx.waker().clone();
        std::thread::spawn(move || {
            let result = std::panic::catch_unwind(|| {
                block_on(poll_fn(|_| -> Poll<()> { panic!("must never poll") }));
            });
            let message = result.expect_err("concurrent call must be rejected");
            assert_eq!(message.downcast_ref::<&str>(), Some(&"block_on does not support nested or concurrent calls"));
            waker.wake();
        }).join().unwrap();
        Poll::Ready(())
    }));
    block_on(core::future::ready(()));
}
