//! Application-side IPC doorbells and defensive SDK message formats.
//!
//! SDK 163d9150 selects two channels, causes26/27 and core IDs app2/control0.
//! Doorbells do not initialize shared rings, negotiate peripheral ownership,
//! flush caches, translate remote pointers or start/reset the control core.
//! Source of truth: hisi-registers, pinned by the PAC generation record. The send command is
//! a halfword, unlike the unpatched ROM. ACK has a separate discard-read view.

use crate::peripherals::Ipc;
use crate::soc::{chip::Interrupt, pac};
use core::sync::atomic::{Ordering, compiler_fence};

/// SDK-layout mailbox transactions on exclusively owned ordinary memory.
/// This does not grant a reference into a concurrently running peer's RAM.
#[path = "ipc_ws53/mailbox.rs"]
pub mod mailbox;

/// Experimental coherent SPSC transport storage with explicit peer contracts.
#[path = "ipc_ws53/spsc.rs"]
pub mod spsc;

/// Maximum inline payload accepted by the SDK application IPC profile.
pub const MAX_PAYLOAD: usize = 60;
/// Header plus maximum inline payload; excludes the twelve-byte ring metadata.
pub const MAX_MESSAGE: usize = 4 + MAX_PAYLOAD;

/// SDK boot-header mailbox placement; parsing does not attach to live memory.
#[path = "ipc_ws53/layout.rs"]
pub mod layout;

/// Known full enable-word state at the external IPC handoff.
/// No register read is evidenced by the SDK; the driver adopts this shadow.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReceiveEnableState {
    /// SDK disable-all word0; all unmodeled bits are also zero.
    Disabled,
    /// SDK enable-all word0xffff; retain the other fourteen low mask bits.
    SdkAllEnabled,
}
impl ReceiveEnableState {
    const fn mask(self) -> u16 {
        match self {
            Self::Disabled => 0,
            Self::SdkAllEnabled => u16::MAX,
        }
    }
}

/// One of the two application/control-core channels in the pinned port.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Channel {
    /// Channel0, local cause26.
    Channel0,
    /// Channel1, local cause27.
    Channel1,
}
impl Channel {
    /// Absolute local interrupt cause, not an offset or PLIC ID.
    pub const fn interrupt(self) -> Interrupt {
        match self {
            Self::Channel0 => Interrupt::APP_INT0,
            Self::Channel1 => Interrupt::APP_INT1,
        }
    }
}

/// Valid SDK message discriminator. This grants no permission to invoke a service.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MessageId(u8);
impl MessageId {
    /// Accept only the nineteen IDs in the fixed SDK protocol.
    pub const fn try_from_raw(raw: u8) -> Option<Self> {
        if raw < 19 { Some(Self(raw)) } else { None }
    }
    /// SDK IPC_MSG_UPDATE_SHARE_RANDOM_NUMBER discriminator.
    pub const UPDATE_SHARED_RANDOM_NUMBER: Self = Self(18);
    /// Wire discriminator; never interpreted as a callback or pointer.
    pub const fn raw(self) -> u8 {
        self.0
    }
}

/// SDK relative message priority. Channel choice is independent of priority.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Priority {
    /// Highest priority, wire value0.
    High = 0,
    /// Lowest priority, wire value1.
    Low = 1,
}

/// Defensive message format rejection; no shared memory is consumed on error.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MessageError {
    /// A payload or its declared length exceeds the SDK sixty-byte bound.
    PayloadTooLong,
    /// Missing header, truncated payload or trailing data.
    Framing,
    /// Discriminator is outside the SDK callback-table range.
    UnknownId,
    /// Priority is neither zero nor one.
    UnknownPriority,
}

/// Validated inline message borrowing ordinary application memory.
///
/// This is not a reference into concurrently written shared memory. First copy
/// a complete, coherently published message into locally owned storage under a
/// qualified ring protocol, then decode. No embedded address is dereferenced.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Message<'a> {
    id: MessageId,
    priority: Priority,
    payload: &'a [u8],
}
impl<'a> Message<'a> {
    /// Construct an inline message; reject overlength without narrowing/clamping.
    pub fn new(id: MessageId, priority: Priority, payload: &'a [u8]) -> Result<Self, MessageError> {
        if payload.len() > MAX_PAYLOAD {
            return Err(MessageError::PayloadTooLong);
        }
        Ok(Self { id, priority, payload })
    }
    /// Validate exactly one header and payload from locally owned memory.
    pub fn decode(bytes: &'a [u8]) -> Result<Self, MessageError> {
        if bytes.len() < 4 {
            return Err(MessageError::Framing);
        }
        let id = MessageId::try_from_raw(bytes[0]).ok_or(MessageError::UnknownId)?;
        let priority = match bytes[1] {
            0 => Priority::High,
            1 => Priority::Low,
            _ => return Err(MessageError::UnknownPriority),
        };
        let length = u16::from_le_bytes([bytes[2], bytes[3]]) as usize;
        if length > MAX_PAYLOAD {
            return Err(MessageError::PayloadTooLong);
        }
        if bytes.len() != 4 + length {
            return Err(MessageError::Framing);
        }
        Self::new(id, priority, &bytes[4..])
    }
    /// Encode one four-byte little-endian header and payload; return used length.
    /// Bytes beyond the returned prefix remain untouched.
    pub fn encode(self, destination: &mut [u8; MAX_MESSAGE]) -> usize {
        // Construction/decoding bounds length to60 before this conversion.
        let length = self.payload.len() as u16;
        destination[..4].copy_from_slice(&[self.id.raw(), self.priority as u8, length as u8, (length >> 8) as u8]);
        destination[4..4 + self.payload.len()].copy_from_slice(self.payload);
        4 + self.payload.len()
    }
    /// Validated wire discriminator; does not grant service ownership.
    pub const fn id(self) -> MessageId {
        self.id
    }
    /// Validated priority.
    pub const fn priority(self) -> Priority {
        self.priority
    }
    /// Locally owned input slice, never an unchecked peer pointer.
    pub const fn payload(self) -> &'a [u8] {
        self.payload
    }
}

/// Existing outstanding notification; no new send command was issued.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NotificationPending;

/// Configured application IPC register view. Shared ring memory is not owned.
///
/// Drop deliberately does not reset IPC, clear pending messages or change the
/// peer's state. These primitives are not a complete message transport.
pub struct Doorbell<'d> {
    _peripheral: Ipc<'d>,
    registers: &'d pac::m_crg::RegisterBlock,
    receive_mask: u16,
}
impl<'d> Doorbell<'d> {
    /// Attach to externally initialized application-side doorbell registers.
    /// No register is written or initialized by attachment.
    ///
    /// # Safety
    /// Establish this core's exclusive access to the IPC registers/causes26/27
    /// (not the whole shared M_CRG bank). Quiesce old ROM/SDK/local ISR users.
    /// Keep IPC reset released, its bus/clock/power accessible and low-power
    /// transitions excluded while attached. Coordinate any accesses by another
    /// master. A control-core peer may run using its own side of the protocol;
    /// this constructor does not negotiate its shared-ring/cache/memory duties.
    /// Hand off shared memory and local interrupt handlers separately. A clock
    /// constant, observed base or Rust singleton alone proves none of these.
    /// Own the entire application receive-enable word and ensure it exactly
    /// matches `initial_enable_state` (upper16 bits zero). No other local or
    /// bus-master writer may change it while attached. Per-channel enable-bit
    /// gating is part of the externally qualified configured profile; source
    /// convention alone is not silicon validation. Unknown mask states cannot
    /// be adopted by this API and must be handed off by the platform first.
    pub unsafe fn from_configured(peripheral: Ipc<'d>, initial_enable_state: ReceiveEnableState) -> Self {
        Self {
            _peripheral: peripheral,
            // SAFETY: caller established accessible configured register bank;
            // methods access only the IPC subset through generated PAC fields.
            registers: unsafe { &*pac::MCrg::PTR },
            receive_mask: initial_enable_state.mask(),
        }
    }
    /// Read pending incoming notification without consuming ring data.
    pub fn receive_pending(&self, channel: Channel) -> bool {
        let value = self.registers.ipc_receive_status().read();
        match channel {
            Channel::Channel0 => value.channel0().bit(),
            Channel::Channel1 => value.channel1().bit(),
        }
    }
    /// Read outgoing pending. Despite the ROM helper name, this is not a clear.
    pub fn send_pending(&self, channel: Channel) -> bool {
        let value = self.registers.ipc_send_status().read();
        match channel {
            Channel::Channel0 => value.channel0().bit(),
            Channel::Channel1 => value.channel1().bit(),
        }
    }
    /// Update one channel in the owned shadow mask, then write the whole word.
    /// Preserve the other handed-off low16 bits; never read an unverified view.
    /// Local interrupt-controller enable/handler installation is separate.
    pub fn set_receive_enabled(&mut self, channel: Channel, enabled: bool) {
        let bit = match channel {
            Channel::Channel0 => 1,
            Channel::Channel1 => 2,
        };
        if enabled {
            self.receive_mask |= bit;
        } else {
            self.receive_mask &= !bit;
        }
        // SAFETY: caller handed off the complete enable word in a known SDK
        // state. The shadow preserves its fourteen unmodeled low bits; upper
        // bits stay zero. No hardware read, dynamic RMW or reset is inferred.
        unsafe { self.registers.ipc_receive_enable().write_with_zero(|w| w.bits(u32::from(self.receive_mask))) };
    }
    /// ACK before draining the ring, then fence and discard the SDK readback.
    /// Recheck queue state and pending after drain to handle arrivals during ACK;
    /// clear the local interrupt controller only after that bounded protocol.
    /// This does not advance cursors, release buffers or certify DMA/cache drain.
    pub fn acknowledge(&mut self, channel: Channel) {
        // SAFETY: complete mask write through a command-only PAC view; zeros
        // leave the other source untouched. No command read/modify operation.
        unsafe {
            self.registers.ipc_receive_ack_command().write_with_zero(|w| match channel {
                Channel::Channel0 => w.channel0().set_bit(),
                Channel::Channel1 => w.channel1().set_bit(),
            })
        };
        io_fence();
        let _ = self.registers.ipc_receive_ack_sync().read().bits();
    }
    /// Notify a peer once, leaving an already outstanding notification intact.
    /// A pending result is not proof that a newly queued message was consumed.
    ///
    /// # Safety
    /// Publish a valid SDK-compatible message/ring state first, including cache
    /// maintenance and the cross-core ordering/ownership protocol. Retain every
    /// referenced buffer until the actual peer consumption contract permits
    /// release. The fence here orders accesses but does not flush caches or
    /// establish those duties. Serialize all local notification producers; the
    /// mutable borrow handles Rust users of this object, not unrelated ROM/ISR
    /// users or other bus masters. Never signal an uninitialized/live foreign
    /// queue or infer remote ownership transfer from a successful register write.
    pub unsafe fn notify(&mut self, channel: Channel) -> Result<(), NotificationPending> {
        if self.send_pending(channel) {
            return Err(NotificationPending);
        }
        compiler_fence(Ordering::Release);
        io_fence();
        // SAFETY: exclusive configured doorbell, fixed channel mask and patched
        // halfword access. The caller establishes the separate publication duty.
        unsafe {
            self.registers.ipc_send_set_command().write_with_zero(|w| match channel {
                Channel::Channel0 => w.channel0().set_bit(),
                Channel::Channel1 => w.channel1().set_bit(),
            })
        };
        Ok(())
    }
}

fn io_fence() {
    compiler_fence(Ordering::SeqCst);
    #[cfg(target_arch = "riscv32")]
    // SAFETY: a local I/O ordering barrier, no pointers or implicit stack use.
    unsafe {
        core::arch::asm!("fence iorw, iorw", options(nostack, preserves_flags))
    };
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "ipc_ws53/tests.rs"]
mod tests;
