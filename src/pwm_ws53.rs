//! WS53 application PWM v151, SDK163d9150 / RDL f188fd4.
//!
//! This is the SDK's 255-tick application profile, not a measured silicon width.
//! Handoff must cover the whole block, shared IRQs, DMA quiescence, clock source
//! and serialized clock-control halfwords. High count halves must already be
//! zero. The driver configures the documented divider16 register sequence; it does
//! not negotiate the upstream clock or cross-core ownership. Waveform, shadow
//! latching, stop completion and electrical output still require acceptance.
use crate::interrupt::{self, Interrupt};
use crate::peripherals::Pwm;
use crate::pinctrl::PadControl;
use crate::soc::pac;
use embedded_hal::delay::DelayNs;

/// SDK-reported rate, not a clock guarantee from this driver. A-board bounded
/// RAM takeover measured an effective 2MHz with the same divider16 sequence.
/// The inherited source and any upstream division require independent handoff.
pub const SDK_REPORTED_CLOCK_HZ: u32 = 32_000_000;

/// Legacy SDK report; do not use to infer actual PWM frequency.
#[deprecated(note = "SDK report only; use SDK_REPORTED_CLOCK_HZ and independently qualify the board clock")]
pub const PWM_CLOCK_HZ: u32 = SDK_REPORTED_CLOCK_HZ;

/// PWM-local divisors verified on A-board with the inherited effective32MHz
/// input. This selects only PWM_M_DIV3, not the upstream clock source. Other
/// field encodings remain unavailable until their behavior is established.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ClockDivider {
    /// Divide by1.
    Div1 = 1,
    /// Divide by2.
    Div2 = 2,
    /// Divide by4.
    Div4 = 4,
    /// Divide by8.
    Div8 = 8,
    /// Divide by16; the SDK initialization sequence uses this value.
    Div16 = 16,
}
impl ClockDivider {
    /// Accept only measured divisors; never clamp or reinterpret an encoding.
    pub const fn from_divisor(value: u8) -> Option<Self> {
        match value {
            1 => Some(Self::Div1),
            2 => Some(Self::Div2),
            4 => Some(Self::Div4),
            8 => Some(Self::Div8),
            16 => Some(Self::Div16),
            _ => None,
        }
    }
    /// The programmed integer divisor, not a counter frequency guarantee.
    pub const fn divisor(self) -> u8 {
        self as u8
    }
}

macro_rules! ids {
    ($name:ident, $doc:literal, $($variant:ident = $value:literal),+) => {
        #[doc=$doc]
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        #[repr(u8)]
        pub enum $name { $(#[doc=concat!(stringify!($name), " ", stringify!($value))] $variant=$value),+ }
        impl $name {
            /// Parse only configured instances; not an SDK pin or board IO number.
            pub const fn from_index(index: u8) -> Option<Self> {
                match index { $($value => Some(Self::$variant),)+ _=>None }
            }
            /// Zero-based hardware identity.
            pub const fn index(self) -> u8 { self as u8 }
        }
    };
}
ids!(Channel, "One of eight application PWM channels.", C0 = 0, C1 = 1, C2 = 2, C3 = 3, C4 = 4, C5 = 5, C6 = 6, C7 = 7);
ids!(Group, "One of four application PWM commit groups.", G0 = 0, G1 = 1, G2 = 2, G3 = 3);

/// Nonzero pulse-block length, within the SDK public limit (not physical width).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PulseCount(u16);
impl PulseCount {
    /// Reject zero and values above32767 instead of truncating.
    pub const fn new(count: u32) -> Option<Self> {
        if count == 0 || count > 32767 { None } else { Some(Self(count as u16)) }
    }
    /// Number of waveform periods per pulse block.
    pub const fn count(self) -> u16 {
        self.0
    }
}
/// Repeat policy belongs to a channel, never a driver-global flag.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PulseMode {
    /// Continuous waveform; initial configuration-load notification may occur.
    Continuous,
    /// One finite block, using the source's two-commit staging sequence.
    Finite(PulseCount),
    /// Repeat pulse blocks, allowing step-end notifications each block.
    Repeating(PulseCount),
}
impl PulseMode {
    fn cycles(self) -> u16 {
        match self {
            Self::Continuous => 0,
            Self::Finite(n) | Self::Repeating(n) => n.count(),
        }
    }
    fn finite(self) -> bool {
        matches!(self, Self::Finite(_))
    }
}
/// Count-based waveform with total1..255, high<=total and offset<=low.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Waveform {
    period: u8,
    high: u8,
    offset: u8,
    mode: PulseMode,
}
impl Waveform {
    /// Validate with wide arithmetic. Zero/overflow/unsupported totals are errors.
    /// Zero/full duty are valid count encodings; physical endpoint behavior is unverified.
    pub const fn new(low: u32, high: u32, offset: u32, mode: PulseMode) -> Option<Self> {
        let total = low as u64 + high as u64;
        if total == 0 || total > 255 || offset > low {
            None
        } else {
            Some(Self { period: total as u8, high: high as u8, offset: offset as u8, mode })
        }
    }
    /// Total input-clock ticks per waveform period.
    pub const fn period(self) -> u16 {
        self.period as u16
    }
    /// Active ticks before polarity inversion (this profile uses normal polarity).
    pub const fn high(self) -> u16 {
        self.high as u16
    }
    /// Initial inactive ticks within the low portion.
    pub const fn offset(self) -> u16 {
        self.offset as u16
    }
    /// Per-channel repeat policy.
    pub const fn mode(self) -> PulseMode {
        self.mode
    }
    fn with_duty(self, high: u16) -> Result<Self, Error> {
        if high > self.period() || self.offset() > self.period() - high {
            return Err(Error::InvalidDuty);
        }
        Ok(Self { high: high as u8, ..self })
    }
}
/// Group layout, validated without hardware writes.
#[derive(Clone, Copy, Debug)]
pub struct GroupConfig {
    channels: [Option<Waveform>; 8],
}
impl GroupConfig {
    /// A group must be nonempty, with each channel appearing once.
    pub fn new(entries: &[(Channel, Waveform)]) -> Result<Self, Error> {
        if entries.is_empty() {
            return Err(Error::EmptyGroup);
        }
        let mut config = Self { channels: [None; 8] };
        for &(ch, wave) in entries {
            let slot = &mut config.channels[ch.index() as usize];
            if slot.is_some() {
                return Err(Error::DuplicateChannel);
            }
            *slot = Some(wave);
        }
        Ok(config)
    }
    fn mask(self) -> u8 {
        self.channels.iter().enumerate().fold(0, |bits, (i, w)| bits | if w.is_some() { 1 << i } else { 0 })
    }
}
/// Driver configuration or operational error; no silent truncation or clamping.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// Empty groups have no defined start action.
    EmptyGroup,
    /// A channel appeared twice in a configuration.
    DuplicateChannel,
    /// A channel already belongs to another group or still has a route attached.
    ChannelInUse,
    /// A live group must be stopped before changing its layout or detaching pins.
    GroupRunning,
    /// No configuration exists for the requested group/channel.
    Unconfigured,
    /// A required physical output route was not handed over.
    MissingRoute,
    /// Duty exceeds period or collides with the configured phase offset.
    InvalidDuty,
    /// The trait channel view requires a single continuous/repeating channel group.
    RequiresSingleContinuousChannel,
    /// A finite block ended before stop staging could be verified. The whole
    /// group was stopped; the requested pulse count is not guaranteed.
    FiniteDeadlineMissed,
}
impl embedded_hal::pwm::Error for Error {
    fn kind(&self) -> embedded_hal::pwm::ErrorKind {
        embedded_hal::pwm::ErrorKind::Other
    }
}
/// Exclusive physical output route. Keeps pad ownership until explicitly detached.
pub struct OutputRoute<'d> {
    channel: Channel,
    _pad: PadControl<'d>,
}
impl<'d> OutputRoute<'d> {
    /// Accept an externally verified pinmux route without changing its electrical settings.
    ///
    /// # Safety
    /// This pad must already route the specified PWM channel, with the required
    /// output mode/power/drive and no conflicting external driver. Verify actual
    /// channel-to-pad mapping; numeric MIO/AIO/MGPIO/SDK identifiers are not interchangeable.
    /// No other alias of this PWM output may drive an unowned pad. The old PWM
    /// owner must be quiesced. Pad clocks and ownership remain valid through Drop.
    pub unsafe fn from_configured(channel: Channel, pad: PadControl<'d>) -> Self {
        Self { channel, _pad: pad }
    }
    /// Hardware channel routed by this owner.
    pub const fn channel(&self) -> Channel {
        self.channel
    }
    /// Recover the exclusive pad permission after detaching from a stopped controller.
    pub fn into_pad(self) -> PadControl<'d> {
        self._pad
    }
}
/// Shared interrupt outputs to unmask; source handlers must be installed separately.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EventEnable {
    /// Mask both outputs.
    Disabled,
    /// Abnormal group/count configuration only.
    Abnormal,
    /// Configuration-load notifications, including initial START.
    ConfigurationLoaded,
    /// Both outputs.
    All,
}
/// Snapshot of both shared error words and eight configuration-load flags.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Events {
    /// SDK16-bit group-conflict bitmap; unknown upper bits are retained.
    pub group_errors: u16,
    /// SDK16-bit count-error bitmap; unknown upper bits are retained.
    pub count_errors: u16,
    /// One bit per channel0..7; initial START also sets it before burst completion.
    pub configuration_loaded: u8,
}
/// Owns all PWM groups, both IRQs and attached routes. Drop stops groups, masks
/// events and gates the PWM clock; no channel alone tears down shared IRQs.
///
/// ```compile_fail
/// use hisi_hal::{pwm::PwmDriver, peripherals::{Pwm,Unclaimed}};
/// fn without_handoff(pwm: Unclaimed<Pwm<'static>>, delay: impl embedded_hal::delay::DelayNs) {
///     let _driver = PwmDriver::new(pwm, delay);
/// }
/// ```
pub struct PwmDriver<'d, D: DelayNs> {
    _token: Pwm<'d>,
    engine: Engine<PacRegisters, D>,
    routes: [Option<OutputRoute<'d>>; 8],
}
impl<'d, D: DelayNs> PwmDriver<'d, D> {
    /// Apply the clock profile and initialize the already-quiesced handed-over block.
    /// Uses no global critical section or instruction-count delay. Delay must be
    /// independent of PWM and remain operational through controller Drop.
    pub fn new(token: Pwm<'d>, delay: D) -> Self {
        let mut engine = Engine::new(PacRegisters, delay);
        engine.initialize();
        Self { _token: token, engine, routes: core::array::from_fn(|_| None) }
    }
    /// Change the PWM-local divider only after stopping every group, including
    /// groups with an observed finite-completion event. Running groups reject
    /// the change before MMIO. This does not establish the upstream clock or
    /// promise glitch-free live switching. Constructor default remains Div16.
    pub fn set_clock_divider(&mut self, divider: ClockDivider) -> Result<(), Error> {
        with_irqs_suspended(|| self.engine.set_clock_divider(divider))
    }
    /// Retain a separately validated route. On failure return its ownership.
    pub fn attach(&mut self, route: OutputRoute<'d>) -> Result<(), (Error, OutputRoute<'d>)> {
        let slot = &mut self.routes[route.channel.index() as usize];
        if slot.is_some() {
            return Err((Error::ChannelInUse, route));
        }
        *slot = Some(route);
        Ok(())
    }
    /// Detach only when no group references the channel, including stopped groups.
    pub fn detach(&mut self, ch: Channel) -> Result<Option<OutputRoute<'d>>, Error> {
        if self.engine.group_for(ch).is_some() {
            return Err(Error::ChannelInUse);
        }
        Ok(self.routes[ch.index() as usize].take())
    }
    /// Configure an idle group with disjoint channels and handed-over output routes.
    /// All checks precede register changes; high count halves are not modified.
    pub fn configure_group(&mut self, group: Group, config: GroupConfig) -> Result<(), Error> {
        let routes = self.routes.iter().enumerate().fold(0, |m, (i, r)| m | if r.is_some() { 1 << i } else { 0 });
        with_irqs_suspended(|| self.engine.configure(group, config, routes))
    }
    /// Start all selected channels together; finite channels get their own stop staging.
    /// Call stop before restarting, including after an observed finite completion.
    /// Finite mode guards the initial enabled START for10us before staging EN=0,
    /// then retains the source1us staging interval. Use an independent clock.
    /// The initial configuration-load flag is acknowledged before stop staging.
    /// Another flag observed during staging stops the group with
    /// `FiniteDeadlineMissed`. This is a conservative observation, not an elapsed
    /// deadline proof: long preemption or short blocks can already emit extras.
    /// Exact pulse counts require a qualified clock, delay and latency bound;
    /// only C0/G0, 100us periods, 8/16 pulses at divisor16 have physical evidence.
    /// The software running state is deliberately not guessed from a latched event.
    pub fn start(&mut self, group: Group) -> Result<(), Error> {
        with_irqs_suspended(|| self.engine.start(group))
    }
    /// Commit disabled outputs for this group and retain its configuration/routes.
    /// Hardware completion timing is not certified by the source1us staging delay.
    pub fn stop(&mut self, group: Group) {
        with_irqs_suspended(|| self.engine.stop(group));
    }
    /// Stop then remove the group's channel assignment, allowing route detachment.
    pub fn clear_group(&mut self, group: Group) {
        with_irqs_suspended(|| {
            self.engine.stop(group);
            self.engine.groups[group.index() as usize] = None;
            self.engine.io.select(group, 0);
        });
    }
    /// Access embedded-hal duty control for an independently assigned, non-finite channel.
    /// Updates use the SDK refresh operation and restart the phase of that group.
    pub fn channel(&mut self, ch: Channel) -> Result<DutyChannel<'_, 'd, D>, Error> {
        let group = self.engine.group_for(ch).ok_or(Error::Unconfigured)?;
        let config = self.engine.groups[group.index() as usize].unwrap();
        if config.mask().count_ones() != 1 || config.channels[ch.index() as usize].unwrap().mode.finite() {
            return Err(Error::RequiresSingleContinuousChannel);
        }
        Ok(DutyChannel { driver: self, ch, group })
    }
    /// Set peripheral event enables without clearing status or enabling local handlers.
    /// The caller installs handlers for PWM_ABNOR/PWM_CFG before enabling those sources.
    pub fn set_event_enable(&mut self, enable: EventEnable) {
        self.engine.io.event_enable(enable);
    }
    /// Poll status. Reading does not clear any event. The words are sampled in
    /// sequence, not atomically; repeated hardware events may coalesce into one bit.
    pub fn events(&self) -> Events {
        events(&PacRegisters)
    }
}
impl<D: DelayNs> Drop for PwmDriver<'_, D> {
    fn drop(&mut self) {
        self.engine.shutdown();
    }
}
/// Mutable duty view; cannot outlive or alias the owning controller.
pub struct DutyChannel<'a, 'd, D: DelayNs> {
    driver: &'a mut PwmDriver<'d, D>,
    ch: Channel,
    group: Group,
}
impl<D: DelayNs> embedded_hal::pwm::ErrorType for DutyChannel<'_, '_, D> {
    type Error = Error;
}
impl<D: DelayNs> embedded_hal::pwm::SetDutyCycle for DutyChannel<'_, '_, D> {
    fn max_duty_cycle(&self) -> u16 {
        self.driver.engine.groups[self.group.index() as usize].unwrap().channels[self.ch.index() as usize]
            .unwrap()
            .period()
    }
    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Error> {
        with_irqs_suspended(|| self.driver.engine.set_duty(self.group, self.ch, duty))
    }
}

/// Snapshot and acknowledge only observed events from either registered shared ISR.
/// No callback, wait, or driver borrow is performed.
/// An empty snapshot is valid: IRQ72 can remain pending in the local controller
/// after a configuration-load flag was cleared while delivery was suspended.
/// Count/wake completed work from the returned event bits, not ISR entry alone.
/// IRQ71 errors reassert while the configuration remains invalid: acknowledgement
/// alone is not recovery. Mask abnormal-event delivery before acknowledging,
/// repair and commit the configuration in normal context, then clear and verify
/// the raw error status before re-enabling delivery. This function does not
/// change event enables or repair configuration on the caller's behalf.
///
/// # Safety
/// A live controller must own the complete PWM block/IRQs with clocks available.
/// Call only from either of its handlers, or with both disabled. Do not race
/// another core/NMI, shutdown or reconfiguration. Record/wake users after return.
pub unsafe fn acknowledge_interrupt() -> Events {
    let mut io = PacRegisters;
    let seen = events(&io);
    io.ack(seen);
    interrupt::clear_pending(Interrupt::PWM_ABNOR);
    interrupt::clear_pending(Interrupt::PWM_CFG);
    seen
}
// Only the two owned sources are suspended. Any delay in the closure executes
// outside global critical sections, and other groups' pending events/priority
// survive. The controller's handoff contract excludes competing source owners.
fn with_irqs_suspended<R>(f: impl FnOnce() -> R) -> R {
    unsafe {
        interrupt::with_source_suspended(Interrupt::PWM_ABNOR, || {
            interrupt::with_source_suspended(Interrupt::PWM_CFG, f)
        })
    }
}
trait Registers {
    fn clock(&mut self, on: bool);
    fn clock_divider(&mut self, divider: ClockDivider);
    fn disable_irqs(&mut self);
    fn event_enable(&mut self, enable: EventEnable);
    fn dma_off(&mut self);
    fn select(&mut self, g: Group, mask: u8);
    fn enable(&mut self, ch: Channel, on: bool);
    fn configure(&mut self, ch: Channel, w: Waveform);
    fn commit(&mut self, g: Group, clear: bool);
    fn errors(&self) -> (u16, u16);
    fn period_flag(&self, ch: Channel) -> bool;
    fn ack(&mut self, events: Events);
}
fn channels(mask: u8) -> impl Iterator<Item = Channel> {
    (0..8).filter(move |i| mask & (1 << i) != 0).map(|i| Channel::from_index(i).unwrap())
}
fn events(io: &impl Registers) -> Events {
    let (group_errors, count_errors) = io.errors();
    let configuration_loaded = channels(255).fold(0, |m, ch| m | if io.period_flag(ch) { 1 << ch.index() } else { 0 });
    Events { group_errors, count_errors, configuration_loaded }
}
struct Engine<I, D> {
    io: I,
    delay: D,
    groups: [Option<GroupConfig>; 4],
    running: [bool; 4],
}
impl<I: Registers, D: DelayNs> Engine<I, D> {
    fn new(io: I, delay: D) -> Self {
        Self { io, delay, groups: [None; 4], running: [false; 4] }
    }
    fn initialize(&mut self) {
        self.io.clock(true);
        self.io.event_enable(EventEnable::Disabled);
        self.io.disable_irqs();
        self.io.dma_off();
        for ch in channels(255) {
            self.io.enable(ch, false);
        }
        for i in 0..4 {
            self.io.select(Group::from_index(i).unwrap(), 0);
        }
        self.io.ack(Events { group_errors: u16::MAX, count_errors: u16::MAX, configuration_loaded: 255 });
    }
    fn set_clock_divider(&mut self, divider: ClockDivider) -> Result<(), Error> {
        if self.running.iter().any(|running| *running) {
            return Err(Error::GroupRunning);
        }
        self.io.clock_divider(divider);
        Ok(())
    }
    fn group_for(&self, ch: Channel) -> Option<Group> {
        self.groups
            .iter()
            .position(|g| g.is_some_and(|g| g.mask() & (1 << ch.index()) != 0))
            .map(|i| Group::from_index(i as u8).unwrap())
    }
    fn configure(&mut self, g: Group, config: GroupConfig, routes: u8) -> Result<(), Error> {
        let i = g.index() as usize;
        let mask = config.mask();
        if self.running[i] {
            return Err(Error::GroupRunning);
        }
        if mask & !routes != 0 {
            return Err(Error::MissingRoute);
        }
        for (j, other) in self.groups.iter().enumerate() {
            if j != i && other.is_some_and(|c| c.mask() & mask != 0) {
                return Err(Error::ChannelInUse);
            }
        }
        for ch in channels(mask) {
            self.io.enable(ch, false);
            self.io.configure(ch, config.channels[ch.index() as usize].unwrap());
        }
        self.io.select(g, mask);
        self.groups[i] = Some(config);
        Ok(())
    }
    fn start(&mut self, g: Group) -> Result<(), Error> {
        let i = g.index() as usize;
        if self.running[i] {
            return Err(Error::GroupRunning);
        }
        let config = self.groups[i].ok_or(Error::Unconfigured)?;
        let mask = config.mask();
        self.io.ack(Events { group_errors: mask as u16, count_errors: mask as u16, configuration_loaded: mask });
        for ch in channels(mask) {
            self.io.enable(ch, true);
        }
        self.io.commit(g, true);
        let finite = channels(mask).fold(0u8, |bits, ch| {
            bits | if config.channels[ch.index() as usize].unwrap().mode.finite() { 1 << ch.index() } else { 0 }
        });
        if finite != 0 {
            // START acceptance is asynchronous on the tested A-board. The SDK
            // clears EN immediately; at the measured 2MHz counter that erased
            // the initial enabled configuration before any pulse appeared.
            // A 10us independent guard passed 8/16-pulse physical comparisons.
            // This is a bounded profile guard, not a completion/status poll.
            self.delay.delay_us(10);
            // Initial configuration acceptance itself asserts the load flag.
            // Acknowledge only this group's finite channels before observing
            // deferred-load timing; it is not proof that a block has ended.
            self.io.ack(Events { configuration_loaded: finite, ..Events::default() });
            for ch in channels(finite) {
                self.io.enable(ch, false);
            }
            self.delay.delay_us(1);
            self.io.commit(g, false);
            // Own PWM IRQs are suspended by the public transaction wrapper,
            // so an ISR cannot consume these sticky flags before this check.
            // Do not report success if stop staging may already have been late.
            if channels(finite).any(|ch| self.io.period_flag(ch)) {
                self.stop(g);
                return Err(Error::FiniteDeadlineMissed);
            }
        }
        self.running[i] = true;
        Ok(())
    }
    fn stop(&mut self, g: Group) {
        let i = g.index() as usize;
        if let Some(config) = self.groups[i] {
            let mask = config.mask();
            for ch in channels(mask) {
                self.io.enable(ch, false);
            }
            self.io.commit(g, true);
            self.delay.delay_us(1);
            self.io.ack(Events { group_errors: mask as u16, count_errors: mask as u16, configuration_loaded: mask });
        }
        self.running[i] = false;
    }
    fn set_duty(&mut self, g: Group, ch: Channel, high: u16) -> Result<(), Error> {
        let i = g.index() as usize;
        let config = self.groups[i].as_mut().ok_or(Error::Unconfigured)?;
        let wave = config.channels[ch.index() as usize].ok_or(Error::Unconfigured)?.with_duty(high)?;
        self.io.configure(ch, wave);
        self.io.commit(g, true);
        config.channels[ch.index() as usize] = Some(wave);
        Ok(())
    }
    fn shutdown(&mut self) {
        self.io.event_enable(EventEnable::Disabled);
        self.io.disable_irqs();
        self.io.dma_off();
        for i in 0..4 {
            self.stop(Group::from_index(i).unwrap());
        }
        self.io.ack(Events { group_errors: u16::MAX, count_errors: u16::MAX, configuration_loaded: 255 });
        self.io.clock(false);
    }
}

struct PacRegisters;
enum ChannelOp {
    Enable(bool),
    Configure(Waveform),
    Flag,
}
enum GroupOp {
    Select(u8),
    Commit(bool),
}
fn regs() -> &'static pac::pwm::RegisterBlock {
    unsafe { &*Pwm::ptr() }
}
macro_rules! channel_registers {
    ($($ch:ident: $en:ident $enf:ident, $pol:ident $polf:ident, $oe:ident $oef:ident,
        $off:ident $offf:ident, $freq:ident $freqf:ident, $duty:ident $dutyf:ident,
        $cycles:ident $cyclesf:ident, $flag:ident $flagf:ident;)+) => {
        fn channel_op(ch:Channel,op:ChannelOp)->bool {
            let r=regs();
            match ch {$(Channel::$ch=>match op {
                ChannelOp::Enable(on)=>{r.$en().modify(|_,w|w.$enf().bit(on));false},
                ChannelOp::Configure(config)=>{
                    r.$pol().modify(|_,w|w.$polf().clear_bit());
                    r.$oe().modify(|_,w|w.$oef().clear_bit());
                    r.$off().modify(|_,w|unsafe {w.$offf().bits(config.offset())});
                    r.$freq().modify(|_,w|unsafe {w.$freqf().bits(config.period())});
                    r.$duty().modify(|_,w|unsafe {w.$dutyf().bits(config.high())});
                    r.$cycles().modify(|_,w|unsafe {w.$cyclesf().bits(config.mode.cycles())});false
                },
                ChannelOp::Flag=>r.$flag().read().$flagf().bit_is_set(),
            },)+}
        }
    };
}
channel_registers! {
    C0: pwm_en0 pwm_en_0, pwm_portity0 pwm_poarity_0, pwm_oen_cfg0 pwm_oen_cfg_0, pwm_offset_l0 pwm_offset_l_0, pwm_freq_l0 pwm_freq_l_0, pwm_duty_l0 pwm_duty_l_0, pwm_period_val0 pwm_period_val_0, pwm_periodload_flag0 pwm_periodload_flag_0;
    C1: pwm_en1 pwm_en_1, pwm_portity1 pwm_poarity_1, pwm_oen_cfg1 pwm_oen_cfg_1, pwm_offset_l1 pwm_offset_l_1, pwm_freq_l1 pwm_freq_l_1, pwm_duty_l1 pwm_duty_l_1, pwm_period_val1 pwm_period_val_1, pwm_periodload_flag1 pwm_periodload_flag_1;
    C2: pwm_en2 pwm_en_2, pwm_portity2 pwm_poarity_2, pwm_oen_cfg2 pwm_oen_cfg_2, pwm_offset_l2 pwm_offset_l_2, pwm_freq_l2 pwm_freq_l_2, pwm_duty_l2 pwm_duty_l_2, pwm_period_val2 pwm_period_val_2, pwm_periodload_flag2 pwm_periodload_flag_2;
    C3: pwm_en3 pwm_en_3, pwm_portity3 pwm_poarity_3, pwm_oen_cfg3 pwm_oen_cfg_3, pwm_offset_l3 pwm_offset_l_3, pwm_freq_l3 pwm_freq_l_3, pwm_duty_l3 pwm_duty_l_3, pwm_period_val3 pwm_period_val_3, pwm_periodload_flag3 pwm_periodload_flag_3;
    C4: pwm_en4 pwm_en_4, pwm_portity4 pwm_poarity_4, pwm_oen_cfg4 pwm_oen_cfg_4, pwm_offset_l4 pwm_offset_l_4, pwm_freq_l4 pwm_freq_l_4, pwm_duty_l4 pwm_duty_l_4, pwm_period_val4 pwm_period_val_4, pwm_periodload_flag4 pwm_periodload_flag_4;
    C5: pwm_en5 pwm_en_5, pwm_portity5 pwm_poarity_5, pwm_oen_cfg5 pwm_oen_cfg_5, pwm_offset_l5 pwm_offset_l_5, pwm_freq_l5 pwm_freq_l_5, pwm_duty_l5 pwm_duty_l_5, pwm_period_val5 pwm_period_val_5, pwm_periodload_flag5 pwm_periodload_flag_5;
    C6: pwm_en6 pwm_en_6, pwm_portity6 pwm_poarity_6, pwm_oen_cfg6 pwm_oen_cfg_6, pwm_offset_l6 pwm_offset_l_6, pwm_freq_l6 pwm_freq_l_6, pwm_duty_l6 pwm_duty_l_6, pwm_period_val6 pwm_period_val_6, pwm_periodload_flag6 pwm_periodload_flag_6;
    C7: pwm_en7 pwm_en_7, pwm_portity7 pwm_poarity_7, pwm_oen_cfg7 pwm_oen_cfg_7, pwm_offset_l7 pwm_offset_l_7, pwm_freq_l7 pwm_freq_l_7, pwm_duty_l7 pwm_duty_l_7, pwm_period_val7 pwm_period_val_7, pwm_periodload_flag7 pwm_periodload_flag_7;
}
macro_rules! group_registers {
    ($($g:ident: $sel:ident $self:ident, $clear:ident $clearf:ident, $start:ident $startf:ident;)+)=>{
        fn group_op(g:Group,op:GroupOp) {
            let r=regs();
            match g {$(Group::$g=>match op {
                GroupOp::Select(mask)=>{r.$sel().modify(|_,w|unsafe {w.$self().bits(mask as u16)});},
                GroupOp::Commit(clear)=>{
                    r.$clear().modify(|_,w|w.$clearf().bit(clear));
                    // Complete command, not the original SDK command-word RMW.
                    unsafe {r.$start().write_with_zero(|w|w.$startf().set_bit());}
                }
            },)+}
        }
    };
}
group_registers! {
    G0: pwm_sel0 pwm_sel_0, pwm_startclrcnt_en0 pwm_startclrcnt_en_0, pwm_start0 pwm_start_0;
    G1: pwm_sel1 pwm_sel_1, pwm_startclrcnt_en1 pwm_startclrcnt_en_1, pwm_start1 pwm_start_1;
    G2: pwm_sel2 pwm_sel_2, pwm_startclrcnt_en2 pwm_startclrcnt_en_2, pwm_start2 pwm_start_2;
    G3: pwm_sel3 pwm_sel_3, pwm_startclrcnt_en3 pwm_startclrcnt_en_3, pwm_start3 pwm_start_3;
}
impl Registers for PacRegisters {
    fn clock_divider(&mut self, divider: ClockDivider) {
        let r = unsafe { &*pac::MCrg::PTR };
        r.pwm_m_div3().modify(|_, w| w.load().clear_bit());
        r.pwm_m_div3().modify(|_, w| unsafe { w.divider().bits(divider.divisor()) });
        r.pwm_m_div3().modify(|_, w| w.load().set_bit());
    }
    fn clock(&mut self, on: bool) {
        let r = unsafe { &*pac::MCrg::PTR };
        if on {
            r.m_clken0().modify(|_, w| w.pwm_enable().set_bit());
            r.pwm_m_div3().modify(|_, w| w.enable().set_bit());
            self.clock_divider(ClockDivider::Div16);
        } else {
            r.m_clken0().modify(|_, w| w.pwm_enable().clear_bit());
            // Preserve unmodeled shared bits instead of the SDK whole-word zero.
            r.pwm_m_div3().modify(|_, w| w.enable().clear_bit().load().clear_bit());
        }
    }
    fn disable_irqs(&mut self) {
        unsafe {
            interrupt::disable(Interrupt::PWM_ABNOR);
            interrupt::disable(Interrupt::PWM_CFG);
        }
    }
    fn event_enable(&mut self, enable: EventEnable) {
        regs().pwm_int_mask().modify(|_, w| match enable {
            EventEnable::Disabled => w.pwm_int_mask().disabled(),
            EventEnable::Abnormal => w.pwm_int_mask().abnormal_only(),
            EventEnable::ConfigurationLoaded => w.pwm_int_mask().configuration_loaded_only(),
            EventEnable::All => w.pwm_int_mask().all_events(),
        });
    }
    fn dma_off(&mut self) {
        regs().pwm_dma_en().modify(|_, w| w.pwm_dma_en().clear_bit());
    }
    fn select(&mut self, g: Group, mask: u8) {
        group_op(g, GroupOp::Select(mask));
    }
    fn enable(&mut self, ch: Channel, on: bool) {
        channel_op(ch, ChannelOp::Enable(on));
    }
    fn configure(&mut self, ch: Channel, w: Waveform) {
        channel_op(ch, ChannelOp::Configure(w));
    }
    fn commit(&mut self, g: Group, clear: bool) {
        group_op(g, GroupOp::Commit(clear));
    }
    fn errors(&self) -> (u16, u16) {
        (
            regs().pwm_abnor_state0().read().pwm_abnor_state0().bits(),
            regs().pwm_abnor_state1().read().pwm_abnor_state1().bits(),
        )
    }
    fn period_flag(&self, ch: Channel) -> bool {
        channel_op(ch, ChannelOp::Flag)
    }
    fn ack(&mut self, e: Events) {
        unsafe {
            if e.group_errors != 0 {
                regs().pwm_abnor_state_clr0().write_with_zero(|w| w.pwm_abnor_state_clr0().bits(e.group_errors));
            }
            if e.count_errors != 0 {
                regs().pwm_abnor_state_clr1().write_with_zero(|w| w.pwm_abnor_state_clr1().bits(e.count_errors));
            }
            if e.configuration_loaded != 0 {
                regs().pwm_cfg_int_clr0().write_with_zero(|w| w.pwm_cfg_int_clr0().bits(e.configuration_loaded as u16));
            }
        }
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "pwm_ws53/tests.rs"]
mod tests;
