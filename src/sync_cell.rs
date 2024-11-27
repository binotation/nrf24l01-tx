use crate::State;
use core::cell::UnsafeCell;
use cortex_m::Peripherals as CorePeripherals;
use static_fifo_queue::Queue;
use stm32l4::stm32l412::Peripherals as DevicePeripherals;

pub struct SyncPeripheral<P>(UnsafeCell<Option<P>>);

impl<P> SyncPeripheral<P> {
    pub const fn new() -> Self {
        SyncPeripheral(UnsafeCell::new(None))
    }

    pub fn set(&self, inner: P) {
        unsafe { *self.0.get() = Some(inner) };
    }

    #[allow(clippy::mut_from_ref)]
    pub const fn get(&self) -> &mut P {
        unsafe { &mut *self.0.get() }.as_mut().unwrap()
    }
}

// SAFETY: CPU is single-threaded. Interrupts cannot execute simultaneously and cannot
// preempt each other (all interrupts have same priority).
unsafe impl Sync for SyncPeripheral<CorePeripherals> {}
unsafe impl Sync for SyncPeripheral<DevicePeripherals> {}

impl<P> Default for SyncPeripheral<P> {
    fn default() -> Self {
        Self::new()
    }
}

pub struct SyncQueue<T, const N: usize>(UnsafeCell<Queue<T, N>>);

impl<T, const N: usize> SyncQueue<T, N> {
    pub const fn new() -> Self {
        Self(UnsafeCell::new(Queue::new()))
    }

    #[allow(clippy::mut_from_ref)]
    pub const fn get(&self) -> &mut Queue<T, N> {
        unsafe { &mut *self.0.get() }
    }
}
unsafe impl Sync for SyncQueue<&[u8], 16> {}

impl<T, const N: usize> Default for SyncQueue<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

pub struct SyncState(UnsafeCell<State>);

impl SyncState {
    pub const fn new() -> Self {
        Self(UnsafeCell::new(State::HandshakePulse))
    }

    #[allow(clippy::mut_from_ref)]
    pub const fn get(&self) -> &mut State {
        unsafe { &mut *self.0.get() }
    }
}
unsafe impl Sync for SyncState {}

impl Default for SyncState {
    fn default() -> Self {
        Self::new()
    }
}
