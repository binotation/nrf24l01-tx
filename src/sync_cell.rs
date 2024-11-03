use crate::state::State;
use core::cell::UnsafeCell;
use cortex_m::Peripherals;
use heapless::spsc::Queue;
use stm32l4::stm32l4x2::{DMA1, EXTI, GPIOA, PWR, SPI1, TIM2, USART2};

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
unsafe impl Sync for SyncPeripheral<GPIOA> {}
unsafe impl Sync for SyncPeripheral<PWR> {}
unsafe impl Sync for SyncPeripheral<USART2> {}
unsafe impl Sync for SyncPeripheral<SPI1> {}
unsafe impl Sync for SyncPeripheral<DMA1> {}
unsafe impl Sync for SyncPeripheral<EXTI> {}
unsafe impl Sync for SyncPeripheral<TIM2> {}
unsafe impl Sync for SyncPeripheral<Peripherals> {}

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

pub struct SyncState(UnsafeCell<State>);

impl SyncState {
    pub const fn new() -> Self {
        Self(UnsafeCell::new(State::Init))
    }

    #[allow(clippy::mut_from_ref)]
    pub const fn get(&self) -> &mut State {
        unsafe { &mut *self.0.get() }
    }
}
unsafe impl Sync for SyncState {}

pub struct SyncNum<T>(UnsafeCell<T>);

impl SyncNum<u8> {
    pub const fn new() -> Self {
        Self(UnsafeCell::new(0))
    }

    #[allow(clippy::mut_from_ref)]
    pub const fn get(&self) -> &mut u8 {
        unsafe { &mut *self.0.get() }
    }
}
unsafe impl Sync for SyncNum<u8> {}
