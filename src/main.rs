#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use cortex_m::asm;
use cortex_m_rt::entry;
// use cortex_m_semihosting::hprintln;
use heapless::spsc::Queue;
use nrf24l01_commands::{commands, registers};
use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use stm32l4::stm32l4x2::{interrupt, Interrupt, Peripherals, DMA1, GPIOA, SPI1, TIM2, USART2};

const USART2_RDR: u32 = 0x4000_4424;
const SPI1_DR: u32 = 0x4001_300C;
const TX_ADDR: u64 = 0xA2891FFF6A;

// Commands
const NOP: [u8; 1] = commands::Nop::bytes();
const W_RF_CH: [u8; 2] = commands::WRegister(registers::RfCh::new().with_rf_ch(110)).bytes();
const W_RF_SETUP: [u8; 2] =
    commands::WRegister(registers::RfSetup::new().with_rf_dr(false)).bytes();
const W_TX_ADDR: [u8; 6] =
    commands::WRegister(registers::TxAddr::<5>::new().with_tx_addr(TX_ADDR)).bytes();
const W_RX_ADDR_P0: [u8; 6] =
    commands::WRegister(registers::RxAddrP0::<5>::new().with_rx_addr_p0(TX_ADDR)).bytes();
const ACTIVATE: [u8; 2] = commands::Activate::bytes();
const W_FEATURE: [u8; 2] =
    commands::WRegister(registers::Feature::new().with_en_dyn_ack(true)).bytes();
const W_CONFIG: [u8; 2] = commands::WRegister(
    registers::Config::new()
        .with_pwr_up(true)
        .with_mask_rx_dr(true),
)
.bytes();
const RESET_TX_DS: [u8; 2] = commands::WRegister(registers::Status::new().with_tx_ds(true)).bytes();

static mut W_TX_PL_NOACK: [u8; 33] = commands::WTxPayloadNoack([0; 32]).bytes();
static mut SPI1_RX_BUFFER: [u8; 33] = [0; 33];
static mut INIT_COMPLETE: bool = false;

struct SyncPeripheral<P>(UnsafeCell<Option<P>>);

impl<P> SyncPeripheral<P> {
    const fn new() -> Self {
        SyncPeripheral(UnsafeCell::new(None))
    }

    fn set(&self, inner: P) {
        unsafe { *self.0.get() = Some(inner) };
    }

    #[allow(clippy::mut_from_ref)]
    const fn get(&self) -> &mut P {
        unsafe { &mut *self.0.get() }.as_mut().unwrap()
    }
}

// SAFETY: CPU is single-threaded. Interrupts cannot execute simultaneously and cannot
// preempt each other (all interrupts have same priority).
unsafe impl Sync for SyncPeripheral<GPIOA> {}
unsafe impl Sync for SyncPeripheral<USART2> {}
unsafe impl Sync for SyncPeripheral<SPI1> {}
unsafe impl Sync for SyncPeripheral<DMA1> {}
unsafe impl Sync for SyncPeripheral<TIM2> {}

static GPIOA_PERIPHERAL: SyncPeripheral<GPIOA> = SyncPeripheral::new();
static USART2_PERIPHERAL: SyncPeripheral<USART2> = SyncPeripheral::new();
static SPI1_PERIPHERAL: SyncPeripheral<SPI1> = SyncPeripheral::new();
static DMA1_PERIPHERAL: SyncPeripheral<DMA1> = SyncPeripheral::new();
static TIM2_PERIPHERAL: SyncPeripheral<TIM2> = SyncPeripheral::new();

struct SyncQueue<T, const N: usize>(UnsafeCell<Queue<T, N>>);

impl<T, const N: usize> SyncQueue<T, N> {
    const fn new() -> Self {
        Self(UnsafeCell::new(Queue::new()))
    }

    #[allow(clippy::mut_from_ref)]
    const fn get(&self) -> &mut Queue<T, N> {
        unsafe { &mut *self.0.get() }
    }
}
unsafe impl Sync for SyncQueue<&[u8], 16> {}

static COMMANDS: SyncQueue<&[u8], 16> = SyncQueue::new();

#[inline]
fn send_command(command: &[u8], dma1: &mut DMA1, spi1: &mut SPI1) {
    // Write memory address for SPI1 TX
    dma1.ch3()
        .mar()
        .write(|w| unsafe { w.bits(command.as_ptr() as u32) });
    let transfer_size = command.len() as u32;
    // Set DMA transfer size for SPI1 RX, TX
    dma1.ch2()
        .ndtr()
        .write(|w| unsafe { w.bits(transfer_size) });
    dma1.ch3()
        .ndtr()
        .write(|w| unsafe { w.bits(transfer_size) });

    // Enable DMA for SPI1 RX, TX
    dma1.ch2().cr().modify(|_, w| w.en().set_bit());
    dma1.ch3().cr().modify(|_, w| w.en().set_bit());
    // Enable SPI1
    spi1.cr1().modify(|_, w| w.spe().enabled());
}

/// USART2 RX DMA
#[interrupt]
fn DMA1_CH6() {
    let dma1 = DMA1_PERIPHERAL.get();
    let spi1 = SPI1_PERIPHERAL.get();

    if dma1.isr().read().tcif6().bit_is_set() {
        dma1.ch6().cr().modify(|_, w| w.en().clear_bit());
        dma1.ifcr().write(|w| w.ctcif6().set_bit());

        #[allow(static_mut_refs)]
        send_command(unsafe { &W_TX_PL_NOACK }, dma1, spi1);
    }
}

/// SPI1 RX DMA
#[interrupt]
fn DMA1_CH2() {
    #[allow(static_mut_refs)]
    let init_complete = unsafe { &mut INIT_COMPLETE };

    let dma1 = DMA1_PERIPHERAL.get();
    let spi1 = SPI1_PERIPHERAL.get();
    let gpioa = GPIOA_PERIPHERAL.get();
    let tim2 = TIM2_PERIPHERAL.get();
    let commands = COMMANDS.get();

    if dma1.isr().read().tcif2().bit_is_set() {
        dma1.ch2().cr().modify(|_, w| w.en().clear_bit());
        dma1.ifcr().write(|w| w.ctcif2().set_bit());

        // Disable SPI1
        spi1.cr1().modify(|_, w| w.spe().clear_bit());

        // Pulse CE if payload written
        #[allow(static_mut_refs)]
        if dma1.ch3().mar().read() == unsafe { W_TX_PL_NOACK.as_ptr() } as u32 {
            gpioa.bsrr().write(|w| w.bs0().set_bit());
            // Enable counter, one-pulse mode
            tim2.cr1().write(|w| w.opm().enabled().cen().enabled());
            // Read next payload
            dma1.ch6().ndtr().write(|w| unsafe { w.bits(32) });
            dma1.ch6().cr().modify(|_, w| w.en().set_bit());
        }

        // Send next command
        if let Some(command) = commands.dequeue() {
            send_command(command, dma1, spi1);
            if !*init_complete && commands.is_empty() {
                *init_complete = true;
                dma1.ch6().ndtr().write(|w| unsafe { w.bits(32) });
                dma1.ch6().cr().modify(|_, w| w.en().set_bit());
            }
        }
    }
}

/// SPI1 TX DMA
#[interrupt]
fn DMA1_CH3() {
    let dma1 = DMA1_PERIPHERAL.get();
    if dma1.isr().read().tcif3().bit_is_set() {
        dma1.ch3().cr().modify(|_, w| w.en().clear_bit());
        dma1.ifcr().write(|w| w.ctcif3().set_bit());
    }
}

#[interrupt]
fn TIM2() {
    let tim2 = TIM2_PERIPHERAL.get();
    let gpioa = GPIOA_PERIPHERAL.get();

    if tim2.sr().read().uif().bit_is_set() {
        gpioa.bsrr().write(|w| w.br0().set_bit());
        tim2.sr().write(|w| w.uif().clear_bit())
    }
}

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let dp = Peripherals::take().unwrap();

    // Enable peripheral clocks: DMA1, GPIOA, USART2, TIM2, SPI1
    dp.RCC.ahb1enr().write(|w| w.dma1en().set_bit());
    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit());
    dp.RCC
        .apb1enr1()
        .write(|w| w.usart2en().enabled().tim2en().set_bit());
    dp.RCC.apb2enr().write(|w| w.spi1en().set_bit());

    // USART2: A2 (TX), A3 (RX) as AF 7
    // SPI1: A4 (NSS), A5 (SCK), A6 (MISO), A7 (MOSI) as AF 5
    // GPIO: A1 (IRQ), A0 (CE)
    dp.GPIOA.moder().write(|w| {
        w.moder0()
            .output()
            .moder1()
            .input()
            .moder2()
            .alternate()
            .moder3()
            .alternate()
            .moder4()
            .alternate()
            .moder5()
            .alternate()
            .moder6()
            .alternate()
            .moder7()
            .alternate()
    });
    dp.GPIOA.otyper().write(|w| w.ot0().push_pull());
    // NSS, IRQ are active low
    dp.GPIOA
        .pupdr()
        .write(|w| w.pupdr1().pull_up().pupdr4().pull_up());
    dp.GPIOA.ospeedr().write(|w| {
        w.ospeedr2()
            .low_speed()
            .ospeedr3()
            .low_speed()
            .ospeedr4()
            .low_speed()
            .ospeedr5()
            .medium_speed()
            .ospeedr6()
            .medium_speed()
            .ospeedr7()
            .medium_speed()
    });
    dp.GPIOA.afrl().write(|w| {
        w.afrl2()
            .af7()
            .afrl3()
            .af7()
            .afrl4()
            .af5()
            .afrl5()
            .af5()
            .afrl6()
            .af5()
            .afrl7()
            .af5()
    });

    // DMA channel selection
    dp.DMA1
        .cselr()
        .write(|w| w.c2s().map1().c3s().map1().c6s().map2());

    // DMA channel 6 USART2 RX
    dp.DMA1
        .ch6()
        .par()
        .write(|w| unsafe { w.pa().bits(USART2_RDR) });
    #[allow(static_mut_refs)]
    dp.DMA1
        .ch6()
        .mar()
        .write(|w| unsafe { w.ma().bits(W_TX_PL_NOACK[1..].as_ptr() as u32) });
    dp.DMA1
        .ch6()
        .cr()
        .write(|w| w.minc().set_bit().tcie().set_bit());

    // DMA channel 2 SPI1 RX
    dp.DMA1
        .ch2()
        .par()
        .write(|w| unsafe { w.pa().bits(SPI1_DR) });
    #[allow(static_mut_refs)]
    dp.DMA1
        .ch2()
        .mar()
        .write(|w| unsafe { w.ma().bits(SPI1_RX_BUFFER.as_ptr() as u32) });
    dp.DMA1
        .ch2()
        .cr()
        .write(|w| w.minc().set_bit().tcie().set_bit());

    // DMA channel 3 SPI1 TX
    dp.DMA1
        .ch3()
        .par()
        .write(|w| unsafe { w.pa().bits(SPI1_DR) });
    dp.DMA1
        .ch3()
        .cr()
        .write(|w| w.minc().set_bit().dir().set_bit().tcie().set_bit());

    // Set 11us interval
    dp.TIM2.arr().write(|w| unsafe { w.arr().bits(44) }); // 44 / 4MHz = 11us

    // Enable TIM2 update interrupt
    dp.TIM2.dier().write(|w| w.uie().set_bit());

    // USART2: Configure baud rate 9600
    dp.USART2.brr().write(|w| unsafe { w.bits(417) }); // 4Mhz / 9600 approx. 417

    // USART2: enable DMA RX
    dp.USART2.cr3().write(|w| w.dmar().set_bit());

    // SPI1: Set FIFO reception threshold to 1/4, enable slave select output, enable DMA
    dp.SPI1.cr2().write(|w| {
        w.frxth()
            .set_bit()
            .ssoe()
            .enabled()
            .txdmaen()
            .set_bit()
            .rxdmaen()
            .set_bit()
    });
    // SPI1: set SPI master
    dp.SPI1.cr1().write(|w| w.mstr().set_bit());

    // Enable USART, receiver
    dp.USART2.cr1().write(|w| w.re().set_bit().ue().set_bit());

    // Initialization commands
    let commands = COMMANDS.get();

    unsafe {
        commands.enqueue_unchecked(&W_RF_SETUP);
        commands.enqueue_unchecked(&W_TX_ADDR);
        commands.enqueue_unchecked(&W_RX_ADDR_P0);
        commands.enqueue_unchecked(&ACTIVATE);
        commands.enqueue_unchecked(&W_FEATURE);
        commands.enqueue_unchecked(&W_CONFIG);

        // Unmask NVIC global interrupts
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH2);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH3);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH6);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2)
    }
    GPIOA_PERIPHERAL.set(dp.GPIOA);
    SPI1_PERIPHERAL.set(dp.SPI1);
    USART2_PERIPHERAL.set(dp.USART2);
    DMA1_PERIPHERAL.set(dp.DMA1);
    TIM2_PERIPHERAL.set(dp.TIM2);

    let dma1 = DMA1_PERIPHERAL.get();
    let spi1 = SPI1_PERIPHERAL.get();

    send_command(&W_RF_CH, dma1, spi1);

    #[allow(clippy::empty_loop)]
    loop {
        asm::wfi();
    }
}
