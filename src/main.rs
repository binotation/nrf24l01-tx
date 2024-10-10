//! Receive a byte over USART2 and send it over SPI1.

#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use cortex_m_rt::entry;
use heapless::{spsc::Queue, Vec};
use nrf24l01_commands::{commands, commands::Command, registers};
use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use stm32l4::stm32l4x2::{interrupt, Interrupt, Peripherals, GPIOA, SPI1, TIM2, USART2};

const TX_ADDR: u64 = 0xA2891FFF6A;
const PAYLOAD: [u8; 32] = [
    b't', b'h', b'e', b' ', b'l', b'a', b'z', b'y', b' ', b'f', b'o', b'x', b' ', b'j', b'u', b'm',
    b'p', b'e', b'd', b' ', b'o', b'v', b'e', b'r', b' ', b't', b'h', b'e', b' ', b'b', b'r', b'o',
];

struct SyncUnsafeCell<P>(UnsafeCell<Option<P>>);

impl<P> SyncUnsafeCell<P> {
    const fn new() -> Self {
        SyncUnsafeCell(UnsafeCell::new(None))
    }

    fn set(&self, inner: P) {
        unsafe { *self.0.get() = Some(inner) };
    }

    fn get(&self) -> &mut P {
        unsafe { &mut *self.0.get() }.as_mut().unwrap()
    }
}

// SAFETY: CPU is single-threaded. Interrupts cannot execute simultaneously and cannot
// preempt each other (all interrupts have same priority).
unsafe impl Sync for SyncUnsafeCell<GPIOA> {}
unsafe impl Sync for SyncUnsafeCell<USART2> {}
unsafe impl Sync for SyncUnsafeCell<SPI1> {}
unsafe impl Sync for SyncUnsafeCell<TIM2> {}
unsafe impl Sync for SyncUnsafeCell<Queue<u8, 64>> {}
unsafe impl Sync for SyncUnsafeCell<Queue<u16, 64>> {}
unsafe impl Sync for SyncUnsafeCell<Queue<Vec<u8, 64>, 16>> {}

static GPIOA_PERIPHERAL: SyncUnsafeCell<GPIOA> = SyncUnsafeCell::new();
static USART2_PERIPHERAL: SyncUnsafeCell<USART2> = SyncUnsafeCell::new();
static SPI1_PERIPHERAL: SyncUnsafeCell<SPI1> = SyncUnsafeCell::new();
static TIM2_PERIPHERAL: SyncUnsafeCell<TIM2> = SyncUnsafeCell::new();
/// Bytes to be transmitted over SPI1
static TX_BUFFER: SyncUnsafeCell<Queue<u8, 64>> = SyncUnsafeCell::new();
/// Bytes received over SPI1
static RX_BUFFER: SyncUnsafeCell<Queue<u16, 64>> = SyncUnsafeCell::new();
/// Queued nRF24l01 commands
static COMMANDS: SyncUnsafeCell<Queue<Vec<u8, 64>, 16>> = SyncUnsafeCell::new();

#[inline(always)]
fn enqueue_command(queue: &mut Queue<Vec<u8, 64>, 16>, command_bytes: &[u8]) {
    let mut command = Vec::new();
    let _ = command.extend_from_slice(command_bytes);
    let _ = queue.enqueue(command);
}

#[interrupt]
fn USART2() {
    let gpioa = GPIOA_PERIPHERAL.get();
    let usart2 = USART2_PERIPHERAL.get();
    let spi1 = SPI1_PERIPHERAL.get();
    let tim2 = TIM2_PERIPHERAL.get();
    let rx_buffer = RX_BUFFER.get();
    let command_queue = COMMANDS.get();

    // Dequeue bytes off rx buffer and transmit over USART2
    if usart2.isr().read().txe().bit_is_set() {
        match rx_buffer.dequeue() {
            Some(byte) => {
                usart2.tdr().write(|w| unsafe { w.tdr().bits(byte) });
                if rx_buffer.is_empty() {
                    usart2.cr1().modify(|_, w| w.txeie().disabled());
                }
            }
            None => usart2.cr1().modify(|_, w| w.txeie().disabled()),
        }
    }

    // Read incoming bytes from USART2 and queue onto tx buffer
    if usart2.isr().read().rxne().bit_is_set() {
        // Read data, this clears RXNE
        let received_byte = usart2.rdr().read().rdr().bits();

        match received_byte {
            97 => {
                // a
                // NOP
                let mut command: Vec<u8, 64> = Vec::new();
                let _ = command.push(commands::Nop::WORD);
                let _ = command_queue.enqueue(command);
            }
            98 => {
                // b
                // Read RF_CH
                enqueue_command(
                    command_queue,
                    &commands::ReadRegister::<registers::RfCh>::bytes(),
                );
            }
            99 => {
                // c
                // Read TX addr
                enqueue_command(
                    command_queue,
                    &commands::ReadRegister::<registers::TxAddr>::bytes(),
                );
            }
            100 => {
                // d
                // Read RX Addr P0
                enqueue_command(
                    command_queue,
                    &commands::ReadRegister::<registers::RxAddrP0>::bytes(),
                );
            }
            101 => {
                // e
                // Read feature register
                enqueue_command(
                    command_queue,
                    &commands::ReadRegister::<registers::Feature>::bytes(),
                );
            }
            102 => {
                // f
                // Read Config
                enqueue_command(
                    command_queue,
                    &commands::ReadRegister::<registers::Config>::bytes(),
                );
            }
            103 => {
                // g
                // Write payload
                enqueue_command(
                    command_queue,
                    &commands::WriteTxPayloadNoAck(PAYLOAD).bytes(),
                );
            }
            104 => {
                // h
                // pulse CE
                gpioa.bsrr().write(|w| w.bs0().set_bit());
                // Enable counter, one-pulse mode
                tim2.cr1().write(|w| w.opm().enabled().cen().enabled());
            }
            105 => {
                // i
                // Clear TX_DS flag
                enqueue_command(
                    command_queue,
                    &commands::WriteRegister(registers::Status::new().with_tx_ds(true)).bytes(),
                );
            }
            _ => (),
        }

        spi1.cr2().modify(|_, w| w.txeie().set_bit());
        spi1.cr1().modify(|_, w| w.spe().enabled());
    }
    if usart2.isr().read().ore().bit_is_set() {
        usart2.icr().write(|w| w.orecf().set_bit());
    }
}

#[interrupt]
fn SPI1() {
    let spi1 = SPI1_PERIPHERAL.get();
    let usart2 = USART2_PERIPHERAL.get();
    let tx_buffer = TX_BUFFER.get();
    let rx_buffer = RX_BUFFER.get();
    let command_queue = COMMANDS.get();

    // Transmit bytes from tx buffer
    if spi1.sr().read().txe().bit_is_set() {
        match tx_buffer.dequeue() {
            Some(byte) => {
                spi1.dr8().write(|w| unsafe { w.dr().bits(byte) });

                if tx_buffer.is_empty() {
                    // Buffer empty -> pull NSS high
                    while spi1.sr().read().bsy().bit_is_set() {}
                    spi1.cr1().modify(|_, w| w.spe().disabled());
                    // Queue bytes of next command
                    if let Some(next_command) = command_queue.dequeue() {
                        for &byte in next_command.iter() {
                            let _ = tx_buffer.enqueue(byte);
                        }
                        // Buffer non-empty -> Pull NSS low
                        spi1.cr1().modify(|_, w| w.spe().enabled());
                    } else {
                        spi1.cr2().modify(|_, w| w.txeie().clear_bit());
                    }
                }
            }
            None => {
                // Initial commands
                if let Some(next_command) = command_queue.dequeue() {
                    spi1.dr8()
                        .write(|w| unsafe { w.dr().bits(next_command[0]) });
                    for &byte in next_command[1..].iter() {
                        let _ = tx_buffer.enqueue(byte);
                    }
                } else {
                    spi1.cr1().modify(|_, w| w.spe().disabled());
                    spi1.cr2().modify(|_, w| w.txeie().clear_bit());
                }
            }
        }
    }

    // Read incoming bytes over SPI1 and queue onto rx buffer
    if spi1.sr().read().rxne().bit_is_set() {
        let received_byte = spi1.dr8().read().dr().bits();
        if rx_buffer.enqueue(received_byte as u16).is_ok() {
            usart2.cr1().modify(|_, w| w.txeie().enabled());
        }
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

    // Enable peripheral clocks - GPIOA, USART2
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
            .very_high_speed()
            .ospeedr3()
            .very_high_speed()
            .ospeedr4()
            .very_high_speed()
            .ospeedr5()
            .very_high_speed()
            .ospeedr6()
            .very_high_speed()
            .ospeedr7()
            .very_high_speed()
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

    // USART2: Configure baud rate 9600
    dp.USART2.brr().write(|w| unsafe { w.bits(417) }); // 4Mhz / 9600 approx. 417

    // SPI1: Set FIFO reception threshold to 1/4, data frame size to 8 bits, enable slave select output,
    // enable RXNE interupt
    dp.SPI1.cr2().write(|w| unsafe {
        w.frxth()
            .set_bit()
            .ds()
            .bits(7)
            .ssoe()
            .enabled()
            .rxneie()
            .set_bit()
    });
    // SPI1: set baud rate fpclk/8, SPI master
    dp.SPI1
        .cr1()
        .write(|w| unsafe { w.br().bits(2) }.mstr().set_bit());

    // Enable USART, transmitter, receiver and RXNE interrupt
    dp.USART2.cr1().write(|w| {
        w.re()
            .set_bit()
            .te()
            .set_bit()
            .ue()
            .set_bit()
            .rxneie()
            .set_bit()
    });

    // Set 11us interval
    dp.TIM2.arr().write(|w| unsafe { w.arr().bits(44) }); // 4MHz / 44 = 11us

    // Enable TIM2 update interrupt
    dp.TIM2.dier().write(|w| w.uie().set_bit());

    // Initial commands
    let mut command_queue = Queue::new();

    enqueue_command(
        &mut command_queue,
        &commands::WriteRegister(registers::RfCh::new().with_rf_ch(110)).bytes(),
    );
    enqueue_command(
        &mut command_queue,
        &commands::ReadRegister::<registers::RfCh>::bytes(),
    );
    enqueue_command(
        &mut command_queue,
        &commands::WriteRegister(registers::TxAddr::new().with_tx_addr(TX_ADDR)).bytes(),
    );
    enqueue_command(
        &mut command_queue,
        &commands::ReadRegister::<registers::TxAddr>::bytes(),
    );
    enqueue_command(
        &mut command_queue,
        &commands::WriteRegister(registers::RxAddrP0::new().with_rx_addr_p0(TX_ADDR)).bytes(),
    );
    enqueue_command(
        &mut command_queue,
        &commands::ReadRegister::<registers::RxAddrP0>::bytes(),
    );
    enqueue_command(&mut command_queue, &commands::Activate::bytes());
    enqueue_command(
        &mut command_queue,
        &commands::WriteRegister(registers::Feature::new().with_en_dyn_ack(true)).bytes(),
    );
    enqueue_command(
        &mut command_queue,
        &commands::ReadRegister::<registers::Feature>::bytes(),
    );
    enqueue_command(
        &mut command_queue,
        &commands::WriteRegister(
            registers::Config::new()
                .with_pwr_up(true)
                .with_mask_max_rt(true)
                .with_mask_rx_dr(true),
        )
        .bytes(),
    );
    enqueue_command(
        &mut command_queue,
        &commands::ReadRegister::<registers::Config>::bytes(),
    );

    COMMANDS.set(command_queue);
    TX_BUFFER.set(Queue::new());
    RX_BUFFER.set(Queue::new());
    unsafe {
        // Unmask NVIC global interrupts
        cortex_m::peripheral::NVIC::unmask(Interrupt::SPI1);
        cortex_m::peripheral::NVIC::unmask(Interrupt::USART2);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2)
    }
    GPIOA_PERIPHERAL.set(dp.GPIOA);
    SPI1_PERIPHERAL.set(dp.SPI1);
    USART2_PERIPHERAL.set(dp.USART2);
    TIM2_PERIPHERAL.set(dp.TIM2);

    // Enable SPI to send initial commands
    let spi1 = SPI1_PERIPHERAL.get();
    spi1.cr2().modify(|_, w| w.txeie().set_bit());
    spi1.cr1().modify(|_, w| w.spe().enabled());

    #[allow(clippy::empty_loop)]
    loop {}
}
