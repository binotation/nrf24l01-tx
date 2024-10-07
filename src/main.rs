//! Receive a byte over USART2 and send it over SPI1.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use heapless::{spsc::Queue, Vec};
use nrf24l01_commands::{registers, Command};
use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use stm32l4::stm32l4x2::{interrupt, Interrupt, Peripherals, SPI1, USART2};

const TX_ADDR: u64 = 0xA2891FFF6A;
const PAYLOAD: [u8; 32] = [97; 32];

static mut USART2_PERIPHERAL: Option<USART2> = None;
static mut SPI1_PERIPHERAL: Option<SPI1> = None;
/// Bytes to be transmitted over SPI1
static mut TX_BUFFER: Option<Queue<u8, 64>> = None;
/// Bytes received over SPI1
static mut RX_BUFFER: Option<Queue<u16, 64>> = None;
/// Queued nRF24l01 commands
static mut COMMANDS: Option<Queue<Vec<u8, 64>, 16>> = None;

#[interrupt]
fn USART2() {
    // SAFETY: race condition where USART2_PERIPHERAL can be accessed before being set
    let usart2 = unsafe { USART2_PERIPHERAL.as_mut() }.unwrap();
    let spi1 = unsafe { SPI1_PERIPHERAL.as_mut() }.unwrap();
    let tx_buffer = unsafe { TX_BUFFER.as_mut() }.unwrap();
    let rx_buffer = unsafe { RX_BUFFER.as_mut() }.unwrap();
    let command_queue = unsafe { COMMANDS.as_mut() }.unwrap();

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
                let _ = command.push(Command::Nop.word());
                let _ = command_queue.enqueue(command);
            }
            98 => {
                // b
                // Read RF_CH
                let _ =
                    tx_buffer.enqueue(Command::ReadRegister(registers::Address::RfCh as u8).word());
                let _ = tx_buffer.enqueue(0);
            }
            99 => {
                // c
                // Read TX addr
                let _ = tx_buffer
                    .enqueue(Command::ReadRegister(registers::Address::TxAddr as u8).word());
                for _ in 0..5 {
                    let _ = tx_buffer.enqueue(0);
                }
            }
            100 => {
                // d
                // Read RX Addr P0
                let _ = tx_buffer
                    .enqueue(Command::ReadRegister(registers::Address::RxAddrP0 as u8).word());
                for _ in 0..5 {
                    let _ = tx_buffer.enqueue(0);
                }
            }
            101 => {
                // e
                // Read feature register
                let _ = tx_buffer
                    .enqueue(Command::ReadRegister(registers::Address::Feature as u8).word());
                let _ = tx_buffer.enqueue(0);
            }
            102 => {
                // f
                // Read Config
                let _ = tx_buffer
                    .enqueue(Command::ReadRegister(registers::Address::Config as u8).word());
                let _ = tx_buffer.enqueue(0);
            }
            103 => {
                // g
                // Write Config
                let _ = tx_buffer
                    .enqueue(Command::WriteRegister(registers::Address::Config as u8).word());
                let _ = tx_buffer.enqueue(
                    registers::Config::default()
                        .with_pwr_up(true)
                        .with_mask_max_rt(true)
                        .with_mask_rx_dr(true)
                        .into_bits(),
                );
            }
            104 => {
                // h
                // Write payload
                let _ = tx_buffer.enqueue(Command::WriteTxPayloadNoAck.word());
                for byte in PAYLOAD {
                    let _ = tx_buffer.enqueue(byte);
                }
            }
            105 => {
                // i
                // Clear TX_DS flag
                let _ = tx_buffer
                    .enqueue(Command::WriteRegister(registers::Address::Status as u8).word());
                let _ =
                    tx_buffer.enqueue(registers::Status::default().with_tx_ds(true).into_bits());
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
    let spi1 = unsafe { SPI1_PERIPHERAL.as_mut() }.unwrap();
    let usart2 = unsafe { USART2_PERIPHERAL.as_mut() }.unwrap();
    let tx_buffer = unsafe { TX_BUFFER.as_mut() }.unwrap();
    let rx_buffer = unsafe { RX_BUFFER.as_mut() }.unwrap();
    let command_queue = unsafe { COMMANDS.as_mut() }.unwrap();

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

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let dp = Peripherals::take().unwrap();

    // Enable peripheral clocks - GPIOA, USART2
    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit());
    dp.RCC.apb1enr1().write(|w| w.usart2en().enabled());
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

    // Initial commands
    let mut command_queue = Queue::default();

    let mut write_rf_ch = Vec::new();
    let _ = write_rf_ch.push(Command::WriteRegister(registers::Address::RfCh as u8).word());
    let _ = write_rf_ch.push(registers::RfCh::default().with_rf_ch(110).into_bits());

    let mut read_rf_ch = Vec::new();
    let _ = read_rf_ch.push(Command::ReadRegister(registers::Address::RfCh as u8).word());
    let _ = read_rf_ch.push(0);

    let mut write_tx_addr = Vec::new();
    let _ = write_tx_addr.push(Command::WriteRegister(registers::Address::TxAddr as u8).word());
    let _ = write_tx_addr.extend_from_slice(
        &registers::TxAddr::default()
            .with_tx_addr(TX_ADDR)
            .as_payload(),
    );

    let mut read_tx_addr = Vec::new();
    let _ = read_tx_addr.push(Command::ReadRegister(registers::Address::TxAddr as u8).word());
    let _ = read_tx_addr.extend_from_slice(&[0; 5]);

    let mut write_rx_addr_p0 = Vec::new();
    let _ =
        write_rx_addr_p0.push(Command::WriteRegister(registers::Address::RxAddrP0 as u8).word());
    let _ = write_rx_addr_p0.extend_from_slice(
        &registers::RxAddrP0::default()
            .with_rx_addr_p0(TX_ADDR)
            .as_payload(),
    );

    let mut read_rx_addr_p0 = Vec::new();
    let _ = read_rx_addr_p0.push(Command::ReadRegister(registers::Address::RxAddrP0 as u8).word());
    let _ = read_rx_addr_p0.extend_from_slice(&[0; 5]);

    let mut activate = Vec::new();
    let _ = activate.push(Command::Activate.word());
    let _ = activate.push(0x73);

    let mut write_feature = Vec::new();
    let _ = write_feature.push(Command::WriteRegister(registers::Address::Feature as u8).word());
    let _ = write_feature.push(
        registers::Feature::default()
            .with_en_dyn_ack(true)
            .into_bits(),
    );

    let mut read_feature = Vec::new();
    let _ = read_feature.push(Command::ReadRegister(registers::Address::Feature as u8).word());
    let _ = read_feature.push(0);

    let _ = command_queue.enqueue(write_rf_ch);
    let _ = command_queue.enqueue(read_rf_ch);
    let _ = command_queue.enqueue(write_tx_addr);
    let _ = command_queue.enqueue(read_tx_addr);
    let _ = command_queue.enqueue(write_rx_addr_p0);
    let _ = command_queue.enqueue(read_rx_addr_p0);
    let _ = command_queue.enqueue(activate);
    let _ = command_queue.enqueue(write_feature);
    let _ = command_queue.enqueue(read_feature);

    unsafe {
        COMMANDS = Some(command_queue);
        TX_BUFFER = Some(Queue::default());
        RX_BUFFER = Some(Queue::default());
        // Unmask NVIC USART2 global interrupt
        cortex_m::peripheral::NVIC::unmask(Interrupt::SPI1);
        cortex_m::peripheral::NVIC::unmask(Interrupt::USART2);
        SPI1_PERIPHERAL = Some(dp.SPI1);
        USART2_PERIPHERAL = Some(dp.USART2);
        // Enable SPI to send initial commands
        SPI1_PERIPHERAL
            .as_mut()
            .unwrap()
            .cr2()
            .modify(|_, w| w.txeie().set_bit());
        SPI1_PERIPHERAL
            .as_mut()
            .unwrap()
            .cr1()
            .modify(|_, w| w.spe().enabled());
    }

    #[allow(clippy::empty_loop)]
    loop {}
}
