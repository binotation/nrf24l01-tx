//! Receive a byte over USART2 and send it over SPI1.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use heapless::spsc::Queue;
use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use stm32l4::stm32l4x2::{interrupt, Interrupt, Peripherals, SPI1, USART2};

static mut USART2_PERIPHERAL: Option<USART2> = None;
static mut SPI1_PERIPHERAL: Option<SPI1> = None;
/// Bytes to be transmitted over SPI1
static mut TX_BUFFER: Option<Queue<u16, 16>> = None;
/// Bytes received over SPI1
static mut RX_BUFFER: Option<Queue<u16, 16>> = None;

#[interrupt]
fn USART2() {
    // SAFETY: race condition where USART2_PERIPHERAL can be accessed before being set
    let usart2 = unsafe { USART2_PERIPHERAL.as_mut() }.unwrap();
    let spi1 = unsafe { SPI1_PERIPHERAL.as_mut() }.unwrap();
    let tx_buffer = unsafe { TX_BUFFER.as_mut() }.unwrap();
    let rx_buffer = unsafe { RX_BUFFER.as_mut() }.unwrap();

    // Dequeue bytes off rx buffer and transmit over USART2
    if usart2.isr.read().txe().bit_is_set() {
        match rx_buffer.dequeue() {
            Some(byte) => {
                usart2.tdr.write(|w| w.tdr().bits(byte));
                if rx_buffer.is_empty() {
                    usart2.cr1.modify(|_, w| w.txeie().disabled());
                }
            }
            None => usart2.cr1.modify(|_, w| w.txeie().disabled()),
        }
    }

    // Read incoming bytes from USART2 and queue onto tx buffer
    if usart2.isr.read().rxne().bit_is_set() {
        // Read data, this clears RXNE
        let received_byte = usart2.rdr.read().rdr().bits();

        // Queue byte, do nothing if queue is full
        if tx_buffer.enqueue(received_byte).is_ok() {
            // Enable TXE interrupt as buffer is now non-empty
            spi1.cr2.modify(|_, w| w.txeie().set_bit());
            spi1.cr1.modify(|_, w| w.spe().enabled());
        }
    }
    if usart2.isr.read().ore().bit_is_set() {
        usart2.icr.write(|w| w.orecf().set_bit());
    }
}

#[interrupt]
fn SPI1() {
    let spi1 = unsafe { SPI1_PERIPHERAL.as_mut() }.unwrap();
    let usart2 = unsafe { USART2_PERIPHERAL.as_mut() }.unwrap();
    let tx_buffer = unsafe { TX_BUFFER.as_mut() }.unwrap();
    let rx_buffer = unsafe { RX_BUFFER.as_mut() }.unwrap();

    // Transmit bytes from tx buffer
    if spi1.sr.read().txe().bit_is_set() {
        match tx_buffer.dequeue() {
            Some(byte) => {
                spi1.dr.write(|w| w.dr().bits(255));
                while spi1.sr.read().bsy().bit_is_set() {}
                spi1.cr1.modify(|_, w| w.spe().disabled());
                if tx_buffer.is_empty() {
                    spi1.cr2.modify(|_, w| w.txeie().clear_bit());
                }
            }
            None => {
                spi1.cr1.modify(|_, w| w.spe().disabled());
                spi1.cr2.modify(|_, w| w.txeie().clear_bit());
            }
        }
    }

    // Read incoming bytes over SPI1 and queue onto rx buffer
    if spi1.sr.read().rxne().bit_is_set() {
        let received_byte = spi1.dr.read().dr().bits();
        if rx_buffer.enqueue(received_byte).is_ok() {
            usart2.cr1.modify(|_, w| w.txeie().enabled());
        }
    }
}

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let dp = Peripherals::take().unwrap();

    // Enable peripheral clocks - GPIOA, USART2
    dp.RCC.ahb2enr.write(|w| w.gpioaen().set_bit());
    dp.RCC.apb1enr1.write(|w| w.usart2en().enabled());
    dp.RCC.apb2enr.write(|w| w.spi1en().set_bit());

    // USART2: A2 (TX), A3 (RX) as AF 7
    // SPI1: A4 (NSS), A5 (SCK), A6 (MISO), A7 (MOSI) as AF 5
    dp.GPIOA.moder.write(|w| {
        w.moder2()
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
    dp.GPIOA.pupdr.write(|w| w.pupdr4().pull_up());
    dp.GPIOA.ospeedr.write(|w| {
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
    dp.GPIOA.afrl.write(|w| {
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
    dp.USART2.brr.write(|w| unsafe { w.bits(417) }); // 4Mhz / 9600 approx. 417

    // SPI1: Set FIFO reception threshold to 1/4, data frame size to 8 bits, enable slave select output,
    // enable RXNE interupt
    dp.SPI1.cr2.write(|w| unsafe {
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
    dp.SPI1.cr1.write(|w| w.br().bits(2).mstr().set_bit());

    // Enable USART, transmitter, receiver and RXNE interrupt
    dp.USART2.cr1.write(|w| {
        w.re()
            .set_bit()
            .te()
            .set_bit()
            .ue()
            .set_bit()
            .rxneie()
            .set_bit()
    });

    unsafe {
        TX_BUFFER = Some(Queue::default());
        RX_BUFFER = Some(Queue::default());
        // Unmask NVIC USART2 global interrupt
        cortex_m::peripheral::NVIC::unmask(Interrupt::SPI1);
        cortex_m::peripheral::NVIC::unmask(Interrupt::USART2);
        SPI1_PERIPHERAL = Some(dp.SPI1);
        USART2_PERIPHERAL = Some(dp.USART2);
    }

    #[allow(clippy::empty_loop)]
    loop {}
}
