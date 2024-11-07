#![no_std]
#![no_main]

use cortex_m::{asm, Peripherals as CorePeripherals};
use cortex_m_rt::entry;
use nrf24l01_tx::sync_cell::{SyncPeripheral, SyncQueue, SyncState};
use nrf24l01_tx::State;
// use cortex_m_semihosting::hprintln;
use nrf24l01_commands::{commands, registers};
use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use stm32l4::stm32l4x2::{interrupt, Interrupt, Peripherals as DevicePeripherals};

const USART2_RDR: u32 = 0x4000_4424;
const SPI1_DR: u32 = 0x4001_300C;
const TX_ADDR: u64 = 0xA2891FFF6A;
const SLEEP_DEEP_ON: u32 = 0b110;
const SLEEP_DEEP_OFF: u32 = 0b010;

static CORE_PERIPHERALS: SyncPeripheral<CorePeripherals> = SyncPeripheral::new();
static DEVICE_PERIPHERALS: SyncPeripheral<DevicePeripherals> = SyncPeripheral::new();

// Commands
const NOP: [u8; 1] = commands::Nop::bytes();
const W_RF_CH: [u8; 2] = commands::WRegister(registers::RfCh::new().with_rf_ch(110)).bytes();
const W_RF_SETUP: [u8; 2] =
    commands::WRegister(registers::RfSetup::new().with_rf_dr(false)).bytes();
const W_TX_ADDR: [u8; 6] =
    commands::WRegister(registers::TxAddr::<5>::new().with_tx_addr(TX_ADDR)).bytes();
const W_RX_ADDR_P0: [u8; 6] =
    commands::WRegister(registers::RxAddrP0::<5>::new().with_rx_addr_p0(TX_ADDR)).bytes();
const W_CONFIG: [u8; 2] = commands::WRegister(
    registers::Config::new()
        .with_pwr_up(true)
        .with_mask_rx_dr(true),
)
.bytes();
const POWER_DOWN: [u8; 2] =
    commands::WRegister(registers::Config::new().with_pwr_up(false)).bytes();
const RESET_INTERRUPTS: [u8; 2] =
    commands::WRegister(registers::Status::new().with_max_rt(true).with_tx_ds(true)).bytes();
const HANDSHAKE: [u8; 33] = commands::WTxPayload([b'C'; 32]).bytes();
const FLUSH_TX: [u8; 1] = commands::FlushTx::bytes();

static mut W_TX_PAYLOAD: [u8; 33] = commands::WTxPayload([0; 32]).bytes();
static mut SPI1_RX_BUFFER: [u8; 33] = [0; 33];
static COMMANDS: SyncQueue<&[u8], 16> = SyncQueue::new();
static STATE: SyncState = SyncState::new();

#[inline]
fn send_command(command: &[u8], dp: &mut DevicePeripherals) {
    // Write memory address for SPI1 TX
    dp.DMA1
        .ch3()
        .mar()
        .write(|w| unsafe { w.bits(command.as_ptr() as u32) });
    let transfer_size = command.len() as u32;
    // Set DMA transfer size for SPI1 RX, TX
    dp.DMA1
        .ch2()
        .ndtr()
        .write(|w| unsafe { w.bits(transfer_size) });
    dp.DMA1
        .ch3()
        .ndtr()
        .write(|w| unsafe { w.bits(transfer_size) });

    // Enable DMA for SPI1 RX, TX
    dp.DMA1.ch2().cr().modify(|_, w| w.en().set_bit());
    dp.DMA1.ch3().cr().modify(|_, w| w.en().set_bit());
    // Enable SPI1
    dp.SPI1.cr1().modify(|_, w| w.spe().enabled());
}

#[inline]
fn pulse_ce(dp: &mut DevicePeripherals) {
    // Set CE high
    dp.GPIOA.bsrr().write(|w| w.bs0().set_bit());
    // Enable counter, one-pulse mode
    dp.TIM2.cr1().write(|w| w.opm().enabled().cen().enabled());
}

#[inline]
fn listen_payload(dp: &mut DevicePeripherals) {
    dp.USART2.cr1().modify(|_, w| w.ue().set_bit());
    dp.DMA1.ch6().ndtr().write(|w| unsafe { w.bits(32) });
    dp.DMA1.ch6().cr().modify(|_, w| w.en().set_bit());
}

#[inline]
fn stop_listen_payload(dp: &mut DevicePeripherals) {
    dp.USART2.cr1().modify(|_, w| w.ue().clear_bit());
    dp.DMA1.ch6().cr().modify(|_, w| w.en().clear_bit());
}

/// USART2 RX DMA
#[interrupt]
fn DMA1_CH6() {
    let dp = DEVICE_PERIPHERALS.get();

    if dp.DMA1.isr().read().tcif6().bit_is_set() {
        dp.DMA1.ch6().cr().modify(|_, w| w.en().clear_bit());
        dp.DMA1.ifcr().write(|w| w.ctcif6().set_bit());
        dp.USART2.cr1().modify(|_, w| w.ue().clear_bit());

        #[allow(static_mut_refs)]
        send_command(unsafe { &W_TX_PAYLOAD }, dp);
    }
}

/// SPI1 RX DMA
#[interrupt]
fn DMA1_CH2() {
    let cp = CORE_PERIPHERALS.get();
    let dp = DEVICE_PERIPHERALS.get();
    let commands = COMMANDS.get();
    let state = STATE.get();
    let status = unsafe { SPI1_RX_BUFFER[0] };

    if dp.DMA1.isr().read().tcif2().bit_is_set() {
        dp.DMA1.ch2().cr().modify(|_, w| w.en().clear_bit());
        dp.DMA1.ifcr().write(|w| w.ctcif2().set_bit());

        // Disable SPI1
        dp.SPI1.cr1().modify(|_, w| w.spe().clear_bit());

        // Send next command
        if let Some(command) = commands.dequeue() {
            send_command(command, dp);
        } else {
            match *state {
                State::Init => pulse_ce(dp),
                State::PulseCe => pulse_ce(dp),
                State::HandleIrq => {
                    let status = registers::Status::from_bits(status);
                    if status.max_rt() {
                        let _ = commands.enqueue(&FLUSH_TX);
                        let _ = commands.enqueue(&POWER_DOWN);
                        dp.GPIOA.bsrr().write(|w| w.br8().set_bit());
                        *state = State::BeginSleep;
                    } else if status.tx_ds() {
                        *state = State::Connected;
                        dp.RTC.cr().modify(|_, w| w.wute().clear_bit());
                        listen_payload(dp);
                        dp.GPIOA.bsrr().write(|w| w.bs8().set_bit());
                    }
                    send_command(&RESET_INTERRUPTS, dp);
                }
                State::BeginSleep => {
                    stop_listen_payload(dp);
                    // go sleep
                    // This is entered twice because commands is empty twice
                    if dp.RTC.cr().read().wute().bit_is_clear() {
                        while dp.RTC.isr().read().wutwf().bit_is_clear() {}
                        dp.RTC.cr().modify(|_, w| w.wute().set_bit());
                    }
                    unsafe {
                        // Turn on deepsleep
                        cp.SCB.scr.write(SLEEP_DEEP_ON);
                    }
                }
                State::Connected => {
                    if dp.DMA1.ch3().mar().read() == NOP.as_ptr() as u32 {
                        let status = registers::Status::from_bits(status);
                        if status.max_rt() {
                            let _ = commands.enqueue(&FLUSH_TX);
                            let _ = commands.enqueue(&POWER_DOWN);
                            dp.GPIOA.bsrr().write(|w| w.br8().set_bit());
                            *state = State::BeginSleep;
                        } else if status.tx_ds() {
                            // *state = State::Connected;
                        }
                        send_command(&RESET_INTERRUPTS, dp);
                    } else if dp.DMA1.ch3().mar().read() == unsafe { W_TX_PAYLOAD.as_ptr() } as u32
                    {
                        pulse_ce(dp);
                        // Read next payload
                        listen_payload(dp);
                    }
                }
            }
        }
    }
}

/// SPI1 TX DMA
#[interrupt]
fn DMA1_CH3() {
    let dp = DEVICE_PERIPHERALS.get();
    if dp.DMA1.isr().read().tcif3().bit_is_set() {
        dp.DMA1.ch3().cr().modify(|_, w| w.en().clear_bit());
        dp.DMA1.ifcr().write(|w| w.ctcif3().set_bit());
    }
}

#[interrupt]
fn EXTI1() {
    let dp = DEVICE_PERIPHERALS.get();
    let state = STATE.get();
    // let commands = COMMANDS.get();

    if dp.EXTI.pr1().read().pr1().bit_is_set() {
        dp.EXTI.pr1().write(|w| w.pr1().clear_bit_by_one());
        if *state == State::Connected {
            send_command(&NOP, dp);
        } else {
            *state = State::HandleIrq;
            send_command(&NOP, dp);
        }
        // if dp.DMA1.ch2().cr().read().en().bit_is_set() {
        //     let _ = commands.enqueue(&NOP);
        // } else {

        // }
    }
}

#[interrupt]
fn TIM2() {
    let dp = DEVICE_PERIPHERALS.get();

    if dp.TIM2.sr().read().uif().bit_is_set() {
        dp.TIM2.sr().write(|w| w.uif().clear_bit());
        dp.GPIOA.bsrr().write(|w| w.br0().set_bit());
    }
}

#[interrupt]
fn RTC_WKUP() {
    let cp = CORE_PERIPHERALS.get();
    let dp = DEVICE_PERIPHERALS.get();
    let commands = COMMANDS.get();
    let state = STATE.get();

    if dp.RTC.isr().read().wutf().bit_is_set() {
        dp.RTC.isr().modify(|_, w| w.wutf().clear_bit());
        dp.EXTI.pr1().write(|w| w.pr20().clear_bit_by_one());
        unsafe {
            // Turn off sleepdeep
            cp.SCB.scr.write(SLEEP_DEEP_OFF);
        }
        *state = State::PulseCe;
        let _ = commands.enqueue(&HANDSHAKE);
        send_command(&W_CONFIG, dp);
    }
}

#[entry]
fn main() -> ! {
    // Device defaults to 4MHz clock

    let cp = CorePeripherals::take().unwrap();
    let dp = DevicePeripherals::take().unwrap();

    dp.RCC.csr().write(|w| w.lsion().set_bit());
    // Set system clock speed to 200 khz
    dp.RCC
        .cr()
        .write(|w| w.msirange().range200k().msirgsel().set_bit());

    // Enable peripheral clocks: DMA1, GPIOA, USART2, TIM2, SPI1
    dp.RCC.ahb1enr().write(|w| w.dma1en().set_bit());
    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit());
    dp.RCC.apb1enr1().write(|w| {
        w.usart2en()
            .enabled()
            .tim2en()
            .set_bit()
            .pwren()
            .set_bit()
            .rtcapben()
            .set_bit()
    });
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
            .moder8()
            .output()
    });
    dp.GPIOA
        .otyper()
        .write(|w| w.ot0().push_pull().ot8().push_pull());
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

    // EXTI
    dp.EXTI.ftsr1().write(|w| w.tr1().set_bit());
    dp.EXTI.imr1().write(|w| w.mr1().set_bit().mr20().set_bit());

    // Set SleepOnExit
    unsafe { cp.SCB.scr.write(SLEEP_DEEP_OFF) };
    // Set Stop 2 low-power mode, remove write protection from BDCR
    dp.PWR
        .cr1()
        .write(|w| unsafe { w.lpms().bits(0b010).dbp().set_bit() });
    // Configure RTC, set 5s periodic wake up
    while dp.RCC.csr().read().lsirdy().bit_is_clear() {}
    dp.RCC.bdcr().write(|w| w.rtcsel().lsi().rtcen().set_bit());
    // Remove write protection from RTC registers
    dp.RTC.wpr().write(|w| unsafe { w.key().bits(0xCA) });
    dp.RTC.wpr().write(|w| unsafe { w.key().bits(0x53) });
    // Enter init mode to set prescaler values
    dp.RTC.isr().write(|w| w.init().set_bit());
    while dp.RTC.isr().read().initf().bit_is_clear() {}
    dp.RTC
        .prer()
        .write(|w| unsafe { w.prediv_a().bits(127).prediv_s().bits(249) });
    dp.RTC.isr().write(|w| w.init().clear_bit());
    // Turn off wake-up timer
    dp.RTC.cr().write(|w| w.wute().clear_bit());
    while dp.RTC.isr().read().wutwf().bit_is_clear() {}
    // Write wake-up timer registers
    dp.RTC.wutr().write(|w| unsafe { w.wut().bits(4) });
    dp.EXTI.rtsr1().write(|w| w.tr20().set_bit());
    dp.RTC
        .cr()
        .write(|w| unsafe { w.wucksel().bits(0b100) }.wutie().set_bit());

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
        .write(|w| unsafe { w.ma().bits(W_TX_PAYLOAD[1..].as_ptr() as u32) });
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
    dp.TIM2.arr().write(|w| unsafe { w.arr().bits(3) }); // 200khz * 11us approx. 3

    // Enable TIM2 update interrupt
    dp.TIM2.dier().write(|w| w.uie().set_bit());

    // USART2: Configure baud rate 9600
    dp.USART2.brr().write(|w| unsafe { w.bits(21) }); // 200khz / 9600 approx. 21

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

    // Enable USART receiver
    dp.USART2.cr1().write(|w| w.re().set_bit());

    // Initialization commands
    let commands = COMMANDS.get();

    unsafe {
        commands.enqueue_unchecked(&W_RF_SETUP);
        commands.enqueue_unchecked(&W_TX_ADDR);
        commands.enqueue_unchecked(&W_RX_ADDR_P0);
        commands.enqueue_unchecked(&W_CONFIG);
        commands.enqueue_unchecked(&HANDSHAKE);

        // Unmask NVIC global interrupts
        cortex_m::peripheral::NVIC::unmask(Interrupt::RTC_WKUP);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH2);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH3);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH6);
        cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI1);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
    }
    CORE_PERIPHERALS.set(cp);
    DEVICE_PERIPHERALS.set(dp);

    let dp = DEVICE_PERIPHERALS.get();

    send_command(&W_RF_CH, dp);

    #[allow(clippy::empty_loop)]
    loop {
        asm::wfi();
    }
}
