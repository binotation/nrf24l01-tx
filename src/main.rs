#![no_std]
#![no_main]

use core::arch::asm;
use cortex_m::{asm, Peripherals as CorePeripherals};
use cortex_m_rt::{entry, pre_init};
use gps_relay::sync_cell::{SyncPeripheral, SyncQueue, SyncState};
use gps_relay::State;
// use cortex_m_semihosting::hprintln;
use nrf24l01_commands::{commands, commands::Command, registers};
use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use stm32l4::stm32l412::{interrupt, Interrupt, Peripherals as DevicePeripherals};

const USART2_RDR: u32 = 0x4000_4424;
const SPI1_DR: u32 = 0x4001_300C;
const TX_ADDRESS: u64 = 0xA2891F;
const SLEEPDEEP_ON: u32 = 0b110;
const SLEEPDEEP_OFF: u32 = 0b010;
const SLEEP_DURATION: u16 = 10;

static CORE_PERIPHERALS: SyncPeripheral<CorePeripherals> = SyncPeripheral::new();
static DEVICE_PERIPHERALS: SyncPeripheral<DevicePeripherals> = SyncPeripheral::new();

// Commands
const SETUP_AW: [u8; 2] = commands::WRegister(registers::SetupAw::new().with_aw(1)).bytes();
const SETUP_RETR: [u8; 2] = commands::WRegister(registers::SetupRetr::new().with_arc(15)).bytes();
const RF_CH: [u8; 2] = commands::WRegister(registers::RfCh::new().with_rf_ch(0)).bytes();
const RF_SETUP: [u8; 2] = commands::WRegister(registers::RfSetup::new().with_rf_dr(false)).bytes();
const RX_ADDR_P0: [u8; 4] =
    commands::WRegister(registers::RxAddrP0::<3>::new().with_rx_addr_p0(TX_ADDRESS)).bytes();
const TX_ADDR: [u8; 4] =
    commands::WRegister(registers::TxAddr::<3>::new().with_tx_addr(TX_ADDRESS)).bytes();
const ACTIVATE: [u8; 2] = commands::Activate::bytes();
const FEATURE: [u8; 2] = commands::WRegister(registers::Feature::new().with_en_dpl(true)).bytes();
const DYNPD: [u8; 2] = commands::WRegister(registers::Dynpd::new().with_dpl_p0(true)).bytes();
const POWER_UP: [u8; 2] = commands::WRegister(
    registers::Config::new()
        .with_pwr_up(true)
        .with_mask_rx_dr(true),
)
.bytes();
const POWER_UP_MASK_TX_DS: [u8; 2] = commands::WRegister(
    registers::Config::new()
        .with_pwr_up(true)
        .with_mask_tx_ds(true)
        .with_mask_rx_dr(true),
)
.bytes();
const HANDSHAKE: [u8; 2] = commands::WTxPayload([0; 1]).bytes();
const POWER_DOWN: [u8; 2] =
    commands::WRegister(registers::Config::new().with_pwr_up(false)).bytes();
const RESET_INTERRUPTS: [u8; 2] =
    commands::WRegister(registers::Status::new().with_max_rt(true).with_tx_ds(true)).bytes();
const FLUSH_TX: [u8; 1] = commands::FlushTx::bytes();

static mut PAYLOAD_DOUBLE_BUFFER: [u8; 65] = [
    commands::WTxPayload::<32>::WORD,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    commands::WTxPayload::<32>::WORD,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
];
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
    dp.DMA1
        .ch2()
        .cr()
        .write(|w| w.minc().set_bit().tcie().set_bit().en().set_bit());
    dp.DMA1.ch3().cr().write(|w| {
        w.minc()
            .set_bit()
            .dir()
            .set_bit()
            .tcie()
            .set_bit()
            .en()
            .set_bit()
    });
    // Enable SPI1
    dp.SPI1.cr1().write(|w| w.mstr().set_bit().spe().enabled());
}

#[inline]
fn pulse_ce(dp: &mut DevicePeripherals) {
    // Set CE high
    dp.GPIOA.bsrr().write(|w| w.bs0().set_bit());
    // Enable counter, one-pulse mode
    dp.TIM6.cr1().write(|w| w.opm().enabled().cen().enabled());
}

#[inline]
fn listen_payload(dp: &mut DevicePeripherals) {
    // Enable DMA Ch6 and USART
    dp.DMA1.ch6().ndtr().write(|w| unsafe { w.bits(64) });
    dp.DMA1.ch6().cr().write(|w| {
        w.minc()
            .set_bit()
            .htie()
            .set_bit()
            .tcie()
            .set_bit()
            .circ()
            .set_bit()
            .en()
            .set_bit()
    });
    dp.USART2.cr1().write(|w| w.re().set_bit().ue().set_bit());
    dp.GPIOA.bsrr().write(|w| w.bs8().set_bit());
}

#[inline]
fn stop_listen_payload(dp: &mut DevicePeripherals) {
    // Power off GPS
    dp.GPIOA.bsrr().write(|w| w.br8().set_bit());
    // Disable USART and DMA Ch6
    dp.USART2
        .cr1()
        .write(|w| w.re().clear_bit().ue().clear_bit());
    dp.DMA1.ch6().cr().write(|w| {
        w.minc()
            .set_bit()
            .htie()
            .set_bit()
            .tcie()
            .set_bit()
            .circ()
            .set_bit()
            .en()
            .clear_bit()
    });
}

/// SPI1 RX DMA - nRF24L01 command complete
#[interrupt]
fn DMA1_CH2() {
    let cp = CORE_PERIPHERALS.get();
    let dp = DEVICE_PERIPHERALS.get();
    let commands = COMMANDS.get();
    let state = STATE.get();

    if dp.DMA1.isr().read().tcif2().bit_is_set() {
        // Disable DMA Ch2
        dp.DMA1
            .ch2()
            .cr()
            .write(|w| w.minc().set_bit().tcie().set_bit().en().clear_bit());

        // Disable SPI1
        dp.SPI1
            .cr1()
            .write(|w| w.mstr().set_bit().spe().clear_bit());

        // Send next command
        if let Some(command) = commands.dequeue() {
            send_command(command, dp);
        } else {
            match *state {
                State::HandshakePulse => {
                    // Wait for nRF24L01 to power up
                    dp.TIM16.cr1().write(|w| w.opm().enabled().cen().enabled());
                    *state = State::HandshakeIrq;
                }
                State::HandshakeIrq => {
                    let status = registers::Status::from_bits(unsafe { SPI1_RX_BUFFER[0] });
                    if status.max_rt() {
                        send_command(&POWER_DOWN, dp);
                        *state = State::ContinueSleep;
                    } else if status.tx_ds() {
                        send_command(&POWER_UP_MASK_TX_DS, dp);
                        // Disable wake-up timer
                        dp.RTC.cr().write(|w| {
                            unsafe { w.wucksel().bits(0b100) }
                                .wutie()
                                .set_bit()
                                .wute()
                                .clear_bit()
                        });
                        listen_payload(dp);
                        *state = State::Connected;
                    }
                }
                State::BeginSleep => {
                    // This state is only entered from State::Connected
                    // Enable wake-up timer
                    dp.RTC.cr().write(|w| {
                        unsafe { w.wucksel().bits(0b100) }
                            .wutie()
                            .set_bit()
                            .wute()
                            .set_bit()
                    });
                    unsafe {
                        cp.SCB.scr.write(SLEEPDEEP_ON);
                        // Prepare to enter Stop 2: enter run mode range 2 with Stop 2 selected
                        dp.PWR.cr1().write(|w| {
                            w.vos()
                                .bits(2)
                                .lpr()
                                .clear_bit()
                                .lpms()
                                .bits(2)
                                .dbp()
                                .set_bit()
                        });
                    }
                    while dp.PWR.sr2().read().reglpf().bit_is_set() {}
                }
                State::ContinueSleep => {
                    unsafe {
                        // This state is only entered from wake-up and the wake-up timer is already enabled.
                        cp.SCB.scr.write(SLEEPDEEP_ON);
                        // Prepare to enter Stop 2: enter run mode range 2 with Stop 2 selected
                        dp.PWR.cr1().write(|w| {
                            w.vos()
                                .bits(2)
                                .lpr()
                                .clear_bit()
                                .lpms()
                                .bits(2)
                                .dbp()
                                .set_bit()
                        });
                    }
                    while dp.PWR.sr2().read().reglpf().bit_is_set() {}
                }
                State::Connected => {
                    #[allow(static_mut_refs)]
                    if dp.DMA1.ch3().mar().read() == RESET_INTERRUPTS.as_ptr() as u32 {
                        // MAX_RT asserted -> receiver has disconnected: power off GPS
                        stop_listen_payload(dp);
                        commands.enqueue(&HANDSHAKE);
                        commands.enqueue(&POWER_DOWN);
                        send_command(&FLUSH_TX, dp);
                        *state = State::BeginSleep;
                    } else if dp.DMA1.ch3().mar().read()
                        == unsafe { PAYLOAD_DOUBLE_BUFFER.as_ptr() } as u32
                        || dp.DMA1.ch3().mar().read()
                            == unsafe { PAYLOAD_DOUBLE_BUFFER[32..].as_ptr() } as u32
                    {
                        pulse_ce(dp);
                    }
                }
            }
        }
        dp.DMA1.ifcr().write(|w| w.ctcif2().set_bit());
    }
}

/// Handle IRQ. Timing calculations for delay between payload upload and active IRQ:
/// minimum delay ~ 1320us + 130 + 305 (nrf24l01 p.38) + 130 + 49 ~ 2ms
/// maximum delay ~ 1320us + (130 + 305 + 250) * 15 (retransmission) - 250 + 130 + 50? ~ 11.5 ms
/// A UART byte is transacted in ~1 ms, so 32 bytes for the next payload will take 32ms.
/// This means it should be safe to assume there is no ongoing SPI transaction.
#[interrupt]
fn EXTI1() {
    let dp = DEVICE_PERIPHERALS.get();

    if dp.EXTI.pr1().read().pr1().bit_is_set() {
        send_command(&RESET_INTERRUPTS, dp);
        dp.EXTI.pr1().write(|w| w.pr1().clear_bit_by_one());
    }
}

/// Periodic wake-up routine. Handshake payload is already in nRF24L01 TX FIFO,
/// so we only need to pulse CE.
#[interrupt]
fn RTC_WKUP() {
    let cp = CORE_PERIPHERALS.get();
    let dp = DEVICE_PERIPHERALS.get();
    let state = STATE.get();

    if dp.RTC.sr().read().wutf().bit_is_set() {
        dp.RTC.scr().write(|w| w.cwutf().set_bit());
        dp.EXTI.pr1().write(|w| w.pr20().clear_bit_by_one());
        unsafe {
            // Don't go back into Stop 2
            cp.SCB.scr.write(SLEEPDEEP_OFF);
            // Enter low-power run mode
            dp.PWR.cr1().write(|w| {
                w.vos()
                    .bits(2)
                    .lpr()
                    .set_bit()
                    .lpms()
                    .bits(1)
                    .dbp()
                    .set_bit()
            });
        }
        *state = State::HandshakePulse;
        // Power up nRF24L01
        send_command(&POWER_UP, dp);
    }
}

/// USART2 RX DMA - Transfers GPS NMEA payload to double buffer
#[interrupt]
fn DMA1_CH6() {
    let dp = DEVICE_PERIPHERALS.get();

    if dp.DMA1.isr().read().htif6().bit_is_set() {
        // Send payload in the first half of the buffer
        send_command(unsafe { &PAYLOAD_DOUBLE_BUFFER[0..33] }, dp);
        dp.DMA1.ifcr().write(|w| w.chtif6().set_bit());
    } else if dp.DMA1.isr().read().tcif6().bit_is_set() {
        // Send payload in the second half of the buffer
        #[allow(static_mut_refs)]
        unsafe {
            /* SPI runs an order of magnitude faster than USART,
            W_TX_PAYLOAD command will complete before this is overwritten */
            PAYLOAD_DOUBLE_BUFFER[32] = commands::WTxPayload::<32>::WORD
        };
        send_command(unsafe { &PAYLOAD_DOUBLE_BUFFER[32..65] }, dp);
        dp.DMA1.ifcr().write(|w| w.ctcif6().set_bit());
    }
}

/// SPI1 TX DMA - Send nRF24L01 command
#[interrupt]
fn DMA1_CH3() {
    let dp = DEVICE_PERIPHERALS.get();
    if dp.DMA1.isr().read().tcif3().bit_is_set() {
        // Disable DMA Ch3
        dp.DMA1.ch3().cr().write(|w| {
            w.minc()
                .set_bit()
                .dir()
                .set_bit()
                .tcie()
                .set_bit()
                .en()
                .clear_bit()
        });
        dp.DMA1.ifcr().write(|w| w.ctcif3().set_bit());
    }
}

/// CE 10us pulse
#[interrupt]
fn TIM6_DACUNDER() {
    let dp = DEVICE_PERIPHERALS.get();

    if dp.TIM6.sr().read().uif().bit_is_set() {
        dp.GPIOA.bsrr().write(|w| w.br0().set_bit());
        dp.TIM6.sr().write(|w| w.uif().clear_bit());
    }
}

/// Wait for nRF24L01 to power up
#[interrupt]
fn TIM1_UP_TIM16() {
    let dp = DEVICE_PERIPHERALS.get();

    if dp.TIM16.sr().read().uif().bit_is_set() {
        pulse_ce(dp);
        dp.TIM16.sr().write(|w| w.uif().clear_bit());
    }
}

#[pre_init]
unsafe fn copy_flash_sections_to_ram() {
    asm!(
        // Copy vector table to SRAM1
        "ldr r0, =__vector_table
         ldr r1, =__evector_table
         ldr r2, =__sivector_table
         0:
         cmp r1, r0
         beq 1f
         ldm r2!, {{r3}}
         stm r0!, {{r3}}
         b 0b
         1:",
        // Copy .rodata to SRAM1
        "ldr r0, =__srodata
         ldr r1, =__erodata
         ldr r2, =__sirodata
         0:
         cmp r1, r0
         beq 1f
         ldm r2!, {{r3}}
         stm r0!, {{r3}}
         b 0b
         1:"
    );
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

    // Enable peripheral clocks
    dp.RCC
        .ahb1enr()
        .write(|w| w.dma1en().set_bit().flashen().set_bit());
    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit());
    dp.RCC.apb1enr1().write(|w| {
        w.usart2en()
            .set_bit()
            .tim6en()
            .set_bit()
            .pwren()
            .set_bit()
            .rtcapben()
            .set_bit()
    });
    dp.RCC
        .apb2enr()
        .write(|w| w.spi1en().set_bit().tim16en().set_bit());
    // Flash memory clock is gated off during sleep and stop
    dp.RCC.ahb1smenr().write(|w| w.flashsmen().clear_bit());

    // Power down flash during sleep
    dp.FLASH.acr().write(|w| w.sleep_pd().set_bit());

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
    // NSS output is active low
    dp.GPIOA.pupdr().write(|w| w.pupdr4().pull_up());
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

    unsafe {
        // Set SleepOnExit
        cp.SCB.scr.write(SLEEPDEEP_OFF);
        // Set voltage scaling to range 2, low-power run mode and
        // remove write protection from BDCR and RTC registers
        dp.PWR.cr1().write(|w| {
            w.vos()
                .bits(2)
                .lpr()
                .set_bit()
                .lpms()
                .bits(1)
                .dbp()
                .set_bit()
        });
    }
    // Configure RTC, set periodic wake up
    while dp.RCC.csr().read().lsirdy().bit_is_clear() {}
    dp.RCC.bdcr().write(|w| w.rtcsel().lsi().rtcen().set_bit());
    // Remove write protection from RTC registers
    dp.RTC.wpr().write(|w| unsafe { w.key().bits(0xCA) });
    dp.RTC.wpr().write(|w| unsafe { w.key().bits(0x53) });
    // Enter init mode to set prescaler values
    dp.RTC.icsr().write(|w| w.init().set_bit());
    while dp.RTC.icsr().read().initf().bit_is_clear() {}
    dp.RTC
        .prer()
        .write(|w| unsafe { w.prediv_a().bits(127).prediv_s().bits(249) });
    dp.RTC.icsr().write(|w| w.init().clear_bit());
    // Turn off wake-up timer
    dp.RTC.cr().write(|w| w.wute().clear_bit());
    while dp.RTC.icsr().read().wutwf().bit_is_clear() {}
    // Write wake-up timer registers
    dp.RTC
        .wutr()
        .write(|w| unsafe { w.wut().bits(SLEEP_DURATION - 1) });
    dp.EXTI.rtsr1().write(|w| w.tr20().set_bit());
    dp.RTC.cr().write(|w| {
        unsafe { w.wucksel().bits(0b100) }
            .wutie()
            .set_bit()
            .wute()
            .set_bit()
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
        .write(|w| unsafe { w.ma().bits(PAYLOAD_DOUBLE_BUFFER[1..].as_ptr() as u32) });

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

    // DMA channel 3 SPI1 TX
    dp.DMA1
        .ch3()
        .par()
        .write(|w| unsafe { w.pa().bits(SPI1_DR) });

    // Set 11us interval
    dp.TIM6.arr().write(|w| unsafe { w.arr().bits(3) }); // 200khz * 11us approx. 3

    // Enable TIM6 update interrupt
    dp.TIM6.dier().write(|w| w.uie().set_bit());

    // TIM16: wait for nRF24L01 to power up
    dp.TIM16.psc().write(|w| unsafe { w.bits(299) });
    dp.TIM16.arr().write(|w| unsafe { w.arr().bits(1) }); // 200khz / (299 + 1) * 1.5ms
    dp.TIM16.dier().write(|w| w.uie().set_bit());

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

    // Initialization commands
    let commands = COMMANDS.get();
    commands.enqueue(&SETUP_RETR);
    commands.enqueue(&RF_CH);
    commands.enqueue(&RF_SETUP);
    commands.enqueue(&RX_ADDR_P0);
    commands.enqueue(&TX_ADDR);
    commands.enqueue(&ACTIVATE);
    commands.enqueue(&FEATURE);
    commands.enqueue(&DYNPD);
    commands.enqueue(&POWER_UP);
    commands.enqueue(&HANDSHAKE);

    unsafe {
        // Unmask NVIC global interrupts
        cortex_m::peripheral::NVIC::unmask(Interrupt::RTC_WKUP);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH2);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH3);
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CH6);
        cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI1);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM6_DACUNDER);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM1_UP_TIM16);
    }
    CORE_PERIPHERALS.set(cp);
    DEVICE_PERIPHERALS.set(dp);

    let dp = DEVICE_PERIPHERALS.get();

    send_command(&SETUP_AW, dp);

    loop {
        asm::wfi();
    }
}
