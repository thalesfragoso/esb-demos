#![no_std]
#![no_main]

// We need to import this crate explicitly so we have a panic handler
use panic_semihosting as _;

/// Configuration macro to be called by the user configuration in `config.rs`.
///
/// Expands to yet another `apply_config!` macro that's called from `init` and performs some
/// hardware initialization based on the config values.
macro_rules! config {
    (
        baudrate = $baudrate:ident;
        tx_pin = $tx_pin:ident;
        rx_pin = $rx_pin:ident;
    ) => {
        macro_rules! apply_config {
            ( $p0:ident, $uart:ident ) => {{
                let rxd = $p0.$rx_pin.into_floating_input().degrade();
                let txd = $p0.$tx_pin.into_push_pull_output(Level::Low).degrade();

                let pins = hal::uarte::Pins {
                    rxd,
                    txd,
                    cts: None,
                    rts: None,
                };

                hal::uarte::Uarte::new($uart, pins, Parity::EXCLUDED, Baudrate::$baudrate)
            }};
        }
    };
}

#[macro_use]
mod config;

// Import the right HAL/PAC crate, depending on the target chip
#[cfg(feature = "51")]
use nrf51_hal as hal;
#[cfg(feature = "52810")]
use nrf52810_hal as hal;
#[cfg(feature = "52832")]
use nrf52832_hal as hal;
#[cfg(feature = "52840")]
use nrf52840_hal as hal;

use {
    core::{
        default::Default,
        fmt::Write,
        sync::atomic::{AtomicBool, Ordering},
    },
    esb::{
        consts::*, Addresses, BBBuffer, Config, ConstBBBuffer, Error, EsbApp, EsbBuffer, EsbHeader,
        EsbIrq, IrqTimer,
    },
    hal::{
        gpio::Level,
        pac::{TIMER0, TIMER1, UARTE0},
        uarte::{Baudrate, Parity, Uarte},
    },
};

const MAX_PAYLOAD_SIZE: u8 = 64;
const MSG: &'static str = "Hello from PTX";

static DELAY_FLAG: AtomicBool = AtomicBool::new(false);

#[rtfm::app(device = crate::hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        esb_app: EsbApp<U512, U256>,
        esb_irq: EsbIrq<U512, U256, TIMER0>,
        esb_timer: IrqTimer<TIMER0>,
        serial: Uarte<UARTE0>,
        delay: TIMER1,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        let _clocks = hal::clocks::Clocks::new(ctx.device.CLOCK).enable_ext_hfosc();

        let p0 = hal::gpio::p0::Parts::new(ctx.device.P0);

        let uart = ctx.device.UARTE0;
        let mut serial = apply_config!(p0, uart);
        writeln!(serial, "\n--- INIT ---").unwrap();

        static BUFFER: EsbBuffer<U512, U256> = EsbBuffer {
            app_to_radio_buf: BBBuffer(ConstBBBuffer::new()),
            radio_to_app_buf: BBBuffer(ConstBBBuffer::new()),
            timer_flag: AtomicBool::new(false),
        };
        let addresses = Addresses::default();
        let config = Config::default();
        let (esb_app, esb_irq, esb_timer) = BUFFER
            .try_split(
                ctx.device.TIMER0,
                ctx.device.RADIO,
                addresses,
                MAX_PAYLOAD_SIZE,
                config,
            )
            .unwrap();

        // setup timer for delay
        let timer = ctx.device.TIMER1;
        timer.bitmode.write(|w| w.bitmode()._32bit());

        // 16 Mhz / 2**9 = 31250 Hz
        timer.prescaler.write(|w| unsafe { w.prescaler().bits(9) });
        timer.shorts.modify(|_, w| w.compare0_clear().enabled());
        timer.cc[0].write(|w| unsafe { w.bits(31250u32) });
        timer.events_compare[0].reset();
        timer.intenset.write(|w| w.compare0().set());

        // Clears and starts the counter
        timer.tasks_clear.write(|w| unsafe { w.bits(1) });
        timer.tasks_start.write(|w| unsafe { w.bits(1) });

        init::LateResources {
            esb_app,
            esb_irq,
            esb_timer,
            serial,
            delay: timer,
        }
    }

    #[idle(resources = [serial, esb_app])]
    fn idle(ctx: idle::Context) -> ! {
        let esb_header = EsbHeader::build()
            .max_payload(MAX_PAYLOAD_SIZE)
            .pid(0)
            .pipe(0)
            .no_ack(false)
            .check()
            .unwrap();
        loop {
            // Do we received any packet ?
            if let Some(packet) = ctx.resources.esb_app.read_packet() {
                ctx.resources.serial.write(b"Payload: ").unwrap();
                ctx.resources.serial.write(&packet[..]).unwrap();
                ctx.resources.serial.write(b" rssi: ").unwrap();
                ctx.resources
                    .serial
                    .write(&[packet.get_header().rssi()])
                    .unwrap();
                ctx.resources.serial.write(b"\n").unwrap();
                packet.release();
            }

            writeln!(ctx.resources.serial, "--- Sending Hello ---\n").unwrap();
            let mut packet = ctx.resources.esb_app.grant_packet(esb_header).unwrap();
            let length = MSG.as_bytes().len();
            &packet[..length].copy_from_slice(MSG.as_bytes());
            packet.commit(length);
            ctx.resources.esb_app.start_tx();

            while !DELAY_FLAG.load(Ordering::Acquire) {}
            DELAY_FLAG.store(false, Ordering::Release);
            //hprintln!("Tick").unwrap();
        }
    }

    #[task(binds = RADIO, resources = [esb_irq], priority = 3)]
    fn radio(ctx: radio::Context) {
        match ctx.resources.esb_irq.radio_interrupt() {
            Err(Error::MaximumAttempts) => {}
            Err(e) => panic!("Found error {:?}", e),
            _ => {}
        }
    }

    #[task(binds = TIMER0, resources = [esb_timer], priority = 3)]
    fn timer0(ctx: timer0::Context) {
        ctx.resources.esb_timer.timer_interrupt();
    }

    #[task(binds = TIMER1, resources = [delay], priority = 1)]
    fn timer1(ctx: timer1::Context) {
        ctx.resources.delay.events_compare[0].reset();
        DELAY_FLAG.store(true, Ordering::Release);
    }
};
