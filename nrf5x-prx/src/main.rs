#![no_std]
#![no_main]

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

                let pins = uarte::Pins {
                    rxd,
                    txd,
                    cts: None,
                    rts: None,
                };

                Uarte::new($uart, pins, Parity::EXCLUDED, Baudrate::$baudrate)
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
        panic::PanicInfo,
        sync::atomic::{compiler_fence, AtomicBool, Ordering},
    },
    esb::{
        consts::*, irq::StatePRX, Addresses, BBBuffer, ConfigBuilder, ConstBBBuffer, Error, EsbApp,
        EsbBuffer, EsbHeader, EsbIrq, IrqTimer,
    },
    hal::{gpio::Level, pac::TIMER0},
    rtt_target::{rprintln, rtt_init_print},
};

#[cfg(not(feature = "51"))]
use hal::{
    pac::UARTE0,
    uarte::{self, Baudrate, Parity, Uarte},
};

#[cfg(feature = "51")]
use hal::{
    pac::UART0 as UARTE0,
    uart::{self as uarte, Baudrate, Parity, Uart as Uarte},
};

const MAX_PAYLOAD_SIZE: u8 = 64;
const RESP: &'static str = "Hello back";

#[rtfm::app(device = crate::hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        esb_app: EsbApp<U1024, U1024>,
        esb_irq: EsbIrq<U1024, U1024, TIMER0, StatePRX>,
        esb_timer: IrqTimer<TIMER0>,
        serial: Uarte<UARTE0>,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        let _clocks = hal::clocks::Clocks::new(ctx.device.CLOCK).enable_ext_hfosc();
        rtt_init_print!();

        #[cfg(not(feature = "51"))]
        let p0 = hal::gpio::p0::Parts::new(ctx.device.P0);

        #[cfg(feature = "51")]
        let p0 = hal::gpio::p0::Parts::new(ctx.device.GPIO);

        #[cfg(not(feature = "51"))]
        let uart = ctx.device.UARTE0;

        #[cfg(feature = "51")]
        let uart = ctx.device.UART0;

        let mut serial = apply_config!(p0, uart);
        writeln!(serial, "\n--- INIT ---").unwrap();

        static BUFFER: EsbBuffer<U1024, U1024> = EsbBuffer {
            app_to_radio_buf: BBBuffer(ConstBBBuffer::new()),
            radio_to_app_buf: BBBuffer(ConstBBBuffer::new()),
            timer_flag: AtomicBool::new(false),
        };
        let addresses = Addresses::default();
        let config = ConfigBuilder::default()
            .maximum_transmit_attempts(1)
            .max_payload_size(MAX_PAYLOAD_SIZE)
            .check()
            .unwrap();
        let (esb_app, esb_irq, esb_timer) = BUFFER
            .try_split(ctx.device.TIMER0, ctx.device.RADIO, addresses, config)
            .unwrap();
        let mut esb_irq = esb_irq.into_prx();
        esb_irq.start_receiving().unwrap();

        init::LateResources {
            esb_app,
            esb_irq,
            esb_timer,
            serial,
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
            // Did we receive any packet ?
            if let Some(packet) = ctx.resources.esb_app.read_packet() {
                //let payload = core::str::from_utf8(&packet[..]).unwrap();

                //ctx.resources.serial.write_str("Payload: ").unwrap();
                //let payload = core::str::from_utf8(&packet[..]).unwrap();
                //ctx.resources.serial.write_str(payload).unwrap();
                //ctx.resources.serial.write_str(" rssi: ").unwrap();
                //ctx.resources
                //    .serial
                //    .write(packet.get_header().rssi())
                //    .unwrap();
                //ctx.resources.serial.write(b'\n').unwrap();
                packet.release();

                // Respond in the next transfer

                let mut response = ctx.resources.esb_app.grant_packet(esb_header).unwrap();
                let length = RESP.as_bytes().len();
                &response[..length].copy_from_slice(RESP.as_bytes());
                response.commit(length);
            }
        }
    }

    #[task(binds = RADIO, resources = [esb_irq], priority = 3)]
    fn radio(ctx: radio::Context) {
        match ctx.resources.esb_irq.radio_interrupt() {
            Err(Error::MaximumAttempts) => {}
            Err(e) => panic!("Found error {:?}", e),
            Ok(_) => {} //rprintln!("{:?}", state).unwrap(),
        }
    }

    #[task(binds = TIMER0, resources = [esb_timer], priority = 3)]
    fn timer0(ctx: timer0::Context) {
        ctx.resources.esb_timer.timer_interrupt();
    }
};

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    rprintln!("{}", info);
    loop {
        compiler_fence(Ordering::SeqCst);
    }
}
