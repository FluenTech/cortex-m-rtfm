#![no_std]
#![no_main]

extern crate panic_rtt;

use core::borrow::Borrow;
use core::convert;
use core::prelude::v1::*;
use nrf52::prelude::*;
use rtic::export::interrupt::Mutex;
use rtic::time::{self, units::*, Instant, Period};

pub mod nrf52 {
    pub use nrf52832_hal::gpio;
    pub use nrf52832_hal::prelude;
    pub use nrf52832_hal::target as pac;
}

pub struct SystemTime;

impl time::Clock for SystemTime {
    type Rep = i64;
    const PERIOD: Period = Period::new(1, 16_000_000);

    fn now() -> Instant<Self> {
        unsafe {
            (*nrf52::pac::EGU0::ptr()).tasks_trigger[0].write(|write| write.bits(1));
        }
        let ticks = unsafe {
            (*nrf52::pac::TIMER0::ptr()).cc[0].read().bits() as u64
                | (((*nrf52::pac::TIMER1::ptr()).cc[0].read().bits() as u64) << 32)
        };

        Instant::new(ticks as Self::Rep)
    }

    fn wake_at(instant: Instant<Self>) {
        panic!();
        unsafe {
            (*nrf52::pac::TIMER0::ptr()).cc[3]
                .write(|writer| writer.bits(instant.ticks_since_epoch() as u32));
        }
        Self::enable_interrupt();
    }

    fn enable_interrupt() {
        unsafe {
            (*nrf52::pac::TIMER0::ptr())
                .intenset
                .write(|writer| writer.compare3().set_bit());
        }
    }

    fn disable_interrupt() {
        unsafe {
            (*nrf52::pac::TIMER0::ptr())
                .intenclr
                .write(|writer| writer.compare3().set_bit())
        }
    }

    fn pend_interrupt() {
        unsafe {
            // (*nrf52::pac::EGU0::ptr()).tasks_trigger[1].write(|write| write.bits(1));
            (*nrf52::pac::TIMER0::ptr()).events_compare[3].write(|writer| writer.bits(1))
        }
    }
}

impl rtic::Monotonic for SystemTime {
    unsafe fn reset() {
        (*nrf52::pac::TIMER0::ptr())
            .tasks_clear
            .write(|write| write.bits(1));
        (*nrf52::pac::TIMER1::ptr())
            .tasks_clear
            .write(|write| write.bits(1));
    }
}

const LED_ON_TIME: Milliseconds<i32> = Milliseconds(250);

#[rtic::app(device = nrf52832_hal::target, peripherals = true, monotonic = crate::SystemTime)]
const APP: () = {
    struct Resources {
        led1: nrf52::gpio::p0::P0_17<nrf52::gpio::Output<nrf52::gpio::OpenDrain>>,
        led2: nrf52::gpio::p0::P0_18<nrf52::gpio::Output<nrf52::gpio::OpenDrain>>,
        led3: nrf52::gpio::p0::P0_19<nrf52::gpio::Output<nrf52::gpio::OpenDrain>>,
        led4: nrf52::gpio::p0::P0_20<nrf52::gpio::Output<nrf52::gpio::OpenDrain>>,
    }

    #[init(spawn = [turn_on_led1])]
    fn init(cx: init::Context) -> init::LateResources {
        cx.spawn.turn_on_led1().unwrap();

        cx.device.TIMER0.mode.write(|w| w.mode().timer());
        cx.device.TIMER0.bitmode.write(|w| w.bitmode()._32bit());
        cx.device
            .TIMER0
            .prescaler
            .write(|w| unsafe { w.prescaler().bits(0) });
        cx.device.TIMER0.cc[1].write(|w| unsafe { w.bits(0xFFFF_FFFF) });
        cx.device
            .TIMER1
            .mode
            .write(|w| w.mode().low_power_counter());
        cx.device.TIMER1.bitmode.write(|w| w.bitmode()._32bit());
        cx.device
            .TIMER1
            .prescaler
            .write(|w| unsafe { w.prescaler().bits(0) });

        unsafe {
            cx.device.PPI.ch[0].eep.write(|w| {
                w.bits(cx.device.TIMER0.events_compare[1].borrow()
                    as *const nrf52::pac::generic::Reg<_, _> as u32)
            });
            cx.device.PPI.ch[0].tep.write(|w| {
                w.bits(cx.device.TIMER1.tasks_count.borrow()
                    as *const nrf52::pac::generic::Reg<_, _> as u32)
            });
            cx.device.PPI.chen.modify(|_, w| w.ch0().enabled());

            cx.device.PPI.ch[1].eep.write(|w| {
                w.bits(cx.device.EGU0.events_triggered[0].borrow()
                    as *const nrf52::pac::generic::Reg<_, _> as u32)
            });
            cx.device.PPI.ch[1].tep.write(|w| {
                w.bits(cx.device.TIMER0.tasks_capture[0].borrow()
                    as *const nrf52::pac::generic::Reg<_, _> as u32)
            });
            cx.device.PPI.fork[1].tep.write(|w| {
                w.bits(cx.device.TIMER1.tasks_capture[0].borrow()
                    as *const nrf52::pac::generic::Reg<_, _> as u32)
            });
            cx.device.PPI.chen.modify(|_, w| w.ch1().enabled());

            // cx.device.PPI.ch[2].eep.write(|w| {
            //     w.bits(cx.device.EGU0.events_triggered[1].borrow()
            //         as *const nrf52::pac::generic::Reg<_, _> as u32)
            // });
            // cx.device.PPI.ch[2].tep.write(|w| {
            //     w.bits(cx.device.TIMER0.tasks_capture[3].borrow()
            //         as *const nrf52::pac::generic::Reg<_, _> as u32)
            // });
            // cx.device.PPI.chen.modify(|_, w| w.ch2().enabled());

            cx.device.TIMER0.tasks_start.write(|write| write.bits(1));
            cx.device.TIMER1.tasks_start.write(|write| write.bits(1));
        }

        let port0 = nrf52::gpio::p0::Parts::new(cx.device.P0);

        let led1 = port0.p0_17.into_open_drain_output(
            nrf52::gpio::OpenDrainConfig::Standard0Disconnect1,
            nrf52::gpio::Level::High,
        );

        let led2 = port0.p0_18.into_open_drain_output(
            nrf52::gpio::OpenDrainConfig::Standard0Disconnect1,
            nrf52::gpio::Level::High,
        );

        let led3 = port0.p0_19.into_open_drain_output(
            nrf52::gpio::OpenDrainConfig::Standard0Disconnect1,
            nrf52::gpio::Level::High,
        );

        let led4 = port0.p0_20.into_open_drain_output(
            nrf52::gpio::OpenDrainConfig::Standard0Disconnect1,
            nrf52::gpio::Level::High,
        );

        init::LateResources {
            led1,
            led2,
            led3,
            led4,
        }
    }

    #[task(resources = [led1], schedule = [turn_off_led1])]
    fn turn_on_led1(cx: turn_on_led1::Context) {
        let led1 = cx.resources.led1;
        led1.set_low().unwrap();

        cx.schedule
            .turn_off_led1(cx.scheduled + LED_ON_TIME)
            .unwrap();
    }

    #[task(resources = [led1], spawn = [turn_on_led2])]
    fn turn_off_led1(cx: turn_off_led1::Context) {
        let led1 = cx.resources.led1;
        led1.set_high().unwrap();

        cx.spawn.turn_on_led2().unwrap();
    }

    #[task(resources = [led2], schedule = [turn_off_led2])]
    fn turn_on_led2(cx: turn_on_led2::Context) {
        cx.resources.led2.set_low().unwrap();

        cx.schedule
            .turn_off_led2(cx.scheduled + LED_ON_TIME)
            .unwrap();
    }

    #[task(resources = [led2], spawn = [turn_on_led4])]
    fn turn_off_led2(cx: turn_off_led2::Context) {
        cx.resources.led2.set_high().unwrap();

        cx.spawn.turn_on_led4().unwrap();
    }

    #[task(resources = [led3], schedule = [turn_off_led3])]
    fn turn_on_led3(cx: turn_on_led3::Context) {
        cx.resources.led3.set_low().unwrap();

        cx.schedule
            .turn_off_led3(cx.scheduled + LED_ON_TIME)
            .unwrap();
    }

    #[task(resources = [led3], spawn = [turn_on_led1])]
    fn turn_off_led3(cx: turn_off_led3::Context) {
        cx.resources.led3.set_high().unwrap();

        cx.spawn.turn_on_led1().unwrap();
    }

    #[task(resources = [led4], schedule = [turn_off_led4])]
    fn turn_on_led4(cx: turn_on_led4::Context) {
        cx.resources.led4.set_low().unwrap();

        cx.schedule
            .turn_off_led4(cx.scheduled + LED_ON_TIME)
            .unwrap();
    }

    #[task(resources = [led4], spawn = [turn_on_led3])]
    fn turn_off_led4(cx: turn_off_led4::Context) {
        cx.resources.led4.set_high().unwrap();

        cx.spawn.turn_on_led3().unwrap();
    }

    // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn UARTE0_UART0();
        fn RTC0();
    }
};
