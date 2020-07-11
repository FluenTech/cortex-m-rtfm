#![no_std]
#![no_main]

use core::convert::Infallible;
use core::prelude::v1::*;
use nrf52::prelude::*;
use panic_halt as _;
use rtic::time::{self, units::*};

pub mod nrf52 {
    pub use nrf52832_hal::{
        gpio, prelude,
        target::{self as pac, Peripherals},
    };
}

pub struct SysClock {
    low: nrf52::pac::TIMER0,
    high: nrf52::pac::TIMER1,
    capture_task: nrf52::pac::EGU0,
}

impl SysClock {
    pub fn take(
        low: nrf52::pac::TIMER0,
        high: nrf52::pac::TIMER1,
        capture_task: nrf52::pac::EGU0,
    ) -> Self {
        Self {
            low,
            high,
            capture_task,
        }
    }
}

impl time::Clock for SysClock {
    type Rep = u64;
    type ImplError = Infallible;
    const PERIOD: time::Period = <time::Period>::new(1, 16_000_000);

    fn now(&self) -> Result<time::Instant<Self>, time::clock::Error<Self::ImplError>> {
        self.capture_task.tasks_trigger[0].write(|write| unsafe { write.bits(1) });

        let ticks =
            self.low.cc[0].read().bits() as u64 | ((self.high.cc[0].read().bits() as u64) << 32);

        Ok(time::Instant::new(ticks as Self::Rep))
    }
}

impl rtic::Monotonic for SysClock {
    unsafe fn reset() {
        (*nrf52::pac::TIMER0::ptr())
            .tasks_clear
            .write(|write| write.bits(1));
        (*nrf52::pac::TIMER1::ptr())
            .tasks_clear
            .write(|write| write.bits(1));
    }
}

const LED_ON_TIME: Milliseconds<u32> = Milliseconds(250);

#[rtic::app(device = nrf52832_hal::target, peripherals = true, monotonic = crate::SysClock, sys_timer_freq = 64_000_000)]
const APP: () = {
    struct Resources {
        clock: crate::SysClock,
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
                w.bits(
                    &cx.device.TIMER0.events_compare[1] as *const nrf52::pac::generic::Reg<_, _>
                        as u32,
                )
            });
            cx.device.PPI.ch[0].tep.write(|w| {
                w.bits(
                    &cx.device.TIMER1.tasks_count as *const nrf52::pac::generic::Reg<_, _> as u32,
                )
            });
            cx.device.PPI.chen.modify(|_, w| w.ch0().enabled());

            cx.device.PPI.ch[1].eep.write(|w| {
                w.bits(
                    &cx.device.EGU0.events_triggered[0] as *const nrf52::pac::generic::Reg<_, _>
                        as u32,
                )
            });
            cx.device.PPI.ch[1].tep.write(|w| {
                w.bits(
                    &cx.device.TIMER0.tasks_capture[0] as *const nrf52::pac::generic::Reg<_, _>
                        as u32,
                )
            });
            cx.device.PPI.fork[1].tep.write(|w| {
                w.bits(
                    &cx.device.TIMER1.tasks_capture[0] as *const nrf52::pac::generic::Reg<_, _>
                        as u32,
                )
            });
            cx.device.PPI.chen.modify(|_, w| w.ch1().enabled());

            cx.device.TIMER0.tasks_start.write(|write| write.bits(1));
            cx.device.TIMER1.tasks_start.write(|write| write.bits(1));
        }

        // This moves these peripherals to prevent conflicting usage, however not the entire EGU0 is
        // used. A ref to EGU0 could be sent instead, although that provides no protection for the
        // fields that are being used by the clock.
        let clock = SysClock::take(cx.device.TIMER0, cx.device.TIMER1, cx.device.EGU0);

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
            clock,
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
