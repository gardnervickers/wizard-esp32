#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use esp32_hal::entry;
use esp32_hal::gpio::{AnyPin, Output, PushPull};
use esp_backtrace as _;

use esp_println::println;

use embassy_executor::Executor;
use embassy_time::{Duration, Timer};
use esp32_hal::{
    clock::ClockControl, embassy, peripherals::Peripherals, prelude::*, timer::TimerGroup, Rtc, IO,
};
use esp_backtrace as _;
use static_cell::StaticCell;

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::task]
async fn blink(mut led: AnyPin<Output<PushPull>>) {
    let mut nr_pings = 0;

    loop {
        led.toggle().unwrap();
        esp_println::println!("Ping {nr_pings}!");
        Timer::after(Duration::from_millis(500)).await;
        nr_pings += 1;
    }
}

#[entry]
fn main() -> ! {
    println!("Starting");
    let peripherals = Peripherals::take();
    let system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    embassy::init(&clocks, timer_group0.timer0);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let led1 = io.pins.gpio15.into_push_pull_output().degrade();

    /*    // Async requires the GPIO interrupt to wake futures
    esp32_hal::interrupt::enable(
        esp32_hal::peripherals::Interrupt::GPIO,
        esp32_hal::interrupt::Priority::Priority1,
    )
    .unwrap(); */

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(blink(led1)).ok();
    });
}
