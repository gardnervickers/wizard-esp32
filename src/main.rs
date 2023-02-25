#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;

use alloc::rc::Rc;
use embassy_executor::Executor;
use embassy_time::{Duration, Timer};
use esp32_hal::gpio::{self, AnyPin, Output, PushPull};
use esp32_hal::i2c::{self, I2C};
use esp32_hal::peripherals::{Peripherals, I2C0};
use esp32_hal::{clock, embassy, prelude::*, Rtc};
use esp32_hal::{ledc, IO};
use esp_backtrace as _;
use esp_println::println;
use ssd1306::mode::{BufferedGraphicsMode, DisplayConfig};
use ssd1306::prelude::*;
use ssd1306::rotation::DisplayRotation;
use ssd1306::size::DisplaySize128x64;
use ssd1306::{I2CDisplayInterface, Ssd1306};
use static_cell::StaticCell;

use embedded_graphics::{
    mono_font::{
        ascii::{FONT_6X10, FONT_9X18_BOLD},
        MonoTextStyleBuilder,
    },
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Alignment, Text},
};

macro_rules! singleton {
    ($val:expr) => {{
        type T = impl Sized;
        static STATIC_CELL: StaticCell<T> = StaticCell::new();
        let (x,) = STATIC_CELL.init(($val,));
        x
    }};
}

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;

    extern "C" {
        static mut _heap_start: u32;
        static mut _heap_end: u32;
    }
    unsafe {
        let heap_start = &_heap_start as *const _ as usize;
        let heap_end = &_heap_end as *const _ as usize;
        assert!(
            heap_end - heap_start > HEAP_SIZE,
            "Not enough available heap memory."
        );
        ALLOCATOR.init(heap_start as *mut u8, HEAP_SIZE);
    }
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::task]
async fn blink(
    ledc: &'static ledc::LEDC<'_>,
    mut led: gpio::GpioPin<
        Output<PushPull>,
        gpio::Bank0GpioRegisterAccess,
        gpio::DualCoreInteruptStatusRegisterAccessBank0,
        gpio::InputOutputAnalogPinType,
        gpio::Gpio15Signals,
        15,
    >,
    display: &'static mut Ssd1306<
        I2CInterface<I2C<'_, I2C0>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    >,
) {
    let mut hstimer0 = ledc.get_timer::<ledc::HighSpeed>(ledc::timer::Number::Timer0);
    hstimer0
        .configure(ledc::timer::config::Config {
            duty: ledc::timer::config::Duty::Duty5Bit,
            clock_source: ledc::timer::HSClockSource::APBClk,
            frequency: 24u32.kHz(),
        })
        .unwrap();
    let mut channel0 = ledc.get_channel(ledc::channel::Number::Channel0, led);
    channel0
        .configure(ledc::channel::config::Config {
            timer: &hstimer0,
            duty_pct: 10,
        })
        .unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_9X18_BOLD)
        .text_color(BinaryColor::On)
        .build();

    let mut iterations = 0;
    loop {
        Text::with_alignment(
            alloc::format!("hello world {}", iterations).as_str(),
            display.bounding_box().center() + Point::new(0, 14),
            text_style,
            Alignment::Center,
        )
        .draw(display)
        .unwrap();
        // Write buffer to display
        display.flush().unwrap();
        // Clear display buffer
        display.clear();
        iterations += 1;

        for duty in 0..100 {
            channel0.set_duty(duty).unwrap();
            Timer::after(Duration::from_millis(10)).await;
        }
        for duty in (0..100).rev() {
            channel0.set_duty(duty).unwrap();
            Timer::after(Duration::from_millis(10)).await;
        }
    }
}

#[entry]
fn main() -> ! {
    init_heap();
    println!("Starting");
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = &*singleton!(clock::ClockControl::boot_defaults(system.clock_control).freeze());
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = esp32_hal::timer::TimerGroup::new(peripherals.TIMG0, clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = esp32_hal::timer::TimerGroup::new(peripherals.TIMG1, clocks);
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    // Setup embassy
    embassy::init(clocks, timer_group0.timer0);
    let executor = EXECUTOR.init(Executor::new());
    // Async requires the GPIO interrupt to wake futures
    esp32_hal::interrupt::enable(
        esp32_hal::peripherals::Interrupt::GPIO,
        esp32_hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let ledc = &*singleton!(ledc::LEDC::new(
        peripherals.LEDC,
        clocks,
        &mut system.peripheral_clock_control,
    ));

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let led = io.pins.gpio15.into_push_pull_output();
    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio22,
        io.pins.gpio21,
        100u32.kHz(),
        &mut system.peripheral_clock_control,
        clocks,
    );

    let interface = I2CDisplayInterface::new(i2c);
    let display = singleton!(
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode()
    );
    display.init().unwrap();

    display.bounding_box();

    // Specify different text styles
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    let text_style_big = MonoTextStyleBuilder::new()
        .font(&FONT_9X18_BOLD)
        .text_color(BinaryColor::On)
        .build();

    // Fill display bufffer with a centered text with two lines (and two text
    // styles)

    Text::with_alignment(
        "esp-hal",
        display.bounding_box().center() + Point::new(0, 0),
        text_style_big,
        Alignment::Center,
    )
    .draw(display)
    .unwrap();

    Text::with_alignment(
        "Chip: ESP32",
        display.bounding_box().center() + Point::new(0, 14),
        text_style,
        Alignment::Center,
    )
    .draw(display)
    .unwrap();

    // Write buffer to display
    display.flush().unwrap();
    // Clear display buffer
    display.clear();

    // The silkscreen is lying to me, the address is actually 0x3c
    //i2c.write(0x3C, "hi".as_bytes()).unwrap();

    executor.run(move |spawner| {
        spawner.must_spawn(blink(ledc, led, display));
    });
}
