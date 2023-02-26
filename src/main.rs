#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;

mod display;

use embassy_executor::Executor;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Ticker, Timer};

use esp32_hal::gpio::{self};
use esp32_hal::i2c::I2C;
use esp32_hal::peripherals::Peripherals;
use esp32_hal::{clock, embassy, prelude::*, timer, Rng, Rtc};
use esp32_hal::{ledc, IO};
use esp_backtrace as _;
use esp_println::logger::init_logger;

use esp_wifi::esp_now::{PeerInfo, BROADCAST_ADDRESS};
use futures_util::StreamExt;
use log::info;
use static_cell::StaticCell;

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
    led: gpio::Gpio15<gpio::Output<gpio::PushPull>>,
    mut display: display::DisplayController,
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

    let mut current = 11;
    loop {
        for duty in 0..100 {
            channel0.set_duty(duty).unwrap();
            Timer::after(Duration::from_millis(current)).await;
        }
        for duty in (0..100).rev() {
            channel0.set_duty(duty).unwrap();
            Timer::after(Duration::from_millis(current)).await;
        }
        if current > 0 {
            current -= 1;
            let message = alloc::format!("LifeTotal: {current}");
            display.draw_message(&message).unwrap();
        } else {
            display.draw_message("rekt").unwrap();

            channel0.set_duty(0).unwrap();
            Timer::after(Duration::from_secs(5)).await;
            current = 11;
        }
    }
}

/// Initialize all the things
fn init_all() {
    init_logger(log::LevelFilter::Info);
    esp_wifi::init_heap();
    init_heap();
}

#[entry]
fn main() -> ! {
    init_all();
    log::info!("Starting");
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = &*singleton!(clock::ClockControl::configure(
        system.clock_control,
        clock::CpuClock::Clock240MHz
    )
    .freeze());
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    rtc.rwdt.disable();

    info!("creating timer groups");
    // Create a timer group which will be used to initialize the executor and wifi timers.
    let mut timg0 = timer::TimerGroup::new(peripherals.TIMG0, clocks);
    let mut timg1 = timer::TimerGroup::new(peripherals.TIMG1, clocks);
    info!("disabling watchdog timers");
    timg0.wdt.disable();
    timg1.wdt.disable();

    info!("initializing esp_wifi");
    // Initialize esp-wifi with the timer group.
    esp_wifi::initialize(timg1.timer0, Rng::new(peripherals.RNG), clocks).unwrap();
    // Initialize embassy with the timer group.
    info!("initializing embassy");
    embassy::init(clocks, timg0.timer0);
    info!("initializing esp_now");
    let esp_now = esp_wifi::esp_now::esp_now().initialize().unwrap();

    info!("started esp_now version={}", esp_now.get_version().unwrap());

    info!("initializing executor");
    let executor = EXECUTOR.init(Executor::new());
    // Async requires the GPIO interrupt to wake futures
    /* info!("setting up interrupt handling");
    esp32_hal::interrupt::enable(
        esp32_hal::peripherals::Interrupt::GPIO,
        esp32_hal::interrupt::Priority::Priority1,
    )
    .unwrap(); */

    info!("creating LEDC PWM controller");
    let ledc = &*singleton!(ledc::LEDC::new(
        peripherals.LEDC,
        clocks,
        &mut system.peripheral_clock_control,
    ));

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Setup our analog pin.
    info!("registering PWM LED pin to GPIO 15");
    let led = io.pins.gpio15.into_push_pull_output();

    info!("creating i2C controller sda=22, scl=21");
    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio22,
        io.pins.gpio21,
        100u32.kHz(),
        &mut system.peripheral_clock_control,
        clocks,
    );

    info!("starting display");
    let mut display = display::DisplayController::from_i2c(i2c).unwrap();
    display.draw_message("starting up").unwrap();

    executor.run(move |spawner| {
        spawner.must_spawn(blink(ledc, led, display));
        spawner.must_spawn(receiver(esp_now))
    });
}

#[embassy_executor::task]
async fn receiver(mut esp_now: esp_wifi::esp_now::EspNow) {
    let mut ticker = Ticker::every(Duration::from_secs(5));
    loop {
        let res = select(ticker.next(), async {
            let r = esp_now.receive_async().await;
            info!("Received {:x?}", r);
            if r.info.dst_address == BROADCAST_ADDRESS {
                if !esp_now.peer_exists(&r.info.src_address).unwrap() {
                    esp_now
                        .add_peer(PeerInfo {
                            peer_address: r.info.src_address,
                            lmk: None,
                            channel: None,
                            encrypt: false,
                        })
                        .unwrap();
                }
                esp_now.send(&r.info.src_address, b"Hello Peer").unwrap();
            }
        })
        .await;

        match res {
            Either::First(_) => {
                info!("Send");
                esp_now.send(&BROADCAST_ADDRESS, b"0123456789").unwrap();
            }
            Either::Second(_) => (),
        }
    }
}
