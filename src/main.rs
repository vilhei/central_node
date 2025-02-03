#![no_std]
#![no_main]

use embedded_graphics::{
    geometry::AnchorPoint,
    pixelcolor::Rgb565,
    prelude::{DrawTarget, RgbColor},
};
use u8g2_fonts::types::{FontColor, HorizontalAlignment, VerticalPosition};

#[rtic::app(device=esp32c3, dispatchers = [FROM_CPU_INTR0])]
mod app {
    use core::cell::RefCell;

    use central_node::{SensorData, FONT0_NORMAL, FONT0_SMALL, FONT0_SMALLER};
    use esp_backtrace as _;
    use esp_hal::{
        clock::CpuClock,
        delay::Delay,
        gpio::{Level, Output},
        peripherals::TIMG1,
        rng::Rng,
        spi::{master::Spi, SpiMode},
        timer::{
            timg::{Timer, TimerGroup, TimerX},
            OneShotTimer, PeriodicTimer,
        },
        Blocking,
    };
    use esp_println::println;
    use esp_wifi::{esp_now::EspNow, EspWifiController};
    use fugit::{Duration, ExtU64, RateExtU32};
    use mipidsi::{
        interface::SpiInterface,
        models::ST7735s,
        options::{Orientation, Rotation},
    };
    use static_cell::{ConstStaticCell, StaticCell};

    use embedded_hal_bus::spi::CriticalSectionDevice;

    use embedded_graphics::{geometry::AnchorPoint, pixelcolor::Rgb565, prelude::*};
    use u8g2_fonts::types::{
        FontColor::{Transparent, WithBackground},
        HorizontalAlignment, VerticalPosition,
    };

    use crate::{render_received, render_time};

    static ESP_WIFI_CONTROLLER: StaticCell<EspWifiController> = StaticCell::new();
    static SPI_BUS: StaticCell<critical_section::Mutex<RefCell<Spi<Blocking>>>> = StaticCell::new();

    static DISPLAY0_BUFFER: ConstStaticCell<[u8; 512]> = ConstStaticCell::new([0u8; 512]);

    #[shared]
    struct Shared {
        // temp: f32,
        // hum: f32,
        filament_box_sensor: SensorData<f32, 2>,
    }

    #[local]
    struct Local {
        esp_now: EspNow<'static>,
        display0: mipidsi::Display<
            SpiInterface<
                'static,
                CriticalSectionDevice<'static, Spi<'static, Blocking>, Output<'static>, Delay>,
                Output<'static>,
            >,
            ST7735s,
            Output<'static>,
        >,
        delay: Delay,
        display_timer: OneShotTimer<'static, Timer<TimerX<TIMG1>, Blocking>>,
    }

    #[init]
    fn init(_cx: init::Context) -> (Shared, Local) {
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::Clock160MHz;
        let peripherals = esp_hal::init(config);
        let timg0 = TimerGroup::new(peripherals.TIMG0);

        esp_alloc::heap_allocator!(72 * 1024);

        let init: &'static mut EspWifiController = ESP_WIFI_CONTROLLER.init_with(|| {
            esp_wifi::init(
                timg0.timer0,
                Rng::new(peripherals.RNG),
                peripherals.RADIO_CLK,
            )
            .expect("Failed to initialize wifi")
        });

        let esp_now =
            EspNow::new(init, peripherals.WIFI).expect("Failed to create esp now instance");

        // 4 SCK
        // 3 SDA
        // 2 reset
        // 1 rs/dc
        // 0 cs

        let spi = Spi::new_with_config(
            peripherals.SPI2,
            esp_hal::spi::master::Config {
                frequency: 40u32.MHz(),
                mode: SpiMode::Mode0,
                ..Default::default()
            },
        )
        .with_mosi(peripherals.GPIO3)
        .with_sck(peripherals.GPIO4);

        let cs = Output::new(peripherals.GPIO0, Level::High);

        let spi_bus = SPI_BUS.init(critical_section::Mutex::new(RefCell::new(spi)));

        let mut delay = Delay::new();
        let display0_spi = CriticalSectionDevice::new(spi_bus, cs, delay).unwrap();

        let mut reset = Output::new(peripherals.GPIO2, Level::Low);
        reset.set_high();
        let dc = Output::new(peripherals.GPIO1, Level::Low);

        let display_interface = SpiInterface::new(display0_spi, dc, DISPLAY0_BUFFER.take());

        let mut display0 = mipidsi::Builder::new(ST7735s, display_interface)
            .reset_pin(reset)
            .display_size(128, 160)
            .orientation(Orientation::new().rotate(Rotation::Deg270))
            .init(&mut delay)
            .unwrap();

        display0.clear(Rgb565::BLACK).unwrap();

        receiver::spawn().unwrap();

        let timg1 = TimerGroup::new(peripherals.TIMG1);

        let mut display_timer = OneShotTimer::new(timg1.timer0);
        display_timer.enable_interrupt(true);
        display_timer.schedule(1u64.secs()).unwrap();

        (
            Shared {
                filament_box_sensor: SensorData::new(),
            },
            Local {
                esp_now,
                display0,
                delay,
                display_timer,
            },
        )
    }

    #[task(shared=[filament_box_sensor], local=[esp_now], priority=2)]
    async fn receiver(mut cx: receiver::Context) {
        println!("Entering receiver task");
        loop {
            println!("waiting for message");
            let response = cx.local.esp_now.receive_async().await;

            if let Ok([tmp, hum]) = central_node::parse_float(response.data()) {
                println!("Temperature : {} - Humidity : {}", tmp, hum);
                let time = esp_hal::time::now();
                let time = time.duration_since_epoch().to_secs();
                cx.shared.filament_box_sensor.lock(|t| {
                    t.sample_time = time;
                    t.samples[0] = tmp;
                    t.samples[1] = hum;
                });
            }
        }
    }

    #[task(binds=TG1_T0_LEVEL ,shared=[filament_box_sensor],local=[display_timer, display0, delay], priority=3)]
    fn display_updater(mut cx: display_updater::Context) {
        cx.local.display_timer.clear_interrupt();
        let display0 = cx.local.display0;
        let time = esp_hal::time::now();
        let time = time.duration_since_epoch().to_secs();
        if display0.is_sleeping() {
            display0.wake(cx.local.delay).unwrap();
        }

        let (temp, hum, sample_time) = cx
            .shared
            .filament_box_sensor
            .lock(|f| (f.samples[0], f.samples[1], f.sample_time));

        render_time(time, display0);

        FONT0_NORMAL
            .render_aligned(
                format_args!("{:.1} °C", temp),
                display0
                    .bounding_box()
                    .anchor_point(AnchorPoint::CenterLeft)
                    + Point::new(0, -10),
                VerticalPosition::Center,
                HorizontalAlignment::Left,
                WithBackground {
                    fg: Rgb565::WHITE,
                    bg: Rgb565::BLACK,
                },
                display0,
            )
            .unwrap();
        FONT0_NORMAL
            .render_aligned(
                format_args!("{:.1} %", hum),
                display0
                    .bounding_box()
                    .anchor_point(AnchorPoint::CenterLeft)
                    + Point::new(0, 20),
                VerticalPosition::Center,
                HorizontalAlignment::Left,
                WithBackground {
                    fg: Rgb565::WHITE,
                    bg: Rgb565::BLACK,
                },
                display0,
            )
            .expect("Failed to render humidity text");

        render_received(sample_time, display0);

        cx.local.display_timer.schedule(1000.millis()).unwrap();
    }
}

fn render_received<E, Err>(sample_time: u64, display: &mut E)
where
    Err: core::fmt::Debug,
    E: DrawTarget<Color = Rgb565, Error = Err>,
{
    central_node::FONT0_SMALLER
        .render_aligned(
            format_args!("Received at : {}", sample_time),
            display.bounding_box().anchor_point(AnchorPoint::BottomLeft),
            VerticalPosition::Bottom,
            HorizontalAlignment::Left,
            FontColor::WithBackground {
                fg: Rgb565::WHITE,
                bg: Rgb565::BLACK,
            },
            display,
        )
        .unwrap();
}

fn render_time<E, Err>(time: u64, display: &mut E)
where
    Err: core::fmt::Debug,
    E: DrawTarget<Color = Rgb565, Error = Err>,
{
    central_node::FONT0_SMALLER
        .render_aligned(
            format_args!("{time}"),
            display.bounding_box().anchor_point(AnchorPoint::TopLeft),
            VerticalPosition::Top,
            HorizontalAlignment::Left,
            FontColor::WithBackground {
                fg: Rgb565::WHITE,
                bg: Rgb565::BLACK,
            },
            display,
        )
        .unwrap();
}
