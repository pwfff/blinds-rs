#![feature(type_alias_impl_trait)]

use embedded_svc::wifi::{AuthMethod, ClientConfiguration, Configuration};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    nvs::EspDefaultNvsPartition,
    wifi::{BlockingWifi, EspWifi},
};
use log::*;

use core::fmt::Debug;
use esp_idf_hal::{
    delay::FreeRtos,
    gpio::{Output, OutputPin, PinDriver},
    ledc::{self, LedcDriver, LedcTimerDriver, Resolution},
    peripheral::Peripheral,
    prelude::Peripherals,
    spi::{self, SpiDeviceDriver, SpiDriver, SpiDriverConfig, SPI2},
    units::FromValueType,
};
use std::{sync::Arc, time::Duration};

//use critical_section::Mutex;
//use embassy_executor::Executor;
//use embassy_time::{Duration, Timer};
//use static_cell::StaticCell;
//
//use esp_backtrace as _;
//use esp_println::println;
//use fugit::HertzU32;
//use hal::{
//    clock::{ClockControl, Clocks},
//    embassy,
//    gpio::{Gpio15, Gpio17, Gpio18, Output, PushPull, IO},
//    ledc::{
//        channel::{self, ChannelIFace},
//        timer::{self, TimerIFace},
//        LSGlobalClkSource, LowSpeed, LEDC,
//    },
//    peripherals::{Peripherals, MCPWM0},
//    prelude::*,
//    spi::{FullDuplexMode, SpiMode},
//    systimer::SystemTimer,
//    timer::TimerGroup,
//    Rtc, Spi, mcpwm::{MCPWM, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode},
//};
//use tmc_rs::{*, registers::tmc2240::TMC2240, registers::*};
use tmc_rs::registers::{tmc2240::TMC2240, *};

//static SPI: Mutex<RefCell<Option<Spi<hal::peripherals::SPI2, FullDuplexMode>>>> =
//    Mutex::new(RefCell::new(None));

//static CLOCKS: Mutex<RefCell<Option<Clocks>>> = Mutex::new(RefCell::new(None));

//fn set_time(t: &mut hal::ledc::timer::Timer<LowSpeed>, freq: HertzU32) {
//    let config = timer::config::Config {
//        duty: timer::config::Duty::Duty1Bit,
//        clock_source: timer::LSClockSource::APBClk,
//        frequency: freq,
//    };
//
//    critical_section::with(|cs| {
//        let mut clocks = CLOCKS.borrow_ref_mut(cs);
//        let clocks = clocks.as_mut().unwrap();
//
//        let src_freq: u32 = clocks.apb_clock.to_Hz();
//        let precision = 1 << config.duty as u32;
//        let frequency: u32 = config.frequency.raw();
//        debug!(
//            "src: {:#?}, prec: {:#?}, freq: {:#?}",
//            src_freq, precision, frequency
//        );
//
//        let mut divisor = ((src_freq as u64) << 8) / frequency as u64 / precision as u64;
//        debug!("div1: {:#?}", divisor);
//
//        const LEDC_TIMER_DIV_NUM_MAX: u64 = 0x3FFFF;
//        if divisor > LEDC_TIMER_DIV_NUM_MAX {
//            // APB_CLK results in divisor which too high. Try using REF_TICK as clock
//            // source.
//            divisor = ((1_000_000 as u64) << 8) / frequency as u64 / precision as u64;
//        }
//        debug!("div2: {:#?}", divisor);
//        debug!("max: {:#?}", LEDC_TIMER_DIV_NUM_MAX);
//
//        t.configure(&clocks, config)
//            .log()
//            .expect("couldn't configure?");
//    });
//}

fn stepper_task<'a, S: OutputPin, D: OutputPin>(
    mut step: PinDriver<'a, S, Output>,
    mut dir: PinDriver<'a, D, Output>,
) -> anyhow::Result<()> {
    let mut moving = true;
    let mut opening = false;
    let mut i = 0u32;
    loop {
        FreeRtos::delay_ms(5);

        //moving = !moving;
        //if moving {
        //    opening = !opening;
        //}

        if i % 10 == 0 {
            debug!("moving: {} opening: {}", moving, opening);
        }
        if moving {
            step.toggle()?;
        }

        //if opening {
        //    dir.set_low()?;
        //} else {
        //    dir.set_high()?;
        //}

        i += 1;
    }
}

//fn write_spi<'a, 'b, D: Borrow<SpiDriver<'b>>>(
//    spi: &'b mut SpiDeviceDriver<'a, D>,
//) -> impl FnMut(u8, u32) + 'a + 'b {
//    |addr: u8, val: u32| {
//        let val_bytes = val.to_be_bytes();
//        let write: [u8; 5] = [
//            // set write bit
//            addr | (TMC_WRITE_BIT as u8),
//            val_bytes[0],
//            val_bytes[1],
//            val_bytes[2],
//            val_bytes[3],
//        ];
//        let mut read = [0; 5];
//        debug!("writing to spi: {:x?} {:x?}", addr, write);
//        spi.transfer(&mut read, &write[..])
//            .expect("Symmetric transfer failed");
//        debug!("wrote to spi, got {:x?}", read);
//        let stat = tmc2240::SPI_STATUS::from_bytes([read[0]]);
//        debug!("spi status: {:#?}", stat);
//    }
//}
//
//fn read_spi<'a>(spi: &'a mut SpiDeviceDriver<'a, SpiDriver<'a>>) -> impl FnMut(u8) -> u32 + 'a {
//    |addr: u8| -> u32 {
//        let mut read = [0u8; 5];
//        let write = [addr & !TMC_WRITE_BIT as u8, 0, 0, 0, 0];
//
//        spi.transfer(&mut read, &write[..])
//            .expect("Symmetric transfer failed");
//        debug!("reading from SPI, last response {:x?}", read);
//        //let stat = tmc2240::SPI_STATUS::from_bytes([write[0]]);
//        //debug!("spi status: {:#?}", stat);
//
//        spi.transfer(&mut read, &write[..])
//            .expect("Symmetric transfer failed");
//        debug!("read from SPI, actual response {:x?}", read);
//        //let stat = tmc2240::SPI_STATUS::from_bytes([write[0]]);
//        //debug!("spi status: {:#?}", stat);
//
//        u32::from_be_bytes([read[1], read[2], read[3], read[4]])
//    }
//}

fn spi_task<'a, CS: Peripheral<P = CSP>, CSP: OutputPin, EN: OutputPin>(
    cs: CS,
    spi_driver: SpiDriver<'a>,
    mut en: PinDriver<'a, EN, Output>,
) -> anyhow::Result<()> {
    let spi_config = spi::config::Config::new().baudrate(5.MHz().into());
    let mut spi = SpiDeviceDriver::new(&spi_driver, Some(cs), &spi_config)?;

    let mut mcu = TMC2240::default();

    mcu.IHOLD_IRUN.set_IRUN(31);
    mcu.IHOLD_IRUN.set_IRUNDELAY(10);
    mcu.IHOLD_IRUN.set_IHOLDDELAY(10);

    mcu.GCONF.set_en_pwm_mode(true);

    mcu.PWMCONF.set_pwm_autoscale(true);
    mcu.PWMCONF.set_pwm_autograd(true);
    mcu.PWMCONF.set_pwm_meas_sd_enable(true);
    mcu.PWMCONF.set_PWM_FREQ(0);

    mcu.CHOPCONF.set_MRES(6);
    mcu.CHOPCONF.set_dedge(true);
    mcu.CHOPCONF.set_intpol(true);
    mcu.CHOPCONF.set_TOFF(3);
    mcu.CHOPCONF.set_TBL(2);
    mcu.CHOPCONF.set_HSTRT_TFD210(4);
    mcu.CHOPCONF.set_HENDOFFSET(0);

    info!("resetting mcu to: {:#?}", mcu);
    info!("chopconf: {:#?}", mcu.CHOPCONF);

    mcu.reset(&mut spi)?;

    let got_chop = mcu.CHOPCONF.read(&mut spi).expect("couldn't read");
    info!("reading back chopconf: {:#?}", got_chop);

    en.set_low().unwrap();

    loop {
        //unsafe {
        //    while (*mcu.config).state > 0 {
        //        periodic(&mut mcu, SystemTimer::now() as u32);
        //        continue;
        //    }
        //}

        FreeRtos::delay_ms(1_000);

        let stat = mcu.DRVSTATUS.read(&mut spi);
        info!("drv status: {:#?}", stat);
    }
}

//static EXECUTOR: StaticCell<Executor> = StaticCell::new();

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    //let mut system = peripherals.SYSTEM.split();
    //let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    //// Disable the RTC and TIMG watchdog timers
    //let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    //let timer_group0 = TimerGroup::new(
    //    peripherals.TIMG0,
    //    &clocks,
    //    &mut system.peripheral_clock_control,
    //);
    //let mut wdt0 = timer_group0.wdt;
    //let timer_group1 = TimerGroup::new(
    //    peripherals.TIMG1,
    //    &clocks,
    //    &mut system.peripheral_clock_control,
    //);
    //let mut wdt1 = timer_group1.wdt;
    //rtc.rwdt.disable();
    //wdt0.disable();
    //wdt1.disable();

    //let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(&clocks, SystemTimer::new(peripherals.SYSTIMER));

    #[cfg(feature = "embassy-time-timg0")]
    embassy::init(&clocks, timer_group0.timer0);

    let mosi = peripherals.pins.gpio4;
    let sclk = peripherals.pins.gpio5;
    let cs = peripherals.pins.gpio6;
    let miso = peripherals.pins.gpio7;
    let mut en = PinDriver::output(peripherals.pins.gpio15)?;
    en.set_high().log().unwrap();

    //let spi = Spi::new(
    //    peripherals.SPI2,
    //    sclk,
    //    mosi,
    //    miso,
    //    cs,
    //    5000u32.kHz(),
    //    SpiMode::Mode3,
    //    &mut system.peripheral_clock_control,
    //    &clocks,
    //);
    let spi = peripherals.spi2;

    let driver = SpiDriver::new::<SPI2>(spi, sclk, mosi, Some(miso), &SpiDriverConfig::new())?;

    let clk = peripherals.pins.gpio16;
    let step = PinDriver::output(peripherals.pins.gpio17)?;
    let mut dir = PinDriver::output(peripherals.pins.gpio18)?;
    dir.set_low().unwrap();

    let clk_config = ledc::config::TimerConfig::new()
        .resolution(Resolution::Bits1)
        .frequency(16.MHz().into());
    let clk_timer = Arc::new(LedcTimerDriver::new(peripherals.ledc.timer1, &clk_config)?);
    let mut clk_ledc = LedcDriver::new(peripherals.ledc.channel1, clk_timer, clk)?;
    clk_ledc.set_duty(50)?;
    clk_ledc.enable()?;

    //critical_section::with(|cs| {
    //    CLOCKS.borrow_ref_mut(cs).replace(clocks);
    //});

    //let clock_cfg = PeripheralClockConfig::with_frequency(&clocks, 625u32.kHz()).unwrap();
    //let mut mcpwm = MCPWM::new(
    //    peripherals.MCPWM0,
    //    clock_cfg,
    //    &mut system.peripheral_clock_control,
    //);
    //// connect operator0 to timer0
    //mcpwm.operator0.set_timer(&mcpwm.timer0);
    //// connect operator0 to pin
    //let mut pwm_pin = mcpwm
    //    .operator0
    //    .with_pin_a(step, PwmPinConfig::UP_ACTIVE_HIGH);

    //// start timer with timestamp values in the range of 0..=99 and a frequency of
    //// 20 kHz
    //let timer_clock_cfg = clock_cfg
    //    .timer_clock_with_frequency(1000, PwmWorkingMode::Increase, 255u32.Hz())
    //    .unwrap();
    //mcpwm.timer0.start(timer_clock_cfg);

    let t0 = std::thread::Builder::new()
        .stack_size(7000)
        .spawn(move || stepper_task(step, dir))?;
    let t1 = std::thread::Builder::new()
        .stack_size(7000)
        .spawn(move || spi_task(cs, driver, en))?;

    //let sys_loop = EspSystemEventLoop::take()?;
    //let nvs = EspDefaultNvsPartition::take()?;

    //let mut wifi = BlockingWifi::wrap(
    //    EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs))?,
    //    sys_loop,
    //)?;
    //connect_wifi(&mut wifi)?;

    info!("Waiting for PWM threads");

    println!("Joined PWM threads");

    println!("Done");

    loop {
        // Don't let the idle task starve and trigger warnings from the watchdog.
        //FreeRtos::delay_ms(1_000);
        std::thread::sleep(Duration::from_millis(100));
    }
}

struct Bytes<'a>(&'a [u8]);

impl<'a> core::fmt::Binary for Bytes<'a> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        writeln!(f, "[")?;
        for byte in self.0 {
            core::fmt::Binary::fmt(byte, f)?;
            writeln!(f, ",")?;
        }
        writeln!(f, "]")?;
        Ok(())
    }
}

trait LogExt {
    fn log(self) -> Self;
}

impl<T, E: Debug> LogExt for Result<T, E> {
    fn log(self) -> Self {
        if let Err(e) = &self {
            error!("An error happened: {:?}", e);
        }
        self
    }
}

const SSID: &str = env!("WIFI_SSID");
const PASSWORD: &str = env!("WIFI_PASS");

fn connect_wifi(wifi: &mut BlockingWifi<EspWifi<'static>>) -> anyhow::Result<()> {
    let wifi_configuration: Configuration = Configuration::Client(ClientConfiguration {
        ssid: SSID.into(),
        bssid: None,
        auth_method: AuthMethod::WPA2Personal,
        password: PASSWORD.into(),
        channel: None,
    });

    wifi.set_configuration(&wifi_configuration)?;

    wifi.start()?;
    info!("Wifi started");

    wifi.connect()?;
    info!("Wifi connected");

    wifi.wait_netif_up()?;
    info!("Wifi netif up");

    Ok(())
}
