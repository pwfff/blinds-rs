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
    ledc::{self, LedcChannel, LedcDriver, LedcTimer, LedcTimerDriver, Resolution},
    peripheral::Peripheral,
    prelude::Peripherals,
    spi::{self, SpiDeviceDriver, SpiDriver, SpiDriverConfig, SPI2},
    units::FromValueType,
};
use std::{sync::Arc, time::Duration};

use tmc_rs::registers::{tmc2240::TMC2240, *};

fn stepper_task<
    'a,
    STEP: Peripheral<P = impl OutputPin>,
    DIR: Peripheral<P = impl OutputPin>,
    T: Peripheral<P = impl LedcTimer>,
    C: Peripheral<P = impl LedcChannel>,
>(
    ledc_timer: T,
    ledc_channel: C,
    step: STEP,
    dir: DIR,
) -> anyhow::Result<()> {
    let mut moving = true;
    let mut opening = false;

    let mut dir = PinDriver::output(dir)?;
    dir.set_low().unwrap();

    let step_config = ledc::config::TimerConfig::new()
        .resolution(Resolution::Bits8)
        .frequency(2.kHz().into());
    let step_timer = Arc::new(LedcTimerDriver::new(ledc_timer, &step_config)?);
    let mut step_ledc = LedcDriver::new(ledc_channel, step_timer, step)?;
    step_ledc.set_duty(50)?;
    step_ledc.enable()?;

    loop {
        FreeRtos::delay_ms(1000);

        moving = !moving;
        if moving {
            opening = !opening;
        }

        step_ledc.set_duty(match moving {
            true => 50,
            false => 0,
        })?;

        if opening {
            dir.set_low()?;
        } else {
            dir.set_high()?;
        }
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

    mcu.CHOPCONF.set_MRES(4);
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
        FreeRtos::delay_ms(1_000);

        let stat = mcu.DRVSTATUS.read(&mut spi);
        info!("drv status: {:#?}", stat);
    }
}

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let mosi = peripherals.pins.gpio4;
    let sclk = peripherals.pins.gpio5;
    let cs = peripherals.pins.gpio6;
    let miso = peripherals.pins.gpio7;
    let mut en = PinDriver::output(peripherals.pins.gpio15)?;
    en.set_high().log().unwrap();

    let spi = peripherals.spi2;
    let spi_driver = SpiDriver::new::<SPI2>(spi, sclk, mosi, Some(miso), &SpiDriverConfig::new())?;

    let clk = peripherals.pins.gpio16;
    let step = peripherals.pins.gpio17;
    let dir = peripherals.pins.gpio18;

    let clk_config = ledc::config::TimerConfig::new()
        .resolution(Resolution::Bits1)
        .frequency(16.MHz().into());
    let clk_timer = Arc::new(LedcTimerDriver::new(peripherals.ledc.timer1, &clk_config)?);
    let mut clk_ledc = LedcDriver::new(peripherals.ledc.channel1, clk_timer, clk)?;
    clk_ledc.set_duty(50)?;
    clk_ledc.enable()?;

    let t0 = std::thread::Builder::new()
        .stack_size(7000)
        .spawn(move || {
            stepper_task(
                peripherals.ledc.timer0,
                peripherals.ledc.channel0,
                step,
                dir,
            )
        })?;
    let t1 = std::thread::Builder::new()
        .stack_size(7000)
        .spawn(move || spi_task(cs, spi_driver, en))?;

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
