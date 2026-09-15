mod motor;

use anyhow::{bail, ensure, Context};
use esp_idf_svc::hal::{
    delay::FreeRtos,
    gpio::PinDriver,
    ledc::{self, LedcDriver, LedcTimerDriver, Resolution},
    peripherals::Peripherals,
    spi::{self, SpiDeviceDriver, SpiDriver, SpiDriverConfig, SPI2},
    sys::link_patches,
    units::FromValueType,
};
use log::*;
use std::sync::Arc;
use tmc_rs::registers::{tmc2240, Register};

fn check_driver_status(status: tmc2240::DRV_STATUS) -> anyhow::Result<()> {
    let flags = status.SPI_STATUS();
    if flags.ot() || flags.otpw() || flags.s2ga() || flags.s2gb() {
        bail!("driver temperature/short-circuit fault: {status:#?}");
    }
    Ok(())
}

fn main() -> anyhow::Result<()> {
    link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    let peripherals = Peripherals::take()?;

    // DRV_ENN is active low. Keep it disabled throughout setup and on any error.
    // A board pull-up is still required to keep it disabled during reset/boot.
    let mut en = PinDriver::output(peripherals.pins.gpio15)?;
    en.set_high()?;
    FreeRtos::delay_ms(1); // allow bridge disable before any configuration writes

    // Keep the clock and PWM handles alive even after a fault; dropping EN must
    // not be our shutdown mechanism. The outer scope parks with EN driven high.
    let clk_config = ledc::config::TimerConfig::new()
        .resolution(Resolution::Bits1)
        .frequency(motor::CLOCK_HZ.Hz());
    let clk_timer = Arc::new(LedcTimerDriver::new(peripherals.ledc.timer1, &clk_config)?);
    let mut clk_ledc = LedcDriver::new(
        peripherals.ledc.channel1,
        clk_timer,
        peripherals.pins.gpio16,
    )?;
    // One bit has two ticks: duty=1 is 50%, not duty=50.
    clk_ledc.set_duty(1)?;
    clk_ledc.enable()?;
    FreeRtos::delay_ms(1);

    let step_config = ledc::config::TimerConfig::new()
        .resolution(Resolution::Bits8)
        .frequency(motor::STEP_HZ.Hz());
    let step_timer = Arc::new(LedcTimerDriver::new(peripherals.ledc.timer0, &step_config)?);
    let mut step_ledc = LedcDriver::new(
        peripherals.ledc.channel0,
        step_timer,
        peripherals.pins.gpio17,
    )?;
    step_ledc.disable()?;
    let mut dir = PinDriver::output(peripherals.pins.gpio18)?;
    dir.set_low()?;

    let spi_driver = SpiDriver::new::<SPI2>(
        peripherals.spi2,
        peripherals.pins.gpio5,
        peripherals.pins.gpio4,
        Some(peripherals.pins.gpio7),
        &SpiDriverConfig::new(),
    )?;
    let spi_config = spi::config::Config::new()
        .baudrate(5.MHz().into())
        .data_mode(spi::config::MODE_3);
    let mut spi = SpiDeviceDriver::new(&spi_driver, Some(peripherals.pins.gpio6), &spi_config)?;
    let mut registers = tmc2240::TMC2240::default();

    let result = (|| -> anyhow::Result<()> {
        info!(
            "nominal current: max {:?}, run {:?}, hold {:?}",
            motor::CURRENT.max_rms(),
            motor::CURRENT.run_rms(),
            motor::CURRENT.hold_rms()
        );
        motor::MOTOR
            .stage(&mut registers)
            .write(&mut spi)
            .context("writing checked motor profile")?;
        // Associated reads do not overwrite the staged configuration cache.
        let observed = tmc2240::CHOPCONF::read_at(&mut spi)?;
        ensure!(observed == registers.CHOPCONF, "CHOPCONF readback mismatch");
        info!("chopconf: {observed:#?}");

        check_driver_status(registers.DRVSTATUS.read(&mut spi)?)?;
        en.set_low()?;
        FreeRtos::delay_ms(1);
        step_ledc.set_duty(128)?; // 50% of the 256-tick STEP period
        step_ledc.enable()?;
        let mut moving = true;
        let mut opening = true; // low DIR, matching the initial pin state
        let mut seconds = 0;
        loop {
            FreeRtos::delay_ms(1_000);
            let status = registers.DRVSTATUS.read(&mut spi)?;
            info!("drv status: {status:#?}");
            check_driver_status(status)?;
            info!("tstep: {:#?}", registers.TSTEP.read(&mut spi)?);
            if !moving {
                info!(
                    "holding: dir={}, mscnt={}",
                    if opening { "low" } else { "high" },
                    registers.MSCNT.read(&mut spi)?.MSCNT()
                );
            }
            seconds += 1;
            if seconds == 5 {
                seconds = 0;
                if moving {
                    step_ledc.disable()?;
                    // Let the PWM duty update settle before sampling the stop position.
                    FreeRtos::delay_ms(1);
                    info!(
                        "stopped: dir={}, mscnt={}",
                        if opening { "low" } else { "high" },
                        registers.MSCNT.read(&mut spi)?.MSCNT()
                    );
                } else {
                    opening = !opening;
                    if opening {
                        dir.set_low()?;
                    } else {
                        dir.set_high()?;
                    }
                    // Change DIR only while STEP is stopped; exceed its setup time.
                    FreeRtos::delay_ms(1);
                    step_ledc.enable()?;
                }
                moving = !moving;
            }
        }
    })();

    // Disable the bridge first, even if stopping STEP also fails.
    if let Err(error) = en.set_high() {
        error!("could not disable driver: {error}");
    }
    if let Err(error) = step_ledc.disable() {
        error!("could not stop STEP: {error}");
    }
    if let Err(error) = result {
        error!("motor stopped: {error:#}");
    }
    loop {
        FreeRtos::delay_ms(1_000);
    }
}
