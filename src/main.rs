#![no_std]
#![no_main]

use core::mem::transmute;

use esp_backtrace as _;
use esp_println::println;
use hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    spi::{FullDuplexMode, SpiMode},
    systimer::SystemTimer,
    timer::TimerGroup,
    Rtc, Spi, IO,
};
use tmc_rs::{
    periodic, tmc2240_defaultRegisterResetState, tmc_ramp_linear_init, ConfigState_CONFIG_RESTORE,
    ConfigurationTypeDef, TMC_LinearRamp, TMC2240_REGISTER_COUNT, 
};

static mut SPI: Option<Spi<hal::peripherals::SPI2, FullDuplexMode>> = None;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mosi = io.pins.gpio5;
    let sclk = io.pins.gpio6;
    let cs = io.pins.gpio7;
    let miso = io.pins.gpio8;
    let mut en = io.pins.gpio15.into_push_pull_output();
    en.set_low().unwrap();

    let spi = Spi::new(
        peripherals.SPI2,
        sclk,
        mosi,
        miso,
        cs,
        5u32.MHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let timer = SystemTimer::now();

    unsafe {
        SPI = Some(spi);
        tmc_rs::SPIWriter = Some(write_spi);
    }

    let config = ConfigurationTypeDef {
        state: ConfigState_CONFIG_RESTORE,
        configIndex: 0,
        shadowRegister: [0i32; 128usize],
        reset: None,
        restore: None,
        callback: None,
        channel: 0,
    };
    let register_reset_state: [i32; TMC2240_REGISTER_COUNT as usize] = unsafe { transmute(tmc2240_defaultRegisterResetState.clone())};
    let mut mcu = tmc_rs::new(0, config, &register_reset_state, None);

    println!("Hello world!");

    let mut ramp = TMC_LinearRamp {
        maxVelocity: 0,
        targetPosition: 0,
        rampPosition: 0,
        targetVelocity: 0,
        rampVelocity: 0,
        acceleration: 0,
        rampEnabled: true,
        accumulatorVelocity: 0,
        accumulatorPosition: 0,
        rampMode: 0,
        state: 0,
        accelerationSteps: 0,
        precision: 0,
        homingDistance: 0,
        stopVelocity: 0,
    };

    loop {
        unsafe {
            // wait for register writes
            while (*mcu.config).state > 0 {
                periodic(&mut mcu, SystemTimer::now() as u32);
                continue;
            }

            tmc_ramp_linear_init(&mut ramp);
        }
    }
}

fn write_spi(address: u8, value: i32) {
    let value_bytes = value.to_be_bytes();
    let data: [u8; 4] = [address, value_bytes[0], value_bytes[1], value_bytes[2]];

    let spi = unsafe { SPI.as_mut().unwrap() };
    spi.write(&data);
}
