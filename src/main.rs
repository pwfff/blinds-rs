#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::{cell::RefCell, fmt::Debug};

use critical_section::Mutex;
use embassy_executor::Executor;
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;

use esp_backtrace as _;
use esp_println::println;
use fugit::HertzU32;
use hal::{
    clock::{ClockControl, Clocks},
    embassy,
    gpio::{Gpio15, Gpio17, Gpio18, Output, PushPull, IO},
    ledc::{
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
        LSGlobalClkSource, LowSpeed, LEDC,
    },
    peripherals::{Peripherals, MCPWM0},
    prelude::*,
    spi::{FullDuplexMode, SpiMode},
    systimer::SystemTimer,
    timer::TimerGroup,
    Rtc, Spi, mcpwm::{MCPWM, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode},
};
//use tmc_rs::{*, registers::tmc2240::TMC2240, registers::*};
use tmc_rs::{
    registers::{tmc2240::TMC2240, *},
    TMC_WRITE_BIT,
};

//static SPI: Mutex<RefCell<Option<Spi<hal::peripherals::SPI2, FullDuplexMode>>>> =
//    Mutex::new(RefCell::new(None));

static CLOCKS: Mutex<RefCell<Option<Clocks>>> = Mutex::new(RefCell::new(None));

fn set_time(t: &mut hal::ledc::timer::Timer<LowSpeed>, freq: HertzU32) {
    let config = timer::config::Config {
        duty: timer::config::Duty::Duty1Bit,
        clock_source: timer::LSClockSource::APBClk,
        frequency: freq,
    };

    critical_section::with(|cs| {
        let mut clocks = CLOCKS.borrow_ref_mut(cs);
        let clocks = clocks.as_mut().unwrap();

        let src_freq: u32 = clocks.apb_clock.to_Hz();
        let precision = 1 << config.duty as u32;
        let frequency: u32 = config.frequency.raw();
        println!("src: {:#?}, prec: {:#?}, freq: {:#?}", src_freq, precision, frequency);

        let mut divisor = ((src_freq as u64) << 8) / frequency as u64 / precision as u64;
        println!("div1: {:#?}", divisor);

        const LEDC_TIMER_DIV_NUM_MAX: u64 = 0x3FFFF;
        if divisor > LEDC_TIMER_DIV_NUM_MAX {
            // APB_CLK results in divisor which too high. Try using REF_TICK as clock
            // source.
            divisor = ((1_000_000 as u64) << 8) / frequency as u64 / precision as u64;
        }
        println!("div2: {:#?}", divisor);
        println!("max: {:#?}", LEDC_TIMER_DIV_NUM_MAX);

        t.configure(&clocks, config).log().expect("couldn't configure?");
    });
}

#[embassy_executor::task]
async fn stepper_task(
    mut pwm_pin: hal::mcpwm::operator::PwmPin<'static, Gpio17<Output<PushPull>>, MCPWM0, 0, true>,
    //step: Gpio17<Output<PushPull>>,
    mut dir: Gpio18<Output<PushPull>>,
) {
    //let mut step_timer = ledc.get_timer::<LowSpeed>(timer::Number::Timer1);
    //set_time(&mut step_timer, 500u32.Hz());

    //let mut step_channel = ledc.get_channel(channel::Number::Channel1, step);
    //step_channel
    //    .configure(channel::config::Config {
    //        timer: &step_timer,
    //        duty_pct: 0,
    //        pin_config: channel::config::PinConfig::PushPull,
    //    })
    //    .log()
    //    .unwrap();

    let mut moving = false;
    let mut opening = false;
    loop {
        Timer::after(Duration::from_millis(1_000)).await;

        moving = !moving;
        if moving {
            opening = !opening;
        }

        println!("moving: {} opening: {}", moving, opening);
        if moving {
            pwm_pin.set_timestamp(50);
        } else {
            pwm_pin.set_timestamp(0);
        }

        if opening {
            dir.set_low().unwrap();
        } else {
            dir.set_high().unwrap();
        }
    }
}

fn write_spi<'a>(
    spi: &'a mut Spi<'static, hal::peripherals::SPI2, FullDuplexMode>,
) -> impl FnMut(u8, u32) + 'a {
    |addr: u8, val: u32| {
        let val_bytes = val.to_be_bytes();
        let mut data: [u8; 5] = [
            // set write bit
            addr | (TMC_WRITE_BIT as u8),
            val_bytes[0],
            val_bytes[1],
            val_bytes[2],
            val_bytes[3],
        ];
        println!("writing to spi: {:x?} {:x?}", addr, data);
        spi.transfer(&mut data[..])
            .expect("Symmetric transfer failed");
        println!("wrote to spi, got {:x?}", data);
        let stat = tmc2240::SPI_STATUS::from_bytes([data[0]]);
        println!("spi status: {:#?}", stat);
    }
}

fn read_spi<'a>(
    spi: &'a mut Spi<'static, hal::peripherals::SPI2, FullDuplexMode>,
) -> impl FnMut(u8) -> u32 + 'a {
    |addr: u8| -> u32 {
        let mut data = [addr & !TMC_WRITE_BIT as u8, 0, 0, 0, 0];
        spi.transfer(&mut data[..])
            .expect("Symmetric transfer failed");
        println!("reading from SPI, last response {:x?}", data);
        //let stat = tmc2240::SPI_STATUS::from_bytes([data[0]]);
        //println!("spi status: {:#?}", stat);

        let mut data = [addr & !TMC_WRITE_BIT as u8, 0, 0, 0, 0];
        spi.transfer(&mut data[..])
            .expect("Symmetric transfer failed");
        println!("read from SPI, actual response {:x?}", data);
        //let stat = tmc2240::SPI_STATUS::from_bytes([data[0]]);
        //println!("spi status: {:#?}", stat);

        u32::from_be_bytes([data[1], data[2], data[3], data[4]])
    }
}

#[embassy_executor::task]
async fn spi_task(
    mut spi: Spi<'static, hal::peripherals::SPI2, FullDuplexMode>,
    mut en: Gpio15<Output<PushPull>>,
) {
    //let mut write_spi = |addr: u8, val: u32| {
    //    let val_bytes = val.to_be_bytes();
    //    let mut data: [u8; 5] = [
    //        // set write bit
    //        addr | (TMC_WRITE_BIT as u8),
    //        val_bytes[0],
    //        val_bytes[1],
    //        val_bytes[2],
    //        val_bytes[3],
    //    ];
    //    println!("writing to spi: {:x?} {:x?}", addr, data);
    //    spi.transfer(&mut data[..])
    //        .expect("Symmetric transfer failed");
    //    println!("wrote to spi, got {:x?}", data);
    //    let stat = tmc2240::SPI_STATUS::from_bytes([data[0]]);
    //    println!("spi status: {:?}", stat);
    //};

    let mcu = TMC2240::default();

    mcu.IHOLD_IRUN().set_IRUN(31);
    mcu.IHOLD_IRUN().set_IRUNDELAY(10);
    mcu.IHOLD_IRUN().set_IHOLDDELAY(10);
    println!("ihold_irun: {:#?}", mcu.IHOLD_IRUN());

    mcu.GCONF().set_en_pwm_mode(true);

    mcu.PWMCONF().set_pwm_autoscale(true);
    mcu.PWMCONF().set_pwm_autograd(true);
    mcu.PWMCONF().set_pwm_meas_sd_enable(true);

    mcu.PWMCONF().set_PWM_FREQ(0);

    mcu.CHOPCONF().set_MRES(0);
    mcu.CHOPCONF().set_intpol(true);
    mcu.CHOPCONF().set_TOFF(3);
    mcu.CHOPCONF().set_TBL(2);
    mcu.CHOPCONF().set_HSTRT_TFD210(4);
    mcu.CHOPCONF().set_HENDOFFSET(0);

    mcu.reset(&mut write_spi(&mut spi));

    en.set_low().unwrap();

    loop {
        //unsafe {
        //    while (*mcu.config).state > 0 {
        //        periodic(&mut mcu, SystemTimer::now() as u32);
        //        continue;
        //    }
        //}

        Timer::after(Duration::from_millis(1_000)).await;

        let stat = mcu.DRVSTATUS().read(&mut read_spi(&mut spi));
        println!("drv status: {:#?}", stat);

        //let resp = read_spi(TMC2240_DRVSTATUS as u8);
        //let stat = u32::from_be_bytes([resp[1], resp[2], resp[3], resp[4]]);
        //println!(
        //    "olb {}",
        //    read_field(stat, TMC2240_OLB_MASK, TMC2240_OLB_SHIFT)
        //);
        //println!(
        //    "ola {}",
        //    read_field(stat, TMC2240_OLA_MASK, TMC2240_OLA_SHIFT)
        //);
        //println!(
        //    "s2gb {}",
        //    read_field(stat, TMC2240_S2GB_MASK, TMC2240_S2GB_SHIFT)
        //);
        //println!(
        //    "s2ga {}",
        //    read_field(stat, TMC2240_S2GA_MASK, TMC2240_S2GA_SHIFT)
        //);
        //println!(
        //    "otpw {}",
        //    read_field(stat, TMC2240_OTPW_MASK, TMC2240_OTPW_SHIFT)
        //);
        //println!("ot {}", read_field(stat, TMC2240_OT_MASK, TMC2240_OT_SHIFT));
        //println!(
        //    "cs_actual {}",
        //    read_field(stat, TMC2240_CS_ACTUAL_MASK, TMC2240_CS_ACTUAL_SHIFT)
        //);
        //println!(
        //    "fsactive {}",
        //    read_field(stat, TMC2240_FSACTIVE_MASK, TMC2240_FSACTIVE_SHIFT)
        //);
        //println!(
        //    "stealth {}",
        //    read_field(stat, TMC2240_STEALTH_MASK, TMC2240_STEALTH_SHIFT)
        //);
        //println!(
        //    "s2vsb {}",
        //    read_field(stat, TMC2240_S2VSB_MASK, TMC2240_S2VSB_SHIFT)
        //);
        //println!(
        //    "s2vsa {}",
        //    read_field(stat, TMC2240_S2VSA_MASK, TMC2240_S2VSA_SHIFT)
        //);
    }
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

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

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(&clocks, SystemTimer::new(peripherals.SYSTIMER));

    #[cfg(feature = "embassy-time-timg0")]
    embassy::init(&clocks, timer_group0.timer0);

    let mosi = io.pins.gpio4;
    let sclk = io.pins.gpio5;
    let cs = io.pins.gpio6;
    let miso = io.pins.gpio7;
    let mut en = io.pins.gpio15.into_push_pull_output();
    en.set_high().log().unwrap();

    let spi = Spi::new(
        peripherals.SPI2,
        sclk,
        mosi,
        miso,
        cs,
        5000u32.kHz(),
        SpiMode::Mode3,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let clk = io.pins.gpio16.into_push_pull_output();
    let step = io.pins.gpio17.into_push_pull_output();
    let mut dir = io.pins.gpio18.into_push_pull_output();
    dir.set_low().unwrap();

    //critical_section::with(|cs| {
    //    CLOCKS.borrow_ref_mut(cs).replace(clocks);
    //});
    
    let clock_cfg = PeripheralClockConfig::with_frequency(&clocks, 625u32.kHz()).unwrap();
    let mut mcpwm = MCPWM::new(
        peripherals.MCPWM0,
        clock_cfg,
        &mut system.peripheral_clock_control,
    );
    // connect operator0 to timer0
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    // connect operator0 to pin
    let mut pwm_pin = mcpwm
        .operator0
        .with_pin_a(step, PwmPinConfig::UP_ACTIVE_HIGH);

    // start timer with timestamp values in the range of 0..=99 and a frequency of
    // 20 kHz
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(1000, PwmWorkingMode::Increase, 255u32.Hz())
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

    //let mut ledc = LEDC::new(peripherals.LEDC, &mut system.peripheral_clock_control);

    //ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    //let mut clk_timer = ledc.get_timer::<LowSpeed>(timer::Number::Timer0);
    //set_time(&mut clk_timer, 16u32.MHz());

    //let mut clk_channel = ledc.get_channel(channel::Number::Channel0, clk);
    //clk_channel
    //    .configure(channel::config::Config {
    //        timer: &clk_timer,
    //        duty_pct: 50,
    //        pin_config: channel::config::PinConfig::PushPull,
    //    })
    //    .log()
    //    .unwrap();

    //let t_step = MicrosDurationU32::micros(3000);
    //let f_step = t_step.into_rate();
    //println!("t_step {} f_step {}", t_step, f_step);

    //let mut ramp = TMC_LinearRamp {
    //    maxVelocity: 0,
    //    targetPosition: 0,
    //    rampPosition: 0,
    //    targetVelocity: 0,
    //    rampVelocity: 0,
    //    acceleration: 0,
    //    rampEnabled: true,
    //    accumulatorVelocity: 0,
    //    accumulatorPosition: 0,
    //    rampMode: 0,
    //    state: 0,
    //    accelerationSteps: 0,
    //    precision: 0,
    //    homingDistance: 0,
    //    stopVelocity: 0,
    //};
    // tmc_ramp_linear_init(&mut ramp);

    //let mcu_new = tmc2240::new();
    //mcu_new.reset(|addr, val| println!("addr: {:04x}\nval: {:08x}", addr, val));

    //unsafe {
    //    tmc_rs::SPIWriter = Some(write_spi);
    //}

    //let config = ConfigurationTypeDef {
    //    state: ConfigState_CONFIG_RESTORE,
    //    configIndex: 0,
    //    shadowRegister: [0i32; 128usize],
    //    reset: None,
    //    restore: None,
    //    callback: None,
    //    channel: 0,
    //};
    //let mut register_reset_state: [u32; TMC2240_REGISTER_COUNT as usize] =
    //    tmc2240_defaultRegisterResetState.clone();

    //set_field(
    //    &mut register_reset_state,
    //    TMC2240_IHOLD_IRUN,
    //    TMC2240_IRUN_MASK,
    //    TMC2240_IRUN_SHIFT,
    //    28,
    //);

    //set_field(
    //    &mut register_reset_state,
    //    TMC2240_IHOLD_IRUN,
    //    TMC2240_IRUNDELAY_MASK,
    //    TMC2240_IRUNDELAY_SHIFT,
    //    10,
    //);

    //set_field(
    //    &mut register_reset_state,
    //    TMC2240_IHOLD_IRUN,
    //    TMC2240_IHOLDDELAY_MASK,
    //    TMC2240_IHOLDDELAY_SHIFT,
    //    10,
    //);

    //// enable PWM mode in general conf
    //set_field(
    //    &mut register_reset_state,
    //    TMC2240_GCONF,
    //    TMC2240_EN_PWM_MODE_MASK,
    //    TMC2240_EN_PWM_MODE_SHIFT,
    //    1,
    //);

    //// enable autograd, autoscale, current measurement
    //set_field(
    //    &mut register_reset_state,
    //    TMC2240_PWMCONF,
    //    TMC2240_PWM_AUTOGRAD_MASK,
    //    TMC2240_PWM_AUTOGRAD_SHIFT,
    //    1,
    //);
    //set_field(
    //    &mut register_reset_state,
    //    TMC2240_PWMCONF,
    //    TMC2240_PWM_AUTOSCALE_MASK,
    //    TMC2240_PWM_AUTOSCALE_SHIFT,
    //    1,
    //);
    //set_field(
    //    &mut register_reset_state,
    //    TMC2240_PWMCONF,
    //    TMC2240_PWM_MEAS_SD_ENABLE_MASK,
    //    TMC2240_PWM_MEAS_SD_ENABLE_SHIFT,
    //    1,
    //);

    //// default 0 recommended for 16MHz clk
    //set_field(
    //    &mut register_reset_state,
    //    TMC2240_PWMCONF,
    //    TMC2240_PWM_FREQ_MASK,
    //    TMC2240_PWM_FREQ_SHIFT,
    //    0,
    //);

    //set_field(
    //    &mut register_reset_state,
    //    TMC2240_CHOPCONF,
    //    TMC2240_MRES_MASK,
    //    TMC2240_MRES_SHIFT,
    //    4,
    //);
    //set_field(
    //    &mut register_reset_state,
    //    TMC2240_CHOPCONF,
    //    TMC2240_INTPOL_MASK,
    //    TMC2240_INTPOL_SHIFT,
    //    1,
    //);

    //// configure CHOPCONF - not used, but must be configured to run the motor
    //set_field(
    //    &mut register_reset_state,
    //    TMC2240_CHOPCONF,
    //    TMC2240_TOFF_MASK,
    //    TMC2240_TOFF_SHIFT,
    //    3,
    //);
    //set_field(
    //    &mut register_reset_state,
    //    TMC2240_CHOPCONF,
    //    TMC2240_TBL_MASK,
    //    TMC2240_TBL_SHIFT,
    //    2,
    //);
    //set_field(
    //    &mut register_reset_state,
    //    TMC2240_CHOPCONF,
    //    TMC2240_HSTRT_TFD210_MASK,
    //    TMC2240_HSTRT_TFD210_SHIFT,
    //    4,
    //);
    //set_field(
    //    &mut register_reset_state,
    //    TMC2240_CHOPCONF,
    //    TMC2240_HEND_OFFSET_MASK,
    //    TMC2240_HEND_OFFSET_SHIFT,
    //    0,
    //);

    //let register_reset_state = unsafe { transmute(register_reset_state) };
    //let mut mcu = tmc_rs::new(0, config, &register_reset_state, None);

    //println!("resetting mcu");
    //unsafe { tmc2240_reset(&mut mcu) };

    //// clear status flags, check GSTAT

    //write_spi_bytes(TMC2240_GSTAT as u8, [255; 4]);
    //let stat = GStat::from(read_spi(TMC2240_GSTAT as u8)[4]);
    //println!("gstat {:?}", stat);

    //interrupt::enable(
    //    peripherals::Interrupt::SYSTIMER_TARGET0,
    //    Priority::Priority1,
    //)
    //.log()
    //.unwrap();

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(stepper_task(pwm_pin, dir)).ok();
        spawner.spawn(spi_task(spi, en)).ok();
    });

    /*
    loop {
        // wait for register writes

        delay.delay_ms(500u32);

    }
    */
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

//fn write_spi_bytes(address: u8, value: [u8; 4]) {
//    let mut data: [u8; 5] = [
//        // set write bit
//        address | 0b10000000,
//        value[0],
//        value[1],
//        value[2],
//        value[3],
//    ];
//
//    critical_section::with(|cs| {
//        let mut spi = SPI.borrow_ref_mut(cs);
//        let spi = spi.as_mut().unwrap();
//
//        println!("writing to spi: {:x?}", data);
//
//        spi.transfer(&mut data[..])
//            .expect("Symmetric transfer failed");
//
//        println!("wrote to spi, got {:x?}", data);
//        println!("spi status: {:?}", SPIStatus::from(data[0]));
//    })
//}

//fn write_spi(address: u8, value: i32) {
//    let value_bytes = value.to_be_bytes();
//    let mut data: [u8; 5] = [
//        // set write bit
//        address | (TMC_WRITE_BIT as u8),
//        value_bytes[0],
//        value_bytes[1],
//        value_bytes[2],
//        value_bytes[3],
//    ];
//
//    critical_section::with(|cs| {
//        let mut spi = SPI.borrow_ref_mut(cs);
//        let spi = spi.as_mut().unwrap();
//
//        println!("writing to spi: {:x?} {:x?}", address, data);
//
//        spi.transfer(&mut data[..])
//            .expect("Symmetric transfer failed");
//
//        println!("wrote to spi, got {:x?}", data);
//        println!("spi status: {:?}", SPIStatus::from(data[0]));
//    })
//}
//
//fn read_spi(address: u8) -> [u8; 5] {
//    critical_section::with(|cs| {
//        let mut spi = SPI.borrow_ref_mut(cs);
//        let spi = spi.as_mut().unwrap();
//
//        let mut data = [address as u8 & 0b01111111, 0, 0, 0, 0];
//        spi.transfer(&mut data[..])
//            .expect("Symmetric transfer failed");
//        println!("reading from SPI, last response {:x?}", data);
//        println!("spi status: {:?}", SPIStatus::from(data[0]));
//
//        let mut data = [address as u8 & 0b01111111, 0, 0, 0, 0];
//        spi.transfer(&mut data[..])
//            .expect("Symmetric transfer failed");
//        println!("read from SPI, actual response {:x?}", data);
//        println!("spi status: {:?}", SPIStatus::from(data[0]));
//
//        data
//    })
//}
//
//fn set_field(
//    reg: &mut [u32; TMC_REGISTER_COUNT as usize],
//    addr: u32,
//    mask: u32,
//    shift: u32,
//    value: u32,
//) {
//    let addr = addr & TMC_ADDRESS_MASK;
//    println!(
//        "setting: {:x?} to {:02x}                         ({:08b})",
//        addr, value, value
//    );
//    println!("mask:             ({:032b})", mask);
//
//    let shifted = ((value << shift) & mask) as u32;
//    println!("shifted:          ({:032b})", shifted);
//
//    let old_value = reg[addr as usize];
//    println!("old:     {:08x?} ({:032b})", old_value, old_value);
//    let cleared = (old_value) & (!(mask));
//    println!("cleared: {:08x?} ({:032b})", cleared, cleared);
//    let new = cleared | shifted;
//    println!("new:     {:08x?} ({:032b})", new, new);
//    println!("");
//    reg[addr as usize] = new;
//}
//
//fn read_field(value: u32, mask: u32, shift: u32) -> u32 {
//    return (value & mask) >> shift;
//}

trait LogExt {
    fn log(self) -> Self;
}

impl<T, E: Debug> LogExt for Result<T, E> {
    fn log(self) -> Self {
        if let Err(e) = &self {
            println!("An error happened: {:?}", e);
        }
        self
    }
}
