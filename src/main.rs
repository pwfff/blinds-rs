#![no_std]
#![no_main]

use core::{fmt::Debug, mem::transmute};

use esp_backtrace as _;
use esp_println::println;
use fugit::MicrosDurationU32;
use hal::{
    clock::ClockControl,
    gpio::{Gpio17, Gpio18, Output, PushPull, IO},
    interrupt,
    ledc::{
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
        LSGlobalClkSource, LowSpeed, LEDC,
    },
    peripherals::{self, Peripherals},
    prelude::*,
    spi::{FullDuplexMode, SpiMode},
    systimer::{Alarm, Periodic, SystemTimer},
    timer::TimerGroup,
    Delay, Priority, Rtc, Spi,
};
use tmc_rs::{
    periodic, tmc2240_defaultRegisterResetState, tmc2240_reset, tmc_ramp_linear_init,
    ConfigState_CONFIG_RESTORE, ConfigurationTypeDef, GStat, SPIStatus, TMC_LinearRamp,
    TMC2240_CHOPCONF, TMC2240_CS_ACTUAL_MASK, TMC2240_CS_ACTUAL_SHIFT, TMC2240_DRVSTATUS,
    TMC2240_EN_PWM_MODE_MASK, TMC2240_EN_PWM_MODE_SHIFT, TMC2240_FSACTIVE_MASK,
    TMC2240_FSACTIVE_SHIFT, TMC2240_GCONF, TMC2240_GSTAT, TMC2240_HEND_OFFSET_MASK,
    TMC2240_HEND_OFFSET_SHIFT, TMC2240_HSTRT_TFD210_MASK, TMC2240_HSTRT_TFD210_SHIFT,
    TMC2240_INTPOL_MASK, TMC2240_INTPOL_SHIFT, TMC2240_IOIN, TMC2240_MRES_MASK, TMC2240_MRES_SHIFT,
    TMC2240_MSCNT, TMC2240_OLA_MASK, TMC2240_OLA_SHIFT, TMC2240_OTPW_MASK, TMC2240_OTPW_SHIFT,
    TMC2240_OT_MASK, TMC2240_OT_SHIFT, TMC2240_PWMCONF, TMC2240_PWM_AUTOGRAD_MASK,
    TMC2240_PWM_AUTOGRAD_SHIFT, TMC2240_PWM_AUTOSCALE_MASK, TMC2240_PWM_AUTOSCALE_SHIFT,
    TMC2240_PWM_FREQ_MASK, TMC2240_PWM_FREQ_SHIFT, TMC2240_PWM_MEAS_SD_ENABLE_MASK,
    TMC2240_PWM_MEAS_SD_ENABLE_SHIFT, TMC2240_REGISTER_COUNT, TMC2240_S2GA_MASK,
    TMC2240_S2GA_SHIFT, TMC2240_S2GB_MASK, TMC2240_S2GB_SHIFT, TMC2240_S2VSA_MASK,
    TMC2240_S2VSA_SHIFT, TMC2240_S2VSB_MASK, TMC2240_S2VSB_SHIFT, TMC2240_STEALTH_MASK,
    TMC2240_STEALTH_SHIFT, TMC2240_TBL_MASK, TMC2240_TBL_SHIFT, TMC2240_TOFF_MASK,
    TMC2240_TOFF_SHIFT, TMC2240_XENC, TMC_ADDRESS_MASK, TMC_REGISTER_COUNT, TMC_WRITE_BIT, TMC2240_IHOLD_IRUN, TMC2240_IRUN_MASK, TMC2240_IRUN_SHIFT,
};

use core::cell::RefCell;

use critical_section::Mutex;

static SPI: Mutex<RefCell<Option<Spi<hal::peripherals::SPI2, FullDuplexMode>>>> =
    Mutex::new(RefCell::new(None));

// static STEP: Mutex<RefCell<Option<Timer<Timer0<TIMG0>>>>> = Mutex::new(RefCell::new(None));

static ALARM0: Mutex<RefCell<Option<Alarm<Periodic, 0>>>> = Mutex::new(RefCell::new(None));

static STEP: Mutex<RefCell<Option<Gpio17<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static DIR: Mutex<RefCell<Option<Gpio18<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));

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

    let mut ledc = LEDC::new(
        peripherals.LEDC,
        &clocks,
        &mut system.peripheral_clock_control,
    );

    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.get_timer::<LowSpeed>(timer::Number::Timer0);

    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty1Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 16u32.MHz(),
        })
        .log()
        .unwrap();

    let mut channel0 = ledc.get_channel(channel::Number::Channel0, clk);
    channel0
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 50,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .log()
        .unwrap();

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

    let t_step = MicrosDurationU32::millis(4);
    let f_step = t_step.into_rate();
    println!("t_step {} f_step {}", t_step, f_step);

    let timer = SystemTimer::new(peripherals.SYSTIMER);
    let alarm = timer.alarm0.into_periodic();
    alarm.set_period(f_step);
    alarm.clear_interrupt();
    alarm.interrupt_enable(true);

    critical_section::with(|cs| {
        SPI.borrow_ref_mut(cs).replace(spi);
        ALARM0.borrow_ref_mut(cs).replace(alarm);
        STEP.borrow_ref_mut(cs).replace(step);
        DIR.borrow_ref_mut(cs).replace(dir);
    });

    unsafe {
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
    let mut register_reset_state: [u32; TMC2240_REGISTER_COUNT as usize] =
        tmc2240_defaultRegisterResetState.clone();

    set_field(
        &mut register_reset_state,
        TMC2240_IHOLD_IRUN,
        TMC2240_IRUN_MASK,
        TMC2240_IRUN_SHIFT,
        28,
    );

    // enable PWM mode in general conf
    set_field(
        &mut register_reset_state,
        TMC2240_GCONF,
        TMC2240_EN_PWM_MODE_MASK,
        TMC2240_EN_PWM_MODE_SHIFT,
        1,
    );

    // enable autograd, autoscale, current measurement
    set_field(
        &mut register_reset_state,
        TMC2240_PWMCONF,
        TMC2240_PWM_AUTOGRAD_MASK,
        TMC2240_PWM_AUTOGRAD_SHIFT,
        1,
    );
    set_field(
        &mut register_reset_state,
        TMC2240_PWMCONF,
        TMC2240_PWM_AUTOSCALE_MASK,
        TMC2240_PWM_AUTOSCALE_SHIFT,
        1,
    );
    set_field(
        &mut register_reset_state,
        TMC2240_PWMCONF,
        TMC2240_PWM_MEAS_SD_ENABLE_MASK,
        TMC2240_PWM_MEAS_SD_ENABLE_SHIFT,
        1,
    );

    // default 0 recommended for 16MHz clk
    set_field(
        &mut register_reset_state,
        TMC2240_PWMCONF,
        TMC2240_PWM_FREQ_MASK,
        TMC2240_PWM_FREQ_SHIFT,
        0,
    );

    set_field(
        &mut register_reset_state,
        TMC2240_CHOPCONF,
        TMC2240_MRES_MASK,
        TMC2240_MRES_SHIFT,
        8,
    );
    set_field(
        &mut register_reset_state,
        TMC2240_CHOPCONF,
        TMC2240_INTPOL_MASK,
        TMC2240_INTPOL_SHIFT,
        1,
    );

    // configure CHOPCONF - not used, but must be configured to run the motor
    set_field(
        &mut register_reset_state,
        TMC2240_CHOPCONF,
        TMC2240_TOFF_MASK,
        TMC2240_TOFF_SHIFT,
        3,
    );
    set_field(
        &mut register_reset_state,
        TMC2240_CHOPCONF,
        TMC2240_TBL_MASK,
        TMC2240_TBL_SHIFT,
        2,
    );
    set_field(
        &mut register_reset_state,
        TMC2240_CHOPCONF,
        TMC2240_HSTRT_TFD210_MASK,
        TMC2240_HSTRT_TFD210_SHIFT,
        4,
    );
    set_field(
        &mut register_reset_state,
        TMC2240_CHOPCONF,
        TMC2240_HEND_OFFSET_MASK,
        TMC2240_HEND_OFFSET_SHIFT,
        0,
    );

    let register_reset_state = unsafe { transmute(register_reset_state) };
    let mut mcu = tmc_rs::new(0, config, &register_reset_state, None);

    println!("resetting mcu");
    unsafe { tmc2240_reset(&mut mcu) };

    // clear status flags, check GSTAT

    write_spi_bytes(TMC2240_GSTAT as u8, [255; 4]);
    let stat = GStat::from(read_spi(TMC2240_GSTAT as u8)[4]);
    println!("gstat {:?}", stat);

    interrupt::enable(
        peripherals::Interrupt::SYSTIMER_TARGET0,
        Priority::Priority1,
    )
    .log()
    .unwrap();

    en.set_low().unwrap();

    let mut delay = Delay::new(&clocks);

    loop {
        // wait for register writes
        unsafe {
            while (*mcu.config).state > 0 {
                periodic(&mut mcu, SystemTimer::now() as u32);
                continue;
            }
        }

        delay.delay_ms(1000u32);

        let resp = read_spi(TMC2240_MSCNT as u8);
        println!("mscnt {:?}", resp[4]);

        let resp = read_spi(TMC2240_CHOPCONF as u8);
        println!("chopconf[4] {:b}", resp[4]);

        // let resp = critical_section::with(|cs| {
        //     let mut spi = SPI.borrow_ref_mut(cs);
        //     let spi = spi.as_mut().unwrap();

        //     spi.transfer(&mut [0, 1, 2, 3, 4]).log().unwrap();
        //     let mut msg = [0, 1, 2, 3, 4];
        //     let resp = spi.transfer(&mut msg).log().unwrap();
        //     println!("{:x?}", resp);
        //     resp[0]
        // });

        // let status = SPIStatus::from(resp);
        // println!("{:#?}", status);

        let resp = read_spi(TMC2240_DRVSTATUS as u8);
        let stat = u32::from_be_bytes([resp[1], resp[2], resp[3], resp[4]]);
        println!(
            "ola {}",
            read_field(stat, TMC2240_OLA_MASK, TMC2240_OLA_SHIFT)
        );
        println!(
            "s2gb {}",
            read_field(stat, TMC2240_S2GB_MASK, TMC2240_S2GB_SHIFT)
        );
        println!(
            "s2ga {}",
            read_field(stat, TMC2240_S2GA_MASK, TMC2240_S2GA_SHIFT)
        );
        println!(
            "otpw {}",
            read_field(stat, TMC2240_OTPW_MASK, TMC2240_OTPW_SHIFT)
        );
        println!("ot {}", read_field(stat, TMC2240_OT_MASK, TMC2240_OT_SHIFT));
        println!(
            "cs_actual {}",
            read_field(stat, TMC2240_CS_ACTUAL_MASK, TMC2240_CS_ACTUAL_SHIFT)
        );
        println!(
            "fsactive {}",
            read_field(stat, TMC2240_FSACTIVE_MASK, TMC2240_FSACTIVE_SHIFT)
        );
        println!(
            "stealth {}",
            read_field(stat, TMC2240_STEALTH_MASK, TMC2240_STEALTH_SHIFT)
        );
        println!(
            "s2vsb {}",
            read_field(stat, TMC2240_S2VSB_MASK, TMC2240_S2VSB_SHIFT)
        );
        println!(
            "s2vsa {}",
            read_field(stat, TMC2240_S2VSA_MASK, TMC2240_S2VSA_SHIFT)
        );
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

fn write_spi_bytes(address: u8, value: [u8; 4]) {
    let mut data: [u8; 5] = [
        // set write bit
        address | 0b10000000,
        value[0],
        value[1],
        value[2],
        value[3],
    ];

    critical_section::with(|cs| {
        let mut spi = SPI.borrow_ref_mut(cs);
        let spi = spi.as_mut().unwrap();

        println!("writing to spi: {:x?}", data);

        spi.transfer(&mut data[..])
            .expect("Symmetric transfer failed");

        println!("wrote to spi, got {:x?}", data);
        println!("spi status: {:?}", SPIStatus::from(data[0]));
    })
}

fn write_spi(address: u8, value: i32) {
    let value_bytes = value.to_be_bytes();
    let mut data: [u8; 5] = [
        // set write bit
        address | (TMC_WRITE_BIT as u8),
        value_bytes[0],
        value_bytes[1],
        value_bytes[2],
        value_bytes[3],
    ];

    critical_section::with(|cs| {
        let mut spi = SPI.borrow_ref_mut(cs);
        let spi = spi.as_mut().unwrap();

        println!("writing to spi: {:x?} {:x?}", address, data);

        spi.transfer(&mut data[..])
            .expect("Symmetric transfer failed");

        println!("wrote to spi, got {:x?}", data);
        println!("spi status: {:?}", SPIStatus::from(data[0]));
    })
}

fn read_spi(address: u8) -> [u8; 5] {
    critical_section::with(|cs| {
        let mut spi = SPI.borrow_ref_mut(cs);
        let spi = spi.as_mut().unwrap();

        let mut data = [address as u8 & 0b01111111, 0, 0, 0, 0];
        spi.transfer(&mut data[..])
            .expect("Symmetric transfer failed");
        println!("reading from SPI, last response {:x?}", data);
        println!("spi status: {:?}", SPIStatus::from(data[0]));

        let mut data = [address as u8 & 0b01111111, 0, 0, 0, 0];
        spi.transfer(&mut data[..])
            .expect("Symmetric transfer failed");
        println!("read from SPI, actual response {:x?}", data);
        println!("spi status: {:?}", SPIStatus::from(data[0]));

        data
    })
}

fn set_field(
    reg: &mut [u32; TMC_REGISTER_COUNT as usize],
    addr: u32,
    mask: u32,
    shift: u32,
    value: u32,
) {
    let addr = addr & TMC_ADDRESS_MASK;
    println!(
        "setting: {:x?} to {:02x}                         ({:08b})",
        addr, value, value
    );
    println!("mask:             ({:032b})", mask);

    let shifted = ((value << shift) & mask) as u32;
    println!("shifted:          ({:032b})", shifted);

    let old_value = reg[addr as usize];
    println!("old:     {:08x?} ({:032b})", old_value, old_value);
    let cleared = (old_value) & (!(mask));
    println!("cleared: {:08x?} ({:032b})", cleared, cleared);
    let new = cleared | shifted;
    println!("new:     {:08x?} ({:032b})", new, new);
    println!("");
    reg[addr as usize] = new;
}

fn read_field(value: u32, mask: u32, shift: u32) -> u32 {
    return (value & mask) >> shift;
}

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

#[interrupt]
fn SYSTIMER_TARGET0() {
    critical_section::with(|cs| {
        let mut step = STEP.borrow_ref_mut(cs);
        let step = step.as_mut().unwrap();
        step.toggle().log().unwrap();

        ALARM0
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();
    });
}
