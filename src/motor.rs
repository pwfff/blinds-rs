//! Board/motor settings for the SpreadCycle smoke test, not finished motor tuning.
use core::time::Duration;
use tmc_rs::tmc2240::{
    BlankTime, ChopperConfig, ClockFrequency, ClockSource, CurrentConfig, CurrentTimingConfig,
    Microsteps, MilliAmps, MotorConfig, SpreadCycleConfig, StepEdge, SteppingConfig,
    ValidatedConfig,
};

// BIGTREETECH TMC2240 V1.0: R1 is 12 kΩ from IREF to ground.
// STEPPERONLINE 17HS19-2004S1: 2 A/phase, 1.8° (200 full steps/rev).
// Existing wired setup with the board's heatsink; sources are in README.md.
pub const REFERENCE_RESISTANCE_OHMS: u32 = 12_000;
// Conservative ceiling: 1.4 A RMS ~= 1.98 A sinusoidal peak. The motor's
// datasheet says 2 A/phase, not explicitly 2 A sinusoidal peak or RMS.
pub const MAX_CURRENT: MilliAmps = MilliAmps(1400);
// Starting values for testing, not a guarantee of sufficient load/holding torque.
pub const RUN_CURRENT: MilliAmps = MilliAmps(900);
pub const HOLD_CURRENT: MilliAmps = MilliAmps(450);

pub const CLOCK_HZ: u32 = 16_000_000;
pub const CLOCK: ClockSource = ClockSource::External(match ClockFrequency::from_hz(CLOCK_HZ) {
    Some(clock) => clock,
    None => panic!("invalid driver clock"),
});
pub const STEPPING: SteppingConfig = SteppingConfig {
    microsteps: Microsteps::M256,
    interpolate_to_256: true,
    step_edge: StepEdge::Rising,
};
// Preserve the old 128 kHz / 256 microsteps = 500 full steps/s demonstration.
// No acceleration ramp or end-stop handling: disconnect the blinds for this test.
pub const STEP_HZ: u32 = 500 * STEPPING.microsteps.per_full_step() as u32;
pub const CURRENT: CurrentConfig = match CurrentConfig::for_motor(
    REFERENCE_RESISTANCE_OHMS,
    MAX_CURRENT,
    RUN_CURRENT,
    HOLD_CURRENT,
) {
    Ok(current) => current,
    Err(_) => panic!("invalid motor current settings"),
};
pub const TIMING: CurrentTimingConfig = match CurrentTimingConfig::new(
    CLOCK.frequency(),
    Duration::from_micros(163_840), // old TPOWERDOWN=10 at 16 MHz
    Duration::from_micros(128),     // old IRUNDELAY=4
    Duration::from_micros(16_384),  // old IHOLDDELAY=1
) {
    Ok(timing) => timing,
    Err(_) => panic!("invalid current transition timing"),
};
pub const MOTOR: ValidatedConfig = match (MotorConfig {
    clock: CLOCK,
    current: CURRENT,
    stepping: STEPPING,
    timing: TIMING,
    chopper: ChopperConfig::SpreadCycle(SpreadCycleConfig {
        // Datasheet Figure 32 starting point; tune on the actual motor.
        off_time_clocks: 184,
        blank_time: BlankTime::Clocks36,
        hysteresis_start_increment: 1,
        hysteresis_end: -3,
        passive_fast_decay_clocks: 512,
    }),
})
.validate()
{
    Ok(profile) => profile,
    Err(_) => panic!("invalid motor profile"),
};

#[cfg(test)]
mod tests {
    use super::*;
    use tmc_rs::registers::tmc2240::TMC2240;

    #[test]
    fn checked_image_matches_motion_and_timing() {
        let mut registers = TMC2240::default();
        assert_eq!(MOTOR.stage(&mut registers).len(), 22);
        assert!(!registers.GCONF.en_pwm_mode());
        assert_eq!(registers.CHOPCONF.MRES(), 0);
        assert!(!registers.CHOPCONF.dedge());
        assert_eq!(STEP_HZ, 128_000);
        assert_eq!(registers.TPOWERDOWN.TPOWERDOWN(), 10);
        assert_eq!(registers.IHOLD_IRUN.IRUNDELAY(), 4);
        assert_eq!(registers.IHOLD_IRUN.IHOLDDELAY(), 1);
        assert_eq!(registers.TCOOLTHRS.TCOOLTHRS(), 0);
        assert_eq!(registers.TPWMTHRS.TPWMTHRS(), 0);
    }

    #[test]
    fn nominal_currents_do_not_exceed_requests() {
        assert!(CURRENT.max_rms() <= MAX_CURRENT);
        assert!(CURRENT.run_rms() <= RUN_CURRENT);
        assert!(CURRENT.hold_rms() <= HOLD_CURRENT);
        assert!(CURRENT.hold_rms().0 > 0);
        // Keep the nominal sinusoidal peak below the motor's 2 A/phase rating.
        let max_rms = CURRENT.max_rms().0 as u64;
        assert!(2 * max_rms * max_rms <= 2000 * 2000);
    }
}
