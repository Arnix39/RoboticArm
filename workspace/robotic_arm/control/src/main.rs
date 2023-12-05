//! Control a continous rotation servo with a Maker Pi RP2040
#![no_std]
#![no_main]

use defmt_brtt as _; // global logger

use panic_probe as _;

use mpu6050::*;

use rp2040_monotonic::{
    fugit::Duration,
    fugit::RateExtU32, // For .kHz() conversion funcs
    Rp2040Monotonic,
};

use cytron_maker_pi_rp2040::{
    XOSC_CRYSTAL_FREQ,
    hal,
    hal::{pac, I2C, gpio, pwm},
    Pins
};

// PWM traits
use embedded_hal::PwmPin;

const MONO_NUM: u32 = 1;
const MONO_DENOM: u32 = 1000000;
const ONE_MSEC_TICKS: u64 = 1000;
const ONE_SEC_TICKS: u64 = 1000000;

type I2CBus = I2C<
    pac::I2C0,
    (
        gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionI2C, gpio::PullDown>,
        gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionI2C, gpio::PullDown>,
    ),
>;

type BasePWMChannel = pwm::Channel<pwm::Slice<pwm::Pwm7, pwm::FreeRunning>, pwm::B>;
type RightPWMChannel = pwm::Channel<pwm::Slice<pwm::Pwm7, pwm::FreeRunning>, pwm::A>;
type LeftPWMChannel = pwm::Channel<pwm::Slice<pwm::Pwm6, pwm::FreeRunning>, pwm::B>;
type GripperPWMChannel = pwm::Channel<pwm::Slice<pwm::Pwm6, pwm::FreeRunning>, pwm::A>;

#[derive(PartialEq)]
pub enum GripperState {
    OPEN,
    CLOSED,
}

const OPEN_PWM: u16 = 55000;
const CLOSED_PWM: u16 = 23750;

#[derive(PartialEq)]
pub enum RotationDirection {
    CLOCKWISE,
    COUNTERCLOCKWISE,
    NONE,
}

const ROT_CLOCKWISE_PWM:u16 = 42500;
const ROT_COUNTERCLOCKWISE_PWM:u16 = 50500;
const NO_ROT_PWM:u16 = 46500; /* 4645 Counter clockwise limit: 4815, Clockwise limit: 4475 */

fn duration_from_ticks(ticks: u64) -> Duration::<u64, MONO_NUM, MONO_DENOM> {
    Duration::<u64, MONO_NUM, MONO_DENOM>::from_ticks(ticks)
}

#[rtic::app(
    device = cytron_maker_pi_rp2040::hal::pac,
    peripherals = true,
    dispatchers = [SW0_IRQ, SW1_IRQ],
)]
mod app {
    use super::*;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Rp2040Mono = Rp2040Monotonic;

    #[shared]
    struct Shared {
        gripper_state: GripperState,
        base_rotation: RotationDirection,
        right_rotation: RotationDirection,
        left_rotation: RotationDirection,
    }

    #[local]
    struct Local {
        base: BasePWMChannel,
        gripper: GripperPWMChannel,
        right_servo: RightPWMChannel,
        left_servo: LeftPWMChannel,
        mpu: Mpu6050<I2CBus>,
    }

    #[init]
    fn init(context: init::Context) -> (Shared, Local, init::Monotonics) {
        // Cortex-M peripherals
        let _core: pac::CorePeripherals = context.core;

        // Cytron board specific peripherals
        let mut pac: pac::Peripherals = context.device;

        // Set up the watchdog driver - needed by the clock setup code
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

        // Configure the clocks
        let clocks = hal::clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // The single-cycle I/O block controls GPIO pins
        let sio = hal::Sio::new(pac.SIO);

        // Set the pins up
        let pins = Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        // Init PWMs
        let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);

        // Configure PWM6
        let mut pwm6 = pwm_slices.pwm6;
        pwm6.set_ph_correct();
        pwm6.set_div_int(2u8); // 500 hz
        pwm6.enable();

        // Configure PWM7
        let mut pwm7 = pwm_slices.pwm7;
        pwm7.set_ph_correct();
        pwm7.set_div_int(2u8); // 500 hz
        pwm7.enable();

        // Configure PWM for base control
        let mut base = pwm7.channel_b;
        base.output_to(pins.servo_4);

        // Configure PWM for right servo control
        let mut right_servo = pwm7.channel_a;
        right_servo.output_to(pins.servo_3);

        // Configure PWM for left servo control
        let mut left_servo = pwm6.channel_b;
        left_servo.output_to(pins.servo_2);
    
        // Configure PWM for gripper control
        let mut gripper = pwm6.channel_a;
        gripper.output_to(pins.servo_1);

        // Configure I2C pins
        let sda_pin = pins.grove_1_a.into_function::<gpio::FunctionI2C>();
        let scl_pin = pins.grove_1_b.into_function::<gpio::FunctionI2C>();

        // Create the I2C drive
        let i2c = I2C::i2c0(
            pac.I2C0,
            sda_pin,
            scl_pin,
            400.kHz(),
            &mut pac.RESETS,
            &clocks.system_clock);

        // Create the MPU6050 sensor
        let mpu = Mpu6050::new(i2c);

        // Create the monotonic timer
        let mono = Rp2040Mono::new(pac.TIMER);

        // Spawn the tasks
        accelerometer_reading::spawn().ok();
        servos_control::spawn().ok();
        base_control::spawn().ok();
        right_control::spawn().ok();
        left_control::spawn().ok();
        gripper_control::spawn().ok();

        (
            Shared {
                gripper_state: GripperState::CLOSED,
                base_rotation: RotationDirection::NONE,
                right_rotation: RotationDirection::NONE,
                left_rotation: RotationDirection::NONE,
            },
            Local {
                gripper,
                base,
                right_servo,
                left_servo,
                mpu,
            },
            init::Monotonics(mono)
        )
    }

    // Task that controls the state of the servos
    #[task(
        local = [
            counter: u32 = 0,
            previous_base_rotation: RotationDirection = RotationDirection::NONE,
            previous_right_rotation: RotationDirection = RotationDirection::NONE,
            previous_left_rotation: RotationDirection = RotationDirection::NONE,
        ],
        shared = [gripper_state, base_rotation, right_rotation, left_rotation],
        priority = 2
    )]
    fn servos_control(mut context: servos_control::Context) {
        // Change base rotation direction every 500 milliseconds
        if *context.local.counter % 50 == 0 {
            context.shared.base_rotation.lock(|base_rotation| {
                match *base_rotation {
                    RotationDirection::CLOCKWISE => {
                        defmt::info!("Base: Stop!");
                        *context.local.previous_base_rotation = RotationDirection::CLOCKWISE;
                        *base_rotation = RotationDirection::NONE;
                    },
                    RotationDirection::COUNTERCLOCKWISE => {
                        defmt::info!("Base: Stop!");
                        *context.local.previous_base_rotation = RotationDirection::COUNTERCLOCKWISE;
                        *base_rotation = RotationDirection::NONE;
                    },
                    RotationDirection::NONE => {
                        if *context.local.previous_base_rotation == RotationDirection::CLOCKWISE {
                            defmt::info!("Base: Counter-clockwise!");
                            *context.local.previous_base_rotation = RotationDirection::NONE;
                            *base_rotation = RotationDirection::COUNTERCLOCKWISE;
                        } else {
                            defmt::info!("Base: Clockwise!");
                            *context.local.previous_base_rotation = RotationDirection::NONE;
                            *base_rotation = RotationDirection::CLOCKWISE;
                        }
                    },
                }
            });
        }

        // Change right direction every 750 milliseconds
        if *context.local.counter % 75 == 0 {
            context.shared.right_rotation.lock(|right_rotation| {
                match *right_rotation {
                    RotationDirection::CLOCKWISE => {
                        defmt::info!("Right: Stop!");
                        *context.local.previous_right_rotation = RotationDirection::CLOCKWISE;
                        *right_rotation = RotationDirection::NONE;
                    },
                    RotationDirection::COUNTERCLOCKWISE => {
                        defmt::info!("Right: Stop!");
                        *context.local.previous_right_rotation = RotationDirection::COUNTERCLOCKWISE;
                        *right_rotation = RotationDirection::NONE;
                    },
                    RotationDirection::NONE => {
                        if *context.local.previous_right_rotation == RotationDirection::COUNTERCLOCKWISE {
                            defmt::info!("Right: Clockwise!");
                            *context.local.previous_right_rotation = RotationDirection::NONE;
                            *right_rotation = RotationDirection::CLOCKWISE;
                        } else {
                            defmt::info!("Right: Counter-clockwise!");
                            *context.local.previous_right_rotation = RotationDirection::NONE;
                            *right_rotation = RotationDirection::COUNTERCLOCKWISE;
                        }
                    },
                }
            });
        }

        // Change left direction every 750 milliseconds
        if *context.local.counter % 75 == 0 {
            context.shared.left_rotation.lock(|left_rotation| {
                match *left_rotation {
                    RotationDirection::CLOCKWISE => {
                        defmt::info!("Left: Stop!");
                        *context.local.previous_left_rotation = RotationDirection::CLOCKWISE;
                        *left_rotation = RotationDirection::NONE;
                    },
                    RotationDirection::COUNTERCLOCKWISE => {
                        defmt::info!("Left: Stop!");
                        *context.local.previous_left_rotation = RotationDirection::COUNTERCLOCKWISE;
                        *left_rotation = RotationDirection::NONE;
                    },
                    RotationDirection::NONE => {
                        if *context.local.previous_left_rotation == RotationDirection::CLOCKWISE {
                            defmt::info!("Left: Counter-clockwise!");
                            *context.local.previous_left_rotation = RotationDirection::NONE;
                            *left_rotation = RotationDirection::COUNTERCLOCKWISE;
                        } else {
                            defmt::info!("Left: Clockwise!");
                            *context.local.previous_left_rotation = RotationDirection::NONE;
                            *left_rotation = RotationDirection::CLOCKWISE;
                        }
                    },
                }
            });
        }

        // Change gripper state every second
        if *context.local.counter % 100 == 0 {
            context.shared.gripper_state.lock(|gripper_state| {
                if *gripper_state == GripperState::OPEN {
                    defmt::info!("Gripper: Closed!");
                    *gripper_state = GripperState::CLOSED;
                } else {
                    defmt::info!("Gripper: Open!");
                    *gripper_state = GripperState::OPEN
                }
            });

            *context.local.counter += 0;
        }

        *context.local.counter += 1;

        // Re-spawn this task after 10 ms
        servos_control::spawn_after(duration_from_ticks(10 * ONE_MSEC_TICKS)).unwrap();
    }

    // Task that actuates the gripper servo
    #[task(local = [gripper], shared = [gripper_state], priority = 1)]
    fn gripper_control(mut context: gripper_control::Context) {
        context.shared.gripper_state.lock(|gripper_state| {
            match *gripper_state {
                GripperState::OPEN => {
                    context.local.gripper.set_duty(OPEN_PWM);
                },
                GripperState::CLOSED => {
                    context.local.gripper.set_duty(CLOSED_PWM);
                }
            }
        });

        // Re-spawn this task after 10 ms
        gripper_control::spawn_after(duration_from_ticks(10 * ONE_MSEC_TICKS)).unwrap();
    }

    // Task that actuates the base servo
    #[task(local = [base], shared = [base_rotation], priority = 1)]
    fn base_control(mut context: base_control::Context) {
        context.shared.base_rotation.lock(|base_rotation| {
            match *base_rotation {
                RotationDirection::COUNTERCLOCKWISE => {
                    context.local.base.set_duty(ROT_COUNTERCLOCKWISE_PWM);
                },
                RotationDirection::CLOCKWISE => {
                    context.local.base.set_duty(ROT_CLOCKWISE_PWM);
                },
                _ => {
                    context.local.base.set_duty(NO_ROT_PWM);
                }
            }
        });

        // Re-spawn this task after 10 ms
        base_control::spawn_after(duration_from_ticks(10 * ONE_MSEC_TICKS)).unwrap();
    }

    // Task that actuates the right servo
    #[task(local = [right_servo], shared = [right_rotation], priority = 1)]
    fn right_control(mut context: right_control::Context) {
        context.shared.right_rotation.lock(|right_rotation| {
            match *right_rotation {
                RotationDirection::COUNTERCLOCKWISE => {
                    context.local.right_servo.set_duty(ROT_COUNTERCLOCKWISE_PWM);
                },
                RotationDirection::CLOCKWISE => {
                    context.local.right_servo.set_duty(ROT_CLOCKWISE_PWM);
                },
                _ => {
                    context.local.right_servo.set_duty(NO_ROT_PWM);
                }
            }
        });

        // Re-spawn this task after 10 ms
        right_control::spawn_after(duration_from_ticks(10 * ONE_MSEC_TICKS)).unwrap();
    }

    // Task that actuates the left servo
    #[task(local = [left_servo], shared = [left_rotation], priority = 1)]
    fn left_control(mut context: left_control::Context) {
        context.shared.left_rotation.lock(|left_rotation| {
            match *left_rotation {
                RotationDirection::COUNTERCLOCKWISE => {
                    context.local.left_servo.set_duty(ROT_COUNTERCLOCKWISE_PWM);
                },
                RotationDirection::CLOCKWISE => {
                    context.local.left_servo.set_duty(ROT_CLOCKWISE_PWM);
                },
                _ => {
                    context.local.left_servo.set_duty(NO_ROT_PWM);
                }
            }
        });

        // Re-spawn this task after 10 ms
        left_control::spawn_after(duration_from_ticks(10 * ONE_MSEC_TICKS)).unwrap();
    }

    // Task that reads values from the MPU6050 IMU
    #[task(local = [mpu], priority = 1)]
    fn accelerometer_reading(context: accelerometer_reading::Context) {
        let temp = context.local.mpu.get_temp().unwrap();
        defmt::info!("MPU temperature: {:?}", temp);

        // Re-spawn this task after 5 s
        accelerometer_reading::spawn_after(duration_from_ticks(5 * ONE_SEC_TICKS)).unwrap();
    }
}

// End of file
