//! Control a continous rotation servo with a Maker Pi RP2040
#![no_std]
#![no_main]

use defmt_brtt as _; // global logger

use panic_probe as _;

use rp2040_monotonic::{
    fugit::Duration,
    fugit::RateExtU32, // For .kHz() conversion funcs
    Rp2040Monotonic,
};

// Pull in any important traits
use cytron_maker_pi_rp2040::{
    XOSC_CRYSTAL_FREQ,
    hal,
    hal::{pac, I2C, gpio, pwm},
    Pins
};

use core::mem::MaybeUninit;

// GPIO traits
use embedded_hal::PwmPin;
// I2C traits
use embedded_hal::blocking::i2c::Write;

const MONO_NUM: u32 = 1;
const MONO_DENOM: u32 = 1000000;
const ONE_MSEC_TICKS: u64 = 1000;

type I2CBus = I2C<
    pac::I2C0,
    (
        gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionI2C, gpio::PullDown>,
        gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionI2C, gpio::PullDown>,
    ),
>;

type BasePWMChannel = pwm::Channel<pwm::Slice<pwm::Pwm7, pwm::FreeRunning>, pwm::B>;
type GripperPWMChannel = hal::pwm::Channel<pwm::Slice<pwm::Pwm6, pwm::FreeRunning>, hal::pwm::A>;

pub enum GripperState {
    OPEN,
    CLOSED,
}

pub enum RotationDirection {
    CLOCKWISE,
    COUNTERCLOCKWISE,
    NONE,
}

fn duration_in_ticks(ticks: u64) -> Duration::<u64, MONO_NUM, MONO_DENOM> {
    Duration::<u64, MONO_NUM, MONO_DENOM>::from_ticks(ticks)
}

#[rtic::app(
    device = cytron_maker_pi_rp2040::hal::pac,
    peripherals = true,
    dispatchers = [TIMER_IRQ_1, TIMER_IRQ_2],
)]
mod app {
    use super::*;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Rp2040Mono = Rp2040Monotonic;

    #[shared]
    struct Shared {
        gripper_state: GripperState,
        base_rotation: RotationDirection,
    }

    #[local]
    struct Local {
        base: BasePWMChannel,
        gripper: GripperPWMChannel,
        i2c: &'static mut I2CBus,
    }

    #[init(local=[
        // Task local initialized resources are static
        // Here we use MaybeUninit to allow for initialization in init()
        // This enables its usage in driver initialization
        i2c_context: MaybeUninit<I2CBus> = MaybeUninit::uninit(),
    ])]
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

        ///////////////////////////////////////////
        /*  PWM channels and servos assignement  */
        ///////////////////////////////////////////
        /* PWM6 channel A => Servo 1 => Gripper  */
        /* PWM7 channel B => Servo 4 => Base     */
        ///////////////////////////////////////////

        // Configure PWM for base control
        let mut base_pwm = pwm_slices.pwm7;
        base_pwm.set_ph_correct();
        base_pwm.set_div_int(20u8); // 50 hz
        base_pwm.enable();
        let mut base = base_pwm.channel_b;
        base.output_to(pins.servo_4);

        // Configure PWM for gripper control
        let mut gripper_pwm = pwm_slices.pwm6;
        gripper_pwm.set_ph_correct();
        gripper_pwm.set_div_int(20u8); // 50 hz
        gripper_pwm.enable();
        let mut gripper = gripper_pwm.channel_a;
        gripper.output_to(pins.servo_1);

        // Configure Gpio0 and Gpio1 as being I2C
        let sda_pin = pins.grove_1_a.into_function::<gpio::FunctionI2C>();
        let scl_pin = pins.grove_1_b.into_function::<gpio::FunctionI2C>();

        // Init the I2C drive itself, using MaybeUninit to overwrite the previously
        // uninitialized i2c_context variable without dropping its value
        // (i2c_context defined in init local resources above)
        let i2c_tmp: &'static mut _ = context.local.i2c_context.write(I2C::i2c0(
            pac.I2C0,
            sda_pin,
            scl_pin,
            400.kHz(),
            &mut pac.RESETS,
            &clocks.system_clock,
        ));

        let mono = Rp2040Mono::new(pac.TIMER);

        accelerometer_reading::spawn().ok();
        servos_control::spawn().ok();
        base_control::spawn().ok();
        gripper_control::spawn().ok();

        (
            Shared {
                gripper_state: GripperState::CLOSED,
                base_rotation: RotationDirection::NONE,
            },
            Local {
                base,
                gripper,
                i2c: i2c_tmp,
            },
            init::Monotonics(mono)
        )
    }

    #[task(local = [i2c], priority = 1)]
    fn accelerometer_reading(context: accelerometer_reading::Context) {
        context.local.i2c.write(0x2c, &[1, 2, 3]).unwrap();
    }

    #[task(local = [counter: u32 = 0], shared = [gripper_state, base_rotation], priority = 2)]
    fn servos_control(mut context: servos_control::Context) {
        context.shared.base_rotation.lock(|base_rotation| {
            *base_rotation = RotationDirection::NONE;
        });

        if *context.local.counter % 100 == 0 {
            context.shared.gripper_state.lock(|gripper_state| {
                *gripper_state = GripperState::OPEN;
            });
            *context.local.counter = 0;
        } else {
            context.shared.gripper_state.lock(|gripper_state| {
                *gripper_state = GripperState::CLOSED;
            });
        }

        *context.local.counter += 1;

        // Re-spawn this task after 10 ms
        servos_control::spawn_after(duration_in_ticks(10 * ONE_MSEC_TICKS)).unwrap();
    }

    #[task(local = [gripper], shared = [gripper_state], priority = 1)]
    fn gripper_control(mut context: gripper_control::Context) {
        // Control the gripper
        context.shared.gripper_state.lock(|gripper_state| {
            match *gripper_state {
                GripperState::OPEN => {
                    defmt::info!("Open!");
                    context.local.gripper.set_duty(5500);
                },
                GripperState::CLOSED => {
                    defmt::info!("Close!");
                    context.local.gripper.set_duty(2350);
                }
            }
        });

        // Re-spawn this task after 10 ms
        gripper_control::spawn_after(duration_in_ticks(10 * ONE_MSEC_TICKS)).unwrap();
    }

    #[task(local = [base], shared = [base_rotation], priority = 1)]
    fn base_control(mut context: base_control::Context) {
        // Control the base
        context.shared.base_rotation.lock(|base_rotation| {
            match *base_rotation {
                RotationDirection::COUNTERCLOCKWISE => {
                    defmt::info!("Counter Clockwise!");
                    context.local.base.set_duty(5500);
                },
                RotationDirection::CLOCKWISE => {
                    defmt::info!("Clockwise!");
                    context.local.base.set_duty(2350);
                },
                _ => {
                    defmt::info!("Stop!");
                    context.local.base.set_duty(4645); /* Counter clockwise limit: 4815, Clockwise limit: 4475 */
                }
            }
        });

        // Re-spawn this task after 10 ms
        base_control::spawn_after(duration_in_ticks(10 * ONE_MSEC_TICKS)).unwrap();
    }
}

// End of file
