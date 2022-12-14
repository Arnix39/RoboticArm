//! Control a continous rotation servo with a Maker Pi RP2040
#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;

use panic_probe as _;

// Make an alias for our board support package so copying examples to other boards is easier
use cytron_maker_pi_rp2040 as bsp;

// Pull in any important traits
use bsp::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use bsp::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use bsp::hal;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure GPIO0 status led
    let led_pin = &mut pins.grove_1_a.into_push_pull_output();

    // Configure PWM7
    let pwm = &mut pwm_slices.pwm7;
    pwm.set_ph_correct();
    pwm.set_div_int(20u8); // 50 hz
    pwm.enable();

    // Output channel B on PWM7 to the servo_4 pin
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.servo_4);

    info!("Stop!");
    channel.set_duty(4645); /* Counter clockwise limit: 4815, Clockwise limit: 4475 */
    delay.delay_ms(1000);

    loop {
        info!("Counter Clockwise!");
        led_pin.set_low().unwrap();
        channel.set_duty(5500);
        delay.delay_ms(100);
        info!("Stop!");
        channel.set_duty(4645);
        delay.delay_ms(1000);
        info!("Clockwise!");
        led_pin.set_high().unwrap();
        channel.set_duty(3500);
        delay.delay_ms(100);
        info!("Stop!");
        channel.set_duty(4645);
        delay.delay_ms(1000);
    }
}

// End of file
