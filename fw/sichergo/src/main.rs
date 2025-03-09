//! Testing PWM output for pre-defined pin combination: all pins for default mapping

#![deny(unsafe_code)]
#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::asm;
use cortex_m_rt::entry;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    time::ms,
    timer::{Channel, Tim1NoRemap},
};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain();

    let mut gpiob = p.GPIOB.split();
    // let mut gpiob = p.GPIOB.split();

    // TIM1
    let c1 = gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh);
    let pins = c1;

    //let mut pwm =
    //    Timer::new(p.TIM2, &clocks).pwm_hz::<Tim2NoRemap, _, _>(pins, &mut afio.mapr, 1.kHz());
    // or
    let mut pwm = p
        .TIM1
        .pwm_hz::<Tim1NoRemap, _, _>(pins, &mut afio.mapr, 1.Hz(), &clocks);

    // Enable clock on each of the channels
    pwm.enable(Channel::C1);

    //// Operations affecting all defined channels on the Timer

    // Adjust period to 0.5 seconds
    // pwm.set_period(ms(500).into_rate());
    // // Return to the original frequency
    // pwm.set_period(1.kHz());

    let max = pwm.get_max_duty();

    //// Operations affecting single channels can be accessed through
    //// the Pwm object or via dereferencing to the pin.

    // Use the Pwm object to set C3 to full strength
    // pwm.set_duty(Channel::C1, max);

    // // Use the Pwm object to set C3 to be dim
    // pwm.set_duty(Channel::C1, max / 4);

    // // Use the Pwm object to set C3 to be zero
    // pwm.set_duty(Channel::C1, 0);

    // // Extract the PwmChannel for C3
    // let mut pwm_channel = pwm.split().2;

    // // Use the PwmChannel object to set C3 to be full strength
    // pwm_channel.set_duty(max);

    // // Use the PwmChannel object to set C3 to be dim
    // pwm_channel.set_duty(max / 4);

    // // Use the PwmChannel object to set C3 to be zero
    // pwm_channel.set_duty(0);

    loop {}
}
