#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

#[entry]
fn main () -> ! {
    let per = stm32f1::stm32f103::Peripherals::take().unwrap();

    let rcc = per.RCC;

    // Enable HSE (External Oscillator)
    rcc.cr.modify(|_, w| w.hseon().set_bit());
    while rcc.cr.read().hserdy().bit_is_clear() {}
    
    // Enable Peripheral clocks (TIM1 and GPIOB)
    rcc.apb2enr.write(|w| {
        w.iopben().set_bit().tim1en().set_bit()
    });
    // Enable Peripheral clock (TIM2)
    rcc.apb1enr.write(|w| w.tim2en().set_bit());

    // Clear gate output by default
    per.GPIOB.crh.write(|w| w.cnf13().push_pull().mode13().output());
    per.GPIOB.odr.write(|w| w.odr13().set_bit());

    // Set PB13 (Gate Pin) as default alternate mode push pull, default speed
    per.GPIOB.crh.write(|w| w.mode13().output().cnf13().alt_push_pull());

    // Activate Timer output compare with PWM mode 1 (0b110)
    // Activate Channel 1 Output (0b00)
    per.TIM1.ccmr1_output().write(|w| unsafe {
        w.oc1m().bits(0b110).cc1s().bits(0b00)
    });

    // Activate Main Output Enable (MOE) for CC1N
    per.TIM1.bdtr.write(|w| w.moe().set_bit().ossr().clear_bit());

    // Set a count of 1.25s (1/8MHz * 1000 * 10000 = 1.25s)
    let psc = 10;
    let cnt = 5000;
    let mut dc = 0;
    per.TIM1.psc.write(|w| w.psc().bits(psc));
    per.TIM1.arr.write(|w| w.arr().bits(cnt));
    per.TIM1.ccr1().write(|w| w.ccr().bits(dc));

    // Activate the Timer (duh!)
    per.TIM1.cr1.modify(|_, w| w.cen().set_bit());

    // Activate complementary output (CC1NE) and disable primary output (CC1E)
    per.TIM1.ccer.write(|w| w.cc1ne().set_bit().cc1e().clear_bit());

    // Set TIM2 as a base counter for loop delay
    per.TIM2.arr.write(|w| w.arr().bits(1000));
    per.TIM2.psc.write(|w| w.psc().bits(1000));
    
    // Enable UF/OF interrupt flag
    per.TIM2.dier.write(|w| w.uie().set_bit());
    
    per.TIM2.cr1.write(|w| w.cen().set_bit());

    let mut i = 0;
    let duties = [200, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 4900, 4900, 4500, 4000, 3500, 3000, 2500, 2000, 1500, 1000, 200, 200, 200, 200];

    /* ! promise loop */
    loop {
        // Block until Update event (Delay)
        while per.TIM2.sr.read().uif().is_clear() {}
        // Clear flag
        per.TIM2.sr.write(|w| w.uif().clear_bit());

        dc = duties[i];
        i = (i + 1) % duties.len();
        per.TIM1.ccr1().write(|w| w.ccr().bits(dc));
    }
}
