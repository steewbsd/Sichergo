#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m::asm::delay;
use keyberon::{debounce::Debouncer, layout::Layout, matrix::Matrix};

use stm32f1xx_hal::{
    gpio::{ErasedPin, Input, Output, PullDown, PushPull},
    prelude::*,
    serial::*,
    timer::{CounterHz, Event},
    usb::{Peripheral, UsbBus, UsbBusType},
};

type KUsbClass = keyberon::Class<'static, UsbBusType, ()>;
type KUsbDevice = usb_device::device::UsbDevice<'static, UsbBusType>;

pub static LAYERS: keyberon::layout::Layers<10, 4, 1, ()> = keyberon::layout::layout! {
    { //[+··· ···+··· ···+··· ···+··· ···+···|···+··· ···+··· ···+··· ···+··· ···+],
        [Q       W       E       R       T       Y       U       I       O       P],
        [A       S       D       F       G       H       J       K       L       -],
        [Z       X       C       V       B       N       M       ,       .       /],
        [n       n    LShift   Space     n       n   RShift    RAlt     n       n],
    }
};

#[derive(Clone, Copy)]
enum PwmBreathDuty {
    MAX = 0,
    STATE1 = 1000,
    STATE2 = 1200,
    STATE3 = 1500,
    STATE4 = 1700,
    STATE5 = 2000,
    STATE6 = 2300,
    STATE7 = 2600,
    STATE8 = 3000,
    STATE9 = 3300,
    STATE10 = 3600,
    STATE11 = 4000,
    STATE12 = 4500,
    MIN = 5000,
}

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true)]
mod app {

    use keyberon::key_code::KbHidReport;
    use usb_device::class::UsbClass;

    use super::*;

    #[shared]
    struct Shared {
        duty_cycle: usize,
        matrix: Matrix<ErasedPin<Output<PushPull>>, ErasedPin<Input<PullDown>>, 5, 4>,
        debouncer: Debouncer<[[bool; 5]; 4]>,
        layout: Layout<10, 4, 1, ()>,
        usb_dev: KUsbDevice,
        usb_class: KUsbClass,
        serial: usbd_serial::SerialPort<'static, UsbBusType>,
    }

    #[local]
    struct Local {
        breath_cycle: usize,
        timer1: stm32f1xx_hal::pac::TIM1,
        timer2: stm32f1xx_hal::pac::TIM2,
        timer3: CounterHz<stm32f1xx_hal::pac::TIM3>,
        repetition: usize,
        breath_pattern: [(PwmBreathDuty, usize); 27],
    }

    #[init(local = [usb_bus: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None])]
    fn init(c: init::Context) -> (Shared, Local) {
        // Enable HSE (External Oscillator)
        // c.device.RCC.cr.modify(|_, w| w.hseon().set_bit());
        // while c.device.RCC.cr.read().hserdy().bit_is_clear() {}

        // Enable Peripheral clocks (TIM1, GPIOA and GPIOB)
        c.device
            .RCC
            .apb2enr()
            .write(|w| w.iopben().set_bit().tim1en().set_bit().iopaen().set_bit());
        // Enable Peripheral clock (TIM2)
        c.device.RCC.apb1enr().write(|w| w.tim2en().set_bit());

        // Activate Timer output compare with PWM mode 1 (0b110)
        // Activate Channel 1 Output (0b00)

        let rcc = c.device.RCC.constrain();
        let mut flash = c.device.FLASH.constrain();
        // let presc_config = Config::default();
        // let presc_config = Config {
        //     hse: None,
        //     pllmul: Some(4),
        //     hpre: HPre::Div1,
        //     ppre1: stm32f1xx_hal::rcc::PPre::Div2,
        //     ppre2: stm32f1xx_hal::rcc::PPre::Div1,
        //     usbpre: UsbPre::Div1,
        //     adcpre: AdcPre::Div2,
        //     hse_bypass: false,
        //     allow_overclock: false,
        // };

        let clocks = rcc
            .cfgr
            .use_hse(12.MHz())
            .sysclk(72.MHz())
            .pclk1(24.MHz())
            .pclk2(48.MHz())
            .freeze(&mut flash.acr);
        //            .freeze_with_config(presc_config, &mut flash.acr);

        if clocks.usbclk_valid() {
            //            asm::bkpt();
        }

        // let clocks = rcc.cfgr.sysclk(8.MHz()).freeze(&mut flash.acr);

        // Manual PAC initialization of the PWM Complementary channel as a
        // workaround for the lack of it on the HAL
        c.device
            .TIM1
            .ccmr1_output()
            .write(|w| unsafe { w.oc1m().bits(0b110).cc1s().bits(0b00) });

        // Activate Main Output Enable (MOE) for CC1N
        c.device
            .TIM1
            .bdtr()
            .write(|w| w.moe().set_bit().ossr().clear_bit());

        // Set a count of 1.25s (1/8MHz * 1000 * 10000 = 1.25s)
        let psc = 10;
        let cnt = 5000;
        c.device.TIM1.psc().write(|w| unsafe { w.psc().bits(psc) });
        c.device.TIM1.arr().write(|w| unsafe { w.arr().bits(cnt) });
        c.device.TIM1.ccr1().write(|w| unsafe { w.ccr().bits(0) });

        // Activate the Timer (duh!)
        c.device.TIM1.cr1().modify(|_, w| w.cen().set_bit());

        // Activate complementary output (CC1NE) and disable primary output (CC1E)
        c.device
            .TIM1
            .ccer()
            .write(|w| w.cc1ne().set_bit().cc1e().clear_bit());

        // Set TIM2 as a base counter for loop delay
        c.device.TIM2.arr().write(|w| unsafe { w.arr().bits(1750) });
        c.device.TIM2.psc().write(|w| unsafe { w.psc().bits(200) });

        // Enable UF/OF interrupt flag
        c.device.TIM2.dier().write(|w| w.uie().set_bit());

        c.device.TIM2.cr1().write(|w| w.cen().set_bit());
        // Finish PWM init

        // Set up the event tick timer
        let mut tick_timer = c.device.TIM3.counter_hz(&clocks);
        tick_timer.start(1.kHz()).unwrap();
        tick_timer.listen(Event::Update);

        let mut gpb = c.device.GPIOB.split();

        // Change PB13 into AF PUPD for LED driving (TIM1 was manually)
        // set up in init_led_drive() )
        gpb.pb13.into_alternate_push_pull(&mut gpb.crh);

        let mut gpa = c.device.GPIOA.split();

        // USB device initialization
        let mut usb_dp = gpa.pa12.into_push_pull_output(&mut gpa.crh);
        // Startup pull down
        usb_dp.set_low();
        delay(clocks.sysclk().raw() / 100);

        let usb_dm = gpa.pa11;
        //let usb_dm = usb_dm.into_floating_input(&mut gpa.crh);

        let usb = Peripheral {
            usb: c.device.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp.into_floating_input(&mut gpa.crh),
        };

        c.local.usb_bus.replace(UsbBus::new(usb));
        let usb_bus = c.local.usb_bus.as_ref().unwrap();

        let _serial = usbd_serial::SerialPort::new(usb_bus);
        
        let _usb_class = keyberon::new_class(usb_bus, ());
        let _usb_dev = keyberon::new_device(usb_bus);


        // // 0x16c0 0x27dd
        // let _usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
        //     .device_class(usbd_serial::USB_CLASS_CDC)
        //     .strings(&[StringDescriptors::default()
        //         .manufacturer("steewBSD")
        //         .product("Serial port")
        //         .serial_number("STEEWBSD")])
        //     .unwrap()
        //     .build();


        // Initialize I2C for OLED
        let _scl = gpa.pa13;
        let _sda = gpa.pa14;

        // Initialize UART pins
        let tx = gpa.pa9.into_alternate_push_pull(&mut gpa.crh);
        let rx = gpa.pa10;

        let afio = c.device.AFIO.constrain();

        let serial = Serial::new(
            c.device.USART1,
            (tx, rx),
            stm32f1xx_hal::serial::Config::default()
                .baudrate(9600.bps())
                .wordlength_9bits()
                .parity_none(),
            &clocks,
        );

        // serial.tx.write_str("Initialized device\n").unwrap();

        let kmat = keyberon::matrix::Matrix::new(
            [
                gpa.pa5.into_push_pull_output(&mut gpa.crl).erase(),
                gpa.pa6.into_push_pull_output(&mut gpa.crl).erase(),
                gpa.pa7.into_push_pull_output(&mut gpa.crl).erase(),
                gpa.pa3.into_push_pull_output(&mut gpa.crl).erase(),
                gpa.pa4.into_push_pull_output(&mut gpa.crl).erase(),
            ],
            [
                gpb.pb10.into_pull_down_input(&mut gpb.crh).erase(),
                gpb.pb0.into_pull_down_input(&mut gpb.crl).erase(),
                gpb.pb1.into_pull_down_input(&mut gpb.crl).erase(),
                gpb.pb2.into_pull_down_input(&mut gpb.crl).erase(),
            ],
        );

        let debnc = Debouncer::new([[false; 5]; 4], [[false; 5]; 4], 5);

        let bp = [
            (PwmBreathDuty::MAX, 70),
            (PwmBreathDuty::STATE1, 10),
            (PwmBreathDuty::STATE2, 10),
            (PwmBreathDuty::STATE3, 10),
            (PwmBreathDuty::STATE4, 10),
            (PwmBreathDuty::STATE5, 10),
            (PwmBreathDuty::STATE6, 10),
            (PwmBreathDuty::STATE7, 10),
            (PwmBreathDuty::STATE8, 10),
            (PwmBreathDuty::STATE9, 10),
            (PwmBreathDuty::STATE10, 10),
            (PwmBreathDuty::STATE11, 10),
            (PwmBreathDuty::STATE12, 10),
            (PwmBreathDuty::MIN, 80),
            (PwmBreathDuty::STATE12, 10),
            (PwmBreathDuty::STATE11, 10),
            (PwmBreathDuty::STATE10, 10),
            (PwmBreathDuty::STATE9, 10),
            (PwmBreathDuty::STATE8, 10),
            (PwmBreathDuty::STATE7, 10),
            (PwmBreathDuty::STATE6, 10),
            (PwmBreathDuty::STATE5, 10),
            (PwmBreathDuty::STATE4, 10),
            (PwmBreathDuty::STATE3, 10),
            (PwmBreathDuty::STATE2, 10),
            (PwmBreathDuty::STATE1, 10),
            (PwmBreathDuty::MAX, 70),
        ];

        (
            Shared {
                duty_cycle: 0,
                matrix: kmat.unwrap(),
                debouncer: debnc,
                layout: Layout::new(&crate::LAYERS),
                serial: _serial,
                usb_dev: _usb_dev,
                usb_class: _usb_class,
            },
            Local {
                breath_cycle: 0,
                timer1: c.device.TIM1,
                timer2: c.device.TIM2,
                timer3: tick_timer,
                repetition: 0,
                breath_pattern: bp,
            },
        )
    }

    #[idle(shared = [serial])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
        }
    }

    #[task(binds = USB_LP_CAN_RX0, shared = [usb_dev, usb_class])]
    fn usb_rx0(c: usb_rx0::Context) {
        (c.shared.usb_dev, c.shared.usb_class).lock(|usb_dev, usb_class| {
            if usb_dev.poll(&mut [usb_class]) {
                usb_class.poll();
            }
        });
    }

    #[task(binds = TIM3,
           shared = [matrix, debouncer, layout, usb_class],
           local = [timer3])]
    fn kbtick(mut c: kbtick::Context) {
        (c.shared.debouncer, c.shared.matrix, c.shared.layout, c.shared.usb_class).lock(|d, m, l, k| {
            for event in d.events(m.get().unwrap()) {
                l.event(event);
            }
            // Tick the layout
            l.tick();
            let report: KbHidReport = l.keycodes().collect();
            k.device_mut().set_keyboard_report(report.clone());

            unsafe {k.write(report.as_bytes()).unwrap_unchecked()};
        }
        );
        
    }

    #[task(binds = TIM2,
           local = [breath_cycle, repetition, breath_pattern, timer1, timer2],
           shared = [duty_cycle])]
    fn gate_drive(mut c: gate_drive::Context) {
        let current = c.local.breath_pattern[*c.local.breath_cycle];

        // Iterate over breathing pattern
        if *c.local.repetition < current.1 {
            *c.local.repetition += 1;
        } else {
            *c.local.breath_cycle = (*c.local.breath_cycle + 1) % c.local.breath_pattern.len();
            *c.local.repetition = 0;
        }
        c.shared.duty_cycle.lock(|v| *v = current.0 as usize);

        // Lock and read the current dutycycle value
        let dc = c.shared.duty_cycle.lock(|dc| *dc);

        // Set the new duty cycle
        c.local
            .timer1
            .ccr1()
            .write(|w| unsafe { w.ccr().bits(dc as u16) });

        // Clean the timer 2 interrupt flag
        c.local.timer2.sr().write(|w| w.uif().clear_bit());
    }
}
