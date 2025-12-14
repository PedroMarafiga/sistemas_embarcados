#![no_std]
#![no_main]

//use cortex_m::Peripherals;
use core::arch::asm;
use cortex_m_rt::pre_init;
use defmt::*;
use embassy_executor::Spawner;
//use embassy_stm32::pac::metadata::Peripheral;
mod utils;

use embassy_stm32::peripherals::{ADC1, ADC2};
use embassy_stm32::time::Hertz;
//use embassy_stm32::rcc::low_level::RccPeripheral;
//use embassy_stm32::timer::low_level::GeneralPurpose16bitInstance;
use adxl345_eh_driver::{Driver, GRange, OutputDataRate};
use embassy_stm32::adc::{self, Adc, AdcChannel, AnyAdcChannel, SampleTime};
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::interrupt;
use embassy_stm32::usart::{self, Uart};
use embassy_stm32::Config;
//use embassy_stm32::timer::pwm_input::PwmInput;
//use embassy_stm32::time::hz;
//use embassy_stm32::timer::CountingMode;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::time::khz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

static TEMP_CHANNEL: Channel<CriticalSectionRawMutex, f32, 1> = Channel::new();

#[embassy_executor::task]
async fn lm35_task(mut adc2: adc::Adc<'static, ADC2>, mut pin: AnyAdcChannel<ADC2>) {
    adc2.set_sample_time(SampleTime::CYCLES247_5);

    loop {
        let mut acc = 0u32;
        for _ in 0..16 {
            acc += adc2.blocking_read(&mut pin) as u32;
        }
        let raw = (acc / 16) as u16;

        let temp_c = (raw as f32) * 3.3 / 4095.0 * 100.0;

        info!("Temp: {} °C", temp_c);

        TEMP_CHANNEL.send(temp_c).await;

        Timer::after_millis(500).await;
    }
}

#[embassy_executor::task]
async fn motor_task(mut pwm: SimplePwm<'static, embassy_stm32::peripherals::TIM3>) {
    let mut ch2 = pwm.ch2();
    ch2.enable();

    loop {
        let temp = TEMP_CHANNEL.receive().await;

        let duty = if temp < 10.0 {
            0
        } else if temp < 20.0 {
            ch2.max_duty_cycle() * 75 / 100
        } else if temp < 30.0 {
            ch2.max_duty_cycle() * 90 / 100
        } else {
            ch2.max_duty_cycle()
        };

        ch2.set_duty_cycle(duty);

        info!("Motor PWM set to {} (temp {}°C)", duty, temp);
    }
}

// Declare async tasks
#[embassy_executor::task]
async fn adc_task(mut adc: adc::Adc<'static, ADC1>, mut adc_pin: AnyAdcChannel<ADC1>) {
    adc.set_sample_time(SampleTime::CYCLES47_5);

    loop {
        let measured = adc.blocking_read(&mut adc_pin);
        info!("measured: {}", measured);
        Timer::after_millis(500).await;
    }
}

// Declare async tasks
#[embassy_executor::task]
async fn button_task(mut button: ExtiInput<'static>) {
    info!("Press the USER button...");

    loop {
        button.wait_for_rising_edge().await;
        info!("Pressed!");
        button.wait_for_falling_edge().await;
        info!("Released!");
    }
}

// Declare async tasks
#[embassy_executor::task]
async fn    pwm_task(mut pwm: SimplePwm<'static, embassy_stm32::peripherals::TIM3>) {
    let mut ch2 = pwm.ch2();
    ch2.enable();

    // Loop to read from UART and echo back
    loop {
         /*  ch1.set_duty_cycle_fully_off();
        Timer::after_millis(300).await; 
         ch1.set_duty_cycle_fraction(1, 4);
        Timer::after_millis(300).await; */
        info!("3 /4 duty cycle");
          ch2.set_duty_cycle_fraction(3, 4);
        Timer::after_millis(3000).await; 
        info!("100% duty cycle");
        ch2.set_duty_cycle(ch2.max_duty_cycle() - 1);
        Timer::after_millis(3000).await; 

        info!("PWM cycle done");
    }
}

// Declare async tasks
#[embassy_executor::task]
async fn accel_task(
    mut accel: Driver<I2c<'static, embassy_stm32::mode::Async, i2c::mode::Master>>,
) {
    //pub async fn i2c_slave_task(mut i2c_slave: I2c<'static, embassy_stm32::mode::Async, i2c::mode::MultiMaster>) {
    accel.set_range(GRange::Two).unwrap();
    accel.set_datarate(OutputDataRate::Hz0_10).unwrap();
    loop {
        if let Ok((x, y, z)) = accel.get_accel() {
            info!("ADXL345 Accel Raw: x={}, y={}, z={}", x, y, z);
        }
        Timer::after_millis(1000).await;
    }
}

//bind_interrupts!(struct Irqs {
//    TIM2 => timer::CaptureCompareInterruptHandler<peripherals::TIM2>;
//});

bind_interrupts!(struct Irqs {
    LPUART1 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::LPUART1>;
    I2C1_EV => i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C1>;
});

//#[link_section = ".ram2bss"]
#[link_section = ".ccmram"]
static mut TESTE: i32 = 60;

#[link_section = ".data2"]
static mut TESTE2: i32 = 70;

#[pre_init]
unsafe fn before_main() {
    unsafe {
        asm! {
            "ldr r0, =__sccmdata
            ldr r1, =__eccmdata
            ldr r2, =__siccmdata
            0:
            cmp r1, r0
            beq 1f
            ldm r2!, {{r3}}
            stm r0!, {{r3}}
            b 0b
            1:"
        }

        asm! {
            "ldr r0, =__sdata2
            ldr r1, =__edata2
            ldr r2, =__sidata2
            2:
            cmp r1, r0
            beq 3f
            ldm r2!, {{r3}}
            stm r0!, {{r3}}
            b 2b
            3:"
        }
    }
}

#[interrupt]
unsafe fn TIM3() {
    // reset interrupt flag
    //unsafe {
    //    let pin = embassy_stm32::peripherals::PA5::steal();
    //    let mut pin = Output::new(pin, Level::High, Speed::Low);
    //    pin.set_high();
    //}
    //pac::TIM3.sr().modify(|r| r.set_uif(false));
    info!("interrupt happens: tim20");
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    //unsafe {
    //TESTE = 10;
    //TESTE2 = 20;
    //}
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(24_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV6,
            mul: PllMul::MUL85,
            divp: None,
            divq: None,
            // Main system clock at 170 MHz
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.sys = Sysclk::PLL1_R;
        //config.rcc.mux = ClockSrc::PLL;

        //config.rcc.adc12_clock_source = AdcClockSource::SYS;
        config.rcc.mux.adc12sel = mux::Adcsel::SYS;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
    }

    let p: embassy_stm32::Peripherals = embassy_stm32::init(config);

    info!("Hello World!");
    unsafe {
        println!("Teste de variável na memória CCMRAM {}", TESTE);
        println!("Teste de variável na memória SRAM2 {}", TESTE2);
    }
    //defmt::println!("Hello, world!");

    // let button = ExtiInput::new(p.PC13, p.EXTI13, Pull::Down);

    //let adc = Adc::new(p.ADC1, &mut Delay);
    //let adc_pin = p.PA1;

    //let adc = Adc::new(p.ADC1);

    let adc2 = Adc::new(p.ADC2);

    // Spawned tasks run in the background, concurrently.
    //spawner.spawn(adc_task(adc, p.PA1.degrade_adc())).unwrap();
    //spawner.spawn(button_task(button)).unwrap();
    spawner.spawn(lm35_task(adc2, p.PA1.degrade_adc())).unwrap();


    let mut config = usart::Config::default();
    config.baudrate = 115_200;
    let lpusart = Uart::new(
        p.LPUART1, p.PA3, p.PA2, Irqs, p.DMA1_CH1, p.DMA1_CH2, config,
    )
    .unwrap();
    spawner.spawn(utils::uart_task(lpusart)).unwrap();

    let ch2_pin = PwmPin::new(p.PC7, OutputType::PushPull);
    let pwm: SimplePwm<'_, embassy_stm32::peripherals::TIM3> = SimplePwm::new(
        p.TIM3,
        None,
        Some(ch2_pin),
        None,
        None,
        khz(500),
        Default::default(),
    );
    //let mut ch1: embassy_stm32::timer::simple_pwm::SimplePwmChannel<'_, embassy_stm32::peripherals::TIM1> = pwm.ch1();
    //ch1.enable();

    spawner.spawn(motor_task(pwm)).unwrap();

    //let mut i2c = I2c::new_blocking(p.I2C1, p.PB8, p.PB9, Hertz(100_000), i2c::Config::default());
    //println!("{:?}", p.PB8.af_num());

    let mut led = Output::new(p.PA5, Level::High, Speed::Low);

    loop {
        //info!("high");
        led.set_high();
        Timer::after_millis(500).await;

        //info!("low");
        led.set_low();
        Timer::after_millis(500).await;
    }
}

/*
// Some panic handler needs to be included. This one halts the processor on panic.
use cortex_m_rt::entry;
use rtt_target::{rtt_init_print, rprintln};

use hal::prelude::*;
use hal::stm32;
use stm32g4xx_hal as hal;

#[link_section = ".ram2bss"]
static mut TESTE: i32 = 10;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Olá mundo!");
    unsafe {
        rprintln!("Teste de variável na memória SRAM2 {}", TESTE);
    }

    let teste2 = 10;
    rprintln!("Teste de variável na memória SRAM2 {}", teste2);

    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();

    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut led = gpioa.pa5.into_push_pull_output();

    let core_periphs=cortex_m::Peripherals::take().unwrap();
    let clocks = rcc.clocks.sys_clk;

    // Create a delay abstraction based on SysTick
    let mut delay = hal::delay::Delay::new(core_periphs.SYST, clocks.0);
    loop{
        rprintln!("High");
        led.set_high().unwrap();
        delay.delay_ms(500_u32);

        rprintln!("Low");
        led.set_low().unwrap();
        delay.delay_ms(500_u32);
    }
}
*/
