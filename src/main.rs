#![no_std]
#![no_main]

use core::arch::asm;
use cortex_m_rt::pre_init;
use defmt::*;
use embassy_executor::Spawner;
mod utils;

use embassy_stm32::adc::{Adc, AdcChannel, AnyAdcChannel, SampleTime};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::interrupt;
use embassy_stm32::peripherals::ADC2;
use embassy_stm32::time::khz;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::Config;
use embassy_time::Timer;
use embassy_stm32::Peri;

use {defmt_rtt as _, panic_probe as _};

use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::i2c;
use embassy_stm32::usart::{self, Uart};

// TEMP_RAW: temperatura em décimos de grau (ex: 235 = 23.5°C)
// Lida pelo interrupt TIM3 para controle hard real-time do motor
#[link_section = ".data2"]
static TEMP_RAW: AtomicU32 = AtomicU32::new(0);

// MOTOR_DUTY: duty cycle do PWM (0-1000)
// Calculado pelo interrupt TIM3 e aplicado instantaneamente ao hardware
#[link_section = ".ccmdata"]
static MOTOR_DUTY: AtomicU16 = AtomicU16::new(0);

static MOTOR_ENABLED: AtomicBool = AtomicBool::new(false);

static MOTOR_START_PULSE: AtomicBool = AtomicBool::new(false);

#[embassy_executor::task]
async fn lm35_task(
    mut adc2: Adc<'static, ADC2>,
    mut pin: AnyAdcChannel<ADC2>,
    mut dma: Peri<'static, embassy_stm32::peripherals::DMA1_CH3>,

) {
    adc2.set_sample_time(SampleTime::CYCLES247_5);

    let mut adc_buf = [0u16; 1];

    loop {
        adc2.read(
            dma.reborrow(),
            core::iter::once((&mut pin, SampleTime::CYCLES247_5)),
            &mut adc_buf,
        ).await;

        let raw = adc_buf[0];

        let temp_c = (((raw as u32 * 3300) >> 12)) as u32;
        // let old_temp_c = (raw as f32) * 3.3 / 4095.0 * 100.0;

        TEMP_RAW.store(temp_c, Ordering::Relaxed);

        let temp_c_int = temp_c / 10;
        let temp_c_frac = temp_c % 10;

        
        // info!("Temp: {} °C (ADC raw: {}, button state: {})", old_temp_c, raw, MOTOR_ENABLED.load(Ordering::Relaxed));

        info!("Temp: {}.{} °C (ADC raw: {}, button state: {})", temp_c_int, temp_c_frac, raw, MOTOR_ENABLED.load(Ordering::Relaxed));

        Timer::after_millis(500).await;
    }
}

#[embassy_executor::task]
async fn button_task(mut button: ExtiInput<'static>) {
    info!("Botão configurado. Pressione para ligar/desligar o motor.");

    loop {
        button.wait_for_rising_edge().await;

        Timer::after_millis(50).await;

        let now_enabled = !MOTOR_ENABLED.load(Ordering::Relaxed);
        MOTOR_ENABLED.store(now_enabled, Ordering::Relaxed);

        if now_enabled {
            info!("Motor LIGADO - solicitando pulso inicial");
            MOTOR_START_PULSE.store(true, Ordering::Relaxed);
        } else {
            info!("Motor DESLIGADO");
        }

        button.wait_for_falling_edge().await;
    }
}

#[embassy_executor::task]
async fn blink_task(mut led: Output<'static>) {
    loop {
        led.set_high();
        Timer::after_millis(2000).await;

        led.set_low();
        Timer::after_millis(500).await;
    }
}

bind_interrupts!(struct Irqs {
    LPUART1 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::LPUART1>;
    I2C1_EV => i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C1>;
});

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
fn TIM3() {
    let motor_enabled: bool = MOTOR_ENABLED.load(Ordering::Relaxed);
    static mut PULSE_START_CNT: u16 = 0;

    let duty = if !motor_enabled {
        0
    } else if MOTOR_START_PULSE.load(Ordering::Relaxed) {        
        unsafe {
            let tim3 = embassy_stm32::pac::TIM3;
            let now = tim3.cnt().read().cnt();
            const PULSE_TICKS: u16 = 400; // depende do clock e prescaler

            // Inicializa contador quando pulso é solicitado
            if PULSE_START_CNT == 0 {
                PULSE_START_CNT = now;
            }

            let elapsed = now.wrapping_sub(PULSE_START_CNT);

            if elapsed >= PULSE_TICKS {
                MOTOR_START_PULSE.store(false, Ordering::Relaxed);
                PULSE_START_CNT = 0;
            }

            1000
        }
    } else {
        let temp = TEMP_RAW.load(Ordering::Relaxed);
        
        const HISTERESE: u32 = 20; // 2°C de histerese
        
        let current_duty = MOTOR_DUTY.load(Ordering::Relaxed);
        
        let (duty, _new_zone) = if current_duty == 0 {
            if temp < (100 + HISTERESE) {
                (0, 0)
            } else if temp < (200 + HISTERESE) {
                (750, 1)
            } else if temp < (300 + HISTERESE) {
                (900, 2)
            } else {
                (1000, 3)
            }
        } else if current_duty <= 750 {
            if temp < (100 - HISTERESE) {
                (0, 0)
            } else if temp < (200 + HISTERESE) {
                (750, 1)
            } else if temp < (300 + HISTERESE) {
                (900, 2)
            } else {
                (1000, 3)
            }
        } else if current_duty <= 900 {
            if temp < (100 - HISTERESE) {
                (0, 0)
            } else if temp < (200 - HISTERESE) {
                (750, 1)
            } else if temp < (300 + HISTERESE) {
                (900, 2)
            } else {
                (1000, 3)
            }
        } else {
            if temp < (100 - HISTERESE) {
                (0, 0)
            } else if temp < (200 - HISTERESE) {
                (750, 1)
            } else if temp < (300 - HISTERESE) {
                (900, 2)
            } else {
                (1000, 3)
            }
        };
        
        duty
    };

    MOTOR_DUTY.store(duty, Ordering::Relaxed);

    let tim3 = embassy_stm32::pac::TIM3;
    tim3.ccr(1).write(|w| w.set_ccr(duty));  
    tim3.sr().modify(|w| w.set_uif(false));  // Limpa flag de interrupção
    
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
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
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.mux.adc12sel = mux::Adcsel::SYS;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
    }
    let p: embassy_stm32::Peripherals = embassy_stm32::init(config);

    info!("Hello World!");

    /* --- BOTÃO --- */
    let button = ExtiInput::new(p.PC13, p.EXTI13, Pull::Down);
    spawner.spawn(button_task(button)).unwrap();

    /* --- SENSOR DE TEMPERATURA --- */
    let adc2 = Adc::new(p.ADC2);
    let adc_pin = p.PA1.degrade_adc();
    spawner.spawn(lm35_task(adc2, adc_pin, p.DMA1_CH3)).unwrap();

    /* --- LED DE STATUS ---  */
    let led = Output::new(p.PA5, Level::High, Speed::Low);
    spawner.spawn(blink_task(led)).unwrap();

    /* --- UART CONSOLE --- */
    let mut config = usart::Config::default();
    config.baudrate = 115_200;
    let lpusart = Uart::new(
        p.LPUART1, p.PA3, p.PA2, Irqs, p.DMA1_CH1, p.DMA1_CH2, config,
    )
    .unwrap();
    spawner.spawn(utils::uart_task(lpusart)).unwrap();


    // Configura PWM TIM3 CH2 (PC7) para controle do motor
    let ch2_pin = PwmPin::new(p.PC7, OutputType::PushPull);
    let mut pwm: SimplePwm<'_, embassy_stm32::peripherals::TIM3> = SimplePwm::new(
        p.TIM3,
        None,
        Some(ch2_pin),
        None,
        None,
        khz(500),
        Default::default(),
    );

    pwm.ch2().enable();

    /* --- TIMER 3 --- */
    unsafe {
        let tim3 = embassy_stm32::pac::TIM3;

        tim3.cr1().modify(|w| w.set_cen(false));

        tim3.psc().write_value(169);

        tim3.arr().write(|w| w.set_arr(1000));

        // Configura modo PWM no canal 2 (índice 1)
        tim3.ccmr_output(1).modify(|w| {
            w.set_ocm(1, embassy_stm32::pac::timer::vals::Ocm::PWM_MODE1);
            w.set_ocpe(1, true); // Preload enable
        });

        tim3.ccr(1).write(|w| w.set_ccr(0));

        tim3.ccer().modify(|w| {
            w.set_cce(1, true);
            w.set_ccp(1, false);
        });

        // Força atualização dos registradores
        tim3.egr().write(|w| w.set_ug(true));

        tim3.sr().modify(|w| w.set_uif(false));
        tim3.dier().modify(|w| w.set_uie(true));

        cortex_m::peripheral::NVIC::unmask(embassy_stm32::interrupt::TIM3);

        tim3.cr1().modify(|w| w.set_cen(true));
    }
    
    core::mem::forget(pwm);
}
