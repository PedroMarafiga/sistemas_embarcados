#![no_std]
#![no_main]

use core::arch::asm;
use cortex_m_rt::pre_init;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::peripherals::{ADC1, ADC2};
use embassy_stm32::time::Hertz;
use adxl345_eh_driver::{Driver, GRange, OutputDataRate};
use embassy_stm32::adc::{self, Adc, AdcChannel, AnyAdcChannel, SampleTime};
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::interrupt;
use embassy_stm32::usart::{self, Uart};
use embassy_stm32::Config;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::time::khz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embedded_io::Write;
use heapless::Vec;
use core::sync::atomic::{AtomicU16, AtomicBool, Ordering};
use embassy_stm32::gpio::Pull;

static UART_TX: Channel<CriticalSectionRawMutex, Vec<u8, 128>, 8> = Channel::new();

// TEMP_RAW: temperatura em décimos de grau (ex: 235 = 23.5°C)
// Lida pelo interrupt TIM3 para controle hard real-time do motor
static TEMP_RAW: AtomicU16 = AtomicU16::new(0);

// MOTOR_DUTY: duty cycle do PWM (0-1000)
// Calculado pelo interrupt TIM3 e aplicado instantaneamente ao hardware
static MOTOR_DUTY: AtomicU16 = AtomicU16::new(0);

// MOTOR_ENABLED: controla se o motor está habilitado (true) ou desabilitado (false)
// Alternado pelo botão do usuário
static MOTOR_ENABLED: AtomicBool = AtomicBool::new(true);

// MOTOR_START_PULSE: flag para solicitar pulso inicial de 100% ao ligar o motor
static MOTOR_START_PULSE: AtomicBool = AtomicBool::new(false);

pub struct Writer;

impl embedded_io::ErrorType for Writer {
    type Error = embedded_io::ErrorKind;
}

impl embedded_io::Write for Writer {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let mut vec = Vec::new();
        vec.extend_from_slice(buf).map_err(|_| embedded_io::ErrorKind::Other)?;
        UART_TX
            .try_send(vec)
            .map_err(|_| embedded_io::ErrorKind::Other)?;
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

pub enum Command {
    Help,
    Tasks,
    Mem,
    Rt,
    Unknown,
}

fn send(msg: &[u8]) {
    let mut v: Vec<u8, 128> = Vec::new();
    let _ = v.extend_from_slice(msg);
    let _ = UART_TX.try_send(v);
}


fn parse_command(input: &str) -> Command {
    match input.trim() {
        "help" => Command::Help,
        "tasks" => Command::Tasks,
        "mem" => Command::Mem,
        "rt" => Command::Rt,
        _ => Command::Unknown,
    }
}

fn execute_command(cmd: Command) {
    match cmd {
        Command::Help => {
            send(b"Available commands:\r\n");
            send(b"  help  - Show this message\r\n");
            send(b"  tasks - List tasks\r\n");
            send(b"  mem   - Memory info\r\n");
            send(b"  rt    - Runtime info\r\n");
        }

        Command::Tasks => {
            send(b"Tasks installed:\r\n");
            send(b"  - lm35_task (reads temperature)\r\n");
            send(b"  - uart_task (CLI interface)\r\n");
            send(b"  - button_task (user button)\r\n");
            send(b"  - TIM3 interrupt (motor control - hard RT)\r\n");
        }

        Command::Mem => {
            send(b"Memory info:\r\n");
            send(b"  Heap: not configured (no_std)\r\n");
            send(b"  Static memory only\r\n");
        }

        Command::Rt => {
            send(b"Runtime info:\r\n");
            send(b"  Executor: Embassy async\r\n");
            send(b"  Scheduling: cooperative\r\n");
            send(b"  Preemption: no\r\n");
        }

        Command::Unknown => {
            send(b"Unknown command. Type 'help'.\r\n");
        }
    }
}

// Task que lê temperatura do LM35 e armazena em TEMP_RAW
// O interrupt TIM3 lê TEMP_RAW para controle hard real-time do motor
#[embassy_executor::task]
async fn lm35_task(mut adc2: Adc<'static, ADC2>, mut pin: AnyAdcChannel<ADC2>) {
    adc2.set_sample_time(SampleTime::CYCLES247_5);

    loop {
        // Faz oversampling: média de 16 leituras para reduzir ruído
        let mut acc = 0u32;
        for _ in 0..16 {
            acc += adc2.blocking_read(&mut pin) as u32;
        }
        let raw = (acc / 16) as u16;

        // Converte para temperatura
        let temp_c = (raw as f32) * 3.3 / 4095.0 * 100.0;

        TEMP_RAW.store((temp_c * 10.0) as u16, Ordering::Relaxed);

        info!("Temp: {} °C (ADC raw: {})", temp_c, raw);
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::task]
async fn adc_task(mut adc: adc::Adc<'static, ADC1>, mut adc_pin: AnyAdcChannel<ADC1>) {
    adc.set_sample_time(SampleTime::CYCLES47_5);

    loop {
        let measured = adc.blocking_read(&mut adc_pin);
        info!("measured: {}", measured);
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::task]
async fn button_task(mut button: ExtiInput<'static>) {
    info!("Botão configurado. Pressione para ligar/desligar o motor.");

    loop {
        button.wait_for_rising_edge().await;
        
        // Debounce simples
        Timer::after_millis(50).await;
        
        // Alterna estado do motor
        let was_enabled = MOTOR_ENABLED.load(Ordering::Relaxed);
        let now_enabled = !was_enabled;
        MOTOR_ENABLED.store(now_enabled, Ordering::Relaxed);
        
        if now_enabled {
            info!("Motor LIGADO - solicitando pulso inicial");
            // Solicita pulso inicial de 100% para vencer inércia
            MOTOR_START_PULSE.store(true, Ordering::Relaxed);
        } else {
            info!("Motor DESLIGADO");
        }
        
        button.wait_for_falling_edge().await;
    }
}

#[embassy_executor::task]
async fn uart_task(uart: Uart<'static, embassy_stm32::mode::Async>) {
    let (mut tx, mut rx) = uart.split();
    tx.write(b"UART started. Type commands...\r\n").await.unwrap();

    let mut line: heapless::String<64> = heapless::String::new();
    let mut rx_buf = [0u8; 1];

    loop {
        // Primeiro, tenta enviar mensagens pendentes do sistema
        while let Ok(msg) = UART_TX.try_receive() {
            tx.write(&msg).await.unwrap();
        }

        // Depois, tenta ler um byte com timeout curto via select
        match embassy_futures::select::select(
            rx.read(&mut rx_buf),
            Timer::after_millis(10)
        ).await {
            embassy_futures::select::Either::First(Ok(_)) => {
                let byte = rx_buf[0];

                if byte == b'\r' || byte == b'\n' {
                    if !line.is_empty() {
                        tx.write(b"\r\n").await.unwrap();
                        let cmd = parse_command(&line);
                        execute_command(cmd);
                        line.clear();
                    }
                } else if byte >= 32 && byte <= 126 {
                    tx.write(&[byte]).await.unwrap();
                    let _ = line.push(byte as char);
                }
            }
            _ => {
                // Timeout ou erro - continua o loop
            }
        }
    }
}

bind_interrupts!(struct Irqs {
    LPUART1 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::LPUART1>;
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

// Interrupt Handler TIM3 - CONTROLE HARD REAL-TIME DO MOTOR
// Executa periodicamente (500 kHz) com determinismo temporal garantido
// Lê temperatura, calcula duty cycle e atualiza PWM instantaneamente
#[interrupt]
fn TIM3() {
    let motor_enabled = MOTOR_ENABLED.load(Ordering::Relaxed);
    
    let duty = if !motor_enabled {
        0
    } else if MOTOR_START_PULSE.load(Ordering::Relaxed) {
        // Pulso inicial de 100% por exatamente 200ms para vencer inércia
        // Contador interno: a cada interrupt (~2ms), decrementa
        static mut PULSE_COUNTER: u8 = 0;
        unsafe {
            // Inicializa contador quando pulso é solicitado
            if PULSE_COUNTER == 0 {
                PULSE_COUNTER = 200;  
            }
            
            if PULSE_COUNTER > 0 {
                PULSE_COUNTER -= 1;
            }
            
            if PULSE_COUNTER == 0 {
                MOTOR_START_PULSE.store(false, Ordering::Relaxed);
            }
            
            1000   
        }
    } else {
        let temp = TEMP_RAW.load(Ordering::Relaxed) as f32 / 10.0;

        if temp < 10.0 {
            0       
        } else if temp < 20.0 {
            750    // 75% velocidade
        } else if temp < 30.0 {
            900    // 90% velocidade
        } else {
            1000   // 100% velocidade
        }
    };

    MOTOR_DUTY.store(duty, Ordering::Relaxed);

    let tim3 = embassy_stm32::pac::TIM3;
    tim3.sr().modify(|w| w.set_uif(false));  // Limpa flag de interrupção
    tim3.ccr(1).write(|w| w.set_ccr(duty));  // Atualiza PWM do canal 2
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
    unsafe {
        println!("Teste de variável na memória CCMRAM {}", TESTE);
        println!("Teste de variável na memória SRAM2 {}", TESTE2);
    }

    let button = ExtiInput::new(p.PC13, p.EXTI13, Pull::Down);

    let adc2 = Adc::new(p.ADC2);

    // Spawned tasks run in the background, concurrently.
    spawner.spawn(button_task(button)).unwrap();
    spawner.spawn(lm35_task(adc2, p.PA1.degrade_adc())).unwrap();

    let mut config = usart::Config::default();
    config.baudrate = 115_200;

    let lpusart = Uart::new(
        p.LPUART1,
        p.PA3,          // RX
        p.PA2,          // TX
        Irqs,
        p.DMA1_CH1,     // TX DMA
        p.DMA1_CH2,     // RX DMA
        config,
    )
    .unwrap();

    spawner.spawn(uart_task(lpusart)).unwrap();

    let mut writer = Writer;
    writer.write(b"Hello from Writer\r\n").unwrap();

    // Configura PWM no canal 2 (PC7) do TIM3 para controle do motor
    let ch2_pin = PwmPin::new(p.PC7, OutputType::PushPull);
    let mut pwm: SimplePwm<'_, embassy_stm32::peripherals::TIM3> = SimplePwm::new(
        p.TIM3,
        None,
        Some(ch2_pin),
        None,
        None,
        khz(500),  // Frequência do PWM = 500 kHz
        Default::default(),
    );
    
    // Habilita canal 2 do PWM
    pwm.ch2().enable();
    
    // Configura registradores do TIM3 para controle hard real-time
    unsafe {
        let tim3 = embassy_stm32::pac::TIM3;
        
        // Configura ARR (auto-reload) para duty cycle de 0-1000
        tim3.arr().write(|w| w.set_arr(1000));
        
        // Força atualização dos registradores
        tim3.egr().write(|w| w.set_ug(true));
        
        // Habilita interrupção de update (será chamada a cada período do timer)
        tim3.dier().modify(|w| w.set_uie(true));
        
        // Habilita interrupt no NVIC
        cortex_m::peripheral::NVIC::unmask(embassy_stm32::interrupt::TIM3);
        
        // Garante que o timer está habilitado
        tim3.cr1().modify(|w| w.set_cen(true));
    }
    
    // IMPORTANTE: Não fazer drop! PWM precisa ficar vivo para o timer continuar rodando
    // Usamos core::mem::forget para prevenir o destructor de desabilitar o timer
    core::mem::forget(pwm);

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