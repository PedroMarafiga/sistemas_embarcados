use defmt::*;
use embassy_stm32::usart::Uart;

#[embassy_executor::task]
pub async fn uart_task(mut lpuart: Uart<'static, embassy_stm32::mode::Async>) {
    info!("System Console Initialized");
    lpuart.write(b"\r\n=== STM32 System Console ===\r\n").await.unwrap();
    lpuart.write(b"Type 'help' for available commands\r\n").await.unwrap();
    lpuart.write(b"> ").await.unwrap();

    let mut cmd_buffer = [0u8; 64];
    let mut cmd_index = 0;

    // Loop to read from UART and process commands
    loop {
        let mut buffer = [0u8; 1];
        lpuart.read(&mut buffer).await.unwrap();
        
        let ch = buffer[0];
        
        // Echo character back (non-intrusive)
        lpuart.write(&buffer).await.unwrap();
        
        if ch == b'\r' || ch == b'\n' {
            lpuart.write(b"\r\n").await.unwrap();
            
            if cmd_index > 0 {
                // Process command
                let cmd = core::str::from_utf8(&cmd_buffer[..cmd_index]).unwrap_or("");
                process_command(&mut lpuart, cmd.trim()).await;
                cmd_index = 0;
            }
            
            lpuart.write(b"> ").await.unwrap();
        } else if ch == 8 || ch == 127 {
            // Backspace
            if cmd_index > 0 {
                cmd_index -= 1;
                lpuart.write(b"\x08 \x08").await.unwrap();
            }
        } else if cmd_index < cmd_buffer.len() {
            cmd_buffer[cmd_index] = ch;
            cmd_index += 1;
        }
    }
}

// Process console commands (non-intrusive)
async fn process_command(uart: &mut Uart<'_, embassy_stm32::mode::Async>, cmd: &str) {
    match cmd {
        "help" => {
            uart.write(b"Available commands:\r\n").await.unwrap();
            uart.write(b"  help      - Show this help message\r\n").await.unwrap();
            uart.write(b"  tasks     - List running tasks\r\n").await.unwrap();
            uart.write(b"  mem       - Show memory information\r\n").await.unwrap();
            uart.write(b"  info      - Show system information\r\n").await.unwrap();
            uart.write(b"  rtinfo    - Show real-time task information\r\n").await.unwrap();
            uart.write(b"  status    - Show task status\r\n").await.unwrap();
        }
        "tasks" => {
            uart.write(b"Running Tasks:\r\n").await.unwrap();
            uart.write(b"  1. adc_task       - ADC sampling (PA1)\r\n").await.unwrap();
            uart.write(b"  2. button_task    - Button monitoring (PC13)\r\n").await.unwrap();
            uart.write(b"  3. uart_task      - Console/UART handler\r\n").await.unwrap();
            uart.write(b"  4. pwm_task       - PWM control (TIM1)\r\n").await.unwrap();
            uart.write(b"  5. main_loop      - LED blinker\r\n").await.unwrap();
        }
        "mem" => {
            uart.write(b"Memory Information:\r\n").await.unwrap();
            uart.write(b"  RAM: 128 KB\r\n").await.unwrap();
            uart.write(b"  Flash: 512 KB\r\n").await.unwrap();
            uart.write(b"  CCMRAM: 32 KB\r\n").await.unwrap();
        }
        "info" => {
            uart.write(b"System Information:\r\n").await.unwrap();
            uart.write(b"  MCU: STM32G474\r\n").await.unwrap();
            uart.write(b"  Core: ARM Cortex-M4F\r\n").await.unwrap();
            uart.write(b"  System Clock: 170 MHz\r\n").await.unwrap();
            uart.write(b"  Runtime: Embassy (async)\r\n").await.unwrap();
            uart.write(b"  Features: ADC, UART, I2C, PWM, EXTI\r\n").await.unwrap();
        }
        "rtinfo" => {
            uart.write(b"Real-Time Task Information:\r\n").await.unwrap();
            uart.write(b"  adc_task:\r\n").await.unwrap();
            uart.write(b"    Period: 500ms\r\n").await.unwrap();
            uart.write(b"    Priority: Normal\r\n").await.unwrap();
            uart.write(b"    Function: ADC sampling\r\n").await.unwrap();
            uart.write(b"  button_task:\r\n").await.unwrap();
            uart.write(b"    Type: Event-driven (EXTI)\r\n").await.unwrap();
            uart.write(b"    Priority: Normal\r\n").await.unwrap();
            uart.write(b"    Function: Button press detection\r\n").await.unwrap();
            uart.write(b"  pwm_task:\r\n").await.unwrap();
            uart.write(b"    Period: 300ms (4 steps)\r\n").await.unwrap();
            uart.write(b"    Priority: Normal\r\n").await.unwrap();
            uart.write(b"    Function: PWM generation\r\n").await.unwrap();
            uart.write(b"  main_loop:\r\n").await.unwrap();
            uart.write(b"    Period: 1000ms\r\n").await.unwrap();
            uart.write(b"    Priority: Low\r\n").await.unwrap();
            uart.write(b"    Function: LED toggle\r\n").await.unwrap();
        }
        "status" => {
            uart.write(b"Task Status:\r\n").await.unwrap();
            uart.write(b"  All tasks: RUNNING\r\n").await.unwrap();
            uart.write(b"  Executor: Embassy async\r\n").await.unwrap();
            uart.write(b"  Total Tasks: 5\r\n").await.unwrap();
        }
        "" => {
            // Empty command, do nothing
        }
        _ => {
            uart.write(b"Unknown command: ").await.unwrap();
            uart.write(cmd.as_bytes()).await.unwrap();
            uart.write(b"\r\n").await.unwrap();
            uart.write(b"Type 'help' for available commands\r\n").await.unwrap();
        }
    }
}

// Simple buffer writer for formatting output (non-intrusive)
struct BufferWriter<'a> {
    buffer: &'a mut [u8],
    pos: usize,
}

impl<'a> BufferWriter<'a> {
    fn new(buffer: &'a mut [u8]) -> Self {
        Self { buffer, pos: 0 }
    }
    
    fn write_str(&mut self, s: &str) {
        let bytes = s.as_bytes();
        let len = bytes.len().min(self.buffer.len() - self.pos);
        self.buffer[self.pos..self.pos + len].copy_from_slice(&bytes[..len]);
        self.pos += len;
    }
    
    fn write_num(&mut self, mut num: usize) {
        if num == 0 {
            self.write_str("0");
            return;
        }
        
        let mut digits = [0u8; 20];
        let mut i = 0;
        
        while num > 0 {
            digits[i] = b'0' + (num % 10) as u8;
            num /= 10;
            i += 1;
        }
        
        while i > 0 {
            i -= 1;
            if self.pos < self.buffer.len() {
                self.buffer[self.pos] = digits[i];
                self.pos += 1;
            }
        }
    }
    
    fn reset(&mut self) {
        self.pos = 0;
    }
    
    fn as_bytes(&self) -> &[u8] {
        &self.buffer[..self.pos]
    }
}