use defmt::*;
use embassy_stm32::usart::Uart;
use crate::MOTOR_START_PULSE;
use crate::MOTOR_ENABLED;
use crate::Ordering;

#[embassy_executor::task]
pub async fn uart_task(mut lpuart: Uart<'static, embassy_stm32::mode::Async>) {
    info!("Console do Sistema Inicializado");
    lpuart
        .write(b"\r\n=== STM32 System Console ===\r\n")
        .await
        .unwrap();
    lpuart
        .write(b"Digite 'help' para ver os comandos disponiveis\r\n")
        .await
        .unwrap();
    lpuart.write(b"> ").await.unwrap();

    let mut cmd_buffer = [0u8; 64];
    let mut cmd_index = 0;

    loop {
        let mut buffer = [0u8; 1];
        lpuart.read(&mut buffer).await.unwrap();

        let ch = buffer[0];

        if ch == b'\r' || ch == b'\n' {
            // tecla enter
            if cmd_index > 0 {
                // Verifica se o comando nao é apenas espaços em branco
                let cmd = core::str::from_utf8(&cmd_buffer[..cmd_index]).unwrap_or("");
                let trimmed_cmd = cmd.trim();

                if !trimmed_cmd.is_empty() {
                    // Só pula linha e processa se houver comando real
                    lpuart.write(b"\r\n").await.unwrap();
                    process_command(&mut lpuart, trimmed_cmd).await;
                    lpuart.write(b"> ").await.unwrap();

                    cmd_index = 0;
                }
            }
            // Se cmd_index == 0 (linha vazia), nao faz nada
        } else if ch == 8 || ch == 127 {
            // Backspace - nao permite apagar além do início do comando
            if cmd_index > 0 {
                cmd_index -= 1;
                // Envia: backspace, espaço (apaga), backspace (volta)
                lpuart.write(b"\x08 \x08").await.unwrap();
            }
        } else if ch == 3 {
            // Ctrl+C
            lpuart.write(b"^C\r\n> ").await.unwrap();
            cmd_index = 0;
        } else if cmd_index < cmd_buffer.len() {
            // Echo character back apenas para caracteres normais
            lpuart.write(&buffer).await.unwrap();
            cmd_buffer[cmd_index] = ch;
            cmd_index += 1;
        }
    }
}

async fn process_command(uart: &mut Uart<'_, embassy_stm32::mode::Async>, cmd: &str) {
    match cmd {
        "help" => {
            uart.write(b"Available commands:\r\n").await.unwrap();
            uart.write(b"  help      - Mostra esta mensagem de ajuda\r\n")
                .await
                .unwrap();
            uart.write(b"  tasks     - Lista tarefas em execucao\r\n")
                .await
                .unwrap();
            uart.write(b"  mem       - Mostra informacoes de memoria\r\n")
                .await
                .unwrap();
            uart.write(b"  info      - Mostra informacoes do sistema\r\n")
                .await
                .unwrap();
            uart.write(b"  rtinfo    - Mostra informacoes de tarefas em tempo real\r\n")
                .await
                .unwrap();
            uart.write(b"  status    - Mostra status das tarefas\r\n")
                .await
                .unwrap();
            uart.write(b"  motor_switch - Liga/Desliga o motor\r\n")
                .await
                .unwrap();
        }
        "tasks" => {
            uart.write(b"Tarefas em Execucao:\r\n").await.unwrap();
            uart.write(b"  1. adc_task       - Amostragem ADC (PA1)\r\n")
                .await
                .unwrap();
            uart.write(b"  2. button_task    - Monitoramento de botao (PC13)\r\n")
                .await
                .unwrap();
            uart.write(b"  3. uart_task      - Console/UART handler\r\n")
                .await
                .unwrap();
            uart.write(b"  4. pwm_task       - Controle PWM (TIM3)\r\n")
                .await
                .unwrap();
            uart.write(b"  5. main_loop      - LED blinker\r\n")
                .await
                .unwrap();
        }
        "mem" => {
            uart.write(b"Configuracao de Memoria (primeiras 10 linhas):\r\n")
                .await
                .unwrap();

            const MEMORY_X: &str = include_str!("../memory.x");

            for (i, line) in MEMORY_X.lines().enumerate() {
                if i >= 7 {
                    break;
                } // Para depois de 10 linhas

                uart.write(line.as_bytes()).await.unwrap();
                uart.write(b"\r\n").await.unwrap();
            }
        }
        "info" => {
            uart.write(b"Informacoes do Sistema:\r\n").await.unwrap();
            uart.write(b"  MCU: STM32G474\r\n").await.unwrap();
            uart.write(b"  Core: ARM Cortex-M4F\r\n").await.unwrap();
            uart.write(b"  Clock do Sistema: 170 MHz\r\n").await.unwrap();
            uart.write(b"  Runtime: Embassy (async)\r\n").await.unwrap();
            uart.write(b"  Features: ADC, UART, I2C, PWM, EXTI\r\n")
                .await
                .unwrap();
        }
        "rtinfo" => {
            uart.write(b"Informacoes de Tarefas em Tempo Real:\r\n")
                .await
                .unwrap();
            uart.write(b"  adc_task:\r\n").await.unwrap();
            uart.write(b"    Periodo: 500ms\r\n").await.unwrap();
            uart.write(b"    Prioridade: Normal\r\n").await.unwrap();
            uart.write(b"    Funcao: Amostragem ADC\r\n").await.unwrap();
            uart.write(b"  button_task:\r\n").await.unwrap();
            uart.write(b"    Tipo: Event-driven (EXTI)\r\n")
                .await
                .unwrap();
            uart.write(b"    Prioridade: Normal\r\n").await.unwrap();
            uart.write(b"    Funcao: Deteccao de pressionamento de botao\r\n")
                .await
                .unwrap();
            uart.write(b"  pwm_task:\r\n").await.unwrap();
            uart.write(b"    Periodo: 300ms (4 steps)\r\n")
                .await
                .unwrap();
            uart.write(b"    Prioridade: Normal\r\n").await.unwrap();
            uart.write(b"    Funcao: Geracao de PWM\r\n")
                .await
                .unwrap();
            uart.write(b"  main_loop:\r\n").await.unwrap();
            uart.write(b"    Periodo: 1000ms\r\n").await.unwrap();
            uart.write(b"    Prioridade: Baixa\r\n").await.unwrap();
            uart.write(b"    Funcao: LED toggle\r\n").await.unwrap();
        }
        "status" => {
            uart.write(b"Task Status:\r\n").await.unwrap();
            uart.write(b"  Todas as tasks: RODANDO\r\n").await.unwrap();
            uart.write(b"  Executor: Embassy async\r\n").await.unwrap();
            uart.write(b"  Total de Tasks: 5\r\n").await.unwrap();
        }
        "motor_switch" => {
            let was_enabled = MOTOR_ENABLED.load(Ordering::Relaxed);
            let now_enabled = !was_enabled;
            MOTOR_ENABLED.store(now_enabled, Ordering::Relaxed);

            if now_enabled {
                uart.write(b"Motor: ON\r\n").await.unwrap();
                MOTOR_START_PULSE.store(true, Ordering::Relaxed);
            } else {
                uart.write(b"Motor: OFF\r\n").await.unwrap();
            }
        }
        "" => {
            // Empty command, do nothing
        }
        _ => {
            info!("Comando desconhecido: {}", cmd);
            uart.write(b"Comando desconhecido: ").await.unwrap();
            uart.write(cmd.as_bytes()).await.unwrap();
            uart.write(b"\r\n").await.unwrap();
            uart.write(b"Digite 'help' para ver os comandos disponiveis\r\n")
                .await
                .unwrap();
        }
    }
}
