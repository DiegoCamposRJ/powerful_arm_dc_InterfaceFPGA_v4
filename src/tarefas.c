//---------------------------------------------------------------------------------------------//
// tarefas.c
//---------------------------------------------------------------------------------------------//
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "perifericos.h"
#include "bitdoglab_pins.h"
#include "vl53l0x.h"
#include "oled_context.h"
#include "numeros_display.h"
#include "tarefas.h"
#include "braco_config.h"
#include "fpga_comms.h" 

//---------------------------------------------------------------------------------------------//
// Variáveis Globais e Externas
//---------------------------------------------------------------------------------------------//
extern SemaphoreHandle_t self_test_sem;
extern SemaphoreHandle_t i2c0_mutex;
extern SemaphoreHandle_t sensor_data_mutex;
extern SemaphoreHandle_t input_mutex;
extern QueueHandle_t servo_command_queue;
ControlMode_t global_control_mode = MODE_IDLE;

// --- VARIÁVEIS PARA GRAVAÇÃO E REPRODUÇÃO ---
#define MAX_RECORDED_STEPS 200 // Aumente se precisar de sequências mais longas
ServoCommand_t recorded_sequence[MAX_RECORDED_STEPS];
int record_index = 0;
bool is_recording = false;

// --- VARIÁVEIS PARA RECEBIMENTO E PLAYBACK DA FPGA ---
#define MAX_FPGA_STEPS 50 // Tamanho do buffer para a sequência vinda da FPGA
ServoCommand_t playback_sequence_from_fpga[MAX_FPGA_STEPS];
int fpga_sequence_length = 0;
// SemaphoreHandle_t fpga_sequence_mutex; // Mutex para proteger o acesso a este buffer

extern ssd1306_t oled;
vl53l0x_dev sensor_dev;
uint16_t distancia_global_mm = 0;
InputData_t shared_input_data = {0};

//---------------------------------------------------------------------------------------------//
// Tarefa 1: Self-Test
//---------------------------------------------------------------------------------------------//
void task_self_test(void *params) {
    printf("\n[TAREFA 1] Iniciando Self-Test...\n");
    vTaskDelay(pdMS_TO_TICKS(2000));

    test_leds_rgb();         vTaskDelay(pdMS_TO_TICKS(1000));
    test_display_oled();     vTaskDelay(pdMS_TO_TICKS(1000));
    test_buzzer_pwm();       vTaskDelay(pdMS_TO_TICKS(1000));
    test_botoes();           vTaskDelay(pdMS_TO_TICKS(1000));
    test_microfone();        vTaskDelay(pdMS_TO_TICKS(1000));
    test_joystick();         vTaskDelay(pdMS_TO_TICKS(1000));
    test_sensor_vl53l0x();   vTaskDelay(pdMS_TO_TICKS(1000));
    test_servos();           vTaskDelay(pdMS_TO_TICKS(1000));

    printf("[TAREFA 1] Self-Test concluido. Sinalizando para as outras tarefas.\n\n");

    xSemaphoreGive(self_test_sem);
    xSemaphoreGive(self_test_sem);

    vTaskDelete(NULL);
}

//---------------------------------------------------------------------------------------------//
// Tarefa de Leitura do Sensor VL53L0X e Exibição no OLED
//---------------------------------------------------------------------------------------------//
void task_distancia_display(void *params) {
    if (xSemaphoreTake(self_test_sem, portMAX_DELAY) == pdTRUE) {
        printf("[TAREFA] Sinal recebido. Iniciando Distancia Display Task...\n");
        
        vTaskDelay(pdMS_TO_TICKS(100));
        vl53l0x_start_continuous(&sensor_dev, 0);

        uint16_t distancia_local_mm;
        bool leitura_ok;

        while (true) {
            if (xSemaphoreTake(i2c0_mutex, portMAX_DELAY) == pdTRUE) {
                distancia_local_mm = vl53l0x_read_range_continuous_millimeters(&sensor_dev);
                xSemaphoreGive(i2c0_mutex);
            }

            leitura_ok = (distancia_local_mm < 8190);

            if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE) {
                distancia_global_mm = leitura_ok ? distancia_local_mm : 9999;
                xSemaphoreGive(sensor_data_mutex);
            }

            if (xSemaphoreTake(i2c0_mutex, portMAX_DELAY) == pdTRUE) {
                oled_clear(&oled);
                oled_centralizar_texto(&oled, "Distancia (mm)", 1);
                
                if (leitura_ok) {
                    oled_exibir_4digitos(&oled, distancia_local_mm);
                } else {
                    oled_exibir_caractere_grande(&oled, '-', 10);
                    oled_exibir_caractere_grande(&oled, '-', 36);
                    oled_exibir_caractere_grande(&oled, '-', 62);
                    oled_exibir_caractere_grande(&oled, '-', 88);
                }
                
                oled_render(&oled);
                xSemaphoreGive(i2c0_mutex);
            }
            
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

//---------------------------------------------------------------------------------------------//
// TAREFA DE ALERTA - MODIFICADA PARA PEDIR E EXECUTAR DADOS DA FPGA
//---------------------------------------------------------------------------------------------//
void task_alerta_proximidade(void *params) {
    // Espera o sinal do self_test para começar a operar.
    if (xSemaphoreTake(self_test_sem, portMAX_DELAY) == pdTRUE) {
        printf("[TAREFA] Sinal recebido. Iniciando Alerta/Acao Task...\n");

        uint16_t distancia_local;
        InputData_t input_local; // Struct para armazenar o estado dos botões
        enum { NORMAL, ATENCAO, ALERTA_SONORO, ACAO_SERVO } estado_atual = NORMAL;
        bool acao_ja_executada = false; // Flag para garantir que a sequência seja acionada apenas uma vez

        while (true) {
            // 1. Obtém os dados de estado mais recentes de forma segura.
            // Obtém a distância do sensor.
            if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE) {
                distancia_local = distancia_global_mm;
                xSemaphoreGive(sensor_data_mutex);
            }
            // Obtém o estado dos botões/joystick.
            if (xSemaphoreTake(input_mutex, portMAX_DELAY) == pdTRUE) {
                input_local = shared_input_data;
                xSemaphoreGive(input_mutex);
            }

            // 2. CONDIÇÃO DE GUARDA: Verifica se o controle manual está inativo.
            bool controle_manual_inativo = (!input_local.btn_a_pressed && global_control_mode == MODE_IDLE);

            // --- INÍCIO DA MÁQUINA DE ESTADOS DE ALERTA ---

            // ESTADO 1: Alerta Crítico (prioridade máxima)
            if (distancia_local < 100) {
                if (estado_atual != ALERTA_SONORO) {
                    estado_atual = ALERTA_SONORO;
                    gpio_put(LED_RGB_R, 1); gpio_put(LED_RGB_G, 0); gpio_put(LED_RGB_B, 0); // Vermelho
                }
                buzzer_set_freq(1500);
                buzzer_set_alarm(true);
                vTaskDelay(pdMS_TO_TICKS(100));
                buzzer_set_alarm(false);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            // ESTADO 2: Ação Automática (só executa se o controle manual estiver inativo)
            else if (distancia_local <= 150 && !acao_ja_executada && controle_manual_inativo) {
                if (estado_atual != ACAO_SERVO) {
                    estado_atual = ACAO_SERVO;
                    gpio_put(LED_RGB_R, 1); gpio_put(LED_RGB_G, 1); gpio_put(LED_RGB_B, 1); // Branco
                    buzzer_set_alarm(false);
                }
                
                printf("ACAO: Objeto detectado. Solicitando sequencia da FPGA...\n");
                
                // Limpa o buffer de playback antes de pedir uma nova sequência.
                if (xSemaphoreTake(fpga_sequence_mutex, portMAX_DELAY) == pdTRUE) {
                    fpga_sequence_length = 0;
                    xSemaphoreGive(fpga_sequence_mutex);
                }

                // Envia o comando de LEITURA (0xB0) para a FPGA, pedindo a sequência no endereço 0.
                uint8_t cmd[2] = {0xB0, 0};
                uart_write_blocking(UART_ID, cmd, 2);

                // Pausa para dar tempo à FPGA de responder e à task_fpga_receiver de preencher o buffer.
                vTaskDelay(pdMS_TO_TICKS(500)); 

                // Notifica o Servo Manager para executar a sequência que acabou de ser recebida.
                printf("... Solicitando execucao da sequencia recebida da FPGA.\n");
                xTaskNotifyGive(handle_servo_manager);
                
                acao_ja_executada = true; // Marca que a ação foi solicitada para não repetir.
            } 
            // ESTADO 3: Atenção
            else if (distancia_local < 300) {
                if (estado_atual != ATENCAO) {
                    estado_atual = ATENCAO;
                    gpio_put(LED_RGB_R, 1); gpio_put(LED_RGB_G, 1); gpio_put(LED_RGB_B, 0); // Amarelo
                    buzzer_set_alarm(false);
                }
                // Rearma a ação automática se o objeto sair da zona de perigo.
                if (distancia_local > 150) {
                    acao_ja_executada = false;
                }
                vTaskDelay(pdMS_TO_TICKS(200));
            } 
            // ESTADO 4: Normal
            else {
                if (estado_atual != NORMAL) {
                    estado_atual = NORMAL;
                    // Só muda o LED para verde se o controle manual não estiver usando o LED.
                    if (controle_manual_inativo) {
                        gpio_put(LED_RGB_R, 0); gpio_put(LED_RGB_G, 1); gpio_put(LED_RGB_B, 0); // Verde
                    }
                    buzzer_set_alarm(false);
                }
                // Rearma a ação automática.
                acao_ja_executada = false;
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
    }
}

//---------------------------------------------------------------------------------------------//
// TAREFA COORDENADOR DE CONTROLE
//---------------------------------------------------------------------------------------------//
void task_coordenador_controle(void *params) {
    printf("[TAREFA] Iniciando Coordenador de Controle...\n");
    InputData_t local_input_data;

    while (true) {
        // --- LEITURA DE TODAS AS ENTRADAS DE CONTROLE ---
        local_input_data.btn_a_pressed = !gpio_get(BTN_A_PIN);
        local_input_data.btn_b_pressed = !gpio_get(BTN_B_PIN);
        
        // -------------------------------------------------------------------
        // --- CORREÇÃO CRÍTICA AQUI ---
        // Adiciona a leitura do botão do joystick que estava faltando.
        // Sem esta linha, a funcionalidade de gravação nunca é acionada.
        local_input_data.btn_sw_pressed = !gpio_get(BTN_SW_PIN);
        // -------------------------------------------------------------------

        adc_select_input(0); // Eixo Y
        local_input_data.joy_y_norm = (adc_read() - 2048.0f) / 2048.0f;
        adc_select_input(1); // Eixo X
        local_input_data.joy_x_norm = (adc_read() - 2048.0f) / 2048.0f;

        // Publica os dados de forma segura
        if (xSemaphoreTake(input_mutex, portMAX_DELAY) == pdTRUE) {
            shared_input_data = local_input_data;
            xSemaphoreGive(input_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}



//---------------------------------------------------------------------------------------------//
// TAREFA DE CONTROLE DA GARRA (BOTÃO A) - LÓGICA PARA ROTAÇÃO CONTÍNUA
//---------------------------------------------------------------------------------------------//
void task_control_garra(void *params) {
    printf("[TAREFA] Iniciando Controle da Garra (Botao A)...\n");
    
    // Struct local para armazenar os dados de entrada.
    InputData_t local_data;
    
    // "Memória" para o último comando de ângulo enviado, para evitar o envio de comandos duplicados.
    // Inicializa com um valor inválido para forçar o primeiro envio.
    static float last_sent_angle = -1.0f; 

    while (true) {
        // 1. Obtém a cópia mais recente dos dados de entrada.
        if (xSemaphoreTake(input_mutex, portMAX_DELAY) == pdTRUE) {
            local_data = shared_input_data;
            xSemaphoreGive(input_mutex);
        }

        // Comando padrão a ser enviado é PARAR.
        float angle_command = 90.0f;

        // 2. A lógica só é executada se o Botão A estiver pressionado.
        if (local_data.btn_a_pressed) {
            // Joystick para cima -> Gira em um sentido
            if (local_data.joy_y_norm > 0.5f) {
                angle_command = 70.0f;
            } 
            // Joystick para baixo -> Gira no outro sentido
            else if (local_data.joy_y_norm < -0.5f) {
                angle_command = 110.0f;
            }
            // Se o joystick estiver no centro, o comando permanece 90.0f (PARAR).

            // 3. Filtra comandos duplicados: só age se o estado do movimento mudou.
            if (angle_command != last_sent_angle) {
                last_sent_angle = angle_command; // Atualiza a memória com o novo comando

                ServoCommand_t cmd = { .servo_id = SERVO_GARRA, .angle = angle_command };
                
                // Se a gravação estiver ativa, salva o comando de mudança de estado.
                if (is_recording && record_index < MAX_RECORDED_STEPS) {
                    recorded_sequence[record_index++] = cmd;
                }
                
                // Envia o comando (Girar ou Parar) para a fila do Servo Manager.
                xQueueSend(servo_command_queue, &cmd, 0);
                printf("COMANDO [Garra]: Enviando comando de velocidade %.1f\n", angle_command);
            }
        } else {
            // Se o botão A foi solto, reseta a memória para garantir que o próximo
            // comando de parada ou movimento seja enviado na próxima vez que for pressionado.
            last_sent_angle = -1.0f;
        }
        
        // Pausa a tarefa por 25ms.
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

//---------------------------------------------------------------------------------------------//
// TAREFA DE CONTROLE DO BRAÇO (BOTÃO B) - COM ENVIO DA GRAVAÇÃO PARA FPGA
//---------------------------------------------------------------------------------------------//
void task_control_braco(void *params) {
    printf("[TAREFA] Iniciando Controle do Braco (Botao B)...\n");
    
    // A máquina de estados para ciclar entre os modos de controle.
    typedef enum { MODE_IDLE, MODE_BASE, MODE_BRACO, MODE_ANGULO } control_mode_t;
    control_mode_t current_mode = MODE_IDLE;

    // Variáveis para a lógica de detecção de clique (toggle) com debounce.
    bool last_btn_b_state = false;
    bool last_btn_sw_state = false; // Para o botão do joystick
    uint32_t last_debounce_time = 0;
    const uint32_t DEBOUNCE_DELAY = 250;
    
    // Struct local para armazenar os dados de entrada.
    InputData_t local_data;

    // --- Posição Virtual ---
    static float current_angles[3] = {90.0f, 90.0f, 90.0f}; // Base, Braço, Ângulo
    const float ROTATION_SPEED = 2.0f;
    
    // "Memória" para o último comando de ângulo enviado.
    static float last_sent_commands[3] = {-1.0f, -1.0f, -1.0f};

    while (true) {
        // 1. Obtém a cópia mais recente dos dados de entrada.
        if (xSemaphoreTake(input_mutex, portMAX_DELAY) == pdTRUE) {
            local_data = shared_input_data;
            xSemaphoreGive(input_mutex);
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());

        // --- FUNÇÃO AUXILIAR PARA ENVIAR SEQUÊNCIA PARA FPGA ---
        void send_sequence_to_fpga(const char* trigger_source) {
            printf("\n--- INICIANDO ENVIO PARA FPGA (%d PASSOS, Acionado por: %s) ---\n", record_index, trigger_source);

            // Posições atuais dos servos para preencher o pacote
            float current_pos[4] = {90.0f, 90.0f, 90.0f, 90.0f}; // Base, Braço, Garra, Ângulo

            for (int i = 0; i < record_index; i++) {
                ServoCommand_t step = recorded_sequence[i];
                
                // Atualiza a posição do servo que se moveu neste passo
                if (step.angle == 70.0f) { // Girar em um sentido
                    current_pos[step.servo_id] -= 10; // Simula um movimento de 10 graus
                } else if (step.angle == 110.0f) { // Girar no outro sentido
                    current_pos[step.servo_id] += 10;
                }
                // Garante que os ângulos fiquem dentro de 0-180 para envio
                if (current_pos[step.servo_id] < 0) current_pos[step.servo_id] = 0;
                if (current_pos[step.servo_id] > 180) current_pos[step.servo_id] = 180;

                // Envia o estado COMPLETO de todos os 4 servos para a FPGA
                fpga_send_sequence_step(i, 
                                      current_pos[SERVO_BASE], 
                                      current_pos[SERVO_BRACO], 
                                      current_pos[SERVO_ANGULO], 
                                      current_pos[SERVO_GARRA]);
                
                // Pausa entre os passos para a FPGA processar
                vTaskDelay(pdMS_TO_TICKS(500)); 
            }
            
            printf("--- ENVIO PARA FPGA CONCLUÍDO ---\n");
        }

        // --- 2. OUVINTE DE NOTIFICAÇÃO PARA ENVIO AUTOMÁTICO ---
        if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
            if (current_mode == MODE_IDLE && !is_recording && record_index > 0) {
                send_sequence_to_fpga("Sensor");
            }
        }

        // --- 3. LÓGICA DE TRIGGERS (GRAVAÇÃO E ENVIO MANUAL) ---
        if (local_data.btn_sw_pressed && !last_btn_sw_state && (now - last_debounce_time > DEBOUNCE_DELAY)) {
            last_debounce_time = now;
            is_recording = !is_recording;
            if (is_recording) {
                record_index = 0;
                printf("\n*** GRAVAÇÃO INICIADA ***\n");
            } else {
                printf("\n*** GRAVAÇÃO PARADA. %d passos gravados. ***\n", record_index);
                // ... (bloco de impressão do resumo da gravação) ...
            }
        }
        last_btn_sw_state = local_data.btn_sw_pressed;

        // 4. Se o Botão A (Garra) estiver ativo, esta tarefa cede a prioridade.
        if (local_data.btn_a_pressed) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // 5. Lógica de clique do Botão B: Alterna modos OU inicia o envio para a FPGA.
        if (local_data.btn_b_pressed && !last_btn_b_state && (now - last_debounce_time > DEBOUNCE_DELAY)) {
            last_debounce_time = now;
            if (current_mode == MODE_IDLE) {
                if (!is_recording && record_index > 0) {
                    send_sequence_to_fpga("Manual");
                } else {
                    current_mode = (control_mode_t)((current_mode + 1) % 4);
                }
            } else {
                current_mode = (control_mode_t)((current_mode + 1) % 4);
            }
            global_control_mode = current_mode;
            const char* modes[] = {"IDLE", "BASE", "BRACO", "ANGULO"};
            printf("MODO (Botao B): %s\n", modes[current_mode]);
            for(int i=0; i<3; i++) last_sent_commands[i] = -1.0f;
        }
        last_btn_b_state = local_data.btn_b_pressed;

        // 6. Se um modo de controle manual (local) estiver ativo, executa a lógica de movimento.
        if (current_mode != MODE_IDLE) {
            float angle_command = 90.0f;
            const char* servo_name = "";
            int servo_idx = -1;

            switch (current_mode) {
                case MODE_BASE:
                    servo_name = "Base";
                    servo_idx = 0;
                    gpio_put(LED_RGB_R, 1); gpio_put(LED_RGB_G, 0); gpio_put(LED_RGB_B, 0);
                    if (local_data.joy_x_norm > 0.5f && current_angles[0] < BASE_MAX_ANGLE) {
                        current_angles[0] += ROTATION_SPEED;
                        angle_command = 110.0f;
                    } else if (local_data.joy_x_norm < -0.5f && current_angles[0] > BASE_MIN_ANGLE) {
                        current_angles[0] -= ROTATION_SPEED;
                        angle_command = 70.0f;
                    }
                    if (current_angles[0] < BASE_MIN_ANGLE) current_angles[0] = BASE_MIN_ANGLE;
                    if (current_angles[0] > BASE_MAX_ANGLE) current_angles[0] = BASE_MAX_ANGLE;
                    break;
                case MODE_BRACO:
                    servo_name = "Braco";
                    servo_idx = 1;
                    gpio_put(LED_RGB_R, 0); gpio_put(LED_RGB_G, 1); gpio_put(LED_RGB_B, 0);
                    if (local_data.joy_y_norm > 0.5f && current_angles[1] < BRACO_MAX_ANGLE) {
                        current_angles[1] += ROTATION_SPEED;
                        angle_command = 70.0f;
                    } else if (local_data.joy_y_norm < -0.5f && current_angles[1] > BRACO_MIN_ANGLE) {
                        current_angles[1] -= ROTATION_SPEED;
                        angle_command = 110.0f;
                    }
                    if (current_angles[1] < BRACO_MIN_ANGLE) current_angles[1] = BRACO_MIN_ANGLE;
                    if (current_angles[1] > BRACO_MAX_ANGLE) current_angles[1] = BRACO_MAX_ANGLE;
                    break;
                case MODE_ANGULO:
                    servo_name = "Angulo";
                    servo_idx = 2;
                    gpio_put(LED_RGB_R, 0); gpio_put(LED_RGB_G, 0); gpio_put(LED_RGB_B, 1);
                    if (local_data.joy_x_norm > 0.5f && current_angles[2] < ANGULO_MAX_ANGLE) {
                        current_angles[2] += ROTATION_SPEED;
                        angle_command = 110.0f;
                    } else if (local_data.joy_x_norm < -0.5f && current_angles[2] > ANGULO_MIN_ANGLE) {
                        current_angles[2] -= ROTATION_SPEED;
                        angle_command = 70.0f;
                    }
                    if (current_angles[2] < ANGULO_MIN_ANGLE) current_angles[2] = ANGULO_MIN_ANGLE;
                    if (current_angles[2] > ANGULO_MAX_ANGLE) current_angles[2] = ANGULO_MAX_ANGLE;
                    break;
            }

            if (servo_idx != -1) {
                if (angle_command != last_sent_commands[servo_idx]) {
                    last_sent_commands[servo_idx] = angle_command;
                    ServoCommand_t cmd = { .servo_id = (ServoID_t)servo_idx, .angle = angle_command };
                    if (is_recording && record_index < MAX_RECORDED_STEPS) {
                        recorded_sequence[record_index++] = cmd;
                    }
                    xQueueSend(servo_command_queue, &cmd, 0);
                    printf("COMANDO [%s]: Posicao Virtual: %.1f | Comando: %.1f\n", servo_name, current_angles[servo_idx], angle_command);
                }
            }
        } else {
            for(int i=0; i<3; i++) last_sent_commands[i] = -1.0f;
        }
        
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

//---------------------------------------------------------------------------------------------//
// TAREFA SERVO MANAGER - MODIFICADA PARA EXECUTAR SEQUÊNCIA DA FPGA
//---------------------------------------------------------------------------------------------//
void task_servo_manager(void *params) {
    printf("[TAREFA] Iniciando Servo Manager...\n");
    ServoCommand_t received_command;

    // --- Máquina de Estados Interna ---
    // Armazena a última posição de comando para cada um dos 4 servos.
    float last_angles[4] = {90.0f, 90.0f, 90.0f, 90.0f}; // Base, Braço, Garra, Ângulo
    // Armazena o tempo do último comando recebido para cada servo.
    uint32_t last_command_time[4] = {0, 0, 0, 0};
    // Tempo em milissegundos para desligar um servo que não recebe comandos.
    const uint32_t SERVO_TIMEOUT_MS = 500;

    while (true) {
        // --- 1. OUVINTE DE NOTIFICAÇÃO PARA EXECUTAR SEQUÊNCIA DA FPGA ---
        // Verifica se recebeu uma notificação da task_alerta_proximidade.
        if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
            printf("-> Servo Mgr: Notificacao recebida para executar sequencia da FPGA.\n");
            
            // Trava o buffer da sequência para lê-lo com segurança.
            if (xSemaphoreTake(fpga_sequence_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (fpga_sequence_length > 0) {
                    // Itera sobre a sequência que foi recebida e armazenada pela task_fpga_receiver.
                    for (int i = 0; i < fpga_sequence_length; i++) {
                        // Envia o comando da sequência para sua própria fila para ser processado.
                        xQueueSend(servo_command_queue, &playback_sequence_from_fpga[i], 0);
                        // Pausa entre os passos para dar tempo ao movimento.
                        vTaskDelay(pdMS_TO_TICKS(500));
                    }
                }
                xSemaphoreGive(fpga_sequence_mutex); // Libera o buffer.
            }
        }

        // --- 2. PROCESSAMENTO DA FILA DE COMANDOS (CONTROLE MANUAL E PLAYBACK) ---
        // Espera por um novo comando na fila por até 100ms.
        if (xQueueReceive(servo_command_queue, &received_command, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Se um comando foi recebido (seja do controle manual ou do playback)...
            uint8_t id = received_command.servo_id;
            
            // Atualiza o estado interno: o novo ângulo e o tempo atual.
            last_angles[id] = received_command.angle;
            last_command_time[id] = to_ms_since_boot(get_absolute_time());

            // Envia o comando para o hardware PWM correspondente.
            switch (id) {
                case SERVO_BASE:
                    pwm_set_chan_level(slice_base, pwm_gpio_to_channel(SERVO_BASE_PIN), angle_to_duty(last_angles[id]));
                    break;
                case SERVO_BRACO:
                    pwm_set_chan_level(slice_braco, pwm_gpio_to_channel(SERVO_BRACO_PIN), angle_to_duty(last_angles[id]));
                    break;
                case SERVO_GARRA:
                    pwm_set_chan_level(slice_garra, pwm_gpio_to_channel(SERVO_GARRA_PIN), angle_to_duty(last_angles[id]));
                    break;
                case SERVO_ANGULO:
                    pwm_set_chan_level(slice_angulo, pwm_gpio_to_channel(SERVO_ANGULO_PIN), angle_to_duty(last_angles[id]));
                    break;
            }
        } else {
            // --- 3. LÓGICA DE TIMEOUT (SE A FILA ESTIVER VAZIA) ---
            // Se nenhum comando foi recebido em 100ms, verifica quais servos estão inativos.
            uint32_t now = to_ms_since_boot(get_absolute_time());
            for (uint8_t id = 0; id < 4; id++) {
                // Se um servo não recebe um comando por mais de SERVO_TIMEOUT_MS...
                if (last_command_time[id] != 0 && (now - last_command_time[id] > SERVO_TIMEOUT_MS)) {
                    // ...desliga o pulso PWM apenas daquele servo para economizar energia.
                    switch (id) {
                        case SERVO_BASE:   pwm_set_chan_level(slice_base,   pwm_gpio_to_channel(SERVO_BASE_PIN),   0); break;
                        case SERVO_BRACO:  pwm_set_chan_level(slice_braco,  pwm_gpio_to_channel(SERVO_BRACO_PIN),  0); break;
                        case SERVO_GARRA:  pwm_set_chan_level(slice_garra,  pwm_gpio_to_channel(SERVO_GARRA_PIN),  0); break;
                        case SERVO_ANGULO: pwm_set_chan_level(slice_angulo, pwm_gpio_to_channel(SERVO_ANGULO_PIN), 0); break;
                    }
                    // Reseta o timer para não tentar desligar repetidamente.
                    last_command_time[id] = 0;
                }
            }
        }
    }
}

// =============================================================================================
// TAREFA RECEPTORA DA FPGA - VERSÃO COM DEPURAÇÃO BYTE-A-BYTE
// =============================================================================================
void task_fpga_receiver(void *params) {
    printf("[TAREFA] Iniciando Receptora da FPGA (Modo de Depuracao)...\n");
    uint8_t rx_buffer[4];
    int rx_count = 0;

    while (true) {
        // Verifica se há pelo menos um byte para ler.
        // uart_is_readable() é a forma mais simples de verificar.
        if (uart_is_readable(UART_ID)) {
            
            // Lê um único byte
            uint8_t ch = uart_getc(UART_ID);

            // --- FEEDBACK IMEDIATO NO MONITOR SERIAL ---
            // Imprime cada byte recebido, em hexadecimal.
            printf("-> FPGA Rx: Recebido byte 0x%02X\n", ch);

            // Armazena no buffer
            if (rx_count < 4) {
                rx_buffer[rx_count] = ch;
                rx_count++;
            }

            // Se completamos 4 bytes, processa o pacote completo
            if (rx_count >= 4) {
                printf("--> PACOTE COMPLETO: [B:0x%02X, R:0x%02X, A:0x%02X, G:0x%02X]\n", 
                       rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]);
                
                // Tenta travar o mutex para atualizar a sequência de playback
                if (xSemaphoreTake(fpga_sequence_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    if (fpga_sequence_length < MAX_FPGA_STEPS) {
                        // Converte os bytes recebidos para comandos de servo
                        // (Esta lógica pode precisar de ajuste dependendo do que a FPGA envia)
                        playback_sequence_from_fpga[fpga_sequence_length++] = (ServoCommand_t){SERVO_BASE, (float)rx_buffer[0]};
                        playback_sequence_from_fpga[fpga_sequence_length++] = (ServoCommand_t){SERVO_BRACO, (float)rx_buffer[1]};
                        playback_sequence_from_fpga[fpga_sequence_length++] = (ServoCommand_t){SERVO_ANGULO, (float)rx_buffer[2]};
                        playback_sequence_from_fpga[fpga_sequence_length++] = (ServoCommand_t){SERVO_GARRA, (float)rx_buffer[3]};
                    }
                    xSemaphoreGive(fpga_sequence_mutex);
                }
                
                rx_count = 0; // Reseta para o próximo pacote
            }
        }

        // Pequena pausa para não sobrecarregar a CPU com polling.
        // A tarefa cede tempo para outras tarefas executarem.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}