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
// TAREFA DE MONITORAMENTO DE ALERTAS - MODIFICADA PARA ACIONAR PLAYBACK VIA NOTIFICAÇÃO
//---------------------------------------------------------------------------------------------//
void task_alerta_proximidade(void *params) {
    if (xSemaphoreTake(self_test_sem, portMAX_DELAY) == pdTRUE) {
        printf("[TAREFA] Sinal recebido. Iniciando Alerta/Acao Task...\n");

        uint16_t distancia_local;
        InputData_t input_local; // Struct para armazenar o estado dos botões
        enum { NORMAL, ATENCAO, ALERTA_SONORO, ACAO_SERVO } estado_atual = NORMAL;
        bool acao_ja_executada = false; // Flag para garantir que a notificação seja enviada apenas uma vez

        while (true) {
            // Obtém a distância mais recente
            if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE) {
                distancia_local = distancia_global_mm;
                xSemaphoreGive(sensor_data_mutex);
            }
            // Obtém o estado mais recente dos botões/joystick
            if (xSemaphoreTake(input_mutex, portMAX_DELAY) == pdTRUE) {
                input_local = shared_input_data;
                xSemaphoreGive(input_mutex);
            }

            // --- CONDIÇÃO DE GUARDA PARA AÇÃO AUTOMÁTICA ---
            // Verifica se o controle manual está inativo
            bool controle_manual_inativo = (!input_local.btn_a_pressed && global_control_mode == MODE_IDLE);

            // 1. Condição de Alerta Crítico (sempre ativo, prioridade máxima)
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
            // 2. Condição de Ação (AGORA ACIONA A REPRODUÇÃO VIA NOTIFICAÇÃO)
            else if (distancia_local <= 150 && !acao_ja_executada && controle_manual_inativo) {
                if (estado_atual != ACAO_SERVO) {
                    estado_atual = ACAO_SERVO;
                    gpio_put(LED_RGB_R, 1); gpio_put(LED_RGB_G, 1); gpio_put(LED_RGB_B, 1); // Branco
                    buzzer_set_alarm(false);
                }
                
                // Só notifica se houver uma sequência gravada para reproduzir
                if (record_index > 0) {
                    printf("ACAO: Objeto detectado. Solicitando reproducao da sequencia gravada...\n");
                    
                    // Envia uma notificação direta para a task_control_braco
                    xTaskNotifyGive(handle_control_braco);
                }
                
                acao_ja_executada = true; // Marca que a ação foi solicitada para não repetir
            } 
            // 3. Lógica de Atenção
            else if (distancia_local < 300) {
                if (estado_atual != ATENCAO) {
                    estado_atual = ATENCAO;
                    gpio_put(LED_RGB_R, 1); gpio_put(LED_RGB_G, 1); gpio_put(LED_RGB_B, 0); // Amarelo
                    buzzer_set_alarm(false);
                }
                // Rearma a ação automática se o objeto sair da zona de perigo
                if (distancia_local > 150) {
                    acao_ja_executada = false;
                }
                vTaskDelay(pdMS_TO_TICKS(200));
            } 
            // 4. Condição Normal
            else {
                if (estado_atual != NORMAL) {
                    estado_atual = NORMAL;
                    // Apenas muda o LED se o controle manual estiver inativo
                    if (controle_manual_inativo) {
                        gpio_put(LED_RGB_R, 0); gpio_put(LED_RGB_G, 1); gpio_put(LED_RGB_B, 0); // Verde
                    }
                    buzzer_set_alarm(false);
                }
                // Rearma a ação automática
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
// TAREFA DE CONTROLE DA GARRA (BOTÃO A) - COM FILTRO DE EVENTOS DUPLICADOS
//---------------------------------------------------------------------------------------------//
void task_control_garra(void *params) {
    printf("[TAREFA] Iniciando Controle da Garra (Botao A)...\n");
    InputData_t local_data;
    
    static float current_garra_angle = 90.0f;
    const float R_SPEED = 2.0f;
    
    // "Memória" para o último comando de ângulo enviado para a garra.
    // Inicializa com um valor inválido para forçar o primeiro envio de comando.
    static float last_sent_angle = -1.0f; 

    while (true) {
        // Obtém a cópia mais recente dos dados de entrada.
        if (xSemaphoreTake(input_mutex, portMAX_DELAY) == pdTRUE) {
            local_data = shared_input_data;
            xSemaphoreGive(input_mutex);
        }

        float angle_command = 90.0f; // Comando padrão = PARAR

        if (local_data.btn_a_pressed) {
            // Determina a intenção de movimento com base no joystick.
            if (local_data.joy_y_norm > 0.5f) { // Joystick para cima -> Abrir
                if (current_garra_angle > GARRA_MIN_ANGLE) {
                    current_garra_angle -= R_SPEED;
                    angle_command = 70.0f; // Comando para girar em um sentido
                }
            } else if (local_data.joy_y_norm < -0.5f) { // Joystick para baixo -> Fechar
                if (current_garra_angle < GARRA_MAX_ANGLE) {
                    current_garra_angle += R_SPEED;
                    angle_command = 110.0f; // Comando para girar no outro sentido
                }
            }
            // Se o joystick estiver no centro ou um limite foi atingido, angle_command permanece 90.0f (PARAR).

            // Garante que a posição virtual não ultrapasse os limites.
            if (current_garra_angle < GARRA_MIN_ANGLE) current_garra_angle = GARRA_MIN_ANGLE;
            if (current_garra_angle > GARRA_MAX_ANGLE) current_garra_angle = GARRA_MAX_ANGLE;

            // --- LÓGICA DE FILTRO ---
            // Só envia/grava/loga se o comando for DIFERENTE do anterior.
            if (angle_command != last_sent_angle) {
                last_sent_angle = angle_command; // Atualiza a memória

                ServoCommand_t cmd = { .servo_id = SERVO_GARRA, .angle = angle_command };
                
                if (is_recording && record_index < MAX_RECORDED_STEPS) {
                    recorded_sequence[record_index++] = cmd;
                }
                
                xQueueSend(servo_command_queue, &cmd, 0);
                printf("COMANDO [Garra]: Posicao Virtual: %.1f | Comando: %.1f\n", current_garra_angle, angle_command);
            }
        } else {
            // Reseta a memória quando o botão é solto para garantir que o próximo comando seja enviado.
            last_sent_angle = -1.0f;
        }
        
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

//---------------------------------------------------------------------------------------------//
// TAREFA DE CONTROLE DO BRAÇO (BOTÃO B) - COM LÓGICA DE GRAVAÇÃO/REPRODUÇÃO E FILTRO
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
    // Estas variáveis 'static' "lembram" a posição simulada de cada servo.
    static float current_angles[3] = {90.0f, 90.0f, 90.0f}; // Base, Braço, Ângulo
    const float ROTATION_SPEED = 2.0f; // Velocidade de rotação em "graus virtuais" por ciclo.
    
    // "Memória" para o último comando de ângulo enviado para cada servo (Base, Braço, Ângulo).
    static float last_sent_commands[3] = {-1.0f, -1.0f, -1.0f};

    while (true) {
        // 1. Obtém a cópia mais recente dos dados de entrada.
        if (xSemaphoreTake(input_mutex, portMAX_DELAY) == pdTRUE) {
            local_data = shared_input_data;
            xSemaphoreGive(input_mutex);
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());

        // --- FUNÇÃO AUXILIAR PARA PLAYBACK INTELIGENTE ---
        void play_sequence() {
            const uint32_t MOVE_DURATION_MS = 300; // Duração de cada passo de movimento

            for (int i = 0; i < record_index; i++) {
                ServoCommand_t step = recorded_sequence[i];
                xQueueSend(servo_command_queue, &step, portMAX_DELAY);

                // Se o comando é de MOVIMENTO (não de parada), segura por mais tempo.
                if (step.angle != 90.0f) {
                    vTaskDelay(pdMS_TO_TICKS(MOVE_DURATION_MS));
                } else {
                    // Se for um comando de PARADA, a pausa pode ser curta.
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }
            // Garante que todos os servos parem no final da sequência
            ServoCommand_t stop_cmd;
            stop_cmd.angle = 90.0f;
            stop_cmd.servo_id = SERVO_BASE;   xQueueSend(servo_command_queue, &stop_cmd, 0);
            stop_cmd.servo_id = SERVO_BRACO;  xQueueSend(servo_command_queue, &stop_cmd, 0);
            stop_cmd.servo_id = SERVO_GARRA;  xQueueSend(servo_command_queue, &stop_cmd, 0);
            stop_cmd.servo_id = SERVO_ANGULO; xQueueSend(servo_command_queue, &stop_cmd, 0);
        }

        // --- 2. OUVINTE DE NOTIFICAÇÃO PARA REPRODUÇÃO AUTOMÁTICA ---
        if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
            if (current_mode == MODE_IDLE && !is_recording && record_index > 0) {
                printf("\n--- REPRODUÇÃO ACIONADA PELO SENSOR ---\n");
                play_sequence();
                printf("--- REPRODUÇÃO CONCLUÍDA ---\n");
            }
        }

        // --- 3. LÓGICA DE TRIGGERS (GRAVAÇÃO E REPRODUÇÃO MANUAL) ---

        // Trigger do Botão do Joystick (SW): Inicia/Para Gravação
        if (local_data.btn_sw_pressed && !last_btn_sw_state && (now - last_debounce_time > DEBOUNCE_DELAY)) {
            last_debounce_time = now;
            is_recording = !is_recording;
            if (is_recording) {
                record_index = 0;
                printf("\n*** GRAVAÇÃO INICIADA ***\n");
            } else {
                printf("\n*** GRAVAÇÃO PARADA. %d passos gravados. ***\n", record_index);
                printf("------------------------------------------\n");
                printf("--- Resumo da Sequencia Gravada ---\n");
                const char* servo_names[] = {"BASE", "BRACO", "GARRA", "ANGULO"};
                for (int i = 0; i < record_index; i++) {
                    ServoCommand_t step = recorded_sequence[i];
                    if (step.servo_id >= SERVO_BASE && step.servo_id <= SERVO_ANGULO) {
                        printf("  Passo %3d: Servo %-6s | Comando de Angulo: %.1f\n", 
                               i + 1, servo_names[step.servo_id], step.angle);
                    }
                }
                printf("------------------------------------------\n\n");
            }
        }
        last_btn_sw_state = local_data.btn_sw_pressed;

        // 4. Se o Botão A (Garra) estiver ativo, esta tarefa cede a prioridade.
        if (local_data.btn_a_pressed) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // 5. Lógica de clique do Botão B: Alterna modos OU inicia a reprodução manual.
        if (local_data.btn_b_pressed && !last_btn_b_state && (now - last_debounce_time > DEBOUNCE_DELAY)) {
            last_debounce_time = now;
            if (current_mode == MODE_IDLE) {
                if (!is_recording && record_index > 0) {
                    printf("\n--- REPRODUZINDO %d PASSOS (MANUAL) ---\n", record_index);
                    play_sequence();
                    printf("--- REPRODUÇÃO CONCLUÍDA ---\n");
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

        // 6. Se um modo de controle manual estiver ativo, executa a lógica de movimento.
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
// TAREFA SERVO MANAGER
//---------------------------------------------------------------------------------------------//
void task_servo_manager(void *params) {
    printf("[TAREFA] Iniciando Servo Manager...\n");
    ServoCommand_t received_command;

    // --- Máquina de Estados Interna ---
    // Armazena a última posição comandada para cada um dos 4 servos
    float last_angles[4] = {90.0f, 90.0f, 90.0f, 90.0f}; // Base, Braço, Garra, Ângulo
    // Armazena o tempo do último comando recebido para cada servo
    uint32_t last_command_time[4] = {0, 0, 0, 0};
    // Tempo em milissegundos para desligar um servo que não recebe comandos
    const uint32_t SERVO_TIMEOUT_MS = 500;

    while (true) {
        // 1. Espera por um novo comando na fila.
        // O timeout de 20ms faz com que o loop execute regularmente para verificar os timeouts dos servos.
        if (xQueueReceive(servo_command_queue, &received_command, pdMS_TO_TICKS(20)) == pdTRUE) {
            // --- Se um comando foi recebido ---
            uint8_t id = received_command.servo_id;
            
            // Atualiza o estado interno: o novo ângulo e o tempo atual
            last_angles[id] = received_command.angle;
            last_command_time[id] = to_ms_since_boot(get_absolute_time());

            // Envia o comando para o hardware PWM correspondente
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
        }

        // 2. Lógica de Timeout (executada a cada ~20ms)
        // Verifica cada servo individualmente para desligá-lo se estiver inativo.
        uint32_t now = to_ms_since_boot(get_absolute_time());
        for (uint8_t id = 0; id < 4; id++) {
            // Se um servo não recebe um comando por mais de SERVO_TIMEOUT_MS...
            if (last_command_time[id] != 0 && (now - last_command_time[id] > SERVO_TIMEOUT_MS)) {
                // ...desliga o pulso PWM apenas daquele servo.
                switch (id) {
                    case SERVO_BASE:   pwm_set_chan_level(slice_base,   pwm_gpio_to_channel(SERVO_BASE_PIN),   0); break;
                    case SERVO_BRACO:  pwm_set_chan_level(slice_braco,  pwm_gpio_to_channel(SERVO_BRACO_PIN),  0); break;
                    case SERVO_GARRA:  pwm_set_chan_level(slice_garra,  pwm_gpio_to_channel(SERVO_GARRA_PIN),  0); break;
                    case SERVO_ANGULO: pwm_set_chan_level(slice_angulo, pwm_gpio_to_channel(SERVO_ANGULO_PIN), 0); break;
                }
                // Reseta o timer para não tentar desligar repetidamente
                last_command_time[id] = 0;
            }
        }
    }
}