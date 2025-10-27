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
// TAREFA DE MONITORAMENTO DE ALERTAS (VERSÃO FINAL E ROBUSTA)
//---------------------------------------------------------------------------------------------//
void task_alerta_proximidade(void *params) {
    if (xSemaphoreTake(self_test_sem, portMAX_DELAY) == pdTRUE) {
        printf("[TAREFA] Sinal recebido. Iniciando Alerta/Acao Task...\n");

        uint16_t distancia_local;
        enum { NORMAL, ATENCAO, ALERTA_SONORO, ACAO_SERVO } estado_atual = NORMAL;

        while (true) {
            if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE) {
                distancia_local = distancia_global_mm;
                xSemaphoreGive(sensor_data_mutex);
            }

            // --- LÓGICA DE ESTADO CORRIGIDA ---

            // 1. Condição de Alerta Crítico (MAIS ESPECÍFICA)
            if (distancia_local < 100) {
                if (estado_atual != ALERTA_SONORO) {
                    estado_atual = ALERTA_SONORO;
                    gpio_put(LED_RGB_R, 1); gpio_put(LED_RGB_G, 0); gpio_put(LED_RGB_B, 0);
                }
                buzzer_set_freq(1500);
                buzzer_set_alarm(true);
                vTaskDelay(pdMS_TO_TICKS(100));
                buzzer_set_alarm(false);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            // 2. Condição de Ação do Servo (MENOS ESPECÍFICA)
            else if (distancia_local <= 150) {
                if (estado_atual != ACAO_SERVO) {
                    estado_atual = ACAO_SERVO;
                    gpio_put(LED_RGB_R, 1); gpio_put(LED_RGB_G, 1); gpio_put(LED_RGB_B, 1); // Branco
                    buzzer_set_alarm(false);
                }
                
                ServoCommand_t cmd = { .servo_id = SERVO_BASE, .angle = 0.0f };
                // **FERRAMENTA DE DIAGNÓSTICO:** Verifica se o envio falhou
                if (xQueueSend(servo_command_queue, &cmd, 0) != pdPASS) {
                    printf("ERRO: Fila de comandos do servo esta cheia! Comando de alerta descartado.\n");
                }
                
                vTaskDelay(pdMS_TO_TICKS(100));
            } 
            // 3. Lógica de Atenção
            else if (distancia_local < 300) {
                if (estado_atual != ATENCAO) {
                    estado_atual = ATENCAO;
                    gpio_put(LED_RGB_R, 1); gpio_put(LED_RGB_G, 1); gpio_put(LED_RGB_B, 0); // Amarelo
                    buzzer_set_alarm(false);
                }
                vTaskDelay(pdMS_TO_TICKS(200));
            } 
            // 4. Condição Normal
            else {
                if (estado_atual != NORMAL) {
                    estado_atual = NORMAL;
                    gpio_put(LED_RGB_R, 0); gpio_put(LED_RGB_G, 1); gpio_put(LED_RGB_B, 0); // Verde
                    buzzer_set_alarm(false);
                }
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
        local_input_data.btn_a_pressed = !gpio_get(BTN_A_PIN);
        local_input_data.btn_b_pressed = !gpio_get(BTN_B_PIN);

        adc_select_input(0); // Eixo Y
        local_input_data.joy_y_norm = (adc_read() - 2048.0f) / 2048.0f;
        adc_select_input(1); // Eixo X
        local_input_data.joy_x_norm = (adc_read() - 2048.0f) / 2048.0f;

        if (xSemaphoreTake(input_mutex, portMAX_DELAY) == pdTRUE) {
            shared_input_data = local_input_data;
            xSemaphoreGive(input_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

//---------------------------------------------------------------------------------------------//
// TAREFA DE CONTROLE DA GARRA (BOTÃO A)
//---------------------------------------------------------------------------------------------//
void task_control_garra(void *params) {
    printf("[TAREFA] Iniciando Controle da Garra (Botao A)...\n");
    float garra_angle = 90.0f;
    const float SMOOTHING_ALPHA = 0.1f;
    InputData_t local_data;

    while (true) {
        if (xSemaphoreTake(input_mutex, portMAX_DELAY) == pdTRUE) {
            local_data = shared_input_data;
            xSemaphoreGive(input_mutex);
        }

        if (local_data.btn_a_pressed) {
            if (fabs(local_data.joy_y_norm) > 0.15f) {
                float target = 90.0f - (local_data.joy_y_norm * 90.0f);
                garra_angle = garra_angle * (1.0f - SMOOTHING_ALPHA) + target * SMOOTHING_ALPHA;
                
                ServoCommand_t cmd = { .servo_id = SERVO_GARRA, .angle = garra_angle };
                xQueueSend(servo_command_queue, &cmd, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

//---------------------------------------------------------------------------------------------//
// TAREFA DE CONTROLE DO BRAÇO (BOTÃO B) - VERSÃO FINAL
//---------------------------------------------------------------------------------------------//
void task_control_braco(void *params) {
    printf("[TAREFA] Iniciando Controle do Braco (Botao B)...\n");
    
    // A máquina de estados para ciclar entre os modos de controle
    typedef enum { MODE_IDLE, MODE_BASE, MODE_BRACO, MODE_ANGULO } control_mode_t;
    control_mode_t current_mode = MODE_IDLE;

    // Variáveis para a lógica de detecção de clique (toggle) com debounce
    bool last_btn_b_state = false;
    uint32_t last_debounce_time = 0;
    const uint32_t DEBOUNCE_DELAY = 250; // Evita múltiplos cliques

    // Armazena a última posição conhecida de cada servo para suavização
    float angles[3] = {90.0f, 90.0f, 90.0f}; // Posições para Base, Braço, Ângulo
    const float SMOOTHING_ALPHA = 0.1f;      // Fator de suavização para movimentos fluidos
    
    // Struct local para armazenar os dados de entrada lidos da variável global
    InputData_t local_data;

    while (true) {
        // 1. Obtém a cópia mais recente dos dados de entrada (joystick e botões)
        if (xSemaphoreTake(input_mutex, portMAX_DELAY) == pdTRUE) {
            local_data = shared_input_data;
            xSemaphoreGive(input_mutex);
        }

        // 2. Verifica se o controle da Garra (Botão A) está ativo. Se estiver, esta tarefa cede a prioridade.
        if (local_data.btn_a_pressed) {
            // Pausa e pula o resto do loop para não interferir com o controle da garra.
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // 3. Lógica de clique do Botão B para alternar (ciclar) os modos de controle
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (local_data.btn_b_pressed && !last_btn_b_state && (now - last_debounce_time > DEBOUNCE_DELAY)) {
            last_debounce_time = now;
            current_mode = (current_mode + 1) % 4; // Cicla os modos: 0->1->2->3->0
            
            // Fornece feedback visual e no monitor serial sobre o modo atual
            const char* modes[] = {"IDLE", "BASE", "BRACO", "ANGULO"};
            printf("MODO (Botao B): %s\n", modes[current_mode]);
        }
        last_btn_b_state = local_data.btn_b_pressed;

        // 4. Executa a ação correspondente ao modo atual, se não for IDLE
        if (current_mode != MODE_IDLE) {
            ServoCommand_t cmd;
            bool send_cmd = false;

            switch (current_mode) {
                case MODE_BASE:
                    // Feedback visual: LED Vermelho
                    gpio_put(LED_RGB_R, 1); gpio_put(LED_RGB_G, 0); gpio_put(LED_RGB_B, 0);
                    // Controla a Base com o eixo X do joystick
                    if (fabs(local_data.joy_x_norm) > 0.15f) {
                        float target = 90.0f + (local_data.joy_x_norm * 90.0f);
                        angles[0] = angles[0] * (1.0f - SMOOTHING_ALPHA) + target * SMOOTHING_ALPHA;
                        cmd = (ServoCommand_t){ .servo_id = SERVO_BASE, .angle = angles[0] };
                        send_cmd = true;
                    }
                    break;

                case MODE_BRACO:
                    // Feedback visual: LED Verde
                    gpio_put(LED_RGB_R, 0); gpio_put(LED_RGB_G, 1); gpio_put(LED_RGB_B, 0);
                    // Controla o Braço (altura) com o eixo Y do joystick
                    if (fabs(local_data.joy_y_norm) > 0.15f) {
                        float target = 90.0f - (local_data.joy_y_norm * 90.0f); // Eixo Y invertido
                        angles[1] = angles[1] * (1.0f - SMOOTHING_ALPHA) + target * SMOOTHING_ALPHA;
                        cmd = (ServoCommand_t){ .servo_id = SERVO_BRACO, .angle = angles[1] };
                        send_cmd = true;
                    }
                    break;

                case MODE_ANGULO:
                    // Feedback visual: LED Azul
                    gpio_put(LED_RGB_R, 0); gpio_put(LED_RGB_G, 0); gpio_put(LED_RGB_B, 1);
                    // Controla o Ângulo com o eixo X do joystick
                    if (fabs(local_data.joy_x_norm) > 0.15f) {
                        float target = 90.0f + (local_data.joy_x_norm * 90.0f);
                        angles[2] = angles[2] * (1.0f - SMOOTHING_ALPHA) + target * SMOOTHING_ALPHA;
                        cmd = (ServoCommand_t){ .servo_id = SERVO_ANGULO, .angle = angles[2] };
                        send_cmd = true;
                    }
                    break;
                
                case MODE_IDLE:
                    // Não faz nada, o LED será controlado pela task de alerta
                    break;
            }

            // Se um movimento foi calculado, envia o comando para a fila do Servo Manager
            if (send_cmd) {
                xQueueSend(servo_command_queue, &cmd, 0);
            }
        }
        
        // Pausa a tarefa para permitir que outras executem
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