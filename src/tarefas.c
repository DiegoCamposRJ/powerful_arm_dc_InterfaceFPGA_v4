//---------------------------------------------------------------------------------------------//
// tarefas.c
//---------------------------------------------------------------------------------------------//
#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <math.h>             // <-- CORREÇÃO: Adicionado para a função fabs()
#include "hardware/adc.h"      // <-- CORREÇÃO: Adicionado para as funções do ADC

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
extern ssd1306_t oled;

vl53l0x_dev sensor_dev;
uint16_t distancia_global_mm = 0;

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
    test_servo_base();       vTaskDelay(pdMS_TO_TICKS(1000));

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
// Tarefa de Monitoramento de Alertas e AÇÃO DO SERVO
//---------------------------------------------------------------------------------------------//
void task_alerta_proximidade(void *params) {
    if (xSemaphoreTake(self_test_sem, portMAX_DELAY) == pdTRUE) {
        printf("[TAREFA] Sinal recebido. Iniciando Alerta/Acao Task...\n");

        uint16_t distancia_local;
        enum { NORMAL, ATENCAO, ALERTA, ACAO_SERVO } estado_atual = NORMAL;
        bool servo_foi_acionado = false;

        while (true) {
            if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE) {
                distancia_local = distancia_global_mm;
                xSemaphoreGive(sensor_data_mutex);
            }

            if (distancia_local <= 150 && !servo_foi_acionado) {
                if (estado_atual != ACAO_SERVO) {
                    printf("ACAO: Objeto detectado a %dmm. Movendo servo...\n", distancia_local);
                    estado_atual = ACAO_SERVO;
                    gpio_put(LED_RGB_R, 1); gpio_put(LED_RGB_G, 1); gpio_put(LED_RGB_B, 1);
                    buzzer_set_alarm(false);
                }
                pwm_set_chan_level(slice_base, pwm_gpio_to_channel(SERVO_BASE_PIN), angle_to_duty(0.0f));
                vTaskDelay(pdMS_TO_TICKS(1000));
                pwm_set_chan_level(slice_base, pwm_gpio_to_channel(SERVO_BASE_PIN), 0);
                servo_foi_acionado = true;
            } 
            else if (distancia_local < 100) {
                if (estado_atual != ALERTA) {
                    estado_atual = ALERTA;
                    gpio_put(LED_RGB_R, 1); gpio_put(LED_RGB_G, 0); gpio_put(LED_RGB_B, 0);
                }
                buzzer_set_freq(1500);
                buzzer_set_alarm(true);
                vTaskDelay(pdMS_TO_TICKS(100));
                buzzer_set_alarm(false);
                vTaskDelay(pdMS_TO_TICKS(100));
            } 
            else if (distancia_local < 300) {
                if (estado_atual != ATENCAO) {
                    estado_atual = ATENCAO;
                    gpio_put(LED_RGB_R, 1); gpio_put(LED_RGB_G, 1); gpio_put(LED_RGB_B, 0);
                    buzzer_set_alarm(false);
                }
                if (distancia_local > 150) {
                    servo_foi_acionado = false;
                }
                vTaskDelay(pdMS_TO_TICKS(200));
            } 
            else {
                if (estado_atual != NORMAL) {
                    estado_atual = NORMAL;
                    gpio_put(LED_RGB_R, 0); gpio_put(LED_RGB_G, 1); gpio_put(LED_RGB_B, 0);
                    buzzer_set_alarm(false);
                }
                servo_foi_acionado = false;
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
    }
}

//---------------------------------------------------------------------------------------------//
// Tarefa de Controle do Braço Robótico com Joystick
//---------------------------------------------------------------------------------------------//
void task_braco_controle(void *params) {
    printf("[TAREFA] Iniciando Tarefa de Controle do Braco...\n");

    float current_base_angle = 90.0f;
    const float SMOOTHING_ALPHA = 0.1f;

    while (true) {
        bool btn_a_pressed = !gpio_get(BTN_A_PIN);

        if (btn_a_pressed) {
            adc_select_input(1); // ADC1 é o Eixo X (GP27)
            uint16_t joy_x_raw = adc_read();

            float normalized = (joy_x_raw - 2048.0f) / 2048.0f;

            if (fabs(normalized) < 0.15f) {
                pwm_set_chan_level(slice_base, pwm_gpio_to_channel(SERVO_BASE_PIN), 0);
            } else {
                float target_angle = 90.0f + (normalized * 90.0f);
                current_base_angle = current_base_angle * (1.0f - SMOOTHING_ALPHA) + target_angle * SMOOTHING_ALPHA;
                pwm_set_chan_level(slice_base, pwm_gpio_to_channel(SERVO_BASE_PIN), angle_to_duty(current_base_angle));
            }
        } else {
            pwm_set_chan_level(slice_base, pwm_gpio_to_channel(SERVO_BASE_PIN), 0);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}