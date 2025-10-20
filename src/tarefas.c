//---------------------------------------------------------------------------------------------//
// tarefas.c
//---------------------------------------------------------------------------------------------//
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "perifericos.h"
#include "bitdoglab_pins.h"
#include "aht10.h"
#include "oled_context.h"
#include "ssd1306_text.h"
#include "numeros_display.h"

//---------------------------------------------------------------------------------------------//
// Variáveis Globais e Externas
//---------------------------------------------------------------------------------------------//

// Primitivas de sincronização (definidas em main.c)
extern SemaphoreHandle_t self_test_sem;
extern SemaphoreHandle_t i2c0_mutex;
extern SemaphoreHandle_t sensor_data_mutex;

// Instância do OLED (definida em oled_context.c)
extern ssd1306_t oled;

// Constante de conversão (definida em perifericos.c)
extern const float ADC_CONVERSION_FACTOR;

// Variável global para compartilhar os dados do sensor entre as tarefas
aht10_data_t dados_sensor_clima;


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
    test_sensor_aht10();     vTaskDelay(pdMS_TO_TICKS(1000));

    printf("[TAREFA 1] Self-Test concluido. Sinalizando para as outras tarefas.\n\n");

    // Libera o semáforo para as 3 tarefas que estão esperando
    xSemaphoreGive(self_test_sem); // Para task_alive
    xSemaphoreGive(self_test_sem); // Para task_aht10_display
    xSemaphoreGive(self_test_sem); // Para task_alerta_clima

    // A tarefa de teste se deleta, pois não é mais necessária
    vTaskDelete(NULL);
}


//---------------------------------------------------------------------------------------------//
// Tarefa 2: Alive Task (Sinal de Vida)
//---------------------------------------------------------------------------------------------//
void task_alive(void *params) {
    if (xSemaphoreTake(self_test_sem, portMAX_DELAY) == pdTRUE) {
        printf("[TAREFA 2] Sinal recebido. Iniciando Alive Task...\n");
        
        while (true) {
            gpio_put(LED_ALIVE_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_put(LED_ALIVE_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}


//---------------------------------------------------------------------------------------------//
// Tarefa 4: Leitura do Sensor AHT10 e Exibição no OLED
//---------------------------------------------------------------------------------------------//
void task_aht10_display(void *params) {
    if (xSemaphoreTake(self_test_sem, portMAX_DELAY) == pdTRUE) {
        printf("[TAREFA 4] Sinal recebido. Iniciando AHT10 Display Task...\n");
        
        aht10_data_t dados_locais; // Variável local para a leitura
        char linha_temp[20];
        char linha_umid[20];
        bool leitura_ok = false;

        while (true) {
            // Bloco de Leitura do Sensor (protegido pelo mutex do I2C)
            if (xSemaphoreTake(i2c0_mutex, portMAX_DELAY) == pdTRUE) {
                leitura_ok = aht10_read_data(i2c0, &dados_locais);
                xSemaphoreGive(i2c0_mutex);
            }

            if (leitura_ok) {
                // Atualiza a variável global de forma segura (protegido pelo mutex de dados)
                if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE) {
                    dados_sensor_clima = dados_locais;
                    xSemaphoreGive(sensor_data_mutex);
                }

                // Formata e exibe no OLED (usando dados locais para evitar re-leitura)
                snprintf(linha_temp, sizeof(linha_temp), "Temp: %.2f C", dados_locais.temperature);
                snprintf(linha_umid, sizeof(linha_umid), "Umid: %.2f %%", dados_locais.humidity);

                // Bloco de Escrita no Display (protegido pelo mutex do I2C)
                if (xSemaphoreTake(i2c0_mutex, portMAX_DELAY) == pdTRUE) {
                    oled_clear(&oled);
                    ssd1306_draw_utf8_multiline(oled.ram_buffer, 0, 16, linha_temp, oled.width, oled.height);
                    ssd1306_draw_utf8_multiline(oled.ram_buffer, 0, 32, linha_umid, oled.width, oled.height);
                    oled_render(&oled);
                    xSemaphoreGive(i2c0_mutex);
                }

            } else {
                // Em caso de falha na leitura, exibe uma mensagem de erro no OLED
                if (xSemaphoreTake(i2c0_mutex, portMAX_DELAY) == pdTRUE) {
                    oled_clear(&oled);
                    oled_centralizar_texto(&oled, "ERRO SENSOR", 3);
                    oled_render(&oled);
                    xSemaphoreGive(i2c0_mutex);
                }
            }
            
            // Aguarda 2 segundos para o próximo ciclo completo de leitura e exibição
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}


//---------------------------------------------------------------------------------------------//
// Tarefa 5: Monitoramento de Alertas de Clima
//---------------------------------------------------------------------------------------------//
void task_alerta_clima(void *params) {
    if (xSemaphoreTake(self_test_sem, portMAX_DELAY) == pdTRUE) {
        printf("[TAREFA 5] Sinal recebido. Iniciando Alerta Clima Task...\n");

        aht10_data_t dados_locais; // Variável local para a cópia segura dos dados
        bool em_alerta = false;

        while (true) {
            // Copia os dados da variável global de forma segura
            if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE) {
                dados_locais = dados_sensor_clima;
                xSemaphoreGive(sensor_data_mutex);
            }

            // Lógica do Alerta: verifica as condições de temperatura e umidade
            if (dados_locais.temperature >= 31.00f || dados_locais.humidity >= 70.0f) {
                // Condição de alerta: Liga o LED Vermelho
                // Liga o LED Vermelho
                em_alerta = true;
                gpio_put(LED_RGB_R, 1);
                gpio_put(LED_RGB_G, 0);
                gpio_put(LED_RGB_B, 0);

                // --- Lógica da Sirene ---
                buzzer_set_freq(1500); // Frequência alta
                buzzer_set_alarm(true);
                vTaskDelay(pdMS_TO_TICKS(100)); // Toca por 100ms

                buzzer_set_freq(800);  // Frequência baixa
                vTaskDelay(pdMS_TO_TICKS(100)); // Toca por 100ms

            } else if(dados_locais.temperature >= 30.10f || dados_locais.humidity >= 65.0f) {
                // led amarelo ligado como atenção
                gpio_put(LED_RGB_R, 1);
                gpio_put(LED_RGB_G, 1);
                gpio_put(LED_RGB_B, 0);
                buzzer_set_alarm(true); 
                buzzer_set_freq(100);  // Frequência baixa
                vTaskDelay(pdMS_TO_TICKS(50)); // Toca por 100ms
            }else {
                // if (em_alerta) {
                //     em_alerta = false;
                    // Condição normal: led verde ligado
                    gpio_put(LED_RGB_R, 0);
                    gpio_put(LED_RGB_G, 1);
                    gpio_put(LED_RGB_B, 0);
                    buzzer_set_alarm(false); 
                // }
                
            }

            if (!em_alerta) {
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
    }
}




    