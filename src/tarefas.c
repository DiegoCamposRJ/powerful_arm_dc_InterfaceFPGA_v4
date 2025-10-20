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
// #include "aht10.h"
#include "oled_context.h"
#include "ssd1306_text.h"
#include "numeros_display.h"
#include "vl53l0x.h"
#include "tarefas.h" // Inclui o próprio header para consistência

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
// aht10_data_t dados_sensor_clima;
uint16_t distancia_global_mm = 0;
vl53l0x_dev sensor_dev; // <-- ADICIONE A INSTÂNCIA GLOBAL DO SENSOR


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
    // test_sensor_aht10();     vTaskDelay(pdMS_TO_TICKS(1000));
    test_sensor_vl53l0x();   vTaskDelay(pdMS_TO_TICKS(1000)); 

    printf("[TAREFA 1] Self-Test concluido. Sinalizando para as outras tarefas.\n\n");

    // Libera o semáforo para as 3 tarefas que estão esperando
    xSemaphoreGive(self_test_sem); // Para task_alive
    xSemaphoreGive(self_test_sem); // Para task_aht10_display
    // xSemaphoreGive(self_test_sem); // Para task_alerta_clima

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

/**
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
*/





/**
//---------------------------------------------------------------------------------------------//
// Tarefa 6: Leitura do Sensor VL53L0X e Exibição no OLED
//---------------------------------------------------------------------------------------------//
void task_distancia_display(void *params) {
    if (xSemaphoreTake(self_test_sem, portMAX_DELAY) == pdTRUE) {
        printf("[TAREFA 6] Sinal recebido. Iniciando Distancia Display Task...\n");

        char linha_dist[20];
        uint16_t distancia_mm;
        bool leitura_ok = false;

        while (true) {
            // Bloco de Leitura do Sensor (protegido pelo mutex do I2C0)
            if (xSemaphoreTake(i2c0_mutex, portMAX_DELAY) == pdTRUE) {
                distancia_mm = vl53l0x_read_range_single_millimeters(i2c0);
                xSemaphoreGive(i2c0_mutex);
            }

            // O sensor retorna valores altos (ex: 8190) quando não consegue ler
            leitura_ok = (distancia_mm < 8190);

            if (leitura_ok) {
                snprintf(linha_dist, sizeof(linha_dist), "Dist: %d mm", distancia_mm);
            } else {
                snprintf(linha_dist, sizeof(linha_dist), "Dist: --- mm");
            }

            // Bloco de Escrita no Display (protegido pelo mutex do I2C)
            // ATENÇÃO: O OLED está no i2c1, mas o recurso é o display em si.
            // Vamos precisar de um mutex para o display para evitar que esta tarefa
            // e a task_aht10_display escrevam ao mesmo tempo.
            if (xSemaphoreTake(i2c0_mutex, portMAX_DELAY) == pdTRUE) { // Reutilizando o mutex por simplicidade
                // Escreve na linha 5 (y=48) para não apagar os dados do AHT10
                ssd1306_draw_utf8_multiline(oled.ram_buffer, 0, 48, "                ", oled.width, oled.height); // Limpa a linha
                ssd1306_draw_utf8_multiline(oled.ram_buffer, 0, 48, linha_dist, oled.width, oled.height);
                oled_render(&oled);
                xSemaphoreGive(i2c0_mutex);
            }
            
            vTaskDelay(pdMS_TO_TICKS(500)); // Lê a distância a cada 500ms
        }
    }
}

*/
    

//---------------------------------------------------------------------------------------------//
// tarefas.c (tarefa de display corrigida)
//---------------------------------------------------------------------------------------------//
void task_distancia_display(void *params) {
    if (xSemaphoreTake(self_test_sem, portMAX_DELAY) == pdTRUE) {
        printf("[TAREFA] Sinal recebido. Iniciando Distancia Display Task...\n");
        
        // --- CORREÇÃO CRÍTICA AQUI ---
        // Adiciona uma pequena pausa para garantir que o sensor esteja totalmente
        // estável após a complexa sequência de inicialização.
        vTaskDelay(pdMS_TO_TICKS(100));

        // Inicia o modo de medição contínua.
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
                    // Exibe "----" em caso de erro de leitura
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
// Tarefa de Monitoramento de Alertas de PROXIMIDADE
//---------------------------------------------------------------------------------------------//
void task_alerta_proximidade(void *params) {
    if (xSemaphoreTake(self_test_sem, portMAX_DELAY) == pdTRUE) {
        printf("[TAREFA] Sinal recebido. Iniciando Alerta Proximidade Task...\n");

        uint16_t distancia_local;
        enum { NORMAL, ATENCAO, ALERTA } estado_atual = NORMAL;

        while (true) {
            if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE) {
                distancia_local = distancia_global_mm;
                xSemaphoreGive(sensor_data_mutex);
            }

            if (distancia_local < 100) {
                if (estado_atual != ALERTA) {
                    estado_atual = ALERTA;
                    gpio_put(LED_RGB_R, 1); gpio_put(LED_RGB_G, 0); gpio_put(LED_RGB_B, 0);
                }
                buzzer_set_freq(1500);
                buzzer_set_alarm(true);
                vTaskDelay(pdMS_TO_TICKS(100));
                buzzer_set_alarm(false);
                vTaskDelay(pdMS_TO_TICKS(100));

            } else if (distancia_local < 300) {
                if (estado_atual != ATENCAO) {
                    estado_atual = ATENCAO;
                    gpio_put(LED_RGB_R, 1); gpio_put(LED_RGB_G, 1); gpio_put(LED_RGB_B, 0);
                    buzzer_set_alarm(false);
                }
                vTaskDelay(pdMS_TO_TICKS(200));

            } else {
                if (estado_atual != NORMAL) {
                    estado_atual = NORMAL;
                    gpio_put(LED_RGB_R, 0); gpio_put(LED_RGB_G, 1); gpio_put(LED_RGB_B, 0);
                    buzzer_set_alarm(false);
                }
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
    }
}