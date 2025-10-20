//---------------------------------------------------------------------------------------------//
// main.c (versão corrigida)
//---------------------------------------------------------------------------------------------//
#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "perifericos.h"
#include "tarefas.h"

// Ponteiros para controlar as tarefas
TaskHandle_t handle_self_test = NULL;
TaskHandle_t handle_distancia_display = NULL;
TaskHandle_t handle_alerta_proximidade = NULL;

// Primitivas de sincronização
SemaphoreHandle_t self_test_sem = NULL;
SemaphoreHandle_t i2c0_mutex = NULL;
SemaphoreHandle_t sensor_data_mutex = NULL;

int main() {
    stdio_init_all();
    sleep_ms(2000);
    // Aguarda em um loop até que a conexão USB seja estabelecida com o computador.
    // while (!stdio_usb_connected()) {
    //     sleep_ms(200);
    // }

    init_perifericos();
    buzzer_pwm_init();

    // --- CORREÇÃO AQUI ---
    // Cria um semáforo de contagem que pode "segurar" até 2 permissões.
    // Começa com 0 permissões disponíveis.
    self_test_sem = xSemaphoreCreateCounting(2, 0); // maxCount = 2, initialCount = 0

    i2c0_mutex = xSemaphoreCreateMutex();
    sensor_data_mutex = xSemaphoreCreateMutex();

    if (self_test_sem == NULL || i2c0_mutex == NULL || sensor_data_mutex == NULL) {
        printf("ERRO: Falha ao criar primitivas RTOS.\n");
        while(1);
    }

    printf("Sistema inicializado. Criando tarefas FreeRTOS...\n");

    xTaskCreate(task_self_test, "Self-Test Task", 2048, NULL, 1, &handle_self_test);
    
    xTaskCreate(task_alerta_proximidade, "Alerta Prox", 512, NULL, 3, &handle_alerta_proximidade);
    xTaskCreate(task_distancia_display, "Distancia Display", 2048, NULL, 2, &handle_distancia_display);

    vTaskStartScheduler();

    while (true) {}
}