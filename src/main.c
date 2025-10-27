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
TaskHandle_t handle_coordenador_controle = NULL;
TaskHandle_t handle_control_garra = NULL;
TaskHandle_t handle_control_braco = NULL;
TaskHandle_t handle_servo_manager = NULL;

// Primitivas de sincronização
SemaphoreHandle_t self_test_sem = NULL;
SemaphoreHandle_t i2c0_mutex = NULL;
SemaphoreHandle_t sensor_data_mutex = NULL;
// SemaphoreHandle_t servo_mutex = NULL;
SemaphoreHandle_t input_mutex = NULL; 
QueueHandle_t servo_command_queue = NULL;

int main() {
    stdio_init_all();
    sleep_ms(2000);
    // Aguarda em um loop até que a conexão USB seja estabelecida com o computador.
    while (!stdio_usb_connected()) {
        sleep_ms(200);
    }

    init_perifericos();
    buzzer_pwm_init();

    // --- CORREÇÃO AQUI ---
    // Cria um semáforo de contagem que pode "segurar" até 2 permissões.
    // Começa com 0 permissões disponíveis.
    self_test_sem = xSemaphoreCreateCounting(2, 0); // maxCount = 2, initialCount = 0

    i2c0_mutex = xSemaphoreCreateMutex();
    sensor_data_mutex = xSemaphoreCreateMutex();
    // servo_mutex = xSemaphoreCreateMutex();
    input_mutex = xSemaphoreCreateMutex();
    // Cria uma fila que pode conter até 10 comandos de servo
    servo_command_queue = xQueueCreate(10, sizeof(ServoCommand_t));

    if (self_test_sem == NULL || i2c0_mutex == NULL || sensor_data_mutex == NULL || input_mutex == NULL || servo_command_queue == NULL)  {
        printf("ERRO: Falha ao criar primitivas RTOS.\n");
        while(1);
    }

    printf("Sistema inicializado. Criando tarefas FreeRTOS...\n");

    xTaskCreate(task_self_test, "Self-Test", 2048, NULL, 1, &handle_self_test); // Prioridade mais baixa
    xTaskCreate(task_distancia_display, "Display", 2048, NULL, 2, &handle_distancia_display);
    xTaskCreate(task_coordenador_controle, "Coord Ctrl", 1024, NULL, 2, &handle_coordenador_controle);

    // Tarefas de controle manual (produtoras)
    xTaskCreate(task_control_garra, "Garra Ctrl", 1024, NULL, 3, &handle_control_garra);
    xTaskCreate(task_control_braco, "Braco Ctrl", 1024, NULL, 3, &handle_control_braco);

    // Tarefa gerente do servo (consumidora) - DEVE ter prioridade alta
    xTaskCreate(task_servo_manager, "Servo Mgr", 1024, NULL, 4, &handle_servo_manager);

    // Tarefa de alerta (crítica) - DEVE ter a prioridade mais alta
    xTaskCreate(task_alerta_proximidade, "Alerta", 512, NULL, 5, &handle_alerta_proximidade);

    vTaskStartScheduler();

    while (true) {}
}