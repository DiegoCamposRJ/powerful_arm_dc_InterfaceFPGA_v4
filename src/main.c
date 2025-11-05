//---------------------------------------------------------------------------------------------//
// main.c
//---------------------------------------------------------------------------------------------//
#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "perifericos.h"
#include "tarefas.h"

//---------------------------------------------------------------------------------------------//
// Handles para Tarefas e Primitivas de Sincronização
//---------------------------------------------------------------------------------------------//

// --- Ponteiros (Handles) para controlar as tarefas ---
TaskHandle_t handle_self_test = NULL;
TaskHandle_t handle_distancia_display = NULL;
TaskHandle_t handle_alerta_proximidade = NULL;
TaskHandle_t handle_coordenador_controle = NULL;
TaskHandle_t handle_control_garra = NULL;
TaskHandle_t handle_control_braco = NULL;
TaskHandle_t handle_servo_manager = NULL;
TaskHandle_t handle_fpga_receiver = NULL;

// --- Primitivas de Sincronização ---
SemaphoreHandle_t self_test_sem = NULL;
SemaphoreHandle_t i2c0_mutex = NULL;
SemaphoreHandle_t sensor_data_mutex = NULL;
SemaphoreHandle_t input_mutex = NULL;
SemaphoreHandle_t fpga_sequence_mutex = NULL;
QueueHandle_t servo_command_queue = NULL;


//---------------------------------------------------------------------------------------------//
// Função Principal (Ponto de Entrada do Programa)
//---------------------------------------------------------------------------------------------//
int main() {
    // Inicializa a comunicação serial via USB para o monitor (printf)
    stdio_init_all();
    // Aguarda um tempo para a conexão do monitor serial ser estabelecida
    sleep_ms(2000);

    // Chama as funções que inicializam todo o hardware da placa
    init_perifericos();
    buzzer_pwm_init();

    // --- Criação das Primitivas do FreeRTOS ---
    self_test_sem = xSemaphoreCreateCounting(2, 0); // Para liberar as 2 tarefas principais após o teste
    i2c0_mutex = xSemaphoreCreateMutex();           // Para proteger o barramento I2C
    sensor_data_mutex = xSemaphoreCreateMutex();    // Para proteger a variável global de distância
    input_mutex = xSemaphoreCreateMutex();          // Para proteger a struct de dados de entrada
    fpga_sequence_mutex = xSemaphoreCreateMutex();  // Para proteger o buffer da sequência da FPGA
    
    
    // Cria uma fila que pode conter até 10 comandos de servo
    servo_command_queue = xQueueCreate(10, sizeof(ServoCommand_t));

    // Verificação de segurança: garante que todas as primitivas foram criadas com sucesso
    if (self_test_sem == NULL || i2c0_mutex == NULL || sensor_data_mutex == NULL || 
        input_mutex == NULL || fpga_sequence_mutex == NULL || servo_command_queue == NULL) {
        printf("ERRO CRITICO: Falha ao criar primitivas RTOS.\n");
        while(1); // Trava o sistema
    }

    printf("Sistema inicializado. Criando tarefas FreeRTOS...\n");

    // --- Criação das Tarefas do FreeRTOS com Prioridades Definidas ---
    // A prioridade define a importância da tarefa. Números maiores = maior prioridade.
    
    // Prioridade 1 (Mais Baixa): Tarefa de teste que roda apenas uma vez.
    xTaskCreate(task_self_test, "Self-Test", 2048, NULL, 1, &handle_self_test);
    
    // Prioridade 2 (Baixa): Tarefas de background e interface com o usuário.
    xTaskCreate(task_distancia_display, "Display", 2048, NULL, 2, &handle_distancia_display);
    xTaskCreate(task_coordenador_controle, "Coord Ctrl", 1024, NULL, 2, &handle_coordenador_controle);
    
    // Prioridade 3 (Média): Tarefas de controle manual e recepção de dados.
    xTaskCreate(task_control_garra, "Garra Ctrl", 1024, NULL, 3, &handle_control_garra);
    xTaskCreate(task_control_braco, "Braco Ctrl", 2048, NULL, 3, &handle_control_braco);
    xTaskCreate(task_fpga_receiver, "FPGA Rx", 2048, NULL, 3, &handle_fpga_receiver);
    
    // Prioridade 4 (Média-Alta): Tarefa que gerencia o hardware do servo. Deve ser mais prioritária que as tarefas que lhe enviam comandos.
    xTaskCreate(task_servo_manager, "Servo Mgr", 2048, NULL, 4, &handle_servo_manager);
    
    // Prioridade 5 (Mais Alta): Tarefa de alerta. Deve ter a resposta mais rápida a eventos críticos.
    xTaskCreate(task_alerta_proximidade, "Alerta", 1024, NULL, 5, &handle_alerta_proximidade);

    // Inicia o escalonador do FreeRTOS. A partir deste ponto, o RTOS assume o controle.
    vTaskStartScheduler();

    // Este loop infinito nunca deve ser alcançado.
    while (true) {}
}