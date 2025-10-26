#ifndef TAREFAS_H
#define TAREFAS_H

#include "FreeRTOS.h"
#include "task.h"
#include "vl53l0x.h" // <-- ADICIONE ESTE INCLUDE para conhecer o tipo 'vl53l0x_dev'

// Handle para a tarefa de autoteste
extern TaskHandle_t handle_self_test;
extern TaskHandle_t handle_alive_task; 
extern TaskHandle_t handle_aht10_display;
// extern TaskHandle_t handle_alerta_clima;
extern TaskHandle_t handle_distancia_display;
extern TaskHandle_t handle_alerta_proximidade;

// --- DECLARAÇÃO PÚBLICA DO SENSOR ---
// Anuncia que a variável 'sensor_dev' existe em algum lugar do projeto.
extern vl53l0x_dev sensor_dev; // <-- ADICIONE ESTA LINHA

extern TaskHandle_t handle_braco_controle;

// Protótipo da tarefa
void task_self_test(void *params);
void task_alive(void *params);
void task_aht10_display(void *params);
// void task_alerta_clima(void *params);
void task_distancia_display(void *params);
void task_alerta_proximidade(void *params);
void task_braco_controle(void *params);

#endif