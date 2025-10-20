#ifndef TAREFAS_H
#define TAREFAS_H

#include "FreeRTOS.h"
#include "task.h"

// Handle para a tarefa de autoteste
extern TaskHandle_t handle_self_test;
extern TaskHandle_t handle_alive_task; 
extern TaskHandle_t handle_aht10_display;
extern TaskHandle_t handle_alerta_clima;

// Protótipo da tarefa
void task_self_test(void *params);
void task_alive(void *params);
void task_aht10_display(void *params);
void task_alerta_clima(void *params);

#endif