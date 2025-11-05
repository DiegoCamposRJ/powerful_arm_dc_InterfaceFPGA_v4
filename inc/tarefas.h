//---------------------------------------------------------------------------------------------//
// tarefas.h
//---------------------------------------------------------------------------------------------//
#ifndef TAREFAS_H
#define TAREFAS_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"      // Necessário para SemaphoreHandle_t
#include "vl53l0x.h"     // Necessário para o tipo vl53l0x_dev

// --- Estrutura para compartilhar dados de entrada ---
typedef struct {
    bool btn_a_pressed;
    bool btn_b_pressed;
    bool btn_sw_pressed;
    float joy_x_norm; // Joystick X normalizado (-1.0 a 1.0)
    float joy_y_norm; // Joystick Y normalizado (-1.0 a 1.0)
} InputData_t;

// --- Enum para o modo de controle do braço ---
typedef enum { MODE_IDLE, MODE_BASE, MODE_BRACO, MODE_ANGULO } ControlMode_t;

// --- ESTRUTURA PARA COMANDOS DO SERVO ---
typedef enum { SERVO_BASE, SERVO_BRACO, SERVO_GARRA, SERVO_ANGULO } ServoID_t;
typedef struct {
    ServoID_t servo_id;
    float angle;
} ServoCommand_t;

// --- Handles das Tarefas ---
extern TaskHandle_t handle_self_test;
extern TaskHandle_t handle_distancia_display;
extern TaskHandle_t handle_alerta_proximidade;
extern TaskHandle_t handle_coordenador_controle;
extern TaskHandle_t handle_control_garra;
extern TaskHandle_t handle_control_braco;
extern TaskHandle_t handle_servo_manager;
extern TaskHandle_t handle_fpga_receiver;

// --- Declaração Pública de Variáveis e Primitivas Globais ---
// Estas variáveis são DEFINIDAS em tarefas.c e declaradas aqui para serem
// visíveis em outros arquivos (como perifericos.c).
extern vl53l0x_dev sensor_dev;
// extern SemaphoreHandle_t servo_mutex;
extern SemaphoreHandle_t input_mutex;
extern QueueHandle_t servo_command_queue; // <-- NOVO: Fila em vez de Mutex
extern ControlMode_t global_control_mode; // <-- NOVO: Variável para compartilhar o modo do botão 
extern SemaphoreHandle_t fpga_sequence_mutex;

// --- Protótipos das Tarefas ---
void task_self_test(void *params);
void task_distancia_display(void *params);
void task_alerta_proximidade(void *params);
void task_coordenador_controle(void *params);
void task_control_garra(void *params);
void task_control_braco(void *params);
void task_servo_manager(void *params);
void task_fpga_receiver(void *params);

#endif