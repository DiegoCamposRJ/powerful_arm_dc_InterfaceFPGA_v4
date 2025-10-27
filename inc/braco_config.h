//---------------------------------------------------------------------------------------------//
// braco_config.h
//---------------------------------------------------------------------------------------------//
#ifndef BRACO_CONFIG_H
#define BRACO_CONFIG_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

// --- Pinos ---
// Por enquanto, só precisamos do pino da base para o teste inicial
#define SERVO_BASE_PIN      16 // Futuro servo da "Garra"
#define SERVO_BRACO_PIN     16 // Futuro servo do "Braço" ou "Altura"
#define SERVO_GARRA_PIN     16 // Futuro servo da "Garra"
#define SERVO_ANGULO_PIN    16 // <-- ADICIONE O PINO PARA O SERVO DE ÂNGULO

// --- Parâmetros de Funcionamento ---
#define PWM_FREQ 50

// --- Variáveis Globais de Hardware (compartilhadas) ---
// 'extern' diz ao compilador: "essa variável existe, mas é definida em outro arquivo .c"
// O tipo 'uint' corresponde ao tipo de retorno da função pwm_gpio_to_slice_num().
// --- Variáveis Globais de Hardware ---
extern uint slice_base;
extern uint slice_braco;
extern uint slice_garra;
extern uint slice_angulo; 

// --- Protótipos das Funções ---
uint16_t angle_to_duty(float angle);

#endif // BRACO_CONFIG_H