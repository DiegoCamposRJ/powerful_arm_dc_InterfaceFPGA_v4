//---------------------------------------------------------------------------------------------//
// braco_config.h
//---------------------------------------------------------------------------------------------//
#ifndef BRACO_CONFIG_H
#define BRACO_CONFIG_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

// --- Pinos ---
// Por enquanto, só precisamos do pino da base para o teste inicial
#define SERVO_BASE_PIN      16 // 17 base
#define SERVO_BRACO_PIN     16 // 19 braco 
#define SERVO_GARRA_PIN     16 // 18 garra
#define SERVO_ANGULO_PIN    16 // 16 angulo 

// --- LIMITES DE MOVIMENTO DOS SERVOS (em graus) ---
#define BASE_MIN_ANGLE      0.0f
#define BASE_MAX_ANGLE      180.0f

#define BRACO_MIN_ANGLE     70.0f  // (90 - 40/2)
#define BRACO_MAX_ANGLE     110.0f // (90 + 40/2) -> Total de 40° de movimento

#define GARRA_MIN_ANGLE     80.0f  // (90 - 25/2)
#define GARRA_MAX_ANGLE     105.0f // (90 + 25/2) -> Total de 25° de movimento

#define ANGULO_MIN_ANGLE    70.0f  // (90 - 40/2)
#define ANGULO_MAX_ANGLE    110.0f // (90 + 40/2) -> Total de 40° de movimento

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