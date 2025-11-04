#ifndef BRACO_CONFIG_H
#define BRACO_CONFIG_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

// --- Pinos ---
#define SERVO_BASE_PIN      17
#define SERVO_BRACO_PIN     19
#define SERVO_GARRA_PIN     18
#define SERVO_ANGULO_PIN    16

// ===================================================================================
// --- INTERRUPTOR DE CONTROLE DE LIMITES ---
//
// Para usar os limites de movimento restritos (ex: Garra 25°, Braço 40°),
// mantenha a linha abaixo DESCOMENTADA.
//
// Para usar o movimento completo de 0° a 180° em TODOS os servos,
// comente a linha abaixo (adicione // no início).
//
// #define USE_LIMITED_RANGE
//
// ===================================================================================


#ifdef USE_LIMITED_RANGE
    // --- MODO COM LIMITES (SEGURO) ---
    #define BASE_MIN_ANGLE      0.0f
    #define BASE_MAX_ANGLE      180.0f

    #define BRACO_MIN_ANGLE     70.0f  // (90 - 40/2)
    #define BRACO_MAX_ANGLE     110.0f // (90 + 40/2) -> Total de 40° de movimento

    #define GARRA_MIN_ANGLE     80.0f  // (90 - 25/2)
    #define GARRA_MAX_ANGLE     105.0f // (90 + 25/2) -> Total de 25° de movimento

    #define ANGULO_MIN_ANGLE    70.0f  // (90 - 40/2)
    #define ANGULO_MAX_ANGLE    110.0f // (90 + 40/2) -> Total de 40° de movimento
#else
    // --- MODO SEM LIMITES (RANGE COMPLETO 0-180°) ---
    #define BASE_MIN_ANGLE      0.0f
    #define BASE_MAX_ANGLE      180.0f

    #define BRACO_MIN_ANGLE     0.0f
    #define BRACO_MAX_ANGLE     180.0f

    #define GARRA_MIN_ANGLE     0.0f
    #define GARRA_MAX_ANGLE     180.0f

    #define ANGULO_MIN_ANGLE    0.0f
    #define ANGULO_MAX_ANGLE    180.0f
#endif


// --- Parâmetros de Funcionamento ---
#define PWM_FREQ 100

// --- Variáveis Globais de Hardware ---
extern uint slice_base;
extern uint slice_braco;
extern uint slice_garra;
extern uint slice_angulo;

// --- Protótipos das Funções ---
uint16_t angle_to_duty(float angle);

#endif // BRACO_CONFIG_H