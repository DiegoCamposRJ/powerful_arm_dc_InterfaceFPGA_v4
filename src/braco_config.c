//---------------------------------------------------------------------------------------------//
// braco_config.c
//---------------------------------------------------------------------------------------------//
/**
 * @file braco_config.c
 * @brief Implementação das funções de hardware para o braço robótico.
 */

#include "braco_config.h"
#include "hardware/clocks.h" // Necessário para obter a frequência do clock do sistema

// --- Definição das Variáveis Globais ---
// Aqui é onde a variável 'slice_base', declarada como 'extern' no arquivo .h,
// é realmente criada na memória. O tipo 'uint' corresponde à declaração.
uint slice_base;

// --- Implementação das Funções ---

/**
 * @brief Converte um ângulo desejado (0-180 graus) em um valor de duty cycle para o PWM.
 * @param angle O ângulo alvo para o servo, em graus.
 * @return uint16_t O valor do nível do canal PWM (duty cycle) correspondente.
 */
uint16_t angle_to_duty(float angle) {
    // Garante que o ângulo esteja dentro dos limites seguros (0 a 180)
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // Mapeia o ângulo para a largura de pulso em microssegundos (geralmente 500µs a 2500µs)
    float pulse_width_us = 500.0f + (angle / 180.0f) * 2000.0f;

    // O período total em microssegundos para 50Hz é 1 / 50 = 0.02s = 20000µs.
    float period_us = 1000000.0f / PWM_FREQ;

    // O valor de "wrap" foi configurado na função init_perifericos para 62499.
    uint32_t wrap = 62499;

    // A fórmula é a proporção da largura do pulso em relação ao período total,
    // multiplicada pelo valor máximo do contador (wrap + 1).
    return (uint16_t)((pulse_width_us / period_us) * (wrap + 1));
}