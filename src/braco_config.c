//---------------------------------------------------------------------------------------------//
// braco_config.c
//---------------------------------------------------------------------------------------------//
/**
 * @file braco_config.c
 * @brief Implementação das funções de hardware para o braço robótico.
 */

#include "braco_config.h"
#include "hardware/clocks.h"

// --- Definição das Variáveis Globais ---
uint slice_base;

// --- Implementação das Funções ---

/**
 * @brief Converte um ângulo de entrada (0-180 graus) em um valor de duty cycle para o PWM,
 *        baseado na calibração específica de -90 a +90 graus (pulso de 1ms a 2ms).
 * @param angle O ângulo alvo para o servo, em uma escala de 0 a 180 graus.
 * @return uint16_t O valor do nível do canal PWM (duty cycle) correspondente.
 */
uint16_t angle_to_duty(float angle) {
    // Garante que o ângulo de entrada esteja dentro da faixa esperada (0 a 180)
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // --- CALIBRAÇÃO PRECISA: Mapeia 0-180 graus para 1000-2000 microssegundos ---
    // Base = 1000.0f (correspondente a -90° do servo, ou nosso 0°)
    // Amplitude (Range) = 2000.0f - 1000.0f = 1000.0f
    float pulse_width_us = 1000.0f + (angle / 180.0f) * 1000.0f;

    // --- CÁLCULO SINCRONIZADO COM O HARDWARE ---
    // Estes valores são baseados na configuração REAL do hardware em perifericos.c
    const float period_us = 20000.0f; // Período total para 50Hz
    const uint16_t wrap = 62499;     // Valor de wrap configurado no hardware

    // Calcula o duty cycle com base na proporção do pulso dentro do período total.
    return (uint16_t)((pulse_width_us / period_us) * (wrap + 1));
}