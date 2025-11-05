#include "fpga_comms.h"
#include "hardware/uart.h"
#include "bitdoglab_pins.h" // Para ter acesso ao UART_ID
#include <stdio.h>

// Converte um ângulo float (0-180) para um byte (0-255)
// O FPGA provavelmente espera um valor de 8 bits.
static uint8_t angle_to_byte(float angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    // Mapeia 0-180 para 0-255
    return (uint8_t)((angle / 180.0f) * 255.0f);
}

void fpga_send_sequence_step(int step_index, float base, float braco, float angulo, float garra) {
    
    printf("--> FPGA: Enviando Passo %d: [B:%.0f, R:%.0f, A:%.0f, G:%.0f]\n", 
           step_index, base, braco, angulo, garra);
    
    // Converte os ângulos para bytes
    uint8_t addr = (uint8_t)step_index;
    uint8_t b = angle_to_byte(base);
    uint8_t r = angle_to_byte(braco);
    uint8_t a = angle_to_byte(angulo);
    uint8_t g = angle_to_byte(garra);

    // Envia o pacote de 6 bytes
    uart_putc_raw(UART_ID, CMD_SAVE_POS);
    uart_putc_raw(UART_ID, addr);
    uart_putc_raw(UART_ID, b);
    uart_putc_raw(UART_ID, r);
    uart_putc_raw(UART_ID, a);
    uart_putc_raw(UART_ID, g);

    // Espera o buffer da UART esvaziar para garantir que tudo foi enviado
    uart_tx_wait_blocking(UART_ID);
}