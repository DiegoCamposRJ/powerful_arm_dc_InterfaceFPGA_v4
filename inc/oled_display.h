// oled_display.h - ETAPA 1
#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Dimensões padrão do display
#define OLED_WIDTH     128
#define OLED_HEIGHT    64
#define OLED_PAGES     (OLED_HEIGHT / 8)
#define OLED_I2C_ADDR  0x3C

// Frequência padrão do barramento I2C (em Hz)
#define OLED_I2C_FREQ  400000

// Pinos padrão (você pode mudar no .c futuramente para tornar ainda mais flexível)
#define OLED_PIN_SDA   14
#define OLED_PIN_SCL   15

// Estrutura encapsulada para o display
typedef struct {
    uint8_t width;
    uint8_t height;
    uint8_t pages;
    uint8_t address;
    i2c_inst_t *i2c_port;
    uint8_t *ram_buffer;
    size_t bufsize;
    uint8_t port_buffer[2];
} ssd1306_t;

// Inicializa o display, I2C, buffer e configurações básicas
bool oled_init(ssd1306_t *oled);

void oled_clear(ssd1306_t *oled);
void oled_render(ssd1306_t *oled);


#endif // OLED_DISPLAY_H
