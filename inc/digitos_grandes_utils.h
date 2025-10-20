#ifndef DIGITOS_GRANDES_UTILS_H
#define DIGITOS_GRANDES_UTILS_H

#include <stdint.h>
#include "oled_display.h"  // para usar ssd1306_t

void exibir_digito_grande(ssd1306_t *oled, uint8_t x, const uint8_t *bitmap);
void exibir_double_dot(ssd1306_t *oled, uint8_t x);
void exibir_ponto_decimal(ssd1306_t *oled, uint8_t x);

#endif

