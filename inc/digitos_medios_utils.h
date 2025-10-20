#ifndef DIGITOS_MEDIOS_UTILS_H
#define DIGITOS_MEDIOS_UTILS_H

#include "oled_display.h"
#include <stdint.h>

// Declaração da função de exibição
void exibir_digito_medio(ssd1306_t *oled, uint8_t x, const uint8_t *bitmap);

#endif
