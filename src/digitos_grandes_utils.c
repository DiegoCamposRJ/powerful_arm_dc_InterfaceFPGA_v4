#include "digitos_grandes_utils.h"

#define DIGITO_LARGURA 25
#define DIGITO_ALTURA_TOTAL 8
#define DIGITO_ALTURA_USADA 6
#define DIGITOS_OFFSET_VERTICAL 2

// Exibe um dígito grande (25x6 páginas = 25x48 pixels)
void exibir_digito_grande(ssd1306_t *oled, uint8_t x, const uint8_t *bitmap) {
    for (uint8_t col = 0; col < DIGITO_LARGURA; col++) {
        for (uint8_t page = 0; page < DIGITO_ALTURA_USADA; page++) {
            int index = col * DIGITO_ALTURA_TOTAL + (page + DIGITOS_OFFSET_VERTICAL);
            int pos = (page + DIGITOS_OFFSET_VERTICAL) * oled->width + (x + col);
            if (pos < oled->bufsize) {
                oled->ram_buffer[pos] = bitmap[index];
            }
        }
    }
}

// Dois pontos (:) entre dígitos
void exibir_double_dot(ssd1306_t *oled, uint8_t x) {
    for (uint8_t page = 0; page < oled->pages; page++) {
        int pos = page * oled->width + x;
        if (pos < oled->bufsize) {
            oled->ram_buffer[pos] = (page == 2 || page == 4) ? 0x18 : 0x00;
        }
    }
}

// Ponto decimal para valores com vírgula
void exibir_ponto_decimal(ssd1306_t *oled, uint8_t x) {
    for (uint8_t page = 0; page < oled->pages; page++) {
        int pos = page * oled->width + x;
        if (pos < oled->bufsize) {
            oled->ram_buffer[pos] = (page == 7) ? 0x1C : 0x00;
        }
    }
}
