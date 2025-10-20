#include "digitos_medios_utils.h"

#define LARGURA 16
#define ALTURA_BITMAP 8
#define PAGINAS_USADAS 3
#define PAGINA_DESTINO 5  // Agora: páginas 5, 6, 7

void exibir_digito_medio(ssd1306_t *oled, uint8_t x, const uint8_t *bitmap) {
    for (uint8_t col = 0; col < LARGURA; col++) {
        for (uint8_t page = 0; page < PAGINAS_USADAS; page++) {
            int index = col * ALTURA_BITMAP + (page + PAGINA_DESTINO);  // lê bytes 5,6,7
            int pos   = (page + PAGINA_DESTINO) * oled->width + (x + col);
            if (index < LARGURA * ALTURA_BITMAP && pos < oled->bufsize) {
                oled->ram_buffer[pos] = bitmap[index];
            }
        }
    }
}

