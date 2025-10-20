#include "numeros_display.h"
#include "digitos_grandes_utils.h"
#include "digitos_medios_utils.h"
#include "numeros_medios.h"
#include "numeros_grandes.h"
#include <string.h>  // ← necessário para strlen()
#include <stdio.h>   // ← necessário para snprintf()
#include "ssd1306_text.h"  // ← contém a declaração de ssd1306_draw_utf8_multiline


// ----------- DÍGITOS GRANDES ------------

void oled_exibir_digito_grande(ssd1306_t *oled, uint8_t digito, uint8_t x) {
    if (digito > 9) return;
    exibir_digito_grande(oled, x, numeros_grandes[digito]);
}

void oled_exibir_4digitos(ssd1306_t *oled, uint16_t valor) {
    if (valor > 9999) return;

    uint8_t d1 = (valor / 1000) % 10;
    uint8_t d2 = (valor / 100) % 10;
    uint8_t d3 = (valor / 10) % 10;
    uint8_t d4 = valor % 10;

    const uint8_t base_x = 10;
    const uint8_t espaco = 26;

    oled_exibir_digito_grande(oled, d1, base_x);
    oled_exibir_digito_grande(oled, d2, base_x + espaco);
    oled_exibir_digito_grande(oled, d3, base_x + espaco * 2);
    oled_exibir_digito_grande(oled, d4, base_x + espaco * 3);
}

void oled_exibir_caractere_grande(ssd1306_t *oled, char c, uint8_t x) {
    const uint8_t *bitmap = NULL;

    if (c >= '0' && c <= '9') {
        bitmap = numeros_grandes[c - '0'];
    } else if (c == 'A') {
        bitmap = numeros_grandes[10];
    } else if (c == 'B') {
        bitmap = numeros_grandes[11];
    } else if (c == 'C') {
        bitmap = numeros_grandes[12];
    } else if (c == 'D') {
        bitmap = numeros_grandes[13];
    } else if (c == 'E') {
        bitmap = numeros_grandes[14];
    } else if (c == 'F') {
        bitmap = numeros_grandes[15];
    } else if (c == '*') {
        bitmap = numeros_grandes[16];
    } else if (c == '#') {
        bitmap = numeros_grandes[17];
    }

    if (bitmap) {
        exibir_digito_grande(oled, x, bitmap);
    }
}

// ----------- DÍGITOS MÉDIOS ------------

void oled_exibir_digito_medio(ssd1306_t *oled, uint8_t digito, uint8_t x) {
    if (digito > 9) return;
    exibir_digito_medio(oled, x, numeros_medios[digito]);
}

void oled_exibir_4digitos_medios(ssd1306_t *oled, uint16_t valor) {
    if (valor > 9999) return;

    uint8_t d1 = (valor / 1000) % 10;
    uint8_t d2 = (valor / 100) % 10;
    uint8_t d3 = (valor / 10) % 10;
    uint8_t d4 = valor % 10;

    const uint8_t base_x = 10;
    const uint8_t espaco = 18;

    oled_exibir_digito_medio(oled, d1, base_x);
    oled_exibir_digito_medio(oled, d2, base_x + espaco);
    oled_exibir_digito_medio(oled, d3, base_x + espaco * 2);
    oled_exibir_digito_medio(oled, d4, base_x + espaco * 3);
}

void oled_exibir_caractere_medio(ssd1306_t *oled, char c, uint8_t x) {
    const uint8_t *bitmap = NULL;

    if (c >= '0' && c <= '9') bitmap = numeros_medios[c - '0'];
    else if (c == '.') bitmap = simbolos_medios[2];
    else if (c == '+') bitmap = simbolos_medios[0];
    else if (c == '-') bitmap = simbolos_medios[1];
    else if (c == ' ') bitmap = simbolos_medios[3];

    if (bitmap) exibir_digito_medio(oled, x, bitmap);
}

// Exibe valor flutuante com fonte média
void oled_exibir_valor_medio(ssd1306_t *oled, const char *valor_str) {
    // Debug opcional
    printf("[OLED DEBUG] String recebida: \"%s\"\n", valor_str);

    const uint8_t largura = 16;  // Largura da fonte média
    uint8_t x = 0;

    for (uint8_t i = 0; valor_str[i] != '\0' && i < 8; i++) {
        oled_exibir_caractere_medio(oled, valor_str[i], x);
        x += largura;
    }

    oled_render(oled);
}

void oled_centralizar_texto(ssd1306_t *oled, const char *texto, uint8_t linha) {
    int len = strlen(texto);
    int x = (oled->width - len * 8) / 2;
    ssd1306_draw_utf8_multiline(oled->ram_buffer, x, linha * 8, texto, oled->width, oled->height);
}
