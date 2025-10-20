#ifndef NUMEROS_DISPLAY_H
#define NUMEROS_DISPLAY_H

#include "oled_display.h"
#include "numeros_grandes.h"
#include "numeros_medios.h"
#include "digitos_grandes_utils.h"
#include "digitos_medios_utils.h"

// ----------- DÍGITOS GRANDES ------------

void oled_exibir_digito_grande(ssd1306_t *oled, uint8_t digito, uint8_t x);
void oled_exibir_4digitos(ssd1306_t *oled, uint16_t valor);
void oled_exibir_caractere_grande(ssd1306_t *oled, char c, uint8_t x);

// ----------- DÍGITOS MÉDIOS ------------

void oled_exibir_digito_medio(ssd1306_t *oled, uint8_t digito, uint8_t x);
void oled_exibir_4digitos_medios(ssd1306_t *oled, uint16_t valor);
void oled_exibir_caractere_medio(ssd1306_t *oled, char c, uint8_t x);
void oled_exibir_valor_medio(ssd1306_t *oled, const char *texto);

// ----------- TEXTO AUXILIAR ------------

void oled_centralizar_texto(ssd1306_t *oled, const char *texto, uint8_t linha);

#endif

