#ifndef SSD1306_TEXT_H
#define SSD1306_TEXT_H

#include <stdint.h>

void ssd1306_draw_char(uint8_t *ssd, int16_t x, int16_t y, uint8_t character, uint8_t width, uint8_t height);
void ssd1306_draw_string(uint8_t *ssd, int16_t x, int16_t y, char *string, uint8_t width, uint8_t height);
void ssd1306_draw_utf8_multiline(uint8_t *ssd, int16_t x, int16_t y, const char *utf8_string, uint8_t width, uint8_t height);

#endif // SSD1306_TEXT_H
