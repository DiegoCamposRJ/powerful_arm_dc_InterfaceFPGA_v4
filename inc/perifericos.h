#ifndef PERIFERICOS_H
#define PERIFERICOS_H

#include "pico/stdlib.h"

void init_perifericos();
void test_leds_rgb();
// void test_buzzer();
void test_botoes();
void test_microfone();

// Funções para o buzzer com PWM
void buzzer_pwm_init();
void buzzer_set_alarm(bool on);
void test_buzzer_pwm();
void test_display_oled();
void test_sensor_aht10();
void buzzer_set_freq(int freq); 

#endif