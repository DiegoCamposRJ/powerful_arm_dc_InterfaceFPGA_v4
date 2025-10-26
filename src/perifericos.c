//---------------------------------------------------------------------------------------------//
// perifericos.c
//---------------------------------------------------------------------------------------------//
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bitdoglab_pins.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"

#include "oled_context.h"
#include "oled_display.h"
#include "ssd1306_text.h"
#include "numeros_display.h" 
#include "vl53l0x.h"
#include "tarefas.h"
#include "braco_config.h"

const float ADC_CONVERSION_FACTOR = 3.3f / (1 << 12);
static uint slice_num_buzzer;
extern ssd1306_t oled;

// --- CORREÇÃO 2: ORDEM DAS FUNÇÕES DO BUZZER ---
// A definição da função vem ANTES de ser chamada.
void buzzer_set_freq(int freq) {
    if (freq <= 0) {
        pwm_set_enabled(slice_num_buzzer, false);
        return;
    }
    float divisor = 40.0f; 
    pwm_set_clkdiv(slice_num_buzzer, divisor);
    uint32_t clock_hz = clock_get_hz(clk_sys);
    uint16_t wrap_value = (uint16_t)((clock_hz / divisor) / freq - 1);
    pwm_set_wrap(slice_num_buzzer, wrap_value);
    pwm_set_chan_level(slice_num_buzzer, pwm_gpio_to_channel(BUZZER_PIN), wrap_value / 2);
}

void init_perifericos() {
    gpio_init(LED_RGB_R); gpio_set_dir(LED_RGB_R, GPIO_OUT);
    gpio_init(LED_RGB_G); gpio_set_dir(LED_RGB_G, GPIO_OUT);
    gpio_init(LED_RGB_B); gpio_set_dir(LED_RGB_B, GPIO_OUT);

    gpio_init(BTN_A_PIN); gpio_set_dir(BTN_A_PIN, GPIO_IN); gpio_pull_up(BTN_A_PIN);
    gpio_init(BTN_B_PIN); gpio_set_dir(BTN_B_PIN, GPIO_IN); gpio_pull_up(BTN_B_PIN);

     // --- INICIALIZAÇÃO DO JOYSTICK ---
    // Botão do Joystick (SW)
    gpio_init(BTN_SW_PIN);
    gpio_set_dir(BTN_SW_PIN, GPIO_IN);
    gpio_pull_up(BTN_SW_PIN);

    // ADC para Microfone e Eixos do Joystick
    adc_init();
    adc_gpio_init(MIC_ADC_PIN);
    adc_gpio_init(JOY_Y_PIN); // ADC0
    adc_gpio_init(JOY_X_PIN); // ADC1

    oled_init(&oled);

    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);

    // --- CORREÇÃO 1: CÁLCULO DO PWM DO SERVO ---
    gpio_set_function(SERVO_BASE_PIN, GPIO_FUNC_PWM);
    slice_base = pwm_gpio_to_slice_num(SERVO_BASE_PIN);

    pwm_config config = pwm_get_default_config();
    // Usamos um divisor maior para que o valor de 'wrap' seja menor que 65535.
    // 125MHz / 40 = 3.125MHz.
    float div = 40.0f; 
    pwm_config_set_clkdiv(&config, div);
    // Wrap = 3.125MHz / 50Hz = 62500. Este valor cabe em um uint16_t.
    uint16_t wrap = 62500 - 1; 
    pwm_config_set_wrap(&config, wrap);
    
    pwm_init(slice_base, &config, true);
    
    pwm_set_chan_level(slice_base, pwm_gpio_to_channel(SERVO_BASE_PIN), angle_to_duty(90.0f));
    sleep_ms(500);
    pwm_set_chan_level(slice_base, pwm_gpio_to_channel(SERVO_BASE_PIN), 0);
}

void test_leds_rgb() {
    printf("  [TESTE] Testando LEDs RGB...\n");
    gpio_put(LED_RGB_R, 1); vTaskDelay(pdMS_TO_TICKS(250)); gpio_put(LED_RGB_R, 0);
    gpio_put(LED_RGB_G, 1); vTaskDelay(pdMS_TO_TICKS(250)); gpio_put(LED_RGB_G, 0);
    gpio_put(LED_RGB_B, 1); vTaskDelay(pdMS_TO_TICKS(250)); gpio_put(LED_RGB_B, 0);
    printf("  [OK] LEDs RGB testados.\n");
}

void test_botoes() {
    printf("  [TESTE] Testando Botoes (0 = Pressionado)...\n");
    printf("    - Botao A: %d\n", gpio_get(BTN_A_PIN));
    printf("    - Botao B: %d\n", gpio_get(BTN_B_PIN));
    printf("  [OK] Botoes testados.\n");
}

void test_microfone() {
    printf("  [TESTE] Testando Microfone...\n");
    adc_select_input(2);
    uint16_t mic_raw = adc_read();
    printf("    - Leitura Microfone (ADC2): %.2f V\n", mic_raw * ADC_CONVERSION_FACTOR);
    printf("  [OK] Microfone testado.\n");
}

void buzzer_pwm_init() {
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    slice_num_buzzer = pwm_gpio_to_slice_num(BUZZER_PIN);
    buzzer_set_freq(500); // Agora esta chamada é válida
    pwm_set_enabled(slice_num_buzzer, false);
}

void buzzer_set_alarm(bool on) {
    pwm_set_enabled(slice_num_buzzer, on);
}

void test_buzzer_pwm() { 
    printf("  [TESTE] Testando Buzzer com PWM...\n");
    buzzer_set_freq(1000);
    buzzer_set_alarm(true);
    vTaskDelay(pdMS_TO_TICKS(150));
    buzzer_set_freq(500);
    vTaskDelay(pdMS_TO_TICKS(150));
    buzzer_set_alarm(false);
    printf("  [OK] Buzzer testado.\n");
}

void test_display_oled() {
    printf("  [TESTE] Testando Display OLED...\n");
    oled_clear(&oled);
    oled_centralizar_texto(&oled, "SISTEMA OK", 3);
    oled_render(&oled);
    vTaskDelay(pdMS_TO_TICKS(2000));
    oled_clear(&oled);
    oled_render(&oled);
    printf("  [OK] Display OLED testado.\n");
}

void test_sensor_vl53l0x() {
    printf("  [TESTE] Testando Sensor VL53L0X...\n");
    if (!vl53l0x_init(&sensor_dev, i2c0)) {
        printf("  [ERRO] Falha ao inicializar o sensor VL53L0X.\n");
    } else {
        printf("  [OK] Sensor VL53L0X inicializado com sucesso.\n");
    }
}

void test_servo_base() {
    printf("  [TESTE] Testando Servo da Base...\n");
    pwm_set_chan_level(slice_base, pwm_gpio_to_channel(SERVO_BASE_PIN), angle_to_duty(0.0f));
    vTaskDelay(pdMS_TO_TICKS(1000));
    pwm_set_chan_level(slice_base, pwm_gpio_to_channel(SERVO_BASE_PIN), angle_to_duty(90.0f));
    vTaskDelay(pdMS_TO_TICKS(1000));
    pwm_set_chan_level(slice_base, pwm_gpio_to_channel(SERVO_BASE_PIN), angle_to_duty(180.0f));
    vTaskDelay(pdMS_TO_TICKS(1000));
    pwm_set_chan_level(slice_base, pwm_gpio_to_channel(SERVO_BASE_PIN), angle_to_duty(90.0f));
    vTaskDelay(pdMS_TO_TICKS(500));
    pwm_set_chan_level(slice_base, pwm_gpio_to_channel(SERVO_BASE_PIN), 0);
    printf("  [OK] Servo da Base testado.\n");
}

// Nova função para testar o joystick
void test_joystick() {
    printf("  [TESTE] Testando Joystick...\n");

    // Seleciona o canal ADC 0 (conectado ao pino GP26 - Eixo Y)
    adc_select_input(0);
    uint16_t joy_y_raw = adc_read();

    // Seleciona o canal ADC 1 (conectado ao pino GP27 - Eixo X)
    adc_select_input(1);
    uint16_t joy_x_raw = adc_read();

    // Lê o estado do botão do joystick (pino digital)
    // Usamos '!' porque o pull-up interno faz o pino ler '1' quando solto e '0' quando pressionado.
    bool btn_sw_pressed = !gpio_get(BTN_SW_PIN);

    printf("    - Leitura Eixo Y (ADC0): %d\n", joy_y_raw);
    printf("    - Leitura Eixo X (ADC1): %d\n", joy_x_raw);
    printf("    - Botao SW Pressionado: %s\n", btn_sw_pressed ? "Sim" : "Nao");
    
    printf("  [OK] Joystick testado.\n");
}