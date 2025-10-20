#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bitdoglab_pins.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"


#include "oled_context.h"
#include "oled_display.h"
#include "ssd1306_text.h"

#include "numeros_display.h" 

#include "aht10.h"
#define I2C_PORT_AHT10 i2c0
#define I2C_SDA_PIN_AHT10 0
#define I2C_SCL_PIN_AHT10 1


// Constante para converter o valor bruto do ADC (0-4095) em tensão (0-3.3V).
const float ADC_CONVERSION_FACTOR = 3.3f / (1 << 12);

// Variável estática para armazenar o número do "slice" do PWM usado pelo buzzer.
// 'static' a torna visível apenas dentro deste arquivo.
static uint slice_num;

extern ssd1306_t oled;

// Inicializa os pinos de GPIO e o hardware do ADC para todos os periféricos usados.
void init_perifericos() {
    // Configura os pinos dos LEDs RGB como saídas digitais.
    gpio_init(LED_RGB_R); gpio_set_dir(LED_RGB_R, GPIO_OUT);
    gpio_init(LED_RGB_G); gpio_set_dir(LED_RGB_G, GPIO_OUT);
    gpio_init(LED_RGB_B); gpio_set_dir(LED_RGB_B, GPIO_OUT);

    // Configura o pino do buzzer como saída digital (será reconfigurado para PWM depois).
    gpio_init(BUZZER_PIN); gpio_set_dir(BUZZER_PIN, GPIO_OUT);

    // Configura os pinos dos botões como entradas com resistores de pull-up internos.
    gpio_init(BTN_A_PIN); gpio_set_dir(BTN_A_PIN, GPIO_IN); gpio_pull_up(BTN_A_PIN);
    gpio_init(BTN_B_PIN); gpio_set_dir(BTN_B_PIN, GPIO_IN); gpio_pull_up(BTN_B_PIN);

    // Habilita o hardware do conversor analógico-digital.
    adc_init();
    // Associa os pinos físicos do joystick e microfone à função de ADC.
    
    adc_gpio_init(MIC_ADC_PIN);

    oled_init(&oled);

    i2c_init(I2C_PORT_AHT10, 100 * 1000); // AHT10 usa 100kHz
    gpio_set_function(I2C_SDA_PIN_AHT10, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN_AHT10, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN_AHT10);
    gpio_pull_up(I2C_SCL_PIN_AHT10);
}

// Acende e apaga sequencialmente os LEDs R, G e B para um teste visual.
void test_leds_rgb() {
    printf("  [TESTE] Testando LEDs RGB...\n");
    gpio_put(LED_RGB_R, 1); vTaskDelay(pdMS_TO_TICKS(250)); gpio_put(LED_RGB_R, 0);
    gpio_put(LED_RGB_G, 1); vTaskDelay(pdMS_TO_TICKS(250)); gpio_put(LED_RGB_G, 0);
    gpio_put(LED_RGB_B, 1); vTaskDelay(pdMS_TO_TICKS(250)); gpio_put(LED_RGB_B, 0);
    printf("  [OK] LEDs RGB testados.\n");
}

// Lê o estado lógico (pressionado/não pressionado) de cada botão e imprime no terminal.
void test_botoes() {
    printf("  [TESTE] Testando Botoes (0 = Pressionado)...\n");
    printf("    - Botao A: %d\n", gpio_get(BTN_A_PIN));
    printf("    - Botao B: %d\n", gpio_get(BTN_B_PIN));
    printf("  [OK] Botoes testados.\n");
}



// Lê o valor analógico do microfone e o converte para tensão.
void test_microfone() {
    printf("  [TESTE] Testando Microfone...\n");

    // Seleciona o canal 2 do ADC (conectado ao microfone).
    adc_select_input(2);
    uint16_t mic_raw = adc_read();
    printf("    - Leitura Microfone (ADC2): %.2f V\n", mic_raw * ADC_CONVERSION_FACTOR);
    
    printf("  [OK] Microfone testado.\n");
}

// função para configurar a frequência do buzzer
void buzzer_set_freq(int freq) {
    if (freq <= 0) {
        pwm_set_enabled(slice_num, false);
        return;
    }
    // Define um fator de divisão para o clock do sistema.
    // Usar um divisor maior ajuda a obter melhor resolução para frequências baixas.
    float divisor = 40.0f; 
    pwm_set_clkdiv(slice_num, divisor);

    // Calcula o valor de "wrap" para gerar a frequência desejada.
    // Fórmula: wrap = (clock_da_cpu / divisor) / frequencia_desejada - 1
    uint16_t wrap_value = (125000000 / divisor) / freq - 1;
    pwm_set_wrap(slice_num, wrap_value);

    // Define o "duty cycle" para 50% do ciclo total para um som mais audível.
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(BUZZER_PIN), wrap_value / 2);
}

// Configura o pino do buzzer para ser controlado pelo hardware de PWM.
void buzzer_pwm_init() {
    // Associa o pino do buzzer à função de PWM.
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    // Obtém o número do "slice" de PWM correspondente ao pino.
    slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);

    // ✅ ALTERAÇÃO: Define uma frequência inicial padrão (ex: 500 Hz)
    buzzer_set_freq(500);

    // Mantém o PWM desativado por padrão.
    pwm_set_enabled(slice_num, false);
}

// Ativa ou desativa o sinal PWM para o buzzer.
void buzzer_set_alarm(bool on) {
    pwm_set_enabled(slice_num, on);
}

// Realiza um teste audível no buzzer, ativando o PWM por 300ms.
void test_buzzer_pwm() { 
    printf("  [TESTE] Testando Buzzer com PWM...\n");
    buzzer_set_freq(1000); // Testa com 1000 Hz
    buzzer_set_alarm(true);
    vTaskDelay(pdMS_TO_TICKS(150));
    buzzer_set_freq(500);  // Testa com 500 Hz
    vTaskDelay(pdMS_TO_TICKS(150));
    buzzer_set_alarm(false);
    printf("  [OK] Buzzer testado.\n");
}

void test_display_oled() {
    printf("  [TESTE] Testando Display OLED...\n");

    oled_clear(&oled);
    
    // Usa a nova função para centralizar texto
    oled_centralizar_texto(&oled, "SISTEMA OK", 3);
    
    oled_render(&oled);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    oled_clear(&oled);
    oled_render(&oled);

    printf("  [OK] Display OLED testado.\n");
}


void test_sensor_aht10() {
    printf("  [TESTE] Testando Sensor AHT10...\n");

    // Inicializa o sensor
    if (!aht10_init(I2C_PORT_AHT10)) {
        printf("  [ERRO] Falha ao inicializar o sensor AHT10.\n");
        return; // Sai da função de teste se a inicialização falhar
    }

    // Cria uma estrutura para armazenar os dados
    aht10_data_t sensor_data;
    
    // Tenta ler os dados do sensor
    if (aht10_read_data(I2C_PORT_AHT10, &sensor_data)) {
        printf("    - Temperatura: %.2f C\n", sensor_data.temperature);
        printf("    - Umidade: %.2f %%\n", sensor_data.humidity);
        printf("  [OK] Sensor AHT10 testado.\n");
    } else {
        printf("  [ERRO] Falha ao ler os dados do sensor AHT10.\n");
    }
}