//---------------------------------------------------------------------------------------------//
// aht10.c (Modificado)
//---------------------------------------------------------------------------------------------//
#include "aht10.h"
#include "FreeRTOS.h" // ✅ ADIÇÃO: Necessário para vTaskDelay
#include "task.h"     // ✅ ADIÇÃO: Necessário para vTaskDelay

// Comandos do AHT10
const uint8_t CMD_INIT[] = {0xE1, 0x08, 0x00};
const uint8_t CMD_MEASURE[] = {0xAC, 0x33, 0x00};

bool aht10_init(i2c_inst_t* i2c) {
    int ret = i2c_write_blocking(i2c, AHT10_ADDR, CMD_INIT, sizeof(CMD_INIT), false);
    if (ret < 0) return false;
    // ⚠️ ALTERAÇÃO: Usar delay não bloqueante do FreeRTOS
    vTaskDelay(pdMS_TO_TICKS(20)); 
    return true;
}

bool aht10_read_data(i2c_inst_t* i2c, aht10_data_t* data) {
    int ret = i2c_write_blocking(i2c, AHT10_ADDR, CMD_MEASURE, sizeof(CMD_MEASURE), false);
    if (ret < 0) return false;

    // ⚠️ ALTERAÇÃO: Usar delay não bloqueante do FreeRTOS
    vTaskDelay(pdMS_TO_TICKS(80));

    uint8_t buf[6];
    ret = i2c_read_blocking(i2c, AHT10_ADDR, buf, sizeof(buf), false);
    if (ret < 0) return false;

    if ((buf[0] & 0x88) != 0x08) {
        // Se o sensor estiver ocupado, podemos tentar de novo, mas por agora retornamos falha.
        // Em um sistema real, poderia haver uma lógica de nova tentativa.
        return false;
    }

    uint32_t raw_humidity = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
    data->humidity = ((float)raw_humidity / 1048576.0f) * 100.0f;

    uint32_t raw_temp = (((uint32_t)buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];
    data->temperature = (((float)raw_temp / 1048576.0f) * 200.0f) - 50.0f;

    return true;
}