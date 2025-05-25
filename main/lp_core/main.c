/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "ulp_lp_core_i2c.h"
#include "ulp_lp_core_utils.h"
#include "HDC1080.h"

#define LP_I2C_TRANS_TIMEOUT_CYCLES 5000
#define LP_I2C_TRANS_WAIT_FOREVER   -1

#define LUX_THRESHOLD_LOW           5
#define LUX_THRESHOLD_HIGH          1000

//static uint32_t sensor_on = 0;
//static uint32_t res_update_done = 0;
//volatile uint32_t lux = 0;

volatile uint32_t raw_temperature;
volatile uint32_t raw_humidity;
volatile uint32_t readTempReady = false;
volatile uint32_t temp_comms_ok = 0;
volatile uint32_t temp_comms_fail = 0;

/*
esp_err_t lp_core_i2c_master_write_to_device(i2c_port_t lp_i2c_num, uint16_t device_addr,
                                             const uint8_t *data_wr, size_t size,
                                             int32_t ticks_to_wait)
*/

static esp_err_t hdc1080_read_register(uint8_t reg, uint8_t *data, size_t len)
{
    esp_err_t err;

    // Step 1: Write register pointer
    err = lp_core_i2c_master_write_to_device(HDC1080_I2C_PORT, HDC1080_ADDR,
                                     &reg, 1,
                                     HDC1080_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
   //     ESP_LOGE(TAG, "Write reg pointer 0x%02X failed: %s", reg, esp_err_to_name(err));
        return err;
    }

    ulp_lp_core_delay_us(HDC1080_CONVERSION_DELAY_MS * 1000);

    // Delay only if temperature or humidity measurement is triggered
  //  if (reg == HDC1080_TEMP_REG || reg == HDC1080_HUMIDITY_REG) {
  ///      vTaskDelay(pdMS_TO_TICKS(HDC1080_CONVERSION_DELAY_MS));
   // }

    // Step 2: Read data
    err = lp_core_i2c_master_read_from_device(HDC1080_I2C_PORT, HDC1080_ADDR,
                                      data, len,
                                      HDC1080_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
      //  ESP_LOGE(TAG, "Read reg 0x%02X failed: %s", reg, esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static esp_err_t hdc1080_write_register(uint8_t reg, uint16_t value)
{
    uint8_t payload[3] = {
        reg,
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF)
    };

    esp_err_t err = lp_core_i2c_master_write_to_device(HDC1080_I2C_PORT, HDC1080_ADDR,
                                               payload, sizeof(payload),
                                               HDC1080_I2C_TIMEOUT_MS);
  //  if (err != ESP_OK) {
   //     ESP_LOGE(TAG, "Write reg 0x%02X failed: %s", reg, esp_err_to_name(err));
   // }

    return err;
}
/*
esp_err_t hdc1080_init_i2c(gpio_num_t sda, gpio_num_t scl, uint32_t freq_hz)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = freq_hz,
    };

    ESP_ERROR_CHECK(i2c_param_config(HDC1080_I2C_PORT, &conf));
    return i2c_driver_install(HDC1080_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}
*/
esp_err_t hdc1080_read_temperature_raw(uint16_t *raw_temp)
{
    uint8_t data[2];
    esp_err_t err = hdc1080_read_register(HDC1080_TEMP_REG, data, 2);
    if (err != ESP_OK) return err;

    *raw_temp = ((uint16_t)data[0] << 8) | data[1];
    return ESP_OK;
}

esp_err_t hdc1080_read_humidity_raw(uint16_t *raw_humid)
{
    uint8_t data[2];
    esp_err_t err = hdc1080_read_register(HDC1080_HUMIDITY_REG, data, 2);
    if (err != ESP_OK) return err;

    *raw_humid = ((uint16_t)data[0] << 8) | data[1];
    return ESP_OK;
}

esp_err_t hdc1080_read_temperature_humidity(volatile float *temperature_c, volatile float *humidity_percent)
{
    uint16_t raw_temp, raw_humid;
    esp_err_t err;

    err = hdc1080_read_temperature_raw(&raw_temp);
    if (err != ESP_OK) return err;

    err = hdc1080_read_humidity_raw(&raw_humid);
    if (err != ESP_OK) return err;

    *temperature_c = ((float)raw_temp / 65536.0f) * 165.0f - 40.0f;
    *humidity_percent = ((float)raw_humid / 65536.0f) * 100.0f;

    return ESP_OK;
}

esp_err_t hdc1080_read_config(uint16_t *config)
{
    uint8_t data[2];
    esp_err_t err = hdc1080_read_register(HDC1080_CONFIG_REG, data, 2);
    if (err != ESP_OK) return err;

    *config = ((uint16_t)data[0] << 8) | data[1];
    return ESP_OK;
}

esp_err_t hdc1080_write_config(uint16_t config)
{
    return hdc1080_write_register(HDC1080_CONFIG_REG, config);
}

esp_err_t hdc1080_read_manufacturer_id(uint16_t *id)
{
    uint8_t data[2];
    esp_err_t err = hdc1080_read_register(HDC1080_MANUFACTURER_ID, data, 2);
    if (err != ESP_OK) return err;

    *id = ((uint16_t)data[0] << 8) | data[1];
    return ESP_OK;
}

esp_err_t hdc1080_read_device_id(uint16_t *id)
{
    uint8_t data[2];
    esp_err_t err = hdc1080_read_register(HDC1080_DEVICE_ID, data, 2);
    if (err != ESP_OK) return err;

    *id = ((uint16_t)data[0] << 8) | data[1];
    return ESP_OK;
}



int main (void)
{
    uint8_t data_wr     = 0;
    esp_err_t ret       = ESP_OK;
    uint16_t tempRead, humidRead, configRead;

    raw_temperature     = 0;
    raw_humidity        = 0;
    readTempReady       = false;

    hdc1080_write_config(0x0000);

    while (1) 
    {
        if (1 != readTempReady)
        {
            if (ESP_OK == hdc1080_read_temperature_raw(&tempRead))
            {
                raw_temperature = (uint32_t)tempRead;
                temp_comms_ok++;
            }
            else
            {
                temp_comms_fail++;
            }

            if (ESP_OK == hdc1080_read_humidity_raw(&humidRead))
            {
                raw_humidity = (uint32_t) humidRead;
                temp_comms_ok++;
            }
            else
            {
                temp_comms_fail++;
            }

            if (ESP_OK == hdc1080_read_config(&configRead))
            {
                if (configRead != 0x0000)
                {
                    hdc1080_write_config(0x0000);
                    temp_comms_fail++;
                }
                else
                {
                    temp_comms_ok++;
                }
            }
            else
            {
                hdc1080_write_config(0x0000);
                temp_comms_fail++;
            }
            readTempReady = 1;
        }        
    }
    return 0;
}
