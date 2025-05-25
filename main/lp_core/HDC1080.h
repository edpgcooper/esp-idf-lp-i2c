#ifndef HDC1080_H
#define HDC1080_H

#include <string.h>
#include <stdbool.h>
#include "esp_err.h"


#ifdef __cplusplus
extern "C" {
#endif

// I2C address of the HDC1080 sensor
#define HDC1080_ADDR             0x40

// Register addresses
#define HDC1080_TEMP_REG         0x00
#define HDC1080_HUMIDITY_REG     0x01
#define HDC1080_CONFIG_REG       0x02
#define HDC1080_MANUFACTURER_ID  0xFE
#define HDC1080_DEVICE_ID        0xFF

// I2C configuration macros (user can override)
#ifndef HDC1080_I2C_PORT
#define HDC1080_I2C_PORT         LP_I2C_NUM_0
#endif

#ifndef HDC1080_I2C_TIMEOUT_MS
#define HDC1080_I2C_TIMEOUT_MS   1000
#endif

#define HDC1080_CONVERSION_DELAY_MS 25


#ifdef __cplusplus
}
#endif

#endif // HDC1080_H