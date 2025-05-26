#pragma once
#include "Adafruit_GFX.h"
#include "driver/gpio.h"


#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_SDA_IO        20
#define I2C_MASTER_SCL_IO        19
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_TIMEOUT_MS           1000

#define IS31FL3733_ADDR          0x50       // or maybe 0x50 0xA0

#define DISP_IICRST_IO           GPIO_NUM_18
#define DISP_SDB_SHDN_IO         GPIO_NUM_14


class IS31FL_Display : public Adafruit_GFX {
public:
    IS31FL_Display();

    void begin();                  // Init I2C + IS31FL3733
    void drawPixel(int16_t x, int16_t y, uint16_t color) override;
    void display();                // Flush framebuffer
    void clear();                  // Clear framebuffer

private:
    uint8_t framebuffer[12][16];   // Adjust for your matrix size
    void set_led_pwm(int x, int y, uint8_t brightness); // Low-level
    void update_display();         // Push buffer to chip
};