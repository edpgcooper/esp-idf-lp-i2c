#include "is31fl_display.h"
#include "is31fl3733.h"
#include "driver/gpio.h"
#include <stdio.h> // For memset
#include <string.h> // For memset
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

#define WIDTH 	16  // Width of your display
#define HEIGHT 	12 // Height of your display


static const char *TAG = "IS31";

// I2C master init
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                                       I2C_MASTER_RX_BUF_DISABLE,
                                       I2C_MASTER_TX_BUF_DISABLE, 0));
    return ESP_OK;
}


void configure_gpio_outputs(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_18) | (1ULL << GPIO_NUM_14),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}


IS31FL_Display::IS31FL_Display() : Adafruit_GFX(16, 12)
{
    memset(framebuffer, 0, sizeof(framebuffer));
}

void IS31FL_Display::begin() 
{
    configure_gpio_outputs();
    gpio_set_level(DISP_SDB_SHDN_IO, 1);        // disable SHDN
    gpio_set_level(DISP_IICRST_IO, 1);        //RESET
    vTaskDelay(pdMS_TO_TICKS(100));  // Hold reset 
    gpio_set_level(DISP_IICRST_IO, 0);        //RESET

    ESP_LOGI(TAG, "Initializing I2C master...");
    ESP_ERROR_CHECK(i2c_master_init());

    printf("Master Init Done\n");

    // Init IS31FL3733 here (I2C config, shutdown disable, etc.)
}

void IS31FL_Display::drawPixel(int16_t x, int16_t y, uint16_t color) 
{
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
    framebuffer[y][x] = (color ? 255 : 0);  // You could support grayscale
}

void IS31FL_Display::clear() 
{
    memset(framebuffer, 0, sizeof(framebuffer));
}

void IS31FL_Display::display() 
{
    update_display();
}

void IS31FL_Display::set_led_pwm(int x, int y, uint8_t brightness) 
{
    
    // Map (x, y) to linear channel, send via I2C to IS31FL3733
}


static esp_err_t is31fl_write_reg(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IS31FL3733_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}


// Write multiple bytes starting at a register
esp_err_t is31fl_write_bytes(uint8_t start_reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IS31FL3733_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, start_reg, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t is31fl_read_reg(uint8_t reg, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IS31FL3733_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);  // Repeated start
    i2c_master_write_byte(cmd, (IS31FL3733_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Select a page in the device
static esp_err_t is31fl_set_page(uint8_t page)
{
    uint8_t readCheck = 0;
    is31fl_write_reg(0xFE, 0xC5);        //Unlock
  //  is31fl_read_reg(0xFE, &readCheck);
 //   printf("Security byte %x\n", readCheck);
    return is31fl_write_reg(0xFD, page);        //IS31FL3733_PSR
}

// Enable the chip (normal mode)
esp_err_t is31fl_enable_chip(void)
{
    ESP_ERROR_CHECK(is31fl_set_page(3));
    return is31fl_write_reg(0x00, 0x01);
    //return is31fl_write_bytes(0x00, (uint8_t[]){0x01}, 1);  // MODE register = normal, SSD is set, Open/short is enabled
}

// Enable osd scan
esp_err_t is31fl_enable_osd(void)
{
    ESP_ERROR_CHECK(is31fl_set_page(3));
    return is31fl_write_reg(0x00, 0x05);
    //return is31fl_write_bytes(0x00, (uint8_t[]){0x05}, 1);  // MODE register = normal, SSD is set, Open/short is enabled
}

// Enable the chip (normal mode)
esp_err_t is31fl_enable_current(uint8_t current)
{
    ESP_ERROR_CHECK(is31fl_set_page(3));
    return is31fl_write_reg(0x01, current);

   // return is31fl_write_bytes(0x01, (uint8_t[]){current}, 1);  // set current
}

// Enable all LEDs (LED Control Register Page 0)
esp_err_t is31fl_enable_all_leds(void)
{
    ESP_ERROR_CHECK(is31fl_set_page(0));
    uint8_t led_on[24];  // 192 bits = 24 bytes
    memset(led_on, 0xFF, sizeof(led_on));  // All LEDs ON
    return is31fl_write_bytes(0x00, led_on, sizeof(led_on));
}

esp_err_t is31fl_read_pg0_reg(uint8_t readReg, uint8_t *readData)
{
    esp_err_t retVal;
    ESP_ERROR_CHECK(is31fl_set_page(0));
    retVal = is31fl_read_reg(readReg, readData) ;
   // printf(" State %x %u\n", readReg, readData);
    return retVal;
}


// Set all PWM values (Page 1)
esp_err_t is31fl_set_all_pwm(uint8_t value)
{
    ESP_ERROR_CHECK(is31fl_set_page(1));
    uint8_t pwm_data[192];           // was 192
    memset(pwm_data, value, sizeof(pwm_data));
    return is31fl_write_bytes(0x00, pwm_data, sizeof(pwm_data));
}


void check_is31fl_status(void)
{
    uint8_t val;

    // Set to Page 0
    if (is31fl_set_page(0) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select page 0");
        return;
    }

    // Read MODE register
    if (is31fl_read_reg(0x00, &val) == ESP_OK)
        ESP_LOGE(TAG, "MODE register: 0x%02X (%s)", val, (val == 0x01) ? "Normal mode" : "Standby");

    // Read STATUS register
    if (is31fl_read_reg(0x06, &val) == ESP_OK)
        ESP_LOGE(TAG, "STATUS register: 0x%02X", val);
}

// Enable the device (mode register)
static esp_err_t is31fl_my_enable(void)
{
    // Page 0, Mode register 0x00, set to 0x01 to enable
    esp_err_t ret = is31fl_write_reg(IS31FL3733_PSWL, IS31FL3733_PSWL_ENABLE);
    if (ret != ESP_OK) return ret;

    return is31fl_write_reg(IS31FL3733_PSR, 0x01); // Normal operation mode
}


// Enable the device (mode register)
static esp_err_t is31fl_enable(void)
{
    // Page 0, Mode register 0x00, set to 0x01 to enable
    esp_err_t ret = is31fl_set_page(0);
    if (ret != ESP_OK) return ret;

    return is31fl_write_reg(0x00, 0x01); // Normal operation mode
}



void openShortDetection(void)
{
    esp_err_t ret;
    uint8_t newData = 2;
    uint8_t reading;

    printf(" *** Performing Open / Short detection ***\n");

    ESP_LOGI(TAG, "Enabling chip...");
    ret = is31fl_enable_chip();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to enable chip");
        return;
    }

    ESP_LOGI(TAG, "Setting current to 0x01 for open short detection..");
    ret = is31fl_enable_current(0x01);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to enable current");
        return;
    }

    ESP_LOGI(TAG, "Enabling all LEDs...");
    ret = is31fl_enable_all_leds();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to enable all LEDs");
        return;
    }

    ESP_LOGI(TAG, "Enabling OSD...");
    ret = is31fl_enable_osd();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to enable OSD");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(50));  // Delay for the open/short detection to be performed.

    uint32_t totalSetBits = 0;
    printf("\nOpen circuits  ");

    for (reading = 0x18; reading <= 0x2f; reading++)
    {
        is31fl_read_pg0_reg(reading, &newData);
        //printf(" %u", newData);
        // Count set bits in this register
        for (int bit = 7; bit >= 0; bit--)
        {
            uint8_t bitValue = (newData >> bit) & 1;
            printf("%u", bitValue);
            totalSetBits += bitValue;
        }
    }
    printf("\nTotal count %lu\n", totalSetBits);


    totalSetBits = 0;
    printf("\nShort circuits ");

    for (reading = 0x30; reading <= 0x47; reading++)
    {
        is31fl_read_pg0_reg(reading, &newData);
       // printf(" %u", newData);
        for (int bit = 7; bit >= 0; bit--)
        {
            uint8_t bitValue = (newData >> bit) & 1;
            printf("%u", bitValue);
            totalSetBits += bitValue;
        }
    }
    printf("\nTotal count %lu\n", totalSetBits);
   // printf("\n");

    ESP_LOGI(TAG, "Setting all LEDs to full brightness...");
    ret = is31fl_set_all_pwm(100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LEDs");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(50)); 

    
}


void IS31FL_Display::update_display() 
{
    esp_err_t ret;
    uint16_t bytesToSend = 12*16;

    bytesToSend = 31;

    configure_gpio_outputs();

    gpio_set_level(DISP_SDB_SHDN_IO, 1);        // disable SHDN
   // gpio_set_level(DISP_IICRST_IO, 1);        //RESET
    //vTaskDelay(pdMS_TO_TICKS(100));  // Hold reset 
    gpio_set_level(DISP_IICRST_IO, 0);        //RESET

   // ESP_LOGI(TAG, "Initializing I2C master...");
   // ESP_ERROR_CHECK(i2c_master_init());
    

    openShortDetection();

    for (int y = 0; y < 12; y++) 
    {
        for (int x = 0; x < 16; x++) 
        {
            set_led_pwm(x, y, framebuffer[y][x]);
        }
    }

    
    ESP_LOGI(TAG, "Enabling chip...");
    ret = is31fl_enable_chip();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable chip");
        return;
    }

    ESP_LOGI(TAG, "Enabling current..");
    ret = is31fl_enable_current(100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable current");
        return;
    }

    ESP_LOGI(TAG, "Enabling all LEDs...");
    ret = is31fl_enable_all_leds();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable all LEDs");
        return;
    }


    ESP_LOGI(TAG, "Setting all LEDs to on ...");
    ret = is31fl_set_all_pwm(24);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set LEDs");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(50));  // 



/*
    
    ESP_LOGI(TAG, "Enabling IS31FL3733...");
    ret = is31fl_my_enable();       //IS31FL3733_PSWL, IS31FL3733_PSWL_ENABLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable IS31FL3733");
        return;
    }

    // Select page 1 where LED PWM registers are located
    ret = is31fl_set_page(1);       //IS31FL3733_PSR, 0x01
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select page 1");
        return;
    }

    // Set PWM of pixel 0 to 128 (50% brightness)
    ret = is31fl_write_reg(0x00, 128);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write pixel PWM");
        return;
    }
*/
    /*
    ret = is31fl_write_reg(0x00, 128);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write pixel PWM");
        return;
    }*/

    ESP_LOGE(TAG, "Pixel 0 set to brightness");

    /*

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, IS31FL3733_PSWL, true);
    i2c_master_write_byte(cmd, IS31FL3733_PSWL_ENABLE, true);
    i2c_master_stop(cmd);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, IS31FL3733_PSR, true);
    i2c_master_write_byte(cmd, 0x01, true);
    i2c_master_stop(cmd);


  	while (bytesToSend > 0) {

        i2c_master_write_byte(cmd, 0x30, true);
        bytesToSend--;

  	}
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);
*/

    vTaskDelay(pdMS_TO_TICKS(100));  // 
    printf("*Display out*\n");
    
}