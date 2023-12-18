#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <rom/ets_sys.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
//#include "hal/spi_types.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif
/*
# TOOLBOX SETUP
#
CONFIG_TOOLBOX_ENABLED=y
# CONFIG_ENABLE_I2C is not set
CONFIG_ENABLE_SPI=y
CONFIG_SPI_HOST=2
CONFIG_SPI_GPIO_MISO=13
CONFIG_SPI_GPIO_MOSI=11
CONFIG_SPI_GPIO_CLK=10
CONFIG_SPI_MODE=0
CONFIG_SPI_CLOCK_FREQ=1000
CONFIG_SPI_MAX_TRANSFER_SZ=32
# end of TOOLBOX SETUP
*/
#ifdef CONFIG_TOOLBOX_ENABLED
#ifdef CONFIG_ENABLE_I2C
#define I2C_MASTER_PORT     CONFIG_I2C_PORT_NUM
#define I2C_GPIO_SCL        CONFIG_I2C_GPIO_SCL
#define I2C_GPIO_SDA        CONFIG_I2C_GPIO_SDA
#define I2C_BUS_SPEED   CONFIG_I2C_MASTER_FREQUENCY
#define I2C_CMD_TIMEOUT CONFIG_I2C_BUS_TIMEOUT

#ifndef CONFIG_I2C_GPIO_SCL_PULLUP
#define I2C_GPIO_SCL_PULLUP 0
#else
#define I2C_GPIO_SCL_PULLUP 1
#endif

#ifndef CONFIG_I2C_GPIO_SDA_PULLUP
#define I2C_GPIO_SDA_PULLUP 0
#else
#define I2C_GPIO_SDA_PULLUP 1
#endif
#else
#define I2C_MASTER_PORT     0
#define I2C_GPIO_SCL        0
#define I2C_GPIO_SDA        0
#define I2C_BUS_SPEED   0
#define I2C_CMD_TIMEOUT 0
#define I2C_GPIO_SCL_PULLUP 0
#define I2C_GPIO_SDA_PULLUP 0

#endif

#ifdef CONFIG_ENABLE_SPI
#define SPI_HOST_NUM         CONFIG_SPI_HOST
#define PIN_NUM_MISO         CONFIG_SPI_GPIO_MISO
#define PIN_NUM_MOSI         CONFIG_SPI_GPIO_MOSI
#define PIN_NUM_CLK          CONFIG_SPI_GPIO_CLK
#define PIN_NUM_CS           CONFIG_SPI_GPIO_CS
#define SPI_CLK_FREQ         (CONFIG_SPI_CLOCK_FREQ * 1000)
#define SPI_MAX_TRANS_SZ     CONFIG_SPI_MAX_TRANSFER_SZ
#define SPI_BUS_MODE         CONFIG_SPI_MODE
#endif

#endif

// shortcuts/maxcro's
#define makeWord(x,y)       ((x << 8) | y)
#define millis()    ( esp_timer_get_time() / 1000 )
#define LOGI(...)     ESP_LOGI(__func__,__VA_ARGS__)
#define LOGD(...)     ESP_LOGD(__func__,__VA_ARGS__)
#define LOGE(...)     ESP_LOGE(__func__,__VA_ARGS__)
#define LOGV(...)     ESP_LOGV(__func__,__VA_ARGS__)

extern bool i2c_is_initialized;

/* ** i2c helpers ** */
esp_err_t _i2c_init(void);
esp_err_t i2c_read(uint8_t i2c_addr, uint8_t *read_buffer, size_t read_size);
esp_err_t i2c_write(uint8_t i2c_addr, uint8_t *write_buffer, size_t write_size);
esp_err_t i2c_write_read(uint8_t i2c_addr, uint8_t read_reg, uint8_t *read_buffer, size_t read_size, uint32_t delay_us);


/***** SPI **** */
typedef struct {
    spi_device_handle_t             spi;
    spi_device_interface_config_t   intcfg;
} tbox_spi_dev_t;

esp_err_t _spi_init(tbox_spi_dev_t *spi_dev);

/* ** tools ** */
char *print_2bytes(uint16_t value);
char *print_byte(uint8_t byte);
int16_t uint2int(uint16_t value);

/* **** useful Arduino Functions ***** */
double map(double x, double in_min, double in_max, double out_min, double out_max);

uint8_t bcd2dec(uint8_t val);
uint8_t dec2bcd(uint8_t val);

#ifdef __cplusplus
}
#endif