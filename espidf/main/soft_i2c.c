/*
* Copyright (c) 2023 MixoSense Technology Ltd <contact@mixosense.com>.
*
* All rights are reserved.
* Proprietary and confidential.
* Unauthorized copying of this file, via any medium is strictly prohibited.
* Any use is subject to an appropriate license granted by MixoSense Technology
* Ltd.
*
*/

/*-----------------------------------------------------------------------------
 * HEADER FILES
 *---------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "esp_err.h"

#include "driver/gpio.h"
#include "esp_log.h"

#include "soft_i2c.h"

#include "esp_attr.h"

/*-----------------------------------------------------------------------------
 * MACROS
 *---------------------------------------------------------------------------*/

 #define DEVICE_ID_TRANSMIT(x) (uint8_t)((x << 1) & 0xFE)
 #define DEVICE_ID_RECEIVE(x)  (uint8_t)((x << 1) | 0x01)

/*-----------------------------------------------------------------------------
 * EXTERNAL FUNCTIONS
 *---------------------------------------------------------------------------*/
extern unsigned xthal_get_ccount(void);

/*-----------------------------------------------------------------------------
 * VARIABLES
 *---------------------------------------------------------------------------*/
soft_i2c_port_t i2c_slaves[8] = {
    {
        .port = 0,
        .sclk = 20,
        .sdio = 21,
        .pullup_en = 0,
        .clk_speed = 100000,
    },
    {
        .port = 1,
        .sclk = 22,
        .sdio = 23,
        .pullup_en = 0,
        .clk_speed = 100000,
    }
};

/*-----------------------------------------------------------------------------
 * STATIC FUNCTIONS
 *---------------------------------------------------------------------------*/

volatile static inline void I2C_DELAY(uint32_t ns) {
    if (ns == 0) {
        return;
    }
    for (uint32_t i = 0; i < ns; i++) {
        __asm__ volatile ("nop");
        __asm__ volatile ("nop");
    }
}

static inline void i2c_start(soft_i2c_port_t* slave)
{
    gpio_set_level(slave->sdio, 1);
    I2C_DELAY(slave->ns);
    gpio_set_level(slave->sclk, 1);
    I2C_DELAY(slave->ns);

    gpio_set_level(slave->sdio, 0); 
    I2C_DELAY(slave->ns);
    gpio_set_level(slave->sclk, 0);
    I2C_DELAY(slave->ns);
}
 
static inline void i2c_stop(soft_i2c_port_t* slave)
{
    gpio_set_level(slave->sdio, 0);
    I2C_DELAY(slave->ns);

    gpio_set_level(slave->sclk, 1);
    I2C_DELAY(slave->ns);

    gpio_set_level(slave->sdio, 1);
    I2C_DELAY(slave->ns);
}

static inline int i2c_send_ack(soft_i2c_port_t* slave, bool ack_or_nack)
{
    uint8_t tmp = 0;

    gpio_set_level(slave->sdio, ack_or_nack ? 0 : 1);
    
    gpio_set_level(slave->sclk, 1);
    I2C_DELAY(slave->ns);

    gpio_set_level(slave->sclk, 0);
    I2C_DELAY(slave->ns);

    return tmp;
}

static inline uint8_t i2c_get_ack(soft_i2c_port_t* slave)
{
    uint8_t tmp = 0;

    gpio_set_level(slave->sclk, 1);
    tmp = gpio_get_level(slave->sdio);  // 在高电平下读取数据
    I2C_DELAY(slave->ns / 2);

    gpio_set_level(slave->sclk, 0);
    I2C_DELAY(slave->ns);

    return tmp;
}

static inline int i2c_read(soft_i2c_port_t* slave, uint8_t* data)
{
    uint8_t tmp[8];
    uint8_t tmp_data = 0;

    for (int i = 0; i < 8; i++)
    {
        gpio_set_level(slave->sclk, 1);
        tmp_data <<= 1;
        tmp[i] = gpio_get_level(slave->sdio);
        tmp_data |= tmp[i];  // lsb
        I2C_DELAY(slave->ns);

        gpio_set_level(slave->sclk, 0);
        I2C_DELAY(slave->ns);
    }

    *data = tmp_data;

    return 0;
}

static inline int i2c_write(soft_i2c_port_t* slave, uint8_t data)
{
    uint8_t tmp[8];

    for (int i = 8; i > 0; i--)
    {
        tmp[i - 1] = (data >> (i - 1)) & 0x01;
        gpio_set_level(slave->sdio, tmp[i - 1]);
        gpio_set_level(slave->sclk, 1);
        I2C_DELAY(slave->ns);

        gpio_set_level(slave->sclk, 0);
        I2C_DELAY(slave->ns);
    }

    return 0;
}
 
/*-----------------------------------------------------------------------------
* FUNCITON DEFINITIONS
*---------------------------------------------------------------------------*/

esp_err_t soft_i2c_init(soft_i2c_port_t* slave)
{
    // Check if the slave is already initialized
    if (slave->inited) {
        ESP_LOGW("I2C", "I2C port %d is already initialized", slave->port);
        return -1;
    }

    esp_err_t ret = ESP_OK;

    // Configure the GPIO pins for I2C communication
    gpio_config_t io_config_sclk = {
        .pin_bit_mask = (1ULL << slave->sclk) ,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD, // Open-drain mode for I2C
        .pull_up_en = slave->pullup_en ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE, // Enable pull-up resistor if configured
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down resistor
        .intr_type = GPIO_INTR_DISABLE, // No interrupt
    };

    gpio_config_t io_config_sdio = {
        .pin_bit_mask = (1ULL << slave->sdio) ,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD, // Open-drain mode for I2C
        .pull_up_en = slave->pullup_en ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE, // Enable pull-up resistor if configured
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down resistor
        .intr_type = GPIO_INTR_DISABLE, // No interrupt
    };

    ret = gpio_config(&io_config_sclk);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to configure GPIO pins for SCLK");
        return -2;
    }

    ret = gpio_config(&io_config_sdio);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to configure GPIO pins for SDIO");
        return -3;
    }

    if (slave->ns == 0)
    {
        // Set the I2C clock speed
        if ((slave->clk_speed > 0) && (slave->clk_speed <= 1000000)) { // 1MHz max
            if (slave->clk_speed <= 100000)
            {
                slave->ns = 113;
            }
            else if (slave->clk_speed <= 400000)
            {
                slave->ns = 15;
            }
            else
            {
                slave->ns = 0;
            }
        }
        else
        {
            ESP_LOGE("I2C", "Invalid I2C clock speed: %ld", slave->clk_speed);
            return -4;
        }
    }

    gpio_set_level(slave->sdio, 1); // Set SDIO high
    I2C_DELAY(slave->ns);
    gpio_set_level(slave->sclk, 1); // Set SCLK high

    slave->inited = true; // Mark the I2C port as initialized

    ESP_LOGI("I2C", "I2C port %d initialized successfully", slave->port);
    return ESP_OK;
}

esp_err_t soft_i2c_deinit(soft_i2c_port_t *port)
{
    // Check if the I2C port is valid
    if (port->port >= (sizeof(i2c_slaves) / sizeof(i2c_slaves[0]))) {
        ESP_LOGE("I2C", "Invalid I2C port: %d", port->port);
        return -1;
    }

    soft_i2c_port_t* slave = port;

    // Check if the I2C port is already deinitialized
    if (!slave->inited) {
        ESP_LOGW("I2C", "I2C port %d is already deinitialized", port->port);
        return -1;
    }

    // Deinitialize the GPIO pins for I2C communication
    gpio_config_t io_config_sclk = {
        .pin_bit_mask = (1ULL << slave->sclk) ,
        .mode = GPIO_MODE_DISABLE, // Disable the GPIO mode
        .pull_up_en = GPIO_PULLUP_DISABLE, // Disable pull-up resistor
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down resistor
        .intr_type = GPIO_INTR_DISABLE, // No interrupt
    };

    gpio_config_t io_config_sdio = {
        .pin_bit_mask = (1ULL << slave->sdio) ,
        .mode = GPIO_MODE_DISABLE, // Disable the GPIO mode
        .pull_up_en = GPIO_PULLUP_DISABLE, // Disable pull-up resistor
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down resistor
        .intr_type = GPIO_INTR_DISABLE, // No interrupt
    };

    esp_err_t ret = gpio_config(&io_config_sclk);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to deinitialize GPIO pins for I2C: %s", esp_err_to_name(ret));
        return -2;
    }

    ret = gpio_config(&io_config_sdio);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to deinitialize GPIO pins for I2C: %s", esp_err_to_name(ret));
        return -3;
    }

    memset(slave, 0, sizeof(soft_i2c_port_t)); // Clear the I2C port structure

    ESP_LOGI("I2C", "I2C port %d deinitialized successfully", port->port);
    return ESP_OK;
}

esp_err_t soft_i2c_write_read(soft_i2c_port_t *port,
     uint8_t slave_id,
     uint8_t *write_list,
     size_t write_length,
     uint8_t *read_list,
     size_t read_length,
     uint32_t ticks_to_wait)
{
    // Check if the I2C port is valid
    if (port == NULL || port->inited == false) {
        // ESP_LOGE("I2C", "Invalid I2C port");
        return -1;
    }

    soft_i2c_port_t* slave = port;
    esp_err_t ret = ESP_OK;
    uint8_t ack = 0;

    if (write_list == NULL && read_length == 0) {
        // ESP_LOGE("I2C", "No data to write or read");
        return -1;
    }

    if (read_list == NULL && read_length > 0) {
        // ESP_LOGE("I2C", "No buffer to read data into");
        return -1;
    }

    if (write_list == NULL && write_length > 0) {
        // ESP_LOGE("I2C", "No buffer to write data from");
        return -1;
    }

    // write only
    if (write_length > 0 && read_length == 0) {
        i2c_start(slave);
        
        i2c_write(slave, DEVICE_ID_TRANSMIT(slave_id));

        ack = i2c_get_ack(slave);
        if (ack != 0) {
            // ESP_LOGE("I2C", "Failed to get ACK from slave device %d", slave_id);
            i2c_stop(slave);
            return -2;
        }

        for (int i = 0; i < write_length; i++) {
            i2c_write(slave, write_list[i]);
            ack = i2c_get_ack(slave);
            if (ack != 0) {
                // ESP_LOGE("I2C", "Failed to get ACK from slave device %d", slave_id);
                i2c_stop(slave);
                return -2;
            }
        }
        i2c_stop(slave);
        
    }
    // read only
    else if (read_length > 0 && write_length == 0) {
        i2c_start(slave);
        
        i2c_write(slave, DEVICE_ID_RECEIVE(slave_id));

        ack = i2c_get_ack(slave);
        if (ack != 0) {
            // ESP_LOGE("I2C", "Failed to get ACK from slave device %d", slave_id);
            i2c_stop(slave);
            return -3;
        }

        for (int i = 0; i < read_length; i++) {
            i2c_read(slave, &read_list[i]);
            if (i < read_length - 1) {
                i2c_send_ack(slave, true); // send ACK for all but the last byte
            } else {
                i2c_send_ack(slave, false); // send NACK for the last byte
            }
        }
        i2c_stop(slave);
    }
    // write and read
    else{
        i2c_start(slave);
        
        i2c_write(slave, DEVICE_ID_TRANSMIT(slave_id));

        ack = i2c_get_ack(slave);
        if (ack != 0) {
            // ESP_LOGE("I2C", "Failed to get ACK from slave device %d", slave_id);
            i2c_stop(slave);
            return -4;
        }

        for (int i = 0; i < write_length; i++) {
            i2c_write(slave, write_list[i]);
            ack = i2c_get_ack(slave);
            if (ack != 0) {
                // ESP_LOGE("I2C", "Failed to get ACK from slave device %d", slave_id);
                i2c_stop(slave);
                return -4;
            }
        }

        // read data
        i2c_start(slave);
        
        i2c_write(slave, DEVICE_ID_RECEIVE(slave_id));

        ack = i2c_get_ack(slave);
        if (ack != 0) {
            // ESP_LOGE("I2C", "Failed to get ACK from slave device %d", slave_id);
            i2c_stop(slave);
            return -5;
        }

        for (int i = 0; i < read_length; i++) {
            i2c_read(slave, &read_list[i]);
            if (i < read_length - 1) {
                i2c_send_ack(slave, true); // send ACK for all but the last byte
            } else {
                i2c_send_ack(slave, false); // send NACK for the last byte
            }
        }
        i2c_stop(slave);
    }

    return ret;
}