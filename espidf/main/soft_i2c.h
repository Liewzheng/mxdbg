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
#include "esp_err.h"

/*-----------------------------------------------------------------------------
 * TYPES
 *---------------------------------------------------------------------------*/

typedef struct {
    uint8_t port;
    uint8_t sclk;
    uint8_t sdio;
    bool pullup_en;
    bool inited;
    uint32_t clk_speed;
    uint32_t ns;
} soft_i2c_port_t;



/*-----------------------------------------------------------------------------
 * FUNCITON PROTOTYPES
 *---------------------------------------------------------------------------*/

esp_err_t soft_i2c_init(soft_i2c_port_t *port);
esp_err_t soft_i2c_deinit(soft_i2c_port_t *port);

esp_err_t soft_i2c_write_read(soft_i2c_port_t *port,
    uint8_t slave_id,
    uint8_t *write_list,
    size_t write_length,
    uint8_t *read_list,
    size_t read_length,
    uint32_t ticks_to_wait);