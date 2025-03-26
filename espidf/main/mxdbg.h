/*
* Copyright (c) 2024 MixoSense Technology Ltd <contact@mixosense.com>.
*
* All rights are reserved.
* Proprietary and confidential.
* Unauthorized copying of this file, via any medium is strictly prohibited.
* Any use is subject to an appropriate license granted by MixoSense Technology
* Ltd.
*
*/

#pragma once

/*-----------------------------------------------------------------------------
 * ENUMS
 *---------------------------------------------------------------------------*/
typedef enum {
    TASK_IDLE = 0,
    TASK_I2C_WRITE_READ = 0x01,
    TASK_I2C_CONFIG = 0x02,
    TASK_GPIO_WRITE_READ = 0x03,
    TASK_GPIO_CONFIG = 0x04,
    TASK_SPI_WRITE_READ = 0x05,
    TASK_SPI_CONFIG = 0x06,
    TASK_PWM_RUN_STOP = 0x07,
    TASK_PWM_CONFIG = 0x08,
    TASK_HIGH_SPEED_PWM_CONFIG = 0x09,
    TASK_HIGH_SPEED_PWM_RUN_STOP = 0x0A,
    TASK_ADC_CONFIG = 0x10,
    TASK_ADC_READ = 0x11,
    TASK_SPI_READ_IMAGE = 0xA0,
    TASK_USB_CONFIG = 0xE0,
    TASK_RESET_DEVICE = 0xF0,
} task_cmd_t;

/*-----------------------------------------------------------------------------
 * MACROS
 *---------------------------------------------------------------------------*/
#define VERSION_MAJOR            1
#define VERSION_MINOR            0
#define ERR_SYNTHESIS(type, ret) ((VERSION_MAJOR << 24) | (VERSION_MINOR << 16) | (uint8_t)type << 8 | (uint8_t)ret)

/* Definitions for error constants. */
#define ESPRESSIF_OK   0x00       /*!< esp_err_t value indicating success (no error) */
#define ESPRESSIF_FAIL 0xFFFFFFFF /*!< Generic esp_err_t code indicating failure */

#define ESPRESSIF_ERR_NO_MEM           0x101 /*!< Out of memory */
#define ESPRESSIF_ERR_INVALID_ARG      0x102 /*!< Invalid argument */
#define ESPRESSIF_ERR_INVALID_STATE    0x103 /*!< Invalid state */
#define ESPRESSIF_ERR_INVALID_SIZE     0x104 /*!< Invalid size */
#define ESPRESSIF_ERR_NOT_FOUND        0x105 /*!< Requested resource not found */
#define ESPRESSIF_ERR_NOT_SUPPORTED    0x106 /*!< Operation or feature not supported */
#define ESPRESSIF_ERR_TIMEOUT          0x107 /*!< Operation timed out */
#define ESPRESSIF_ERR_INVALID_RESPONSE 0x108 /*!< Received response was invalid */
#define ESPRESSIF_ERR_INVALID_CRC      0x109 /*!< CRC or checksum was invalid */
#define ESPRESSIF_ERR_INVALID_VERSION  0x10A /*!< Version was invalid */
#define ESPRESSIF_ERR_INVALID_MAC      0x10B /*!< MAC address was invalid */
#define ESPRESSIF_ERR_NOT_FINISHED     0x10C /*!< There are items remained to retrieve */

#define ESPRESSIF_ERR_WIFI_BASE      0x3000 /*!< Starting number of WiFi error codes */
#define ESPRESSIF_ERR_MESH_BASE      0x4000 /*!< Starting number of MESH error codes */
#define ESPRESSIF_ERR_FLASH_BASE     0x6000 /*!< Starting number of flash error codes */
#define ESPRESSIF_ERR_HW_CRYPTO_BASE 0xc000 /*!< Starting number of HW cryptography module error codes */
#define ESPRESSIF_ERR_MEMPROT_BASE   0xd000 /*!< Starting number of Memory Protection API error codes */

// task
#define MXDBG_ERR_UNKNOWN_TASK ERR_SYNTHESIS(0xFF, 0x00) /*!< Unknown task */

// data unpack
#define MXDBG_ERR_DATA_UNPACK_FAILED          ERR_SYNTHESIS(0xE0, 0x00) /*!< Data unpack failed */
#define MXDBG_ERR_DATA_UNPACK_HEADER_ERROR    ERR_SYNTHESIS(0xE0, 0x01) /*!< Data unpack header error */
#define MXDBG_ERR_DATA_UNPACK_SEPARATOR_ERROR ERR_SYNTHESIS(0xE0, 0x02) /*!< Data unpack separator error */

// tinyusb_cdc_rx_callback
#define MXDBG_ERR_USB_RX_DATA_FAILED ERR_SYNTHESIS(0xE1, 0x00) /*!< USB RX data failed */
#define MXDBG_ERR_USB_RX_DATA_OVERFLOW \
    ERR_SYNTHESIS(0xE1, 0x01) /*!< USB RX data overflow, please decrease the data transfered. */
#define MXDBG_ERR_USB_RX_DATA_TIMEOUT      ERR_SYNTHESIS(0xE1, 0x02) /*!< USB RX data timeout */
#define MXDBG_ERR_USB_RX_DATA_HEADER_ERROR ERR_SYNTHESIS(0xE1, 0x03) /*!< USB RX data header error */

// PWM run stop
#define MXDBG_ERR_PWM_RUN_ALREADY  ERR_SYNTHESIS(TASK_PWM_RUN_STOP, 0x01) /*!< PWM is running already.*/
#define MXDBG_ERR_PWM_STOP_ALREADY ERR_SYNTHESIS(TASK_PWM_RUN_STOP, 0x02) /*!< PWM is stopped already.*/

// pwm config
#define MXDBG_ERR_PWM_CONFIG_FAILED       ERR_SYNTHESIS(TASK_PWM_CONFIG, 0x00) /*!< PWM config failed */
#define MXDBG_ERR_PWM_INVALID_CHANNEL     ERR_SYNTHESIS(TASK_PWM_CONFIG, 0x01) /*!< Invalid PWM channel */
#define MXDBG_ERR_PWM_INVALID_PARAMS      ERR_SYNTHESIS(TASK_PWM_CONFIG, 0x02) /*!< Invalid PWM parameters */
#define MXDBG_ERR_PWM_INVALID_DUTY_CYCLES ERR_SYNTHESIS(TASK_PWM_CONFIG, 0x03) /*!< Invalid PWM duty cycles */

// gpio read write
#define MXDBG_ERR_GPIO_INVALID_OPERATION  ERR_SYNTHESIS(TASK_GPIO_WRITE_READ, 0x00) /*!< Invalid GPIO operation */

// adc read
#define MXDBG_ERR_ADC_READ_FAILED         ERR_SYNTHESIS(TASK_ADC_READ, 0x00) /*!< ADC read failed */

// adc config
#define MXDBG_ERR_ADC_INIT_FAILED          ERR_SYNTHESIS(TASK_ADC_CONFIG, 0x00) /*!< ADC init failed */
#define MXDBG_ERR_ADC_INVALID_CHANNEL_LIST ERR_SYNTHESIS(TASK_ADC_CONFIG, 0x01) /*!< Invalid ADC channel config list */

// task_spi_read_image
#define MXDBG_ERR_SPI_READ_IMAGE_FAILED ERR_SYNTHESIS(TASK_SPI_READ_IMAGE, 0x00) /*!< SPI read image failed */

// task_i2c
#define MXDBG_ERR_I2C_INIT_FAILED       ERR_SYNTHESIS(TASK_I2C_CONFIG, 0x00)     /*!< I2C init failed */
#define MXDBG_ERR_I2C_DEINIT_FAILED     ERR_SYNTHESIS(TASK_I2C_CONFIG, 0x01)     /*!< I2C deinit failed */
#define MXDBG_ERR_I2C_INVALID_OPERATION ERR_SYNTHESIS(TASK_I2C_WRITE_READ, 0x02) /*!< Invalid I2C operation */
#define MXDBG_ERR_I2C_WRITE_READ_FAILED ERR_SYNTHESIS(TASK_I2C_WRITE_READ, 0x03) /*!< I2C write read failed */
#define MXDBG_ERR_I2C_WRITE_FAILED      ERR_SYNTHESIS(TASK_I2C_WRITE_READ, 0x04) /*!< I2C write failed */
#define MXDBG_ERR_I2C_READ_FAILED       ERR_SYNTHESIS(TASK_I2C_WRITE_READ, 0x05) /*!< I2C read failed */

// task_spi
#define MXDBG_ERR_SPI_INIT_FAILED        ERR_SYNTHESIS(TASK_SPI_CONFIG, 0x00)     /*!< SPI init failed */
#define MXDBG_ERR_SPI_DEINIT_FAILED      ERR_SYNTHESIS(TASK_SPI_CONFIG, 0x01)     /*!< SPI deinit failed */
#define MXDBG_ERR_SPI_INVALID_OPERATION  ERR_SYNTHESIS(TASK_SPI_WRITE_READ, 0x02) /*!< Invalid SPI operation */
#define MXDBG_ERR_SPI_READ_LEN_TOO_LONG  ERR_SYNTHESIS(TASK_SPI_WRITE_READ, 0x03) /*!< SPI read length too long */
#define MXDBG_ERR_SPI_TRANSACTION_FAILED ERR_SYNTHESIS(TASK_SPI_WRITE_READ, 0x04) /*!< SPI transaction failed */

// task_gpio
#define MXDBG_ERR_GPIO_CONFIG_FAILED ERR_SYNTHESIS(TASK_GPIO_CONFIG, 0x00) /*!< GPIO config failed */