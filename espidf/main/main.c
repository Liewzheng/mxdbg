/*
 * Copyright (c) 2024 MixoSense Technology Ltd <contact@mixosense.com>.
 *
 * All rights are reserved.
 * Proprietary and confidential.
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Any use is subject to an appropriate license granted by MixoSense Technology
 * Ltd.
 */

/*-----------------------------------------------------------------------------
 * HEADER FILES
 *---------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/mcpwm_prelude.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "sdkconfig.h"

#include "mxdbg.h"

/*-----------------------------------------------------------------------------
 * MACROS
 *---------------------------------------------------------------------------*/

#define COM_DATA_BUFFER_LENGTH 1024
static uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];

#define COM_PROTOCOL_HEADER "mxdbg:"

#define DATA_SYNTHESIS_4_BYTES(data) ((*(data + 0)) << 24 | (*(data + 1)) << 16 | (*(data + 2)) << 8 | (*(data + 3)))
#define DATA_SYNTHESIS_2_BYTES(data) ((*(data + 0)) << 8 | (*(data + 1)))

/*-----------------------------------------------------------------------------
 * FUNCTION PROTOTYPES
 *---------------------------------------------------------------------------*/
void task_duty_call(void *pvParameters);
void task_notify(void *pvParameters);
void task_i2c_write_read(void *pvParameters);
void task_i2c_config(void *pvParameters);
void task_gpio_write_read(void *pvParameters);
void task_gpio_config(void *pvParameters);
void task_spi_write_read(void *pvParameters);
void task_spi_config(void *pvParameters);
void task_pwm_run_stop(void *pvParameters);
void task_pwm_config(void *pvParameters);
void task_spi_read_image(void *pvParameters);
void task_usb_config(void *pvParameters);

void spi_callback(spi_transaction_t *t);

/*-----------------------------------------------------------------------------
 * GLOBAL VARIABLES
 *---------------------------------------------------------------------------*/

// Task semaphores
SemaphoreHandle_t semaphore_duty_call;
SemaphoreHandle_t semaphore_i2c_write_read;
SemaphoreHandle_t semaphore_task_notify;
SemaphoreHandle_t semaphore_gpio_write_read;
SemaphoreHandle_t semaphore_gpio_config;
SemaphoreHandle_t semaphore_i2c_config;
SemaphoreHandle_t semaphore_spi_write_read;
SemaphoreHandle_t semaphore_spi_config;
SemaphoreHandle_t semaphore_pwm_run_stop;
SemaphoreHandle_t semaphore_pwm_config;

SemaphoreHandle_t semaphore_pwm_running_state;
SemaphoreHandle_t semaphore_usb_total_rx_size;

SemaphoreHandle_t semaphore_spi_read_image;

SemaphoreHandle_t semaphore_usb_config;
SemaphoreHandle_t semaphore_reset_device;

// USB

uint8_t com_data_content[COM_DATA_BUFFER_LENGTH] = { 0 };
uint8_t com_data_send_buffer[COM_DATA_BUFFER_LENGTH] = { 0 };

bool usb_connected_state = false;
uint64_t usb_data_rx_start_time = 0;
size_t total_rx_size = 0;
size_t tx_size = 0;
size_t rx_size = 0;
int usb_cdc_itf = 0;

bool crc_enable = true;

// I2C

i2c_config_t i2c_conf0 = { .mode = I2C_MODE_MASTER,
                           .sda_io_num = 10,
                           .scl_io_num = 11,
                           .sda_pullup_en = GPIO_PULLUP_ENABLE,
                           .scl_pullup_en = GPIO_PULLUP_ENABLE,
                           .master.clk_speed = 400000 };
i2c_config_t i2c_conf1 = { .mode = I2C_MODE_MASTER,
                           .sda_io_num = 42,
                           .scl_io_num = 41,
                           .sda_pullup_en = GPIO_PULLUP_ENABLE,
                           .scl_pullup_en = GPIO_PULLUP_ENABLE,
                           .master.clk_speed = 400000 };

// PWM

#define MAX_PWM_CHANNELS 3

typedef struct {
    mcpwm_timer_handle_t timer;
    mcpwm_timer_config_t timer_config;

    mcpwm_oper_handle_t operator;
    mcpwm_operator_config_t operator_config;

    mcpwm_cmpr_handle_t comparator;
    mcpwm_comparator_config_t comparator_config;
    mcpwm_gen_compare_event_action_t compare_action_start;
    mcpwm_gen_compare_event_action_t compare_action_stop;

    mcpwm_gen_handle_t generator;
    mcpwm_generator_config_t generator_config;

    uint32_t duty_cycle_ticks;
    bool run_state;

} pwm_channel_t;

pwm_channel_t pwm_channels[MAX_PWM_CHANNELS];

// SPI

spi_device_handle_t spi;
spi_bus_config_t buscfg = { .miso_io_num = 12,
                            .mosi_io_num = 13,
                            .sclk_io_num = 14,
                            .quadwp_io_num = -1,
                            .quadhd_io_num = -1,
                            .max_transfer_sz = 1000 };

spi_device_interface_config_t devcfg = {
    .clock_speed_hz = 1 * 1000 * 1000,
    .mode = 3,
    .spics_io_num = 15,
    .queue_size = 7,
    .pre_cb = spi_callback,
    .cs_ena_posttrans = 0,
    .cs_ena_pretrans = 0,
};

/*-----------------------------------------------------------------------------
 * STATIC VARIABLES
 *---------------------------------------------------------------------------*/
static const char *TAG = "MXDBG";

/*-----------------------------------------------------------------------------
 * STATIC FUNCTIONS
 *---------------------------------------------------------------------------*/

static inline void crc16(uint8_t *data, size_t len, uint8_t *crc_high, uint8_t *crc_low)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (crc & 0x0001 ? 0xA001 : 0);
        }
    }
    *crc_high = crc >> 8;
    *crc_low = crc & 0xFF;
}

static inline void data_pack(uint8_t *data_sent, size_t data_length, task_cmd_t cmd, int ret)
{
    const char *header = COM_PROTOCOL_HEADER;
    size_t pos = 0;
    uint8_t crc_high = 0, crc_low = 0;

    for (size_t i = 0; i < strlen(header); i++) {
        com_data_send_buffer[pos++] = header[i];
    }

    // 2. 添加 cmd 和 ret 值
    com_data_send_buffer[pos++] = (uint8_t)cmd; // cmd as uint8_t
    com_data_send_buffer[pos++] = ':';          // 分隔符

    // 3. 添加 ret 值
    com_data_send_buffer[pos++] = (ret & 0xFF000000) >> 24;
    com_data_send_buffer[pos++] = (ret & 0x00FF0000) >> 16;
    com_data_send_buffer[pos++] = (ret & 0x0000FF00) >> 8;
    com_data_send_buffer[pos++] = (ret & 0x000000FF);
    com_data_send_buffer[pos++] = ':'; // 分隔符

    // 4. add data_list
    if (data_length > 0 && data_sent != NULL) {
        for (size_t i = 0; i < data_length; i++) {
            com_data_send_buffer[pos++] = data_sent[i];
        }

        com_data_send_buffer[pos++] = ':'; // 分隔符
    }

    if (crc_enable) {
        crc16(com_data_send_buffer, pos, &crc_high, &crc_low);
    }

    com_data_send_buffer[pos++] = crc_high;
    com_data_send_buffer[pos++] = crc_low;

    tx_size = pos;
}

static inline int data_unpack(uint8_t *data, size_t data_length, task_cmd_t *cmd, uint8_t *content,
                              size_t *content_length)
{
    size_t pos = 0;

    // mxdbg:
    for (size_t i = 0; i < 5; i++) {
        if (data[i] != COM_PROTOCOL_HEADER[i]) {
            ESP_LOGE(TAG, "Protocol header error");
            return MXDBG_ERR_DATA_UNPACK_HEADER_ERROR;
        }
    }
    pos += 6;

    // cmd:
    *cmd = (task_cmd_t)data[pos++];
    if (data[pos++] != ':') {
        ESP_LOGE(TAG, "Protocol separator error");
        return MXDBG_ERR_DATA_UNPACK_SEPARATOR_ERROR;
    }

    // data:
    if (data_length <= 10) { // no data
        content = NULL;
        content_length = 0;
    } else {
        if (data[data_length - 3] != ':') { // format error
            ESP_LOGE(TAG, "Protocol separator error");
            return MXDBG_ERR_DATA_UNPACK_SEPARATOR_ERROR;
        } else {                                // data exists, extract data
            *content_length = data_length - 11; // 5 bytes header + 1 byte cmd + 3 bytes separator + 2 bytes crc
            for (size_t i = 0; i < *content_length; i++) {
                content[i] = data[pos++];
                // ESP_LOGI(TAG, "content[%d]: %d", i, content[i]);
            }
        }
    }

    return ESP_OK;
}

esp_err_t i2c_init(i2c_port_t port)
{
    esp_err_t ret = ESP_OK;

    ret = i2c_param_config((i2c_port_t)port, port == 0 ? (&i2c_conf0) : (&i2c_conf1));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed");
    }

    ret = i2c_driver_install((i2c_port_t)port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
    }

    return ret;
}

esp_err_t i2c_deinit(i2c_port_t port)
{
    esp_err_t ret = 0;

    ret = i2c_driver_delete((i2c_port_t)port);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C deinit failed");
    }

    return ret;
}

void pwm_init(pwm_channel_t *channel)
{
    // 1. 创建 MCPWM 定时器
    ESP_ERROR_CHECK(mcpwm_new_timer(&(channel->timer_config), &(channel->timer)));

    // 2. 创建 MCPWM 操作器
    ESP_ERROR_CHECK(mcpwm_new_operator(&(channel->operator_config), &(channel->operator)));

    // 3. 将操作器连接到定时器
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(channel->operator, channel->timer));

    // 4. 创建 MCPWM 比较器
    ESP_ERROR_CHECK(mcpwm_new_comparator(channel->operator, &(channel->comparator_config), &(channel->comparator)));
    channel->compare_action_start.comparator = channel->comparator;

    // 5. 创建 MCPWM 生成器并绑定到 GPIO
    ESP_ERROR_CHECK(mcpwm_new_generator(channel->operator, &(channel->generator_config), &(channel->generator)));

    // 6. 设置初始的生成器动作（PWM 停止状态，输出低电平）
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        channel->generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));

    // 设置比较值为 0，确保 PWM 停止时输出低电平
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(channel->comparator, 0));

    // 设置比较事件动作，在比较事件发生时输出低电平
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(channel->generator, channel->compare_action_start));

    // 7. 启用并启动 MCPWM 定时器
    ESP_ERROR_CHECK(mcpwm_timer_enable(channel->timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(channel->timer, MCPWM_TIMER_START_NO_STOP));
}

void pwm_deinit(pwm_channel_t *channel)
{
    ESP_ERROR_CHECK(mcpwm_timer_disable(channel->timer));
    ESP_ERROR_CHECK(mcpwm_del_generator(channel->generator));
    ESP_ERROR_CHECK(mcpwm_del_comparator(channel->comparator));
    ESP_ERROR_CHECK(mcpwm_del_operator(channel->operator));
    ESP_ERROR_CHECK(mcpwm_del_timer(channel->timer));
}

esp_err_t spi_init()
{
    esp_err_t ret = 0;

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus initialize failed");
        return ret;
    }

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed");
        return ret;
    }

    return ret;
}

esp_err_t spi_deinit()
{
    esp_err_t ret = 0;

    ret = spi_bus_remove_device(spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device remove failed");
        return ret;
    }

    ret = spi_bus_free(SPI2_HOST);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus free failed");
        return ret;
    }

    return ret;
}

void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    usb_cdc_itf = itf;
    bool header_valid = false;
    int ret = 0;

    ret = tinyusb_cdcacm_read(itf, buf + total_rx_size, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK) {
        if (xSemaphoreTake(semaphore_usb_total_rx_size, 0) == pdTRUE) {
            total_rx_size += rx_size;
            xSemaphoreGive(semaphore_usb_total_rx_size);
        }

        // ESP_LOGI(TAG, "total_rx_size: %d", total_rx_size);
        // ESP_LOG_BUFFER_HEXDUMP(TAG, buf, total_rx_size, ESP_LOG_INFO);

        // 如果数据量超过缓冲区大小，直接丢弃
        if (total_rx_size > COM_DATA_BUFFER_LENGTH) {
            ret = MXDBG_ERR_USB_RX_DATA_OVERFLOW;
            goto ERROR;
        }
        // 如果数据量刚开始收到，并且时间戳为 0，开始计时
        else if (total_rx_size > 0 && usb_data_rx_start_time == 0) {
            usb_data_rx_start_time = esp_timer_get_time();
        }

        // 如果数据接收超时，则报错
        if (esp_timer_get_time() - usb_data_rx_start_time > 200000) // 200ms
        {
            ret = MXDBG_ERR_USB_RX_DATA_TIMEOUT;
            goto ERROR;
        }

        // 如果数据量大于等于协议头长度
        if (total_rx_size >= 5) {
            for (int i = 0; i < 5; i++) {
                if (buf[i] == COM_PROTOCOL_HEADER[i]) {
                    header_valid = true;
                } else {
                    header_valid = false;
                    break;
                }
            }

            // 如果协议头不正确
            if (!header_valid) {
                ret = MXDBG_ERR_USB_RX_DATA_HEADER_ERROR;
                goto ERROR;
            } else {
                uint8_t crc_high = 0, crc_low = 0;
                crc16(buf, total_rx_size - 2, &crc_high, &crc_low);

                // 如果 CRC 校验通过
                if ((buf[total_rx_size - 2] == crc_high) && (buf[total_rx_size - 1] == crc_low)) {
                    header_valid = false;
                    usb_data_rx_start_time = 0;
                    xSemaphoreGive(semaphore_duty_call);
                }
            }
        }
    } else {
        ret = MXDBG_ERR_USB_RX_DATA_FAILED;
        goto ERROR;
    }

    return;

ERROR:

    if (xSemaphoreTake(semaphore_usb_total_rx_size, 0) == pdTRUE) {
        total_rx_size = 0;
        xSemaphoreGive(semaphore_usb_total_rx_size);
    }

    usb_data_rx_start_time = 0;
    ESP_LOGE(TAG, "error code: %d", ret);
    memset(buf, 0, sizeof(buf));
    data_pack(NULL, 0, TASK_IDLE, ret);
    xSemaphoreGive(semaphore_duty_call);

    return;
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;

    if (dtr == 1) {
        usb_connected_state = true;
    } else {
        usb_connected_state = false;
    }

    ESP_LOGI(TAG, "Line state changed on channel %d: DTR:%d, RTS:%d", itf, dtr, rts);
}

void tiny_usb_init()
{
    // ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = NULL,
        .hs_configuration_descriptor = NULL,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = NULL,
#endif // TUD_OPT_HIGH_SPEED
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = { .usb_dev = TINYUSB_USBDEV_0,
                                        .cdc_port = TINYUSB_CDC_ACM_0,
                                        .rx_unread_buf_sz = 64,
                                        .callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
                                        .callback_rx_wanted_char = NULL,
                                        .callback_line_state_changed = NULL,
                                        .callback_line_coding_changed = NULL };

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    /* the second way to register a callback */
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(TINYUSB_CDC_ACM_0, CDC_EVENT_LINE_STATE_CHANGED,
                                                     &tinyusb_cdc_line_state_changed_callback));

    // ESP_LOGI(TAG, "USB initialization DONE");
}

void spi_callback(spi_transaction_t *t)
{
    // Should not put any suspendable function here
    // Or it will cause a crash
}

/*
 * COMMAND RECEIVED:
 *
 * | 5     | 1 | 1   | 1 | n bytes | 1 | 2   |
 * |-------|---|-----|---|---------|---|-----|
 * | mxdbg | : | cmd | : | data    | : | crc |
 *
 * 
 * ====================================================================================================================
 * 
 * TASK - I2C WRITE READ:
 * 
 * value format received:
 * | 5     | 1 | 1   | 1 | 1        | 2        | 4 bytes      | 4 bytes     | n bytes         | 1 | 2   |
 * |-------|---|-----|---|----------|----------|--------------|-------------|-----------------|---|-----|
 * | mxdbg | : | cmd | : | i2c_port | slave_id | write_length | read_length | write_data_list | : | crc |
 *
 * value format to be sent if read something:
 * | 5     | 1 | 1   | 1 | 4   | 1 | nbytes data | 1 | 2   |
 * |-------|---|-----|---|-----|---|-------------|---|-----|
 * | mxdbg | : | cmd | : | ret | : | data        | : | crc |
 *
 * value format to be sent if read nothing:
 * | 5     | 1 | 1   | 1 | 4   | 1 | 2   |
 * |-------|---|-----|---|-----|---|-----|
 * | mxdbg | : | cmd | : | ret | : | crc |
 * 
 * 
 * ====================================================================================================================
 * 
 * TASK - I2C CONFIG:
 * 
 * value format received:
 * | 5     | 1 | 1   | 1 | 1    | 4 bytes | 1       | 1       | 1 | 2   |
 * |-------|---|-----|---|------|---------|---------|---------|---|-----|
 * | mxdbg | : | cmd | : | port | freq    | sda pin | scl pin | : | crc |
 *
 * value format to be sent:
 * | 5     | 1 | 1   | 1 | 4   | 1 | 2   |
 * |-------|---|-----|---|-----|---|-----|
 * | mxdbg | : | cmd | : | ret | : | crc |
 * 
 * 
 * ====================================================================================================================
 *
 * TASK - GPIO WRITE READ:
 * 
 * | 5     | 1 | 1   | 1 | 1         | 1       | 1      | 1 | 2   |
 * |-------|---|-----|---|-----------|---------|--------|---|-----|
 * | mxdbg | : | cmd | : | operation | pin num | level  | : | crc |
 *
 * operation: 0 - write, 1 - read
 *
 * value format to be sent if read gpio level:
 * | 5     | 1 | 1   | 1 | 4       | 1 | 1      | 1 | 2   |
 * |-------|---|-----|---|---------|---|--------|---|-----|
 * | mxdbg | : | cmd | : | ret     | : | level  | : | crc |
 *
 * value format to be sent if write gpio level:
 * | 5     | 1 | 1   | 1 | 4   | 1 | 2   |
 * |-------|---|-----|---|-----|---|-----|
 * | mxdbg | : | cmd | : | ret | : | crc |
 *
 *
 * ====================================================================================================================
 * 
 * TASK - GPIO CONFIG:
 * 
 * value format received:
 * | 5     | 1 | 1   | 1 | 1       | 1    | 1      | 1        | 1 | 2   |
 * |-------|---|-----|---|---------|------|--------|----------|---|-----|
 * | mxdbg | : | cmd | : | pin num | mode | pullup | pulldown | : | crc |
 *
 * value format to be sent:
 * | 5     | 1 | 1   | 1 | 4   | 1 | 2   |
 * |-------|---|-----|---|-----|---|-----|
 * | mxdbg | : | cmd | : | ret | : | crc |
 *
 *
 */
void task_duty_call(void *pvParameters)
{
    task_cmd_t task_cmd = TASK_IDLE;
    size_t content_length = 0;
    int ret = 0;

    while (1) {
        if (xSemaphoreTake(semaphore_duty_call, portMAX_DELAY) == pdTRUE) {
            ret = data_unpack(buf, total_rx_size, &task_cmd, com_data_content, &content_length);
            if (ret != 0) {
                memset(buf, 0, sizeof(buf));

                if (xSemaphoreTake(semaphore_usb_total_rx_size, 0) == pdTRUE) {
                    total_rx_size = 0;
                    xSemaphoreGive(semaphore_usb_total_rx_size);
                }

                ESP_LOGE(TAG, "Data unpack failed");
                data_pack(NULL, 0, task_cmd, ret);

                xSemaphoreGive(semaphore_task_notify);
                continue;
            }

            memset(buf, 0, sizeof(buf));

            if (xSemaphoreTake(semaphore_usb_total_rx_size, 0) == pdTRUE) {
                total_rx_size = 0;
                xSemaphoreGive(semaphore_usb_total_rx_size);
            }

            switch (task_cmd) {
                case TASK_IDLE:
                    break;

                case TASK_I2C_WRITE_READ:

                    xSemaphoreGive(semaphore_i2c_write_read);
                    break;

                case TASK_I2C_CONFIG:

                    xSemaphoreGive(semaphore_i2c_config);
                    break;

                case TASK_GPIO_WRITE_READ:

                    xSemaphoreGive(semaphore_gpio_write_read);
                    break;

                case TASK_GPIO_CONFIG:

                    xSemaphoreGive(semaphore_gpio_config);
                    break;

                case TASK_SPI_WRITE_READ:

                    xSemaphoreGive(semaphore_spi_write_read);
                    break;

                case TASK_SPI_CONFIG:

                    xSemaphoreGive(semaphore_spi_config);
                    break;

                case TASK_PWM_RUN_STOP:

                    xSemaphoreGive(semaphore_pwm_run_stop);
                    break;

                case TASK_PWM_CONFIG:

                    xSemaphoreGive(semaphore_pwm_config);
                    break;

                case TASK_SPI_READ_IMAGE:

                    xSemaphoreGive(semaphore_spi_read_image);
                    break;

                case TASK_USB_CONFIG:

                    xSemaphoreGive(semaphore_usb_config);
                    break;

                case TASK_RESET_DEVICE:

                    xSemaphoreGive(semaphore_reset_device);
                    break;

                default:

                    ret = MXDBG_ERR_UNKNOWN_TASK;
                    ESP_LOGE(TAG, "Unknown task command");
                    data_pack(NULL, 0, task_cmd, ret);
                    xSemaphoreGive(semaphore_task_notify);
                    break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*-----------------------------------------------------------------------------
 * TASK FUNCTIONS
 *---------------------------------------------------------------------------*/

void task_notify(void *pvParameters)
{
    while (1) {
        if (xSemaphoreTake(semaphore_task_notify, portMAX_DELAY) == pdTRUE) {
            if (usb_connected_state) {
                // ESP_LOG_BUFFER_HEXDUMP(TAG, com_data_send_buffer, tx_size, ESP_LOG_INFO);

                tinyusb_cdcacm_write_queue(usb_cdc_itf, com_data_send_buffer, tx_size);
                tinyusb_cdcacm_write_flush(usb_cdc_itf, 0);

                tx_size = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void task_i2c_write_read(void *pvParameters)
{
    int ret = 0;
    uint8_t i2c_port = 0;
    uint16_t slave_id = 0x00;
    size_t write_length = 0, read_length = 0;
    uint8_t *write_data_list = NULL;
    uint8_t *read_data_list = NULL;

    while (1) {
        if (xSemaphoreTake(semaphore_i2c_write_read, portMAX_DELAY) == pdTRUE) {
            // ESP_LOGI(TAG, "I2C write read task");

            memset(com_data_send_buffer, 0, sizeof(com_data_send_buffer));

            i2c_port = com_data_content[0];
            slave_id = DATA_SYNTHESIS_2_BYTES(&(com_data_content[1]));
            write_length = DATA_SYNTHESIS_4_BYTES(&(com_data_content[3]));
            read_length = DATA_SYNTHESIS_4_BYTES(&(com_data_content[7]));

            // ESP_LOGI(TAG, "I2C port: %d, slave id: %d, write length: %d, read length: %d", i2c_port, slave_id,
            //          write_length, read_length);

            if (write_length > 0) {
                write_data_list = (uint8_t *)malloc(write_length);
                if (write_data_list) {
                    memcpy(write_data_list, &com_data_content[11], write_length);
                } else {
                    ESP_LOGE(TAG, "Memory allocation failed");
                }

                if (read_length > 0) {
                    read_data_list = (uint8_t *)malloc(read_length);
                    if (read_data_list) {
                        // Write and read data
                        ret = i2c_master_write_read_device(i2c_port, slave_id, write_data_list, write_length,
                                                           read_data_list, read_length, 1000 / portTICK_PERIOD_MS);
                        if (ret != ESP_OK) {
                            ESP_LOGE(TAG, "I2C write read failed, ret: %d", ret);
                            ret = MXDBG_ERR_I2C_WRITE_READ_FAILED;
                        }
                    } else {
                        ESP_LOGE(TAG, "Memory allocation failed, ret: %d", ret);
                        ret = ESPRESSIF_ERR_NO_MEM;
                    }
                } else {
                    // Write data only
                    ret = i2c_master_write_to_device(i2c_port, slave_id, write_data_list, write_length,
                                                     1000 / portTICK_PERIOD_MS);
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "I2C write failed, ret: %d", ret);
                        ret = MXDBG_ERR_I2C_WRITE_FAILED;
                    }
                }
            } else {
                if (read_length > 0) {
                    read_data_list = (uint8_t *)malloc(read_length);
                    if (read_data_list) {
                        // Read data only
                        ret = i2c_master_read_from_device(i2c_port, slave_id, read_data_list, read_length,
                                                          1000 / portTICK_PERIOD_MS);
                        if (ret != ESP_OK) {
                            ESP_LOGE(TAG, "I2C read failed");
                            ret = MXDBG_ERR_I2C_READ_FAILED;
                        }
                    } else {
                        ESP_LOGE(TAG, "Memory allocation failed");
                        ret = ESPRESSIF_ERR_NO_MEM;
                    }
                } else {
                    // Invalid I2C operation
                    ESP_LOGE(TAG, "Invalid I2C operation");

                    ret = MXDBG_ERR_I2C_INVALID_OPERATION;
                    data_pack(NULL, 0, TASK_I2C_WRITE_READ, ret);
                    xSemaphoreGive(semaphore_task_notify);

                    continue;
                }
            }

            data_pack(read_data_list, read_length, TASK_I2C_WRITE_READ, ret);

            //--------------------------------------------------------------------------------

            // 确保在释放前进行非空检查
            if (write_data_list) {
                free(write_data_list);
                write_data_list = NULL; // 避免悬挂指针
            }
            if (read_data_list) {
                free(read_data_list);
                read_data_list = NULL; // 避免悬挂指针
            }

            xSemaphoreGive(semaphore_task_notify);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void task_i2c_config(void *pvParameters)
{
    int ret = 0;

    while (1) {
        if (xSemaphoreTake(semaphore_i2c_config, portMAX_DELAY) == pdTRUE) {
            uint8_t i2c_port = com_data_content[0];
            uint32_t freq = DATA_SYNTHESIS_4_BYTES(&(com_data_content[1]));
            uint8_t sda_pin = com_data_content[5];
            uint8_t scl_pin = com_data_content[6];
            bool sda_pullup = (com_data_content[7] & 0xF0) >> 4;
            bool scl_pullup = (com_data_content[7] & 0x0F) >> 0;

            ret = i2c_deinit(i2c_port);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "I2C deinit failed");
                ret = MXDBG_ERR_I2C_DEINIT_FAILED;
            } else {
                i2c_config_t *i2c_conf = i2c_port == 0 ? (&i2c_conf0) : (&i2c_conf1);

                i2c_conf->mode = I2C_MODE_MASTER;
                i2c_conf->sda_io_num = sda_pin;
                i2c_conf->scl_io_num = scl_pin;
                i2c_conf->sda_pullup_en = sda_pullup;
                i2c_conf->scl_pullup_en = scl_pullup;
                i2c_conf->master.clk_speed = freq;

                ret = i2c_init(i2c_port);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "I2C config failed");
                    ret = MXDBG_ERR_I2C_INIT_FAILED;
                }
            }

            data_pack(NULL, 0, TASK_I2C_CONFIG, ret);

            xSemaphoreGive(semaphore_task_notify);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void task_gpio_write_read(void *pvParameters)
{
    int ret = 0;

    uint8_t operation = 0;
    uint8_t pin_num = 0;
    uint8_t level = 0;

    while (1) {
        if (xSemaphoreTake(semaphore_gpio_write_read, portMAX_DELAY) == pdTRUE) {
            operation = com_data_content[0];
            pin_num = com_data_content[1];
            level = com_data_content[2];

            if (operation == 0) // write
            {
                ret = gpio_set_level((gpio_num_t)pin_num, level);
                data_pack(NULL, 0, TASK_GPIO_WRITE_READ, ret);
            } else if (operation == 1) // read
            {
                level = gpio_get_level((gpio_num_t)pin_num);
                data_pack(&level, 1, TASK_GPIO_WRITE_READ, 0);
            }

            xSemaphoreGive(semaphore_task_notify);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void task_gpio_config(void *pvParameters)
{
    int ret = 0;

    while (1) {
        if (xSemaphoreTake(semaphore_gpio_config, portMAX_DELAY) == pdTRUE) {
            uint8_t pin_num = com_data_content[0];
            uint8_t mode = com_data_content[1];
            uint8_t pullup = com_data_content[2];
            uint8_t pulldown = com_data_content[3];

            gpio_config_t io_conf;
            io_conf.pin_bit_mask = (1ULL << pin_num);
            io_conf.mode = (gpio_mode_t)mode;
            io_conf.pull_up_en = (gpio_pullup_t)pullup;
            io_conf.pull_down_en = (gpio_pulldown_t)pulldown;
            io_conf.intr_type = GPIO_INTR_DISABLE;

            ret = gpio_config(&io_conf);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "GPIO config failed");
                ret = MXDBG_ERR_GPIO_CONFIG_FAILED;
            }

            data_pack(NULL, 0, TASK_GPIO_CONFIG, ret);
            xSemaphoreGive(semaphore_task_notify);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void task_spi_write_read(void *pvParameters)
{
    int ret;
    spi_transaction_t t;

    uint32_t write_len = 0;
    uint32_t read_len = 0;
    uint8_t *write_buffer = NULL;
    uint8_t *read_buffer = NULL;
    bool critical_mode = false;
    uint8_t justice = 0x00;
    uint8_t justice_index = 0;
    uint8_t examine_period = 1;
    uint8_t timeout = 0;
    uint8_t counts = 0;
    bool examine_result = false;

    while (1) {
        if (xSemaphoreTake(semaphore_spi_write_read, portMAX_DELAY) == pdTRUE) {
            memset(&t, 0, sizeof(t));

            write_len = DATA_SYNTHESIS_4_BYTES(&(com_data_content[0]));
            read_len = DATA_SYNTHESIS_4_BYTES(&(com_data_content[4]));

            if (write_len <= 0 && read_len <= 0) {
                ret = MXDBG_ERR_SPI_INVALID_OPERATION;
                ESP_LOGE(TAG, "Wrong write_len and read_len");
                data_pack(NULL, 0, TASK_SPI_WRITE_READ, ret);
                xSemaphoreGive(semaphore_task_notify);
                continue;
            }

            if (read_len > write_len) {
                ret = MXDBG_ERR_SPI_READ_LEN_TOO_LONG;
                ESP_LOGE(TAG, "Read length should not be larger than write length");
                data_pack(NULL, 0, TASK_SPI_WRITE_READ, ret);
                xSemaphoreGive(semaphore_task_notify);
                continue;
            }

            critical_mode = com_data_content[8 + write_len] & 0x01 ? true : false;
            justice = com_data_content[9 + write_len];
            justice_index = com_data_content[10 + write_len];
            examine_period = com_data_content[11 + write_len];
            timeout = com_data_content[12 + write_len];

            if (critical_mode)
            {
                ESP_LOGI(TAG, "Critical mode: %s, justice: 0x%02X, justice_index: %d, examine_period: %d, timeout: %d",critical_mode ? "True" : "False",  justice, justice_index, examine_period, timeout);
            }

            spi_device_acquire_bus(spi, portMAX_DELAY);

            // in half-duplex mode
            if ((devcfg.flags & 0x00000010) >> 4) {
                if (write_len > 0) {
                    t.flags = SPI_TRANS_CS_KEEP_ACTIVE;

                    t.length = write_len * 8; // transaction length is in bits
                    write_buffer = (uint8_t *)heap_caps_malloc(write_len, MALLOC_CAP_DMA);
                    if (write_buffer == NULL) {
                        ret = ESPRESSIF_ERR_NO_MEM;
                        ESP_LOGE(TAG, "Memory allocation failed");
                        data_pack(NULL, 0, TASK_SPI_WRITE_READ, ret);
                        xSemaphoreGive(semaphore_task_notify);
                        continue;
                    }
                    memcpy(write_buffer, &com_data_content[8], write_len);

                    t.tx_buffer = write_buffer;

                    ret = spi_device_transmit(spi, &t); // Transmit!
                }

                if (read_len > 0) {
                    // memset(&t, 0, sizeof(t));
                    t.tx_buffer = NULL;
                    t.length = 0;

                    read_buffer = (uint8_t *)heap_caps_malloc(read_len, MALLOC_CAP_DMA);
                    if (read_buffer == NULL) {
                        ret = ESPRESSIF_ERR_NO_MEM;
                        ESP_LOGE(TAG, "Memory allocation failed");
                        data_pack(NULL, 0, TASK_SPI_WRITE_READ, ret);
                        xSemaphoreGive(semaphore_task_notify);
                        continue;
                    }
                    memset(read_buffer, 0, read_len);

                    t.rxlength = read_len * 8;
                    t.rx_buffer = read_buffer;
                    ret = spi_device_transmit(spi, &t);
                }

                // depress the single ringing in MOSI line after the transaction
                spi_transaction_t t_null = { 0 };
                spi_device_transmit(spi, &t_null);

            }
            // not in half-duplex mode
            else {
                if (write_len > 0) {
                    t.length = write_len * 8; // transaction length is in bits
                    write_buffer = (uint8_t *)heap_caps_malloc(write_len, MALLOC_CAP_DMA);
                    if (write_buffer == NULL) {
                        ret = ESPRESSIF_ERR_NO_MEM;
                        ESP_LOGE(TAG, "Memory allocation failed");
                        data_pack(NULL, 0, TASK_SPI_WRITE_READ, ret);
                        xSemaphoreGive(semaphore_task_notify);
                        continue;
                    }
                    memcpy(write_buffer, &com_data_content[8], write_len);

                    t.tx_buffer = write_buffer;
                }

                if (read_len > 0) {
                    t.rxlength = read_len * 8;
                    read_buffer = (uint8_t *)heap_caps_malloc(read_len + 1, MALLOC_CAP_DMA);
                    if (read_buffer == NULL) {
                        ret = ESPRESSIF_ERR_NO_MEM;
                        ESP_LOGE(TAG, "Memory allocation failed");
                        data_pack(NULL, 0, TASK_SPI_WRITE_READ, ret);
                        xSemaphoreGive(semaphore_task_notify);
                        continue;
                    }
                    memset(read_buffer, 0, read_len);

                    t.rx_buffer = read_buffer;
                }

                if (critical_mode)
                {

                    counts = timeout / examine_period;

                    for (uint32_t i = 0; i < counts; i+=examine_period)
                    {
                        ret = spi_device_transmit(spi, &t); // Transmit and receive
                        if (ret != ESP_OK) {
                            ret = MXDBG_ERR_SPI_TRANSACTION_FAILED;
                            ESP_LOGE(TAG, "SPI transaction failed");
                            data_pack(NULL, 0, TASK_SPI_WRITE_READ, ret);

                            if (write_len > 0) {
                                heap_caps_free(write_buffer);
                                write_buffer = NULL;
                            }

                            if (read_len > 0) {
                                heap_caps_free(read_buffer);
                                read_buffer = NULL;
                            }

                            xSemaphoreGive(semaphore_task_notify);
                            continue;
                        }
                        else
                        {
                            esp_rom_delay_us(examine_period * 1000);

                            if (read_buffer[justice_index] == justice)
                            {
                                examine_result = true;
                                break;
                            }
                        }

                    }
                }
                else  // not in critical mode
                {
                    ret = spi_device_transmit(spi, &t); // Transmit and receive
                    if (ret != ESP_OK) {
                        ret = MXDBG_ERR_SPI_TRANSACTION_FAILED;
                        ESP_LOGE(TAG, "SPI transaction failed");
                        data_pack(NULL, 0, TASK_SPI_WRITE_READ, ret);

                        if (write_len > 0) {
                            heap_caps_free(write_buffer);
                            write_buffer = NULL;
                        }

                        if (read_len > 0) {
                            heap_caps_free(read_buffer);
                            read_buffer = NULL;
                        }

                        xSemaphoreGive(semaphore_task_notify);
                        continue;
                    }
                }

            }

            spi_device_release_bus(spi); // When using SPI

            if (critical_mode)
            {
                ((uint8_t *)t.rx_buffer)[read_len] = examine_result ? 0x01 : 0x00;
                data_pack(t.rx_buffer, read_len + 1, TASK_SPI_WRITE_READ, 0);
            }
            else
            {
                data_pack(t.rx_buffer, read_len, TASK_SPI_WRITE_READ, 0);
            }
            xSemaphoreGive(semaphore_task_notify);

            if (write_len > 0) {
                heap_caps_free(write_buffer);
                write_buffer = NULL;
            }

            if (read_len > 0) {
                heap_caps_free(read_buffer);
                read_buffer = NULL;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void task_spi_config(void *pvParameters)
{
    esp_err_t ret = 0;
    while (1) {
        if (xSemaphoreTake(semaphore_spi_config, portMAX_DELAY) == pdTRUE) {
            uint8_t *pCOM = &(com_data_content[0]);

            int mosi_io_num = (int)DATA_SYNTHESIS_4_BYTES(pCOM);
            int miso_io_num = (int)DATA_SYNTHESIS_4_BYTES(pCOM + 4);
            int sclk_io_num = (int)DATA_SYNTHESIS_4_BYTES(pCOM + 8);
            int quadwp_io_num = (int)DATA_SYNTHESIS_4_BYTES(pCOM + 12);
            int quadhd_io_num = (int)DATA_SYNTHESIS_4_BYTES(pCOM + 16);
            int data4_io_num = (int)DATA_SYNTHESIS_4_BYTES(pCOM + 20);
            int data5_io_num = (int)DATA_SYNTHESIS_4_BYTES(pCOM + 24);
            int data6_io_num = (int)DATA_SYNTHESIS_4_BYTES(pCOM + 28);
            int data7_io_num = (int)DATA_SYNTHESIS_4_BYTES(pCOM + 32);
            uint16_t max_transfer_sz = DATA_SYNTHESIS_2_BYTES(pCOM + 36);
            uint32_t common_bus_flags = DATA_SYNTHESIS_4_BYTES(pCOM + 38);
            uint8_t isr_cpu_id = pCOM[42];
            int intr_flags = DATA_SYNTHESIS_4_BYTES(pCOM + 43);
            uint8_t command_bits = pCOM[47];
            uint8_t address_bits = pCOM[48];
            uint8_t dummy_bits = pCOM[49];
            uint8_t mode = pCOM[50];
            uint16_t duty_cycle_pos = DATA_SYNTHESIS_2_BYTES(pCOM + 51);
            uint16_t cs_ena_pretrans = DATA_SYNTHESIS_2_BYTES(pCOM + 53);
            uint8_t cs_ena_posttrans = pCOM[55];
            int clock_speed_hz = DATA_SYNTHESIS_4_BYTES(pCOM + 56);
            int input_delay_ns = DATA_SYNTHESIS_4_BYTES(pCOM + 60);
            int spics_io_num = DATA_SYNTHESIS_4_BYTES(pCOM + 64);
            uint32_t device_interface_flags = DATA_SYNTHESIS_4_BYTES(pCOM + 68);
            int queue_size = DATA_SYNTHESIS_4_BYTES(pCOM + 72);

            ret = spi_deinit();
            if (ret != ESP_OK) {
                ret = MXDBG_ERR_SPI_DEINIT_FAILED;
                ESP_LOGE(TAG, "Deinit spi failed.");
                data_pack(NULL, 0, TASK_SPI_CONFIG, ret);
                xSemaphoreGive(semaphore_task_notify);
                continue;
            }

            // ESP_LOGI(TAG, "clock speed hz: %d", clock_speed_hz);
            // ESP_LOGI(TAG, "MISO_IO_NUM: %d", miso_io_num);

            buscfg.data0_io_num = mosi_io_num;
            buscfg.data1_io_num = miso_io_num;
            buscfg.sclk_io_num = sclk_io_num;
            buscfg.data2_io_num = quadwp_io_num;
            buscfg.data3_io_num = quadhd_io_num;
            buscfg.data4_io_num = data4_io_num;
            buscfg.data5_io_num = data5_io_num;
            buscfg.data6_io_num = data6_io_num;
            buscfg.data7_io_num = data7_io_num;
            buscfg.max_transfer_sz = max_transfer_sz;
            buscfg.flags = common_bus_flags;
            buscfg.isr_cpu_id = isr_cpu_id;
            buscfg.intr_flags = intr_flags;

            devcfg.command_bits = command_bits;
            devcfg.address_bits = address_bits;
            devcfg.dummy_bits = dummy_bits;
            devcfg.mode = mode;
            devcfg.duty_cycle_pos = duty_cycle_pos;
            devcfg.cs_ena_pretrans = cs_ena_pretrans;
            devcfg.cs_ena_posttrans = cs_ena_posttrans;
            devcfg.clock_speed_hz = clock_speed_hz;
            devcfg.input_delay_ns = input_delay_ns;
            devcfg.spics_io_num = spics_io_num;
            devcfg.flags = device_interface_flags;
            devcfg.queue_size = queue_size;

            ret = spi_init();
            if (ret != ESP_OK) {
                ret = MXDBG_ERR_SPI_INIT_FAILED;
                ESP_LOGE(TAG, "Initialize spi failed.");
                data_pack(NULL, 0, TASK_SPI_CONFIG, ret);
                xSemaphoreGive(semaphore_task_notify);
                continue;
            }

            data_pack(NULL, 0, TASK_SPI_CONFIG, 0);
            xSemaphoreGive(semaphore_task_notify);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void task_pwm_run_stop(void *pvParameters)
{
    uint8_t channel = 0;
    bool pwm_run_state = false;
    pwm_channel_t *pPwmChannel = NULL;

    int ret = 0;

    while (1) {
        if (xSemaphoreTake(semaphore_pwm_run_stop, portMAX_DELAY) == pdTRUE) {
            channel = com_data_content[0];
            pwm_run_state = com_data_content[1] != 0 ? true : false;

            ESP_LOGI(TAG, "Channel: %d; Run state: %s", channel, pwm_run_state ? "True" : "False");

            pPwmChannel = &(pwm_channels[channel]);

            if (pwm_run_state) {              // True
                if (pPwmChannel->run_state) { // True
                    ESP_LOGE(TAG, "PWM is already running");
                    ret = MXDBG_ERR_PWM_RUN_ALREADY;
                } else {
                    // make it run
                    ESP_ERROR_CHECK(
                        mcpwm_comparator_set_compare_value(pPwmChannel->comparator, pPwmChannel->duty_cycle_ticks));

                    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                        pPwmChannel->generator,
                        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
                                                     MCPWM_GEN_ACTION_HIGH)));

                    pPwmChannel->compare_action_start.comparator = pPwmChannel->comparator;
                    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(pPwmChannel->generator,
                                                                                pPwmChannel->compare_action_start));

                    ret = ESP_OK;

                    if (xSemaphoreTake(semaphore_pwm_running_state, 0) == pdTRUE) {
                        pPwmChannel->run_state = true;
                        xSemaphoreGive(semaphore_pwm_running_state);
                    }
                }
            } else {
                if (pPwmChannel->run_state) {
                    // make it stop
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(pPwmChannel->comparator, 0));

                    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                        pPwmChannel->generator,
                        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
                                                     MCPWM_GEN_ACTION_LOW)));

                    pPwmChannel->compare_action_stop.comparator = pPwmChannel->comparator;
                    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(pPwmChannel->generator,
                                                                                pPwmChannel->compare_action_stop));

                    ret = ESP_OK;

                    if (xSemaphoreTake(semaphore_pwm_running_state, 0) == pdTRUE) {
                        pPwmChannel->run_state = false;
                        xSemaphoreGive(semaphore_pwm_running_state);
                    }

                } else {
                    ESP_LOGE(TAG, "PWM is already stopped");
                    ret = MXDBG_ERR_PWM_STOP_ALREADY;
                }
            }

            data_pack(NULL, 0, TASK_PWM_RUN_STOP, ret);
            xSemaphoreGive(semaphore_task_notify);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    pwm_deinit(&(pwm_channels[channel]));
    vTaskDelete(NULL);
}

void task_pwm_config(void *pvParameters)
{
    int ret = 0;

    uint8_t channel = 0;
    uint8_t gen_gpio_num = 0;
    uint32_t period_ticks = 0;
    uint32_t duty_cycle_ticks = 0;
    uint32_t resolution_hz = 0;

    pwm_channel_t *pPwmChannel = NULL;

    while (1) {
        if (xSemaphoreTake(semaphore_pwm_config, portMAX_DELAY) == pdTRUE) {
            channel = com_data_content[0];
            gen_gpio_num = com_data_content[1];
            period_ticks = DATA_SYNTHESIS_4_BYTES(&(com_data_content[2]));
            duty_cycle_ticks = DATA_SYNTHESIS_4_BYTES(&(com_data_content[6]));
            resolution_hz = DATA_SYNTHESIS_4_BYTES(&(com_data_content[10]));

            ESP_LOGI(TAG, "Channel: %d; GPIO: %d; period_ticks: %ld; duty_cycle_ticks: %ld", channel, gen_gpio_num,
                     period_ticks, duty_cycle_ticks);

            if (channel >= MAX_PWM_CHANNELS) {
                ESP_LOGE(TAG, "Invalid PWM channel");
                data_pack(NULL, 0, TASK_PWM_CONFIG, MXDBG_ERR_PWM_INVALID_CHANNEL);
                xSemaphoreGive(semaphore_task_notify);
                continue;
            } else {
                pPwmChannel = &(pwm_channels[channel]);
            }

            if (period_ticks == 0 || duty_cycle_ticks == 0) {
                ESP_LOGE(TAG, "Invalid PWM configuration");
                data_pack(NULL, 0, TASK_PWM_CONFIG, MXDBG_ERR_PWM_INVALID_PARAMS);
                xSemaphoreGive(semaphore_task_notify);
                continue;
            }

            if (duty_cycle_ticks > period_ticks) {
                ESP_LOGE(TAG, "Duty cycle is larger than period");
                data_pack(NULL, 0, TASK_PWM_CONFIG, MXDBG_ERR_PWM_INVALID_DUTY_CYCLES);
                xSemaphoreGive(semaphore_task_notify);
                continue;
            }

            if (xSemaphoreTake(semaphore_pwm_running_state, 0) == pdTRUE) {
                if (pwm_channels[channel].run_state) {
                    pwm_channels[channel].run_state = false;
                }

                // 重新配置 PWM
                pwm_deinit(&(pwm_channels[channel]));

                pPwmChannel->timer_config.resolution_hz = resolution_hz;
                pPwmChannel->timer_config.period_ticks = period_ticks;
                pPwmChannel->generator_config.gen_gpio_num = gen_gpio_num;
                pPwmChannel->duty_cycle_ticks = duty_cycle_ticks;

                pwm_init(&(pwm_channels[channel]));
                data_pack(NULL, 0, TASK_PWM_CONFIG, 0);
                xSemaphoreGive(semaphore_pwm_running_state);
            } else {
                ret = MXDBG_ERR_PWM_CONFIG_FAILED;
                ESP_LOGE(TAG, "Time out to take semaphore_pwm_running_state, please retry again.");
                data_pack(NULL, 0, TASK_PWM_CONFIG, ret);
            }

            xSemaphoreGive(semaphore_task_notify);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelete(NULL);
}

void task_spi_read_image(void *pvParameters)
{
    int ret = 0;
    
    extern int get_raw_image();
    extern void paw3311dw_init();
    extern uint8_t image_data[];

    bool chip_init = false;

    while (1) {
        if (xSemaphoreTake(semaphore_spi_read_image, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "SPI read image task");

            spi_device_acquire_bus(spi, portMAX_DELAY);

            if (chip_init == false) {
                paw3311dw_init();
                chip_init = true;
            }

            if (get_raw_image() != 0) {
                ret = MXDBG_ERR_SPI_READ_IMAGE_FAILED;
                ESP_LOGE(TAG, "Get raw image failed");
                data_pack(NULL, 0, TASK_SPI_READ_IMAGE, ret);
            } else {
                ret = ESP_OK;
                ESP_LOGI(TAG, "Get raw image completed");
                data_pack(image_data, 900, TASK_SPI_READ_IMAGE, ret);
            }

            spi_device_release_bus(spi); // When using SPI
            xSemaphoreGive(semaphore_task_notify);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelete(NULL);
}

void task_usb_config(void *pvParameters)
{
    while (1) {
        if (xSemaphoreTake(semaphore_usb_config, portMAX_DELAY) == pdTRUE) {
            crc_enable = com_data_content[0] != 0 ? true : false;

            data_pack(NULL, 0, TASK_USB_CONFIG, 0);
            xSemaphoreGive(semaphore_task_notify);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*-----------------------------------------------------------------------------
 * MAIN FUNCTION
 *---------------------------------------------------------------------------*/

void app_main()
{
    pwm_channels[0] = (pwm_channel_t){
        .timer = NULL,
        .operator = NULL,
        .generator = NULL,
        .comparator = NULL,
        .timer_config = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 80000000, // 80MHz
            .period_ticks = 100,      // 周期计数值，对应频率为 800KHz
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        },
        .operator_config = {
            .group_id = 0,
        },
        .generator_config = {
            .gen_gpio_num = 16,
        },
        .comparator_config = {
            .flags.update_cmp_on_tez = true, // 在计数器等于零时更新比较值
        },
        .compare_action_start = {
            .direction = MCPWM_TIMER_DIRECTION_UP,
            .comparator = NULL,
            .action = MCPWM_GEN_ACTION_LOW,
        },
        .compare_action_stop = {
            .direction = MCPWM_TIMER_DIRECTION_UP,
            .comparator = NULL,
            .action = MCPWM_GEN_ACTION_LOW,
        },
        .duty_cycle_ticks = 25,
        .run_state = false
    };
    pwm_channels[1] = (pwm_channel_t){
        .timer = NULL,
        .operator = NULL,
        .generator = NULL,
        .comparator = NULL,
        .timer_config = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 80000000, // 80MHz
            .period_ticks = 1000,      // 周期计数值，对应频率为 80KHz
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        },
        .operator_config = {
            .group_id = 0,
        },
        .generator_config = {
            .gen_gpio_num = 17,
        },
        .comparator_config = {
            .flags.update_cmp_on_tez = true, // 在计数器等于零时更新比较值
        },
        .compare_action_start = {
            .direction = MCPWM_TIMER_DIRECTION_UP,
            .comparator = NULL,
            .action = MCPWM_GEN_ACTION_LOW,
        },
        .compare_action_stop = {
            .direction = MCPWM_TIMER_DIRECTION_UP,
            .comparator = NULL,
            .action = MCPWM_GEN_ACTION_LOW,
        },
        .duty_cycle_ticks = 25,
        .run_state = false
    };
    pwm_channels[2] = (pwm_channel_t){
        .timer = NULL,
        .operator = NULL,
        .generator = NULL,
        .comparator = NULL,
        .timer_config = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 80000000, // 80MHz
            .period_ticks = 1000,      // 周期计数值，对应频率为 80KHz
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        },
        .operator_config = {
            .group_id = 0,
        },
        .generator_config = {
            .gen_gpio_num = 18,
        },
        .comparator_config = {
            .flags.update_cmp_on_tez = true, // 在计数器等于零时更新比较值
        },
        .compare_action_start = {
            .direction = MCPWM_TIMER_DIRECTION_UP,
            .comparator = NULL,
            .action = MCPWM_GEN_ACTION_LOW,
        },
        .compare_action_stop = {
            .direction = MCPWM_TIMER_DIRECTION_UP,
            .comparator = NULL,
            .action = MCPWM_GEN_ACTION_LOW,
        },
        .duty_cycle_ticks = 25,
        .run_state = false
    };

    tiny_usb_init();
    i2c_init(0);
    i2c_init(1);
    spi_init();
    pwm_init(&(pwm_channels[0]));
    pwm_init(&(pwm_channels[1]));
    pwm_init(&(pwm_channels[2]));

    semaphore_duty_call = xSemaphoreCreateBinary();
    if (semaphore_duty_call == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore_duty_call");
    }

    semaphore_i2c_write_read = xSemaphoreCreateBinary();
    if (semaphore_i2c_write_read == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore_i2c_write_read");
    }

    semaphore_task_notify = xSemaphoreCreateBinary();
    if (semaphore_task_notify == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore_task_notify");
    }

    semaphore_gpio_write_read = xSemaphoreCreateBinary();
    if (semaphore_gpio_write_read == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore_gpio_write_read");
    }

    semaphore_gpio_config = xSemaphoreCreateBinary();
    if (semaphore_gpio_config == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore_gpio_config");
    }

    semaphore_i2c_config = xSemaphoreCreateBinary();
    if (semaphore_i2c_config == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore_i2c_config");
    }

    semaphore_spi_write_read = xSemaphoreCreateBinary();
    if (semaphore_spi_write_read == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore_spi_write_read");
    }

    semaphore_spi_config = xSemaphoreCreateBinary();
    if (semaphore_spi_config == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore_spi_config");
    }

    semaphore_pwm_run_stop = xSemaphoreCreateBinary();
    if (semaphore_pwm_run_stop == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore_pwm_run_stop");
    }

    semaphore_pwm_config = xSemaphoreCreateBinary();
    if (semaphore_pwm_config == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore_pwm_config");
    }

    semaphore_pwm_running_state = xSemaphoreCreateMutex();
    if (semaphore_pwm_running_state == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore_pwm_running_state");
    }

    semaphore_usb_total_rx_size = xSemaphoreCreateMutex();
    if (semaphore_usb_total_rx_size == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore_usb_total_rx_size");
    }

    semaphore_spi_read_image = xSemaphoreCreateBinary();
    if (semaphore_spi_read_image == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore_spi_read_image");
    }

    semaphore_usb_config = xSemaphoreCreateBinary();
    if (semaphore_usb_config == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore_usb_config");
    }

    semaphore_reset_device = xSemaphoreCreateBinary();
    if (semaphore_reset_device == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore_reset_device");
    }

    xTaskCreate(task_duty_call, "task_duty_call", 4096, NULL, 5, NULL);
    xTaskCreate(task_notify, "task_notify", 4096, NULL, 6, NULL);

    xTaskCreate(task_i2c_write_read, "task_i2c_write_read", 8192, NULL, 4, NULL);
    xTaskCreate(task_i2c_config, "task_i2c_config", 2048, NULL, 4, NULL);

    xTaskCreate(task_gpio_write_read, "task_gpio_write_read", 2048, NULL, 4, NULL);
    xTaskCreate(task_gpio_config, "task_gpio_config", 4096, NULL, 4, NULL);

    xTaskCreate(task_spi_write_read, "task_spi_write_read", 8192, NULL, 4, NULL);
    xTaskCreate(task_spi_config, "task_spi_config", 4096, NULL, 4, NULL);

    xTaskCreate(task_pwm_run_stop, "task_pwm_run_stop", 4096, NULL, 4, NULL);
    xTaskCreate(task_pwm_config, "task_pwm_config", 4096, NULL, 4, NULL);

    xTaskCreate(task_spi_read_image, "task_spi_read_image", 8192, NULL, 4, NULL);

    xTaskCreate(task_usb_config, "task_usb_config", 1024, NULL, 4, NULL);

    // ESP_LOGI(TAG, "Free DMA heap size: %d bytes", heap_caps_get_free_size(MALLOC_CAP_DMA));

    while (1) {

        if (xSemaphoreTake(semaphore_reset_device, portMAX_DELAY) == pdTRUE) {
            ESP_LOGW(TAG, "Reset device");

            pwm_deinit(&(pwm_channels[0]));
            pwm_deinit(&(pwm_channels[1]));
            pwm_deinit(&(pwm_channels[2]));

            spi_deinit();
            i2c_deinit(0);
            i2c_deinit(1);

            data_pack(NULL, 0, TASK_RESET_DEVICE, 0);
            xSemaphoreGive(semaphore_task_notify);

            esp_restart();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    pwm_deinit(&(pwm_channels[0]));
    pwm_deinit(&(pwm_channels[1]));
    pwm_deinit(&(pwm_channels[2]));

    spi_deinit();
    i2c_deinit(0);
    i2c_deinit(1);
}