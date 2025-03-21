
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "driver/spi_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"

#define PAW3395 1

#ifdef PAW3395
#include "paw3395.h"
#endif

extern spi_device_handle_t spi;

static const char *TAG = "PAW33xx";

uint8_t image_data[2000];

static int paw33xx_write_read(uint8_t *write_list, size_t write_len, uint8_t *read_list, size_t read_len)
{
    spi_transaction_t t;
    int ret = 0;

    memset(&t, 0, sizeof(t));
    t.flags = SPI_TRANS_CS_KEEP_ACTIVE;

    if (write_len > 0 && write_list != NULL) {
        t.length = write_len * 8; // transaction length is in bits
        t.tx_buffer = write_list;

        if (read_len == 0) {
            write_list[0] |= 0x80;
        }
    }

    if (read_len > 0 && read_list != NULL) {
        t.rx_buffer = read_list;
        t.rxlength = read_len * 8;
    }

    ret = spi_device_transmit(spi, &t);
    if (ret) {
        ESP_LOGE(TAG, "SPI transaction failed");
        return -1;
    }

    spi_transaction_t t_null = { 0 };
    spi_device_transmit(spi, &t_null);

    return 0;
}

int paw33xx_get_image(uint16_t width, uint16_t height)
{
    int max_retry_times = 1000;
    size_t image_size = width * height;

    memset(image_data, 0, sizeof(image_data));

    const uint8_t reg_list_1[][2] = {
        { 0x7F, 0x00 }, { 0x40, 0x80 }, 
    };

    const uint8_t reg_list_2[][2] = {
        { 0x50, 0x01}, {0x55, 0x04}, {0x58, 0xFF}
    };

    const uint8_t reg_list_3[][2] = {
        { 0x40, 0x00 }, { 0x50, 0x00 }, { 0x55, 0x00 }
    };

    uint8_t reg_address_temp[2] = {0x02, 0x00};
    uint8_t reg_value_temp[2] = {0};

    //! STEP 1
    for (size_t i = 0; i < sizeof(reg_list_1) / sizeof(reg_list_1[0]); i++) {
        paw33xx_write_read((uint8_t *)reg_list_1[i], sizeof(reg_list_1[i]), NULL, 0);
    }

    //! STEP 2
    ESP_LOGI(TAG, "Wait for Motion available....");
    
    // Continuously read register 0x02 until getting both bit 1 and bit 0 are 0
    while (max_retry_times) {
        paw33xx_write_read(reg_address_temp, 2, reg_value_temp, 2);
        if ((reg_value_temp[1] & 0x03) == 0) {
            break;
        }
        max_retry_times--;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (!max_retry_times) {
        ESP_LOGE(TAG, "Read register 0x02 failed");
        return -1;
    }

    //! STEP 3
    for (size_t i = 0; i < sizeof(reg_list_2) / sizeof(reg_list_2[0]); i++) {
        paw33xx_write_read((uint8_t *)reg_list_2[i], sizeof(reg_list_2[i]), NULL, 0);
    }


    //! STEP 4 (read image data)
    ESP_LOGI(TAG, "Wait for Image available....");

    max_retry_times = 200000;
    reg_address_temp[0] = 0x59;

    // Continuously read register 0x59 until getting bit 7 as “1”.
    while (max_retry_times) {
        paw33xx_write_read(reg_address_temp, 2, reg_value_temp, 0);

        if ((reg_value_temp[1] & 0xC0) == 0x00) {
            break;
        }
        max_retry_times--;

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (!max_retry_times) {
        ESP_LOGE(TAG, "Read register 0x59 for PG_VALID and PG_FIRST failed, raw data: 0x%02X", reg_value_temp[1]);
        return -1;
    }

    reg_address_temp[0] = 0x58;
    // read 0x58 register and save it to image_data
    paw33xx_write_read(reg_address_temp, 2, reg_value_temp, 2);
    image_data[0] = reg_value_temp[1];

    // 读取剩余的 image_size -1 个像素
    for (int i = 0; i < image_size - 1; i++) {
        // Continuously read register 0x59 until getting bit 7 as “1”.
        max_retry_times = 1000;
        reg_address_temp[0] = 0x59;
        while (max_retry_times) {
            paw33xx_write_read(reg_address_temp, 2, reg_value_temp, 2);

            if ((reg_value_temp[1] & 0x80) == 0x80) {
                break;
            }
            max_retry_times--;

            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (!max_retry_times) {
            ESP_LOGE(TAG, "Read register 0x59 for RDG_VALID failed");
            return -1;
        }

        reg_address_temp[0] = 0x58;
        // read 0x58 register and save it to image_data
        paw33xx_write_read(reg_address_temp, 2, reg_value_temp, 2);
        image_data[i+1] = reg_value_temp[1];
    }

    //! STEP 5
    for (size_t i = 0; i < sizeof(reg_list_3) / sizeof(reg_list_3[0]); i++) {
        paw33xx_write_read((uint8_t *)reg_list_3[i], sizeof(reg_list_3[i]), NULL, 0);
    }

    return 0;
}

void paw33xx_init(void)
{
    int ret = 0;
    gpio_config_t paw_nreset = {
        .pin_bit_mask = (1ULL << GPIO_NUM_40),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&paw_nreset);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "paw_nreset config failed");
        return;
    }

    // pull up
    gpio_set_level(GPIO_NUM_40, 1);

    for(int i = 0; i < sizeof(reg_list_1) / sizeof(reg_list_1[0]); i++) {
        paw33xx_write_read((uint8_t *)reg_list_1[i], sizeof(reg_list_1[i]), NULL, 0);
    }

    // delay for 1ms
    esp_rom_delay_us(1000);

    // read 0x6C at 1ms interval until value 0x80 is obtained or read up to 60 times
    uint8_t reg_address_temp[2] = {0x6C, 0x00};
    uint8_t reg_value_temp[2] = {0};
    int max_retry_times = 60;

    while (max_retry_times) {
        paw33xx_write_read(reg_address_temp, 2, reg_value_temp, 2);
        if (reg_value_temp[1] == 0x80) {
            break;
        }
        max_retry_times--;

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (!max_retry_times) {
        ESP_LOGE(TAG, "Read register 0x6C failed");

        for(int i = 0; i < sizeof(reg_list_2) / sizeof(reg_list_2[0]); i++) {
            paw33xx_write_read((uint8_t *)reg_list_2[i], sizeof(reg_list_2[i]), NULL, 0);
        }
    }
    else {
        for (int i = 0; i < sizeof(reg_list_3) / sizeof(reg_list_3[0]); i++) {
            paw33xx_write_read((uint8_t *)reg_list_3[i], sizeof(reg_list_3[i]), NULL, 0);
        }
    }
}

int paw33xx_read_dxdy(uint8_t *motion, int16_t *dx, int16_t *dy)
{
    int ret = 0;

    // read 0x02, 0x03, 0x04, 0x05, 0x06 for Motion, DX_L, DX_H, DY_L, DY_H
    uint8_t reg_address_temp[2] = {0x02, 0x00};
    uint8_t reg_value_temp[2] = {0};

    ret = paw33xx_write_read(reg_address_temp, 2, reg_value_temp, 2);
    if(ret != 0) {
        ESP_LOGE(TAG, "Read register 0x02 failed");
        return -1;
    }
    *motion = reg_value_temp[1];

    reg_address_temp[0] = 0x03;
    ret = paw33xx_write_read(reg_address_temp, 2, reg_value_temp, 2);
    if(ret != 0) {
        ESP_LOGE(TAG, "Read register 0x03 failed");
        return -1;
    }
    *dx = reg_value_temp[1];

    reg_address_temp[0] = 0x04;
    ret = paw33xx_write_read(reg_address_temp, 2, reg_value_temp, 2);
    if(ret != 0) {
        ESP_LOGE(TAG, "Read register 0x04 failed");
        return -1;
    }
    *dx |= (reg_value_temp[1] << 8);

    reg_address_temp[0] = 0x05;
    ret = paw33xx_write_read(reg_address_temp, 2, reg_value_temp, 2);
    if(ret != 0) {
        ESP_LOGE(TAG, "Read register 0x05 failed");
        return -1;
    }
    *dy = reg_value_temp[1];

    reg_address_temp[0] = 0x06;
    ret = paw33xx_write_read(reg_address_temp, 2, reg_value_temp, 2);
    if(ret != 0) {
        ESP_LOGE(TAG, "Read register 0x06 failed");
        return -1;
    }
    *dy |= (reg_value_temp[1] << 8);

    return 0;
}

int paw33xx_main(void)
{
    paw33xx_init();

    uint8_t motion = 0;
    int16_t dx = 0, dy = 0;

    while(1) {
        if(paw33xx_read_dxdy(&motion, &dx, &dy) == 0) {
            ESP_LOGI(TAG, "Motion: %d, dx: %d, dy: %d", motion, dx, dy);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return 0;
}