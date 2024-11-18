
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "driver/spi_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

extern spi_device_handle_t spi;

static const char *TAG = "PAW3311DW";

uint8_t image_data[900];

static int paw3311dw_write_read(uint8_t *write_list, size_t write_len, uint8_t *read_list, size_t read_len)
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

        ret = spi_device_transmit(spi, &t); // Transmit!
        if (ret) {
            ESP_LOGE(TAG, "SPI transaction failed");
            return -1;
        }
    }

    if (read_len > 0 && read_list != NULL) {
        t.tx_buffer = NULL;
        t.length = 0;

        t.rx_buffer = read_list;
        t.rxlength = read_len * 8;

        ret = spi_device_transmit(spi, &t);
        if (ret) {
            ESP_LOGE(TAG, "SPI transaction failed");
            return -1;
        }
    }

    spi_transaction_t t_null = { 0 };
    spi_device_transmit(spi, &t_null);

    return 0;
}

void paw3311dw_init()
{
    const uint8_t reg_list_1[][2] = { { 0x40, 0x80 }, { 0x55, 0x01 } };

    const uint8_t reg_list_2[][2] = { { 0x7F, 0x0E }, { 0x43, 0x1D } };

    const uint8_t reg_list_3[][2] = { { 0x43, 0x1e } };

    uint8_t reg_value_r1 = 0;
    uint8_t reg_value_r2 = 0;

    const uint8_t reg_list_4[][2] = { { 0x7F, 0x14 } };

    const uint8_t reg_list_5[][2] = {
        { 0x7f, 0x00 }, { 0x55, 0x00 }, { 0x4d, 0xd0 }, { 0x4e, 0x23 }, { 0x4f, 0x46 }, { 0x77, 0x24 }, { 0x7f, 0x05 },
        { 0x44, 0xa8 }, { 0x4a, 0x14 }, { 0x53, 0x0c }, { 0x5b, 0xea }, { 0x61, 0x13 }, { 0x62, 0x07 }, { 0x64, 0xd8 },
        { 0x6d, 0x86 }, { 0x7d, 0x84 }, { 0x7e, 0x00 }, { 0x7f, 0x06 }, { 0x60, 0x70 }, { 0x61, 0x00 }, { 0x7e, 0x40 },
        { 0x7f, 0x07 }, { 0x42, 0x16 }, { 0x7f, 0x09 }, { 0x40, 0x03 }, { 0x7f, 0x0a }, { 0x49, 0x00 }, { 0x4a, 0x23 },
        { 0x4c, 0x28 }, { 0x4f, 0x02 }, { 0x7f, 0x0c }, { 0x40, 0xa0 }, { 0x41, 0x70 }, { 0x42, 0x20 }, { 0x43, 0xc5 },
        { 0x44, 0x44 }, { 0x45, 0x04 }, { 0x4c, 0x60 }, { 0x54, 0x00 }, { 0x55, 0x40 }, { 0x59, 0x93 }, { 0x7f, 0x0d },
        { 0x4f, 0x02 }, { 0x4e, 0x6b }, { 0x7f, 0x14 }, { 0x4a, 0x67 }, { 0x62, 0x1c }, { 0x63, 0x1c }, { 0x6d, 0x82 },
        { 0x6f, 0xd8 }, { 0x73, 0x83 }, { 0x74, 0x00 }, { 0x7a, 0x16 }, { 0x7f, 0x10 }, { 0x48, 0x0f }, { 0x49, 0x88 },
        { 0x4c, 0x1d }, { 0x4f, 0x08 }, { 0x51, 0x6f }, { 0x52, 0x90 }, { 0x54, 0x64 }, { 0x55, 0xf0 }, { 0x5c, 0x40 },
        { 0x61, 0xee }, { 0x62, 0xe5 }, { 0x7f, 0x00 }, { 0x5b, 0x40 }, { 0x61, 0xad }, { 0x51, 0xea }, { 0x19, 0x9f }
    };

    const uint8_t reg_list_6[][2] = { { 0x19, 0x10 }, { 0x40, 0x00 }, { 0x61, 0xd5 }, { 0x7f, 0x00 } };

    // ----------------------------------------

    for (size_t i = 0; i < sizeof(reg_list_1) / sizeof(reg_list_1[0]); i++) {
        paw3311dw_write_read((uint8_t *)reg_list_1[i], sizeof(reg_list_1[i]), NULL, 0);
    }

    // delay for 1ms
    vTaskDelay(pdMS_TO_TICKS(1));

    for (size_t i = 0; i < sizeof(reg_list_2) / sizeof(reg_list_2[0]); i++) {
        paw3311dw_write_read((uint8_t *)reg_list_2[i], sizeof(reg_list_2[i]), &reg_value_r1, 1);
    }

    // read 0x46 register and save it to reg_value_r1
    uint8_t reg_address_temp = 0x46;
    paw3311dw_write_read(&reg_address_temp, 1, &reg_value_r1, 1);

    for (size_t i = 0; i < sizeof(reg_list_3) / sizeof(reg_list_3[0]); i++) {
        paw3311dw_write_read((uint8_t *)reg_list_3[i], sizeof(reg_list_3[i]), &reg_value_r2, 1);
    }

    // read 0x46 register and save it to reg_value_r2
    paw3311dw_write_read(&reg_address_temp, 1, &reg_value_r2, 1);

    for (size_t i = 0; i < sizeof(reg_list_4) / sizeof(reg_list_4[0]); i++) {
        paw3311dw_write_read((uint8_t *)reg_list_4[i], sizeof(reg_list_4[i]), NULL, 0);
    }

    uint8_t reg_list_temp[2][2] = { { 0x6A, reg_value_r1 }, { 0x6C, reg_value_r2 } };

    for (size_t i = 0; i < sizeof(reg_list_temp) / sizeof(reg_list_temp[0]); i++) {
        paw3311dw_write_read((uint8_t *)reg_list_temp[i], sizeof(reg_list_temp[i]), NULL, 0);
    }

    for (size_t i = 0; i < sizeof(reg_list_5) / sizeof(reg_list_5[0]); i++) {
        paw3311dw_write_read((uint8_t *)reg_list_5[i], sizeof(reg_list_5[i]), NULL, 0);
    }

    reg_address_temp = 0x20;

    // get timestamps ms
    uint32_t timestamp = esp_timer_get_time() / 1000;

    // Read register 0x20 at 1ms interval until 0x0F is obtained of read up to 55ms, this register read interval must
    //be carried out at 1ms interval with timing tolerance of +/- 1%.

    uint8_t reg_value_temp = 0;

    while (1) {
        paw3311dw_write_read(&reg_address_temp, 1, &reg_value_temp, 1);

        if (reg_value_temp == 0x0F) {
            break;
        }

        if (esp_timer_get_time() / 1000 - timestamp >= 55) {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    for (size_t i = 0; i < sizeof(reg_list_6) / sizeof(reg_list_6[0]); i++) {
        paw3311dw_write_read((uint8_t *)reg_list_6[i], sizeof(reg_list_6[i]), NULL, 0);
    }

    ESP_LOGI(TAG, "PAW3311DW initialization completed, 0x6A: 0x%02X, 0x6C: 0x%02X", reg_value_r1, reg_value_r2);
}

int get_raw_image()
{
    int max_retry_times = 100000;

    memset(image_data, 0, sizeof(image_data));

    const uint8_t reg_list_1[][2] = {
        { 0x7F, 0x00 }, { 0x40, 0x80 }, { 0x7F, 0x13 }, { 0x47, 0x30 }, { 0x7F, 0x00 }, { 0x55, 0x04 },
    };

    const uint8_t reg_list_2[][2] = {
        { 0x58, 0xFF },
    };

    const uint8_t reg_list_3[][2] = {
        { 0x55, 0x00 }, { 0x40, 0x00 }, { 0x7F, 0x13 }, { 0x47, 0x20 }, { 0x7F, 0x00 },
    };

    uint8_t reg_address_temp = 0x02;
    uint8_t reg_value_temp = 0;

    for (size_t i = 0; i < sizeof(reg_list_1) / sizeof(reg_list_1[0]); i++) {
        paw3311dw_write_read((uint8_t *)reg_list_1[i], sizeof(reg_list_1[i]), NULL, 0);
    }

    // Continuously read register 0x02 until getting both bit 1 and bit 0 are 0
    while (max_retry_times) {
        paw3311dw_write_read(&reg_address_temp, 1, &reg_value_temp, 1);

        if ((reg_value_temp & 0x03) == 0) {
            break;
        }
        max_retry_times--;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (!max_retry_times) {
        ESP_LOGE(TAG, "Read register 0x02 failed");
        return -1;
    }

    for (size_t i = 0; i < sizeof(reg_list_2) / sizeof(reg_list_2[0]); i++) {
        paw3311dw_write_read((uint8_t *)reg_list_2[i], sizeof(reg_list_2[i]), NULL, 0);
    }

    max_retry_times = 20000;
    reg_address_temp = 0x59;
    // Continuously read register 0x59 until getting bit 6  as “1”
    while (max_retry_times) {
        paw3311dw_write_read(&reg_address_temp, 1, &reg_value_temp, 1);

        if ((reg_value_temp & 0x40) == 0x40) {
            break;
        }
        max_retry_times--;

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (!max_retry_times) {
        ESP_LOGE(TAG, "Read register 0x59 for RDG_FIRST failed");
        return -1;
    }

    for (int i = 0; i < 900; i++) {
        // Continuously read register 0x59 until getting bit 7 as “1”.
        max_retry_times = 20000;
        while (max_retry_times) {
            paw3311dw_write_read(&reg_address_temp, 1, &reg_value_temp, 1);

            if ((reg_value_temp & 0x80) == 0x80) {
                break;
            }
            max_retry_times--;

            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (!max_retry_times) {
            ESP_LOGE(TAG, "Read register 0x59 for RDG_VALID failed");
            return -1;
        }

        // read 0x58 register and save it to image_data
        paw3311dw_write_read(&reg_address_temp, 1, image_data+i, 1);
    }

    for (size_t i = 0; i < sizeof(reg_list_3) / sizeof(reg_list_3[0]); i++) {
        paw3311dw_write_read((uint8_t *)reg_list_3[i], sizeof(reg_list_3[i]), NULL, 0);
    }

    return 0;
}
