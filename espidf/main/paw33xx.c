
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "driver/spi_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

extern spi_device_handle_t spi;

static const char *TAG = "PAW3311DW";

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

int paw33xx_get_image()
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
        paw33xx_write_read((uint8_t *)reg_list_1[i], sizeof(reg_list_1[i]), NULL, 0);
    }

    // Continuously read register 0x02 until getting both bit 1 and bit 0 are 0
    while (max_retry_times) {
        paw33xx_write_read(&reg_address_temp, 1, &reg_value_temp, 1);

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
        paw33xx_write_read((uint8_t *)reg_list_2[i], sizeof(reg_list_2[i]), NULL, 0);
    }

    max_retry_times = 20000;
    reg_address_temp = 0x59;
    // Continuously read register 0x59 until getting bit 6  as “1”
    while (max_retry_times) {
        paw33xx_write_read(&reg_address_temp, 1, &reg_value_temp, 1);

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
            paw33xx_write_read(&reg_address_temp, 1, &reg_value_temp, 1);

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
        paw33xx_write_read(&reg_address_temp, 1, image_data+i, 1);
    }

    for (size_t i = 0; i < sizeof(reg_list_3) / sizeof(reg_list_3[0]); i++) {
        paw33xx_write_read((uint8_t *)reg_list_3[i], sizeof(reg_list_3[i]), NULL, 0);
    }

    return 0;
}
