#include "driver/spi_master.h"
#include "esp_log.h"
#include <string.h>

#define PIN_NUM_MISO 5
#define PIN_NUM_MOSI 6
#define PIN_NUM_CLK  7
#define PIN_NUM_CS   10

spi_device_handle_t flash_spi;

void init_external_flash_spi() 
{
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000 * 10, // 10 MHz
        .mode = 0,                         // SPI mode 0
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
    };

    // Инициализация шины SPI
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // Добавление устройства (флешки) на шину
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &flash_spi);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI("FLASH", "SPI initialized on GPIO 5,6,7,10");
}

void read_flash_id() 
{
    uint8_t tx_data[4] = {0x9F, 0x00, 0x00, 0x00}; // Команда + 3 пустых байта
    uint8_t rx_data[4];

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 32;          // Всего 32 бита (8 команда + 24 ответ)
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;

    esp_err_t ret = spi_device_polling_transmit(flash_spi, &t);
    if (ret == ESP_OK) {
        // ID начинается со второго байта (rx_data[1]), т.к. первый был во время отправки 0x9F
        ESP_LOGI("FLASH", "JEDEC ID: %02X %02X %02X", rx_data[1], rx_data[2], rx_data[3]);
    }
}

uint8_t flash_read_status() 
{
    uint8_t cmd = 0x05;
    uint8_t status = 0;
    
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;
    // Используем rxlength, чтобы прочитать ответ сразу за командой
    t.rxlength = 8;
    t.rx_buffer = &status;
    
    spi_device_polling_transmit(flash_spi, &t);
    return status;
}

void flash_wait_until_ready() 
{
    // Ждем, пока нулевой бит (WIP) не станет 0
    while (flash_read_status() & 0x01) {
        // Минимальная пауза, чтобы не забивать шину SPI проверками
        vTaskDelay(1); 
    }
}

void flash_write_enable() {
    uint8_t cmd = 0x06;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;
    flash_wait_until_ready(); // Ждем завершения предыдущих дел
    spi_device_polling_transmit(flash_spi, &t);
}

void flash_erase_sector(uint32_t addr) 
{
    flash_wait_until_ready(); // Ждем завершения предыдущих дел
    flash_write_enable();
    
    uint8_t tx_data[4] = {
        0x20,
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >> 8),
        (uint8_t)addr
    };

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 32;
    t.tx_buffer = tx_data;
    spi_device_polling_transmit(flash_spi, &t);
}

