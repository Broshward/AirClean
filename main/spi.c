/* This file works with p25q32sh flash*/

#include "driver/spi_master.h"
#include "esp_log.h"
#include <string.h>
#include "spi.h"

#define PIN_NUM_MISO 5
#define PIN_NUM_MOSI 6
#define PIN_NUM_CLK  7
#define PIN_NUM_CS   10

spi_device_handle_t flash_spi;

uint32_t current_head_addr = 0;
uint32_t current_tail_addr = 0;

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

void spi_flash_send_raw(uint8_t *cmd, uint8_t cmd_len, uint8_t *data, uint16_t data_len) 
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    
    // Если данных мало, можно собрать в один буфер, если много — используем DMA
    uint8_t total_buf[256 + 4]; 
    memcpy(total_buf, cmd, cmd_len);
    memcpy(total_buf + cmd_len, data, data_len);

    t.length = (cmd_len + data_len) * 8;
    t.tx_buffer = total_buf;
    spi_device_polling_transmit(flash_spi, &t);
}

void flash_write_data(uint8_t *data, uint16_t len) 
{
    uint16_t sent = 0;

    while (sent < len) {
        // 1. Проверка конца флешки
        if (current_head_addr >= FLASH_TOTAL_SIZE) {
            ESP_LOGW("FLASH", "End of flash reached. Circling to 0...");
            current_head_addr = 0;
            flash_erase_sector(0); 
        }

        // 2. Стирание сектора при переходе границы 4КБ
        // Если текущая позиция кратна 4096 — пора чистить место
        if (current_head_addr % FLASH_SECTOR_SIZE == 0) {
            flash_erase_sector(current_head_addr);
        }

        // 3. Вычисляем порцию для записи (граница страницы 256б)
        uint16_t space_in_page = 256 - (current_head_addr % 256);
        uint16_t to_write = (len - sent < space_in_page) ? (len - sent) : space_in_page;

        // 4. Физическая запись
        flash_wait_until_ready();
        flash_write_enable();

        uint8_t cmd_addr[4];
        cmd_addr[0] = 0x02; // Page Program
        cmd_addr[1] = (current_head_addr >> 16) & 0xFF;
        cmd_addr[2] = (current_head_addr >> 8) & 0xFF;
        cmd_addr[3] = current_head_addr & 0xFF;

        // Отправляем пакет
        spi_flash_send_raw(cmd_addr, 4, &data[sent], to_write);

        // Сдвигаем глобальный адрес и счетчик отправленного
        current_head_addr += to_write;
        sent += to_write;
    }
}

void spi_flash_read_raw(uint32_t addr, uint8_t *dest, uint16_t len) 
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    // Формируем заголовок: команда 0x03 + 3 байта адреса
    uint8_t cmd_buf[4];
    cmd_buf[0] = 0x03; // Read Data
    cmd_buf[1] = (addr >> 16) & 0xFF;
    cmd_buf[2] = (addr >> 8) & 0xFF;
    cmd_buf[3] = addr & 0xFF;

    // В SPI транзакциях мы можем использовать 'user' или 'address' поля,
    // но для простоты и надежности (учитывая DMA) сделаем так:
    
    // Сначала отправляем адрес (4 байта) без поднятия CS в конце
    // (Или используем одну транзакцию с rxlength)
    
    t.length = 8 * 4;       // Команда + Адрес (32 бита)
    t.tx_buffer = cmd_buf;
    t.rxlength = 8 * len;  // Сколько бит хотим прочитать ПОСЛЕ команды
    t.rx_buffer = dest;

    // Поллинг-транзакция (блокирует поток до завершения, что для SPI быстро)
    esp_err_t ret = spi_device_polling_transmit(flash_spi, &t);
    
    if (ret != ESP_OK) {
        ESP_LOGE("FLASH", "SPI Read Error: %d", ret);
    }
}

void flash_read_data(uint32_t addr, uint8_t *dest, uint16_t len) 
{
    uint16_t read_now = 0;
    uint32_t curr_addr = addr;

    // Если адрес за пределами (например, после прошлой записи), сбрасываем в 0
    if (curr_addr >= FLASH_TOTAL_SIZE) curr_addr = 0;

    // Проверяем: пересекаем ли мы границу конца флешки?
    if (curr_addr + len > FLASH_TOTAL_SIZE) {
        // Читаем первую часть до конца флешки
        uint16_t part1 = FLASH_TOTAL_SIZE - curr_addr;
        spi_flash_read_raw(curr_addr, dest, part1);
        
        // Читаем вторую часть с самого начала
        spi_flash_read_raw(0, dest + part1, len - part1);
    } else {
        // Пакет лежит ровно, читаем одним куском
        spi_flash_read_raw(curr_addr, dest, len);
    }
}

