#include "driver/spi_master.h"
#include <string.h>


// This code for working with SPI-eeprom p25cXX series (p25c256)
#define CMD_WREN  0x06  // Write Enable
#define CMD_WRITE 0x02  // Page Program (Запись)
#define CMD_READ  0x03  // Read Data (Чтение)
#define CMD_RDSR  0x05  // Read Status Register (Проверка готовности)

spi_device_handle_t eeprom_handle;

void init_spi_eeprom() 
{
    spi_bus_config_t buscfg = {
        .miso_io_num = 5,   // MISO
        .mosi_io_num = 6,   // MOSI 
        .sclk_io_num = 7,   // SCK
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz (для надежности на макетке)
        .mode = 0,                         // SPI mode 0
        .spics_io_num = 10,                 // CS пин
        .queue_size = 7,
    };

    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &eeprom_handle);
}

uint8_t eeprom_read_byte(uint16_t address) {
    uint8_t data = 0;
    
    // Подготавливаем команду и адрес
    // P25C256 ждет: [Команда 0x03] [MSB Адреса] [LSB Адреса]
    uint8_t tx_buf[4] = {
        CMD_READ, 
        (uint8_t)(address >> 8), 
        (uint8_t)(address & 0xFF),
        0x00 // Байт-заполнитель, во время которого придет ответ
    };
    uint8_t rx_buf[4] = {0};

    spi_transaction_t t = {
        .length = 8 * 4,           // Всего 4 байта (3 передаем, 1 принимаем)
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    // Используем polling версию для маленьких данных — это быстрее и без паники
    esp_err_t ret = spi_device_polling_transmit(eeprom_handle, &t);
    
    if (ret == ESP_OK) {
        // Данные от EEPROM начнут поступать сразу после передачи адреса
        // Т.е. на 4-м байте транзакции
        data = rx_buf[3];
    }
    
    return data;
}

void eeprom_write_byte(uint16_t address, uint8_t data) 
{
    // 1. Разрешаем запись
    spi_transaction_t wren_t = {.length = 8, .tx_data = {CMD_WREN}, .flags = SPI_TRANS_USE_TXDATA};
    spi_device_polling_transmit(eeprom_handle, &wren_t);

    // 2. Сама запись
    uint8_t addr_h = (address >> 8) & 0xFF;
    uint8_t addr_l = address & 0xFF;
    spi_transaction_t write_t = {
        .length = 8 * 4,
        .tx_data = {CMD_WRITE, addr_h, addr_l, data},
        .flags = SPI_TRANS_USE_TXDATA
    };
    spi_device_polling_transmit(eeprom_handle, &write_t);

    // 3. Ждем, пока WIP (Write In Progress) станет 0
    uint8_t status = 0;
    do {
        spi_transaction_t status_t = {
            .length = 8 * 2,
            .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
            .tx_data = {CMD_RDSR, 0x00}
        };
        spi_device_polling_transmit(eeprom_handle, &status_t);
        status = status_t.rx_data[1];
    } while (status & 0x01); // Бит 0 — это busy флаг
}

uint32_t eeprom_read_timestamp(uint16_t address) 
{
    uint8_t tx_header[3] = { CMD_READ, (uint8_t)(address >> 8), (uint8_t)(address & 0xFF) };
    uint8_t rx_data[4];

    // Сначала шлем заголовок (команда + адрес), затем читаем 4 байта
    spi_transaction_t t;
    // Простой вариант через структуру:
    t.length = 8 * (3 + 4); 
    uint8_t full_tx[7] = {CMD_READ, tx_header[1], tx_header[2], 0, 0, 0, 0};
    uint8_t full_rx[7];
    t.tx_buffer = full_tx;
    t.rx_buffer = full_rx;
    
    spi_device_polling_transmit(eeprom_handle, &t);
    
    // Данные начинаются с 4-го байта (индекс 3)
    return ((uint32_t)full_rx[3] << 24) | ((uint32_t)full_rx[4] << 16) | ((uint32_t)full_rx[5] << 8)  | (uint32_t)full_rx[6];
}

void eeprom_write_timestamp(uint16_t address, uint32_t timestamp) 
{
    // 1. Разрешаем запись (WREN)
    spi_transaction_t wren_t = { .length = 8, .tx_data = {CMD_WREN}, .flags = SPI_TRANS_USE_TXDATA };
    spi_device_polling_transmit(eeprom_handle, &wren_t);

    // 2. Разбиваем 32-битное число на 4 байта (Big Endian)
    uint8_t data[4];
    data[0] = (timestamp >> 24) & 0xFF;
    data[1] = (timestamp >> 16) & 0xFF;
    data[2] = (timestamp >> 8) & 0xFF;
    data[3] = timestamp & 0xFF;

    // 3. Формируем транзакцию: Команда(1) + Адрес(2) + Данные(4) = 7 байт
    uint8_t tx_buffer[7];
    tx_buffer[0] = CMD_WRITE;
    tx_buffer[1] = (address >> 8) & 0xFF; // Addr High
    tx_buffer[2] = address & 0xFF;        // Addr Low
    memcpy(&tx_buffer[3], data, 4);

    spi_transaction_t write_t = {
        .length = 8 * 7,
        .tx_buffer = tx_buffer,
    };
    spi_device_polling_transmit(eeprom_handle, &write_t);

    // 4. Ожидание завершения записи (WIP bit)
    uint8_t status;
    do {
        spi_transaction_t status_t = {
            .length = 8,
            .rxlength = 8,
            .cmd = CMD_RDSR,
            .flags = SPI_TRANS_USE_RXDATA
        };
        spi_device_polling_transmit(eeprom_handle, &status_t);
        status = status_t.rx_data[0];
    } while (status & 0x01); // Ждем, пока бит Busy не станет 0
}
