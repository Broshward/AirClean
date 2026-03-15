#include "driver/spi_master.h"
#include "esp_log.h"
#include <string.h>


// This code for working with SPI-eeprom p25cXX series (p25c256)
#define CMD_WREN  0x06  // Write Enable
#define CMD_WRITE 0x02  // Page Program (Запись)
#define CMD_READ  0x03  // Read Data (Чтение)
#define CMD_RDSR  0x05  // Read Status Register (Проверка готовности)


spi_device_handle_t eeprom_handle;

static void eeprom_write_enable()
{
    spi_transaction_t wren_t = {.length = 8, .tx_data = {CMD_WREN}, .flags = SPI_TRANS_USE_TXDATA};
    spi_device_polling_transmit(eeprom_handle, &wren_t);
}

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

static void wait_for_eeprom_ready() {
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

uint8_t eeprom_read_byte(uint16_t address) 
{
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
    wait_for_eeprom_ready();
    // 1. Включаем запись
    eeprom_write_enable();

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
}

// Функция записи u32
void eeprom_write_u32(uint16_t address, uint32_t data) 
{
    wait_for_eeprom_ready(); 

    // 1. WREN (Включение записи)
	eeprom_write_enable();
    // 2. Подготовка буфера: CMD(1) + ADDR(2) + DATA(4) = 7 байт
    static uint8_t tx_buf[7];
    tx_buf[0] = CMD_WRITE;
    tx_buf[1] = (address >> 8) & 0xFF;
    tx_buf[2] = address & 0xFF;
    // Пишем в Big Endian (стандарт для EEPROM)
    tx_buf[3] = (data >> 24) & 0xFF;
    tx_buf[4] = (data >> 16) & 0xFF;
    tx_buf[5] = (data >> 8) & 0xFF;
    tx_buf[6] = data & 0xFF;

    spi_transaction_t t = { .length = 8 * 7, .tx_buffer = tx_buf };
    spi_device_polling_transmit(eeprom_handle, &t);
}

// Функция чтения u32
uint32_t eeprom_read_u32(uint16_t address) 
{
    wait_for_eeprom_ready();
    static uint8_t tx_buf[7] = {CMD_READ, 0, 0, 0, 0, 0, 0};
    static uint8_t rx_buf[7];
    
    tx_buf[1] = (address >> 8) & 0xFF;
    tx_buf[2] = address & 0xFF;

    spi_transaction_t t = {
        .length = 8 * 7,
		.rxlength = 8*7,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf
    };

    if (spi_device_polling_transmit(eeprom_handle, &t) == ESP_OK) {
        // КРИТИЧЕСКИЙ МОМЕНТ: Собираем данные с индекса 3!
        return ((uint32_t)rx_buf[3] << 24) |
               ((uint32_t)rx_buf[4] << 16) |
               ((uint32_t)rx_buf[5] << 8)  |
                (uint32_t)rx_buf[6];
    }
    return 0;
}

void eeprom_write_float(uint16_t address, float value) 
{
    uint32_t raw_bits;
    memcpy(&raw_bits, &value, 4);
	eeprom_write_u32(address,raw_bits);
}

float eeprom_read_float(uint16_t address) 
{
    uint32_t temp = eeprom_read_u32(address);
    float val;
    memcpy(&val, &temp, 4); // Возвращаем биты обратно во float
    return val;
}

void eeprom_erase_range(uint16_t start_addr, uint16_t length) 
{
    uint16_t current_addr = start_addr;
    uint16_t remaining = length;

    ESP_LOGI("EEPROM", "Стирание %d байт с адреса 0x%04X...", length, start_addr);

    while (remaining > 0) {
        wait_for_eeprom_ready(); // Проверка перед каждой записью

        // WREN
		eeprom_write_enable();
        // Определяем, сколько писать в этой итерации (не более 64 байт и не заходя за границу страницы)
        // Для простоты на макетке будем писать по 16 байт — это гарантированно быстро и безопасно
#define ONE_COUNT 64 //Количество байт за одну запись.
        uint8_t chunk = (remaining > ONE_COUNT) ? ONE_COUNT : remaining;
        
        uint8_t tx_buf[3+ONE_COUNT]; // CMD(1) + ADDR(2) + DATA(16)
        tx_buf[0] = CMD_WRITE;
        tx_buf[1] = (current_addr >> 8) & 0xFF;
        tx_buf[2] = current_addr & 0xFF;
        memset(&tx_buf[3], 0xFF, chunk);

        spi_transaction_t t = {
            .length = 8 * (3 + chunk),
            .tx_buffer = tx_buf
        };
        
        spi_device_polling_transmit(eeprom_handle, &t);

        current_addr += chunk;
        remaining -= chunk;
    }
    
    wait_for_eeprom_ready(); // Ждем финализации последней записи
    ESP_LOGI("EEPROM", "Стирание завершено.");
}
