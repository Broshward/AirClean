
void init_spi_eeprom();
uint8_t eeprom_read_byte(uint16_t address);
void eeprom_write_byte(uint16_t address, uint8_t data);
uint32_t eeprom_read_timestamp(uint16_t address);
void eeprom_write_timestamp(uint16_t address, uint32_t timestamp);
