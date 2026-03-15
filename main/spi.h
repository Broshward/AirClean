
void init_spi_eeprom();
uint8_t eeprom_read_byte(uint16_t address);
void eeprom_write_byte(uint16_t address, uint8_t data);
uint32_t eeprom_read_u32(uint16_t address);
void eeprom_write_u32(uint16_t address, uint32_t u32);
void eeprom_write_float(uint16_t address, float value);
float eeprom_read_float(uint16_t address);

void eeprom_erase_range(uint16_t start_addr, uint16_t length);
