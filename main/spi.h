#define FLASH_SECTOR_SIZE 4096
#define FLASH_TOTAL_SIZE  (4 * 1024 * 1024)

void init_external_flash_spi();
void read_flash_id();
void flash_write_data(uint8_t *data, uint16_t len);
void flash_read_data(uint32_t addr, uint8_t *dest, uint16_t len);


extern uint32_t current_head_addr;
extern uint32_t current_tail_addr;
extern uint32_t prev_tail_addr;
