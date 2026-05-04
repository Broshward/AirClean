// Типы данных, чтобы приложение знало, как их показывать
typedef enum {
    VAL_TYPE_FLOAT,
    VAL_TYPE_INT,
    VAL_TYPE_BOOL
} value_type_t;

// Контейнер для самого значения (занимает 4 байта)
typedef union {
    float f;
    int i;
    bool b;
} sensor_data_t;

// Наша "умная" структура датчика
typedef struct {
    int id;
    value_type_t val_type;
    const char* type_name;
    const char* label;
    sensor_data_t value;    // ТЕКУЩЕЕ ЗНАЧЕНИЕ ХРАНИМ ТУТ
} sensor_t;

typedef sensor_data_t (*read_func_t)(void);

void send_sensors();
void create_data(char *data);
void sensor_set_value(int id, value_type_t v_type, const char* name, const char* label, sensor_data_t new_val);
float temperature_calc(uint8_t *data);
void send_sensors_values(void);
void get_narodmon_string(char *data, size_t max_len);
void flash_log_all_sensors(uint32_t timestamp);

void save_head_to_eeprom(uint32_t head);
void save_tail_to_eeprom(uint32_t tail);
uint32_t read_head_from_eeprom();
uint32_t read_tail_from_eeprom();
void init_flash_logger();
void dump_history_safe(int target_id, uint16_t max_packets, uint8_t step);
