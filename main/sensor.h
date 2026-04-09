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

extern bool is_ble_ready;
