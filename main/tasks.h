#include <stdint.h>
#include <time.h>
#include "esp_err.h"
#include "esp_bt_defs.h"
#include "esp_timer.h"


void configure_led(void);
void sensorsTask(void *pvParameters); 
void tcp_clientTask(void *pvParameters); // prototype of task function from tcp_client_v4.c file
void timeTask(void *pvParameters);
void test(void *pvParameters);
void history_export_task(void *pvParameters);
void save_log_interval_to_nvs(uint32_t interval);


extern float gl_luminosity;
extern float gl_temp;
extern float gl_chip_temp;
extern uint8_t gl_temperature[2];

extern time_t gl_last_send_time;
extern uint32_t log_interval_sec;

extern esp_timer_handle_t log_timer, send_timer;

extern bool gl_unsent; 



