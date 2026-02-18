#include <stdint.h>


void configure_led(void);
void Temp_sensor_Task(void * pvParameters);		// Ping task
void LightTask(void *pvParameters);		// Luminosity task
void I2C_Task(void *pvParameters);		// Temperature task function prototype
void tcp_clientTask(void *pvParameters); // prototype of task function from tcp_client_v4.c file

