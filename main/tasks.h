#include <stdint.h>


void pingTask(void * pvParameters);		// Ping task
void LightTask(void *pvParameters);		// Luminosity task
void TempTask(void *pvParameters);		// Temperature task function prototype
void tcp_clientTask(void *pvParameters); // prototype of task function from tcp_client_v4.c file

