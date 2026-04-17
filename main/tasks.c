#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <errno.h>
#include <netdb.h>            // struct addrinfo
#include <arpa/inet.h>
#include "driver/temperature_sensor.h"
#include "driver/gpio.h"
#include "esp_gap_ble_api.h"

#include "tasks.h"
#include "blufi.h"
#include "times.h"
#include "ping.h"
#include "i2c.h"
#include "oneshot_read_adc_main.c"
#include "spi.h"
#include "ota.h"
#include "sensor.h"
#include "gatts.h"



/*ADC Light sensor Task*/
#define ADC_PERIOD 10			//Период измерения
#define TRANSMIT_PERIOD 1000*1	//Период передачи показаний
float gl_luminosity;
float gl_temp;
float gl_chip_temp;
uint8_t gl_temperature[2];

static const char *TCP_TAG = "TCP_IP";
RTC_DATA_ATTR time_t gl_last_send_time=0; // Last time in RTC SRAM part

QueueHandle_t blufi_tx_queue = NULL;
static uint32_t last_send_attempt_time = 0;

#define BLINK_GPIO 8
void configure_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BLINK_GPIO, 1);
}

#define TEMPERATURE_RESOLUTION		0.0625 
float temperature_calc(uint8_t *data)
{
	return ((int8_t)data[0]) + (data[1]>>4)*TEMPERATURE_RESOLUTION;
}

float get_kty81_210_temp(int in_volt)
{
    const float VCC = 3.3;
    const float R_PULLUP = 2000.0;
    
    // 1. Находим текущее сопротивление датчика
    //float voltage = (adc_raw * VCC) / 4080.0;
    float voltage = (float)in_volt/1000;
    // Схема: VCC -> R_PULLUP -> [ADC] -> KTY -> GND
    float r_kty = (voltage * R_PULLUP) / (VCC - voltage);

    // 2. Линейная аппроксимация (R = m*T + c)
    // По даташиту: 25°C = 2000 Ом, 100°C = 3392 Ом.
    // Наклон (m) ≈ 18.56 Ом/°C
    float temp = (r_kty - R_PULLUP) / 18.56 + 25.0;
    //float temp = R_PULLUP * voltage/(VCC*18.56 - voltage*18.56) - 1536/18.56;
    return temp;
}

void sensorsTask(void *pvParameters)
{
//	sensors_init();
	config_MCP9800(); // Датчик температуры
	
	// CHip temperature sensor init
	temperature_sensor_handle_t temp_handle = NULL;
	temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(0, 50);
	ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_handle));

	// Аналоговые датчики
	adc_config();
	// Переменная для хранения предыдущего значения
	static int filtered_value1;
	adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &filtered_value1); //Начальное значение 
	static int filtered_value2;
	adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &filtered_value2); //Начальное значение 
	const int coeff = 64; // Коэффициент фильтрации
	int count=0;
    while (1) {
        if (do_calibration1_chan0) {

			int raw_value;
			int voltage;
			adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &raw_value);

			// Применяем фильтр: y = (y * (k-1) + x) / k 
			filtered_value1 = (filtered_value1 * (coeff - 1) + raw_value) / coeff;

            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, filtered_value1, &voltage));

			gl_luminosity = pow(10,((float)voltage-253.0)/390); // 10**((V-Vdark)/S) V,Vdark[mV], S [V/decade] 
        if (do_calibration1_chan1) {
			int raw_value;
			int voltage;
			adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &raw_value);
			filtered_value2 = (filtered_value2 * (coeff - 1) + raw_value) / coeff;
            adc_cali_raw_to_voltage(adc1_cali_chan1_handle, filtered_value2, &voltage);
			gl_temp = get_kty81_210_temp(voltage);
			//ESP_LOGI("ADC", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN1, filtered_value2);
            //ESP_LOGI("ADC", "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN1, voltage);
        }
			
			// Цикл отправки данных(медленный) (TRANSMIT_PERIOD) 
			if (count >= TRANSMIT_PERIOD/ADC_PERIOD){
				i2c_register_read(MCP9800_handle, MCP9800_TEMPERATURE_REG, gl_temperature, 2);
				//char payload[40];
				//snprintf(payload, sizeof(payload), "Amb_Temp:%.2f", temperature_calc(gl_temperature));
				sensor_data_t val;
				val.f = temperature_calc(gl_temperature);
				sensor_set_value(1, VAL_TYPE_FLOAT, "t", "MCP9800", val);
				//queue_blufi_data((uint8_t *)payload, strlen(payload));
			
				// Enable temperature sensor
				ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));
				// Get converted sensor data
				ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &val.f));
				sensor_set_value(2, VAL_TYPE_FLOAT, "t", "ESP32C3_temp", val);
				// Disable the temperature sensor if it is not needed and save the power
				ESP_ERROR_CHECK(temperature_sensor_disable(temp_handle));

				val.f = gl_temp;
				sensor_set_value(3, VAL_TYPE_FLOAT, "t", "KTY81_210", val);
				val.f = gl_luminosity;
				sensor_set_value(4, VAL_TYPE_FLOAT, "l", "APDS-9007", val);
				count=0;

				if (is_ble_ready)
					send_sensors_values();
			}
			count++;
        }

        //vTaskDelay(pdMS_TO_TICKS(1000));
        vTaskDelay(pdMS_TO_TICKS(ADC_PERIOD));
    }
}

void after_success()
{
//    ESP_LOGI(TCP_TAG, "Narodmon: Success! Syncing state...");

    // 1. Фиксируем сдвиг хвоста
    // get_narodmon_string уже сдвинула current_tail_addr в ОЗУ в процессе сборки
	save_tail_to_eeprom(current_tail_addr);

    // 2. Обновляем время последней УДАЧНОЙ синхронизации в SRAM часов
    // Это нам пригодится, чтобы понимать, как долго мы без связи
//    time_t now;
//    time(&now);
//    mcp_write_bytes(MCP79410_RTC_RAM_ADDR, 0x20, (uint8_t*)&now, 4);

	gpio_set_level(BLINK_GPIO, 0); // LED is on
}
void after_failure()
{
    ESP_LOGE(TCP_TAG, "Failure sendig to server.");

    // 1. Откатываем хвост (не подтверждаем отправку из флешки)
    current_tail_addr = read_tail_from_eeprom();

    // 2. Проверяем время и пишем текущие показания в лог (если время ок)
	flash_log_all_sensors(gl_last_send_time);
	ESP_LOGI(TCP_TAG, "Current sensors state saved to Flash.");

	gpio_set_level(BLINK_GPIO, 1); // LED is off
}

#define HOST_IP_ADDR  "narodmon.ru"//"192.168.43.105" //"narodmon.ru"
#define PORT 8283			// narodmon.com TCP-port address
#define TIME_PERIOD 1000*(60*5+1) // Perod between sends
#define TIME_PERIOD_CONN 1000*1 // Period between reconnects

void tcp_clientTask(void *pvParameters)
{
	ulTaskNotifyTake(true, portMAX_DELAY); // Wait for time syncing
    configure_led();
	init_flash_logger();
	gl_last_send_time = load_last_send_time(); //Load from nvs last sending time
	printf("Last sending time from after reset ESP32 is %d\n", (int)gl_last_send_time);

    char rx_buffer[128];
    int addr_family = 0;
    int ip_protocol = 0;

	int sock=-1;
    while (1) {
		time_t now;
		time(&now);
		printf("Time NOW: %d\n", (int)now);
		printf("Time of last send: %d\n", (int)gl_last_send_time);
		if (now-gl_last_send_time<0 || gl_last_send_time==0) //Последняя передача в будущем или мы считали 0
			vTaskDelay(TIME_PERIOD);
		else if (now-gl_last_send_time < TIME_PERIOD/1000) {
			int time_to_wait = (TIME_PERIOD-1000*(now-gl_last_send_time)); // ms
			printf("Time to wait : %d second\n", time_to_wait/1000);
			if (time_to_wait>0)
				vTaskDelay(pdMS_TO_TICKS(TIME_PERIOD-1000*(now-gl_last_send_time))); //
		}
					time(&gl_last_send_time);  //Set last attempt send to server time
					save_last_send_time(gl_last_send_time); // Write last send time to flash

		struct hostent *hp = gethostbyname(HOST_IP_ADDR);
		if (hp == NULL) {
			ESP_LOGE(TCP_TAG, "DNS lookup failed");
			after_failure();
			continue;
		}
		char *host_ip = inet_ntoa(*(struct in_addr *)hp->h_addr_list[0]);

        struct sockaddr_in dest_addr;
        inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        if (sock != -1) {
            ESP_LOGE(TCP_TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
		sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TCP_TAG, "Unable to create socket: errno %d", errno);
			after_failure();
            continue;
        }
        ESP_LOGI(TCP_TAG, "Socket created, connecting to %s:%d", host_ip, PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TCP_TAG, "Socket unable to connect: errno %d", errno);
			after_failure();
            continue;
        }
        ESP_LOGI(TCP_TAG, "Successfully connected");

		char  data[4096]; //Max length tcp buffer of narodmon server
		get_narodmon_string(data, sizeof(data)); 
		printf(data);
		ESP_LOGI(TCP_TAG, "Data length = %d\n",strlen(data));

	// Send to server
			err = send(sock, data, strlen(data), 0);
            if (err < 0) {
                ESP_LOGE(TCP_TAG, "Error occurred during sending: errno %d", errno);
				after_failure();
				continue;
            }

			struct timeval tv;
			tv.tv_sec = 10; // 10 секунд ждать ответа
			tv.tv_usec = 0;
			setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TCP_TAG, "recv failed: errno %d", errno);
				after_failure();
				continue;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                //ESP_LOGI(TCP_TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TCP_TAG, "Answer: %s", rx_buffer);
				if (strncmp(rx_buffer,"OK",2)) 
					after_failure();
				else{
					after_success();
				}
				
            }
			ESP_LOGI(TCP_TAG, "----------");
    }
}

#define TIME_SYNC_FROM_RTC 1000*60*10 //10 minutes
#define TIME_SEND_TO_APP 1000*1 // 1 second
void timeTask(void *pvParameters)
{
	bool tcp_notify_given=false; //Эта переменная нужна, чтобы задача не надоедала уведомлениями
	int count=TIME_SYNC_FROM_RTC/TIME_SEND_TO_APP;
	while(1){
		if (count >= TIME_SYNC_FROM_RTC/TIME_SEND_TO_APP){
			rtc_to_system_time(); // Синхронизируем с внутренними часами 
			count=0;
		}
		time_t now;
		time(&now);
		//printf("Time = %u (0x%X)\n", (unsigned )now, (unsigned)now);
		
		char payload[35];
		char timestr[20];
		struct tm timeinfo;
		localtime_r(&now, &timeinfo);
		if (timeinfo.tm_year > 2025-1900) {
			if (tcp_notify_given==false){
				xTaskNotifyGive(tcptask);
				tcp_notify_given=true;
			}
			if (is_ble_ready){
				//Time
				strftime(timestr, sizeof(timestr), "%H:%M:%S", &timeinfo);
				snprintf(payload, sizeof(payload), "Time:%s", timestr);
				//queue_blufi_data((uint8_t *)payload, strlen(payload));
				send_ble_data(payload);
				//Date
				strftime(timestr, sizeof(timestr), "%d - %m - %Y", &timeinfo);
				snprintf(payload, sizeof(payload), "Date:%s", timestr);
				send_ble_data(payload);
			}
		}
		else {
			if (is_ble_ready){
				snprintf(payload, sizeof(payload), "Time_sync_sntp:Not sync");
				send_ble_data(payload);
			}
			ESP_LOGI("Time", "Resync time");
			resync_time();
		}
		vTaskDelay(pdMS_TO_TICKS(TIME_SEND_TO_APP));
		count++;
	}
}

void test(void *pvParameters)
{
	int i=0;
	while(1){
		vTaskDelay(pdMS_TO_TICKS(1000));
		i+=4;
	}
}

uint16_t g_blufi_conn_id = 0xffff; 
esp_bd_addr_t remote_bda_global;

//Эта задача нужна для того, чтобы разблокировать blufi_sender_task когда она зависнет.
void blufi_monitor_task(void *pvParameters) 
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Проверка раз в 5 секунд
		//ESP_LOGI("MONITOR","I am Monitor!!!");

        if (last_send_attempt_time != 0) {
            uint32_t duration = (xTaskGetTickCount() - last_send_attempt_time) * portTICK_PERIOD_MS;
            
            if (duration > 10000) { // Если функция висит дольше 10 сек
                ESP_LOGE("MONITOR", "BLUFI SEND HANG DETECTED! Force Disconnect...");
                
                //esp_ble_gap_disconnect(remote_bda_global); 
				esp_restart(); // Самый надежный способ очистить зависший DMA/контроллер
                
                // Сбрасываем метку после дисконнекта 
				if (g_blufi_conn_id == 0xffff) 
					last_send_attempt_time = 0; 
            }
        }
    }
}

void blufi_sender_task(void *pvParameters) 
{
	//Создаём очередь сообщений
	blufi_tx_queue = xQueueCreate(20, sizeof(blufi_msg_t));
	// Создаём задачу монитор, которая нужна, чтобы разорвать связь, если эта задача зависла.
	xTaskCreate( blufi_monitor_task, "Custom Data sender monitor", 4096, NULL, 10, NULL); //Task for test SPI

	blufi_msg_t msg;
    while (1) {
        if (xQueueReceive(blufi_tx_queue, &msg, portMAX_DELAY)) {
            // Обновляем метку времени ПЕРЕД вызовом
            last_send_attempt_time = xTaskGetTickCount(); 
            
			//Если нет подключения не пытаемся ничего слать, просто выкидываем сообщение
			if (g_blufi_conn_id == 0xffff) {
                free(msg.payload);
                continue; 
            }
//            ESP_LOGI("BLUFI_TX", "Attempting send...");
            esp_blufi_send_custom_data(msg.payload, msg.length);
            
            // Если мы здесь — функция разблокировалась
            last_send_attempt_time = 0; 
            
            free(msg.payload);
        }
    }
}

esp_err_t queue_blufi_data(uint8_t *data, size_t len) 
{
    if (blufi_tx_queue == NULL) return ESP_FAIL;
	
	// Если нет соединения
    if (g_blufi_conn_id == 0xffff) {
        // ESP_LOGD("BLUFI_TX", "No connection, skipping data"); // Опционально для отладки
        return ESP_ERR_INVALID_STATE; 
    }

    blufi_msg_t msg;
    msg.length = len;
    msg.payload = malloc(len); // Выделяем память под копию данных
    
    if (msg.payload == NULL) return ESP_ERR_NO_MEM;
    
    memcpy(msg.payload, data, len);

    // Отправляем в очередь (не ждем долго, если очередь полна — значит данных слишком много)
    if (xQueueSend(blufi_tx_queue, &msg, pdMS_TO_TICKS(50)) != pdPASS) {
        free(msg.payload);
        ESP_LOGE("BLUFI_TX", "Queue full, dropping packet!");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

