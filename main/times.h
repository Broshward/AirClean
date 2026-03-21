#include <time.h>

extern time_t last_sync_sntp;

void sntp_init_and_sync();
void resync_time();
void send_last_sync_sntp();

void rtc_to_system_time() ;
void system_time_to_rtc() ;
void get_current_time_str(char* buf, size_t size);
//void get_current_time_str(char* buf, size_t size);
void save_last_send_time(uint32_t timestamp);
uint32_t load_last_send_time();
void set_timezone(int offset);
void save_tz_to_nvs(int8_t offset);
int8_t load_tz_from_nvs();
