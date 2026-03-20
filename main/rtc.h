
void rtc_to_system_time() ;
void system_time_to_rtc() ;
void get_current_time_str(char* buf, size_t size);

//void get_current_time_str(char* buf, size_t size);
void save_last_send_time(uint32_t timestamp);
uint32_t load_last_send_time();
void set_timezone(int offset);
