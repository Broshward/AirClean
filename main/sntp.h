void sntp_init_and_sync();
void save_last_send_time(uint32_t timestamp);
uint32_t load_last_send_time();
void resync_time();

//void get_current_time_str(char* buf, size_t size);
