#include <time.h>

extern time_t last_sync_sntp;

void sntp_init_and_sync();
void resync_time();
void send_last_sync_sntp();

