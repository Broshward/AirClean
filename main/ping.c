#include "ping/ping_sock.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "lwip/netdb.h"

#include "ping.h"

const static char *PING_TAG = "PING:";
bool gl_ping=0;
time_t gl_time=0;
char gl_narodmon_addr[44];   // Заменить на максимальную длину строки IP-адреса

static void cmd_ping_on_ping_success(esp_ping_handle_t hdl, void *args)
{
    uint8_t ttl;
    uint16_t seqno;
    uint32_t elapsed_time, recv_len;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TTL, &ttl, sizeof(ttl));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    esp_ping_get_profile(hdl, ESP_PING_PROF_SIZE, &recv_len, sizeof(recv_len));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed_time, sizeof(elapsed_time));
    ESP_LOGI(PING_TAG,"%" PRIu32 " bytes from %s icmp_seq=%" PRIu16 " ttl=%" PRIu16 " time=%" PRIu32 " ms",
           recv_len, ipaddr_ntoa((ip_addr_t*)&target_addr), seqno, ttl, elapsed_time);
	strcpy(gl_narodmon_addr, ipaddr_ntoa((ip_addr_t*)&target_addr));
}

static void cmd_ping_on_ping_timeout(esp_ping_handle_t hdl, void *args)
{
    uint16_t seqno;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    printf("From %s icmp_seq=%d timeout\n",ipaddr_ntoa((ip_addr_t*)&target_addr), seqno);
}

static void cmd_ping_on_ping_end(esp_ping_handle_t hdl, void *args)
{
    ip_addr_t target_addr;
    uint32_t transmitted;
    uint32_t received;
    uint32_t total_time_ms;
    uint32_t loss;

    esp_ping_get_profile(hdl, ESP_PING_PROF_REQUEST, &transmitted, sizeof(transmitted));
    esp_ping_get_profile(hdl, ESP_PING_PROF_REPLY, &received, sizeof(received));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    esp_ping_get_profile(hdl, ESP_PING_PROF_DURATION, &total_time_ms, sizeof(total_time_ms));

    if (transmitted > 0) {
        loss = (uint32_t)((1 - ((float)received) / transmitted) * 100);
    } else {
        loss = 0;
    }
//#ifdef CONFIG_LWIP_IPV4
//    if (IP_IS_V4(&target_addr)) {
//        printf("\n--- %s ping statistics ---\n", inet_ntoa(*ip_2_ip4(&target_addr)));
//    }
//#endif
//#ifdef CONFIG_LWIP_IPV6
//    if (IP_IS_V6(&target_addr)) {
//        printf("\n--- %s ping statistics ---\n", inet6_ntoa(*ip_2_ip6(&target_addr)));
//    }
//#endif
    ESP_LOGI(PING_TAG, "%" PRIu32 " packets transmitted, %" PRIu32 " received, %" PRIu32 "%% packet loss, time %" PRIu32 "ms\n",
           transmitted, received, loss, total_time_ms);
	if(received==0) {
		gl_ping=false;
		gpio_set_level(BLINK_GPIO, 1);
	}
	else {
		gl_ping=true;
		gpio_set_level(BLINK_GPIO, 0);
	}
    // delete the ping sessions, so that we clean up all resources and can create a new ping session
    // we don't have to call delete function in the callback, instead we can call delete function from other tasks
    esp_ping_delete_session(hdl);
}

int do_ping_cmd(char *addr)
{
    esp_ping_config_t config = ESP_PING_DEFAULT_CONFIG();
	config.count = 1;
	config.timeout_ms = 1000; // Ping timeout

    // parse IP address
    ip_addr_t target_addr;
    memset(&target_addr, 0, sizeof(target_addr));

//    if (inet_pton(AF_INET6, ping_args.host->sval[0], &sock_addr6.sin6_addr) == 1) {
//        /* convert ip6 string to ip6 address */
//        ipaddr_aton(ping_args.host->sval[0], &target_addr);
//    } else {
        struct addrinfo hint;
        struct addrinfo *res = NULL;
        memset(&hint, 0, sizeof(hint));
hint.ai_family = AF_UNSPEC; // Важно: разрешить оба протокола
hint.ai_socktype = SOCK_RAW; // Для пинга лучше использовать RAW или ANY
hint.ai_flags = AI_ADDRCONFIG; // Использовать только те протоколы, которые настроены на интерфейсах

        /* convert ip4 string or hostname to ip4 or ip6 address */
        if (getaddrinfo(addr, NULL, &hint, &res) != 0) {
            //printf("ping: unknown host %s\n", addr);
			gl_ping=false;
            return 1;
        }
#ifdef CONFIG_LWIP_IPV4
        if (res->ai_family == AF_INET) {
            struct in_addr addr4 = ((struct sockaddr_in *) (res->ai_addr))->sin_addr;
            inet_addr_to_ip4addr(ip_2_ip4(&target_addr), &addr4);
        }
#endif
#ifdef CONFIG_LWIP_IPV6
        if (res->ai_family == AF_INET6) {
            struct in6_addr addr6 = ((struct sockaddr_in6 *) (res->ai_addr))->sin6_addr;
            inet6_addr_to_ip6addr(ip_2_ip6(&target_addr), &addr6);
        }
#endif
        freeaddrinfo(res);
    //}
    config.target_addr = target_addr;

    /* set callback functions */
    esp_ping_callbacks_t cbs = {
        .cb_args = NULL,
        .on_ping_success = cmd_ping_on_ping_success,
        .on_ping_timeout = cmd_ping_on_ping_timeout,
        .on_ping_end = cmd_ping_on_ping_end
    };
    esp_ping_handle_t ping;
    ESP_RETURN_ON_FALSE(esp_ping_new_session(&config, &cbs, &ping) == ESP_OK, -1, PING_TAG, "esp_ping_new_session failed");
    ESP_RETURN_ON_FALSE(esp_ping_start(ping) == ESP_OK, -1, PING_TAG, "esp_ping_start() failed");
    return 0;
}

#define CONFIG_PING_PERIOD 1000*10 // 10 sec
void configure_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BLINK_GPIO, 1);
}

