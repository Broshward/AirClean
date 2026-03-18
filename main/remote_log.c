#include "lwip/sockets.h"

void remote_log(const char *fmt, ...) 
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr("192.168.1.75"); // IP ноута
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(5273);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    sendto(sock, buf, strlen(buf), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    close(sock);
}

