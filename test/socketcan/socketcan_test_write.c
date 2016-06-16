#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>

#include "../../src/target/linux/socketcan/aj_socketcan.h"

#define VCAN_SPEED_LIMIT

static uint32_t getElapsedMillis(struct timespec *pFrom)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    return (1000 * (now.tv_sec - pFrom->tv_sec)) + ((now.tv_nsec - pFrom->tv_nsec) / 1000000);
}

int main() {
    uint8_t buf[1024];
    int fd;
    canid_t can_id = 0x4BA;
    int overall, counter;
    struct timespec from;

    fd = AJ_SocketCAN_Open("vcan0");

    if (fd < 0) {
        printf("failed to open\n");
        return -1;
    }

    //printf("Wrote 1, Written: %d\n", write_to_can(fd, buf, 1, can_id));
    //printf("Wrote 8, Written: %d\n", write_to_can(fd, buf, 8, can_id));
    //printf("Wrote 224, Written: %d\n", write_to_can(fd, buf, 224, can_id));
    //printf("Wrote 1024, Written: %d\n", write_to_can(fd, buf, 1024, can_id));


    //sleep(3);

    overall = 0;
    counter = 0;
    clock_gettime(CLOCK_MONOTONIC, &from);
    for (;;) {
        int i;
#define MIN_SIZE 1
        int len = MIN_SIZE + (rand() % (sizeof(buf) - MIN_SIZE));
        if (len == 0) {
            continue;
        }

        overall += len;

#ifdef VCAN_SPEED_LIMIT
#   define MAX_BYTES_W_NO_DELAY (2000)
#   define MIN_CAN_TIME_MILLIS (MAX_BYTES_W_NO_DELAY * 8 / 1000) /* assume 1Mbps speed */
        if (overall >= MAX_BYTES_W_NO_DELAY) {
            overall %= MAX_BYTES_W_NO_DELAY;
            do {} while (getElapsedMillis(&from) < MIN_CAN_TIME_MILLIS);
            clock_gettime(CLOCK_MONOTONIC, &from);
        }
#endif // VCAN_SPEED_LIMIT

        for (i = 0; i < len; i++) {
            buf[i] = (uint8_t)(counter++ % 256);
        }

        if (AJ_SocketCAN_Write(fd, buf, len, can_id) != len) {
            printf("write failed\n");
            return -1;
        }
        //printf("%03d\r", len);
        //putchar('.');
    }   

    return 0; 
}
