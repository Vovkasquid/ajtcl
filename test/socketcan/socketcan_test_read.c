#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "../../src/target/linux/socketcan/aj_socketcan.h"

int main() {
    uint8_t buf[1024];
    int fd;
    canid_t can_id = 0x4BA;
    int counter, seqcounter, statcounter;
    uint64_t overall;
    struct timespec start;

    fd = AJ_SocketCAN_Open("vcan0");

    if (fd < 0) {
        printf("failed to open\n");
        return -1;
    }

    clock_gettime(CLOCK_MONOTONIC, &start);
    statcounter = seqcounter = counter = 0;
    overall = 0;
    for (;;) {
        int i;
        int ret = AJ_SocketCAN_Read(fd, buf, sizeof(buf), can_id);
        if (ret < 0) {
            break;
        }

        overall += ret;
        statcounter += ret;
#define STAT_BYTES_LIMIT (100000)
        if (statcounter > STAT_BYTES_LIMIT) {
            struct timespec now;
            uint64_t elapsed;
            double speed;
            clock_gettime(CLOCK_MONOTONIC, &now);
            elapsed = (1000000000 * (now.tv_sec - start.tv_sec) + now.tv_nsec - start.tv_nsec) / 1000; // microseconds
            speed = (overall * 8) / (double)elapsed;
            printf("%.2fKBytes recv'd, speed %.3fMbps\n", overall / 1024.0, speed);
            statcounter %= STAT_BYTES_LIMIT;
        }

        seqcounter++;
        for (i = 0; i < ret; i++) {
            if (buf[i] != counter) {
                printf("byte order failed in seq %d, offset %d (%02x instead of %02x)\n", 
                        seqcounter, i, buf[i], counter);
                counter = buf[i] + 1;
                if (i) {
                    // fatal
                    return -1;
                }
            } else {
                counter++;
            }
            counter %= 256;
        }
    }
    return 0; 
}
