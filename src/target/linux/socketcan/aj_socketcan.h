#ifndef _AJ_SOCKETCAN_H
#define _AJ_SOCKETCAN_H

#include <stdint.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @return binded socket fd or -1
 */
extern int AJ_SocketCAN_Open(const char *ifname);

int AJ_SocketCAN_Read(int fd, uint8_t *buffer, int len, canid_t can_id);
int AJ_SocketCAN_Write(int fd, uint8_t *buffer, int len, canid_t can_id);

#ifdef __cplusplus
}
#endif

#endif /* _AJ_SOCKETCAN_H */
