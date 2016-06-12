/**
 * @file
 */
/******************************************************************************
 * Copyright AllSeen Alliance. All rights reserved.
 *
 *    Permission to use, copy, modify, and/or distribute this software for any
 *    purpose with or without fee is hereby granted, provided that the above
 *    copyright notice and this permission notice appear in all copies.
 *
 *    THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *    WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *    MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *    ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *    WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *    ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *    OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 ******************************************************************************/

/**
 * Per-module definition of the current module for debug logging.  Must be defined
 * prior to first inclusion of aj_debug.h
 */
#define AJ_MODULE TARGET_SERIO

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/file.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include <ajtcl/aj_target.h>
#include <ajtcl/aj_status.h>
#include <ajtcl/aj_debug.h>
#include <ajtcl/aj_serial.h>
#include <ajtcl/aj_serial_rx.h>
#include <ajtcl/aj_serial_tx.h>
#include <ajtcl/aj_serio.h>

#include "aj_socketcan.h"

/**
 * Turn on per-module debug printing by setting this variable to non-zero value
 * (usually in debugger).
 */
#ifndef NDEBUG
uint8_t dbgTARGET_SERIO = 0;
#endif

static int canFD = -1;
static canid_t server_can_id = (canid_t)-1;
static canid_t target_can_id = (canid_t)-1;

int AJ_SerialIOBlockingRead(uint8_t* data, uint32_t len)
{
    int ret = 0;

    if ((ret = AJ_SocketCAN_Read(canFD, data, len, server_can_id)) <= 0) {
        AJ_ErrPrintf(("AJ_SerialIOBlockingRead: AJ_SocketCAN_Read() failed, ret=%d, errno=\"%s\"\n",
            ret, strerror(errno)));
    }
    return ret;
}


int AJ_SerialIOWriteBytes(uint8_t* data, uint32_t len)
{
    //sleep(1);
    return AJ_SocketCAN_Write(canFD, data, len, target_can_id);
}

AJ_Status AJ_SerialIOInit(AJ_SerIOConfig* config)
{
    int ret;

    AJ_InfoPrintf(("AJ_SerialIOInit\n"));

    /*
     * Validate and set parameters
     */

    /* TODO */
    target_can_id = 0x4BA;
    server_can_id = 0x3BA;    


    ret = AJ_SocketCAN_Open((const char *)config->config);
    if (ret == -1) {
        AJ_ErrPrintf(("failed to open socketcan device %s. ret = %d, %d - %s\n", (const char *)config->config, ret, errno, strerror(errno)));
        goto error;
    }
    canFD = ret;

    return AJ_OK;

error:
    if (canFD != -1) {
        close(canFD);
        canFD = -1;
    }
    return AJ_ERR_DRIVER;
}

void AJ_SerialIOClose(void)
{
    AJ_InfoPrintf(("AJ_SerialIOClose\n"));
    if (canFD != -1) {
        close(canFD);
        canFD = -1;
    }
}

