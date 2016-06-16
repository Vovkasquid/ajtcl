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

/**
 * Turn on per-module debug printing by setting this variable to non-zero value
 * (usually in debugger).
 */
#ifndef NDEBUG
uint8_t dbgTARGET_SERIO = 0;
#endif

/** Serial Port FD */
static int serialFD = -1;

static const AJ_SerIOConfig* serio_config = NULL;

int AJ_SerialIOBlockingRead(uint8_t* data, uint32_t len)
{
    fd_set fds;
    int rc;
    int ret = 0;

    if (serio_config == NULL) {
        AJ_ErrPrintf(("AJ_SerialIOBlockingRead: not initialized\n"));
        return -1;
    }

    for (;;) {
        FD_ZERO(&fds);
        FD_SET(serialFD, &fds);

        rc = select(serialFD + 1, &fds, NULL, NULL, NULL);
        if (rc <= 0) {
            AJ_ErrPrintf(("AJ_SerialIOBlockingRead: select() failed, exiting, rc=%d, errno=\"%s\"\n", rc, strerror(errno)));
            return -1;
        }

        if (FD_ISSET(serialFD, &fds)) {
            if ((ret = read(serialFD, data, len)) <= 0) {
                AJ_ErrPrintf(("AJ_SerialIOBlockingRead: read() failed, ret=%d, errno=\"%s\"\n", ret, strerror(errno)));
                continue;
            }
	    break;
        }
    }

    return ret;
}


int AJ_SerialIOWriteBytes(uint8_t* data, uint32_t len)
{

    if (serio_config == NULL) {
        AJ_ErrPrintf(("AJ_SerialIOWriteBytes: not initialized\n"));
        return -1;
    }

    return write(serialFD, data, len);
}

AJ_Status AJ_SerialIOInit(const AJ_SerIOConfig* config)
{
    int ret;
    struct termios ttySettings;
    speed_t speed;

    AJ_InfoPrintf(("AJ_SerialIOInit\n"));

    serio_config = NULL;

    serialFD = -1;

    /*
     * Validate and set parameters
     */
    memset(&ttySettings, 0, sizeof(ttySettings));
    ttySettings.c_cflag |= CLOCAL | CREAD;

    /*
     * Set input and output baudrate
     */
    switch (config->bitrate) {
    case 2400:
        speed = B2400;
        break;
    case 9600:
        speed = B9600;
        break;
    case 19200:
        speed = B19200;
        break;
    case 38400:
        speed = B38400;
        break;
    case 57600:
        speed = B57600;
        break;
    case 115200:
        speed = B115200;
        break;
    case 230400:
        speed = B230400;
        break;
    case 460800:
        speed = B460800;
        break;
    case 921600:
        speed = B921600;
        break;
    case 1000000:
        speed = B1000000;
        break;
    case 1152000:
        speed = B1152000;
        break;
    case 1500000:
        speed = B1500000;
        break;
    case 2000000:
        speed = B2000000;
        break;
    case 2500000:
        speed = B2500000;
        break;
    case 3000000:
        speed = B3000000;
        break;
    case 3500000:
        speed = B3500000;
        break;
    case 4000000:
        speed = B4000000;
        break;

    default:
        AJ_ErrPrintf(("Invalid bitrate %d\n", config->bitrate));
        return AJ_ERR_INVALID;
    }
    cfsetospeed(&ttySettings, speed);
    cfsetispeed(&ttySettings, speed);

    switch (config->devconfig.uart.bits) {
    case 5:
        ttySettings.c_cflag |= CS5;
        break;
    case 6:
        ttySettings.c_cflag |= CS6;
        break;
    case 7:
        ttySettings.c_cflag |= CS7;
        break;
    case 8:
        ttySettings.c_cflag |= CS8;
        break;

    default:
        AJ_ErrPrintf(("Invalid databits %d\n", config->devconfig.uart.bits));
        return AJ_ERR_INVALID;
    }

    switch (config->devconfig.uart.parity) {
    case 0 /* 'n' */:
        ttySettings.c_cflag &= ~(PARENB | PARODD);
        break;
    case 1 /* 'o' */:
        ttySettings.c_iflag |= INPCK;
        ttySettings.c_cflag |= PARENB | PARODD;
        break;
    case 2 /* 'e' */:
        ttySettings.c_iflag |= INPCK;
        ttySettings.c_cflag |= PARENB;
        break;

    default:
        AJ_ErrPrintf(("Invalid parity %s\n", config->devconfig.uart.parity));
        return AJ_ERR_INVALID;
    }

    switch (config->devconfig.uart.stopBits) {
    case 1:
        ttySettings.c_cflag &= ~CSTOPB;
        break;
    case 2:
        ttySettings.c_cflag |= CSTOPB;
        break;

    default:
        AJ_ErrPrintf(("Invalid Invalid stopbits %d\n", config->devconfig.uart.stopBits));
        return AJ_ERR_INVALID;
    }

    ret = open(config->dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (ret == -1) {
        AJ_ErrPrintf(("failed to open serial device %s. ret = %d, %d - %s\n", config->dev, ret, errno, strerror(errno)));
        goto error;
    }
    serialFD = ret;

    /* Lock this FD, to ensure exclusive access to this serial port. */
    ret = flock(serialFD, LOCK_EX | LOCK_NB);
    if (ret) {
        AJ_ErrPrintf(("Lock serialFD %d failed with '%s'\n", serialFD, strerror(errno)));
        goto error;
    }

    AJ_InfoPrintf(("opened serial device %s successfully. serialFD = %d\n", (const char *)config->config, serialFD));

    ret = tcflush(serialFD, TCIOFLUSH);
    if (ret) {
        AJ_ErrPrintf(("Flush serialFD %d failed with '%s'\n", serialFD, strerror(errno)));
        goto error;
    }

    /**
     * Set the new options on the port
     */
    ret = tcsetattr(serialFD, TCSANOW, &ttySettings);
    if (ret) {
        AJ_ErrPrintf(("Set parameters serialFD %d failed with '%s'\n", serialFD, strerror(errno)));
        goto error;
    }

    ret = tcflush(serialFD, TCIOFLUSH);
    if (ret) {
        AJ_ErrPrintf(("Flush serialFD %d failed with '%s'\n", serialFD, strerror(errno)));
        goto error;
    }

    serio_config = config;
    return AJ_OK;

error:
    if (serialFD != -1) {
        close(serialFD);
        serialFD = -1;
    }
    return AJ_ERR_DRIVER;

}

void AJ_SerialIOClose(void)
{
    AJ_InfoPrintf(("AJ_SerialIOClose\n"));
    if (serialFD != -1) {
        close(serialFD);
        serialFD = -1;
    }
}

