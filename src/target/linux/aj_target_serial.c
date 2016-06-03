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
#define AJ_MODULE TARGET_SERIAL

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

#ifdef AJ_DEBUG_TARGET_SERIAL
#define AJ_DebugDumpSerialRX(a, b, c) AJ_DumpBytes(a, b, c)
#define AJ_DebugDumpSerialTX(a, b, c) AJ_DumpBytes(a, b, c)
#else
#define AJ_DebugDumpSerialRX(a, b, c)
#define AJ_DebugDumpSerialTX(a, b, c)
#endif

/**
 * Turn on per-module debug printing by setting this variable to non-zero value
 * (usually in debugger).
 */
#ifndef NDEBUG
uint8_t dbgTARGET_SERIAL = 0;
#endif

static AJ_SerIORxCompleteFunc RecvCB;
static AJ_SerIOTxCompleteFunc SendCB;
static void *runRx(void *arg);

/**
 * global function pointer for serial transmit funciton
 */
AJ_SerialTxFunc g_AJ_TX;

/** Serial Port FD */
static int serialFD = -1;

static struct {
    uint8_t* data;
    uint32_t len;
    uint8_t* pos;
} tx_buf = { NULL, 0, NULL }, rx_buf = { NULL, 0, NULL };

#define AJ_SERIAL_WINDOW_SIZE   2
#define AJ_SERIAL_PACKET_SIZE   512 + AJ_SERIAL_HDR_LEN
#define AJ_SERIAL_DEVICE "/dev/ttyUSB0"
#define AJ_SERIAL_BITRATE 115200
#define AJ_SERIAL_BITS 8
#define AJ_SERIAL_STOPBITS 1
#define AJ_SERIAL_PARITY 0 /* Zero disables parity checking, one means odd and two means even parity */

static pthread_mutex_t rx_lock;
static pthread_mutex_t tx_lock;
static pthread_t rx_t;

AJ_Status AJ_Serial_Up()
{
    AJ_Status status;
    int err;

    AJ_InfoPrintf(("AJ_Serial_Up\n"));

    if (pthread_mutex_init(&rx_lock, NULL) != 0)
    {
        printf("\n rx mutex init failed\n");
        return AJ_ERR_DRIVER;
    }

    if (pthread_mutex_init(&tx_lock, NULL) != 0)
    {
        printf("\n tx mutex init failed\n");
        return AJ_ERR_DRIVER;
    }

    status = AJ_SerialInit(AJ_SERIAL_DEVICE, AJ_SERIAL_BITRATE, AJ_SERIAL_WINDOW_SIZE, AJ_SERIAL_PACKET_SIZE);

    if (status == AJ_OK) {
        err = pthread_create(&rx_t, NULL, &runRx, NULL);
        if (err != 0) {
            printf("\ncan't create thread :[%s]", strerror(err));
            return AJ_ERR_DRIVER;
        }
    }

    return status;
}

AJ_Status AJ_SerialTargetInit(const char* ttyName, uint32_t bitRate)
{
    AJ_SerIOConfig config;

    AJ_InfoPrintf(("AJ_SerialTargetInit %s\n", ttyName));

    /* TODO: read from nvram, etc? */
    config.bitrate = bitRate;
    config.config = (const void *)ttyName;
    config.bits = AJ_SERIAL_BITS;
    config.stopBits = AJ_SERIAL_STOPBITS;
    config.parity = AJ_SERIAL_PARITY;

    return  AJ_SerialIOInit(&config);
}

/**
 * Interrupt handler for data arriving on the UART
 */
static void readBytesFromUart(uint8_t* data, uint32_t len)
{
    static uint8_t ReadingMsg = FALSE;

    // if there is data ready,
    //AJ_InfoPrintf(("readBytesFromUart: %d\n", len));
    AJ_ASSERT(rx_buf.data != NULL);
    while (len) {
        if (rx_buf.pos >= rx_buf.data + rx_buf.len) {
            // throw data away until we see a new frame
            if (*data == BOUNDARY_BYTE) {
                ReadingMsg = FALSE;
                rx_buf.pos = rx_buf.data;
            }
            data++;
            len--;
            continue;
        }

        *(rx_buf.pos++) = *data;

        if (*data == BOUNDARY_BYTE) {
            if (ReadingMsg == TRUE) {
                uint8_t*buf = rx_buf.data;
                uint32_t cnt = rx_buf.pos - rx_buf.data;
                rx_buf.pos = rx_buf.data = NULL;
                rx_buf.len = 0;
                ReadingMsg = FALSE;
                AJ_InfoPrintf(("readBytesFromUart: got BOUNDARY_BYTE, data count %d\n", cnt));
                RecvCB(buf, cnt);
            } else {
                ReadingMsg = TRUE;
            }
        }
        data++;
        len--;
    }
}

/* This function sets up a buffer for us to fill with RX data */
void AJ_RX(uint8_t* buf, uint32_t len)
{
    AJ_InfoPrintf(("AJ_RX: %d\n", len));
    AJ_ASSERT(buf != NULL);
    rx_buf.data = buf;
    rx_buf.pos = buf;
    rx_buf.len = len;
}

void AJ_PauseRX()
{
    // Disable RX IRQ
    AJ_InfoPrintf(("AJ_PauseRX\n"));
    pthread_mutex_trylock(&rx_lock);   
}

void AJ_ResumeRX()
{
    // Enable RX IRQ
    AJ_InfoPrintf(("AJ_ResumeRX\n"));
    pthread_mutex_unlock(&rx_lock);
}

static void *runRx(void *arg)
{
    fd_set fds;
    int rc;
    int ret = 0;

    AJ_InfoPrintf(("runRx\n"));

    for (;;) {
        FD_ZERO(&fds);
        FD_SET(serialFD, &fds);

        rc = select(serialFD + 1, &fds, NULL, NULL, NULL);
        if (rc <= 0) {
            AJ_ErrPrintf(("runRx(): select() failed, rc=%d, errno=\"%s\"\n", rc, strerror(errno)));
            return NULL;
        }

        if (FD_ISSET(serialFD, &fds)) {
            uint8_t buf[7]; /* TODO? */
            if ((ret = read(serialFD, buf, sizeof(buf))) <= 0) {
                AJ_ErrPrintf(("runRx(): read() failed, ret=%d, errno=\"%s\"\n", ret, strerror(errno)));
                continue;
            }

            //AJ_InfoPrintf(("runRx: read %d\n", ret));

            pthread_mutex_lock(&rx_lock);   
            readBytesFromUart(buf, ret);
            pthread_mutex_unlock(&rx_lock);
        }
    }

    return NULL;
}


static void runTx()
{
    uint32_t len = tx_buf.len - (tx_buf.pos - tx_buf.data);

    AJ_InfoPrintf(("runTx: (%d)\n", len));

    if (!len) {
    return;
    }

    if (!tx_buf.data || !tx_buf.pos || !tx_buf.len) {
        AJ_InfoPrintf(("runTx: bad status, return\n"));
        return;
    }

    while (len) {
        int ret = write(serialFD, tx_buf.pos, len);
        if (ret == -1) {
        if (errno == EAGAIN) {
            /* ? */
            AJ_InfoPrintf(("runTx: write() EAGAIN\n"));
        } else {
            AJ_InfoPrintf(("runTx: write() failed (fd = %u): %d - %s\n", serialFD, errno, strerror(errno)));
        }
        break;
        } else {
        len -= ret;
                tx_buf.pos += ret;
        }
    }

    AJ_InfoPrintf(("runTx: finished, remaining bytes %d\n", len));
}

/* This function is requesting us to send data over our UART */
void __AJ_TX(uint8_t* buf, uint32_t len)
{
    AJ_InfoPrintf(("__AJ_TX: %d\n", len));
    tx_buf.data = buf;
    tx_buf.pos = buf;
    tx_buf.len = len;

    //    pthread_mutex_lock(&tx_lock);
    runTx();
    if (tx_buf.data && (tx_buf.pos >= tx_buf.data + tx_buf.len)) {
        uint8_t* b = tx_buf.data;
        uint32_t l = tx_buf.pos - tx_buf.data;

        /* Reset Tx data structure */
        tx_buf.data = tx_buf.pos = NULL;
        tx_buf.len = 0;

        /* Acknowledge Send completion to upper layer */
        AJ_InfoPrintf(("__AJ_TX: Acknowledge Send completion to upper layer, len=%d\n", l));
        SendCB(b, l);
    } else {
        AJ_InfoPrintf(("__AJ_TX: No Acknowledge yet after runTx\n"));
    }
    //    pthread_mutex_unlock(&tx_lock);
}

void AJ_TX(uint8_t* buf, uint32_t len)
{
    AJ_InfoPrintf(("AJ_TX: %d\n", len));
    g_AJ_TX(buf, len); // should call the inner implementation
}

void AJ_PauseTX()
{
    // Disable TX IRQ
//    AJ_InfoPrintf(("AJ_PauseTX\n"));
    pthread_mutex_trylock(&tx_lock);
}

void AJ_ResumeTX()
{
    // Enable TX IRQ
//    AJ_InfoPrintf(("AJ_ResumeTX\n"));
    pthread_mutex_unlock(&tx_lock);
}


AJ_Status AJ_SerialIOEnable(uint32_t direction, uint8_t enable)
{
    AJ_InfoPrintf(("AJ_SerialIOEnable -->%d: %d\n", direction, enable));
    AJ_Status status = AJ_OK;

    if (direction == AJ_SERIO_RX) {
        if (enable) {
        } else {
        }
    } else if (direction == AJ_SERIO_TX) {
        if (enable) {
        } else {
        }
    }

    return status;
}

void AJ_SetRxCB(AJ_SerIORxCompleteFunc rx_cb)
{
    AJ_InfoPrintf(("AJ_SetRxCB\n"));
    RecvCB = rx_cb;
}

void AJ_SetTxCB(AJ_SerIOTxCompleteFunc tx_cb)
{
    AJ_InfoPrintf(("AJ_SetTxCB\n"));
    SendCB = tx_cb;
}

void AJ_SetTxSerialTransmit(AJ_SerialTxFunc tx_func)
{
    AJ_InfoPrintf(("AJ_SetTxSerialTransmit\n"));
    g_AJ_TX = tx_func;
}


AJ_Status AJ_SerialIOInit(AJ_SerIOConfig* config)
{
    int ret;
    struct termios ttySettings;
    speed_t speed;

    AJ_InfoPrintf(("AJ_SerialIOInit\n"));

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

    switch (config->bits) {
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
        AJ_ErrPrintf(("Invalid databits %d\n", config->bits));
        return AJ_ERR_INVALID;
    }

    switch (config->parity) {
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
        AJ_ErrPrintf(("Invalid parity %s\n", config->parity));
        return AJ_ERR_INVALID;
    }

    switch (config->stopBits) {
    case 1:
        ttySettings.c_cflag &= ~CSTOPB;
        break;
    case 2:
        ttySettings.c_cflag |= CSTOPB;
        break;

    default:
        AJ_ErrPrintf(("Invalid Invalid stopbits %d\n", config->stopBits));
        return AJ_ERR_INVALID;
    }

    ret = open((const char *)config->config, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (ret == -1) {
        AJ_ErrPrintf(("failed to open serial device %s. ret = %d, %d - %s\n", (const char *)config->config, ret, errno, strerror(errno)));
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

//    AJ_SetSioCheck(aci_loop);
    return AJ_OK;

error:
    if (serialFD != -1) {
        close(serialFD);
        serialFD = -1;
    }
    return AJ_ERR_DRIVER;

}

AJ_Status AJ_SerialIOShutdown(void)
{
    AJ_InfoPrintf(("AJ_SerialIOShutdown\n"));
    pthread_mutex_destroy(&rx_lock);
    pthread_mutex_destroy(&tx_lock);
    /* TODO: stop rx thread */
    return AJ_OK;
}

