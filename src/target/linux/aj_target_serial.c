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

static struct {
    uint8_t* data;
    uint32_t len;
    uint8_t* pos;
} tx_buf = { NULL, 0, NULL }, rx_buf = { NULL, 0, NULL };

#define AJ_SERIAL_WINDOW_SIZE   2
#define AJ_SERIAL_PACKET_SIZE   512 + AJ_SERIAL_HDR_LEN

static const AJ_SerIOConfig serio_config = {
#if defined(AJ_SOCKETCAN) // CAN
    .dev = "vcan0",
    .bitrate = 1000000,
    .devconfig.can.target_can_id = 0x4BA,
    .devconfig.can.server_can_id = 0x3BA
#else // UART
    .dev = "/dev/ttyUSB0",
    .bitrate = 115200,
    .devconfig.uart.bits = 8,
    .devconfig.uart.stopBits = 1,
    .devconfig.uart.parity = 0
#endif
};

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
        AJ_ErrPrintf(("\n rx mutex init failed\n"));
        return AJ_ERR_DRIVER;
    }

    if (pthread_mutex_init(&tx_lock, NULL) != 0)
    {
        AJ_ErrPrintf(("\n tx mutex init failed\n"));
        return AJ_ERR_DRIVER;
    }

    status = AJ_SerialInit(serio_config.dev, serio_config.bitrate, AJ_SERIAL_WINDOW_SIZE, AJ_SERIAL_PACKET_SIZE);

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
    AJ_InfoPrintf(("AJ_SerialTargetInit %s\n", ttyName));
    return  AJ_SerialIOInit(&serio_config);
}

/**
 * Interrupt handler for data arriving on the UART
 */
void AJ_ReadBytesFromUart(uint8_t* data, uint32_t len)
{
    static uint8_t ReadingMsg = FALSE;

    // if there is data ready,
    //AJ_InfoPrintf(("AJ_ReadBytesFromUart: %d\n", len));
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
                AJ_InfoPrintf(("AJ_ReadBytesFromUart: got BOUNDARY_BYTE, data count %d\n", cnt));
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
    //AJ_InfoPrintf(("AJ_PauseRX\n"));
    pthread_mutex_trylock(&rx_lock);   
}

void AJ_ResumeRX()
{
    // Enable RX IRQ
    //AJ_InfoPrintf(("AJ_ResumeRX\n"));
    pthread_mutex_unlock(&rx_lock);
}

/* This frame size is chosen so that most of the SLAP packets fit into one frame.
 * If the packet doesnt fit within this, it will be read using two calls to read().
 */
#define RX_BUFSIZE  (640)
static uint8_t RxBuffer[RX_BUFSIZE];

static void *runRx(void *arg)
{

    AJ_InfoPrintf(("runRx thread started\n"));

    for (;;) {
        int ret;
        ret = AJ_SerialIOBlockingRead(RxBuffer, sizeof(RxBuffer));
        if (ret == -1) {
            AJ_ErrPrintf(("runRx(): AJ_SerialIOBlockingRead failed, exiting\n"));
            return NULL;
        }
        //AJ_InfoPrintf(("runRx: read %d\n", ret));

        if (pthread_mutex_lock(&rx_lock) == 0) {   
            AJ_ReadBytesFromUart(RxBuffer, ret);
        } else {
            AJ_ErrPrintf(("runRx(): pthread_mutex_lock failed, assume shutdown, exiting.\n"));
            return NULL;
        }
        pthread_mutex_unlock(&rx_lock);
    }

    return NULL;
}


static void runTx()
{
    uint32_t len = tx_buf.len - (tx_buf.pos - tx_buf.data);

    //AJ_InfoPrintf(("runTx: (%d)\n", len));

    if (!len) {
    return;
    }

    if (!tx_buf.data || !tx_buf.pos || !tx_buf.len) {
        AJ_ErrPrintf(("runTx: bad status, return\n"));
        return;
    }

    while (len) {
        int ret = AJ_SerialIOWriteBytes(tx_buf.pos, len);
        if (ret == -1) {
            AJ_ErrPrintf(("runTx: AJ_SerialIOWriteBytes failed, exiting\n"));
            break;
        } else {
            len -= ret;
            tx_buf.pos += ret;
        }
    }
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
        AJ_InfoPrintf(("__AJ_TX: WARN: No Acknowledge yet after runTx\n"));
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
//    pthread_mutex_trylock(&tx_lock);
}

void AJ_ResumeTX()
{
    // Enable TX IRQ
//    AJ_InfoPrintf(("AJ_ResumeTX\n"));
//    pthread_mutex_unlock(&tx_lock);
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


AJ_Status AJ_SerialIOShutdown(void)
{
    AJ_InfoPrintf(("AJ_SerialIOShutdown\n"));
    pthread_cancel(rx_t);
    pthread_mutex_destroy(&rx_lock);
    pthread_mutex_destroy(&tx_lock);
    AJ_SerialIOClose();
    pthread_join(rx_t, NULL);
    return AJ_OK;
}

