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

#include <sys/types.h>

#include <ajtcl/aj_target.h>
#include <ajtcl/aj_status.h>
#include <ajtcl/aj_debug.h>
#include <ajtcl/aj_serial.h>
#include <ajtcl/aj_serial_rx.h>
#include <ajtcl/aj_serial_tx.h>
#include <ajtcl/aj_serio.h>

#ifdef AJ_DEBUG_SERIAL_TARGET
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
#define AJ_SERIAL_DEVICE "/dev/ttyUSB0"

AJ_Status AJ_Serial_Up()
{
    AJ_Status status;

    AJ_Printf("AJ_Serial_Up\n");
    
    return AJ_SerialInit(AJ_SERIAL_DEVICE, 115200, AJ_SERIAL_WINDOW_SIZE, AJ_SERIAL_PACKET_SIZE);
}

AJ_Status AJ_SerialTargetInit(const char* ttyName, uint32_t bitRate)
{
    AJ_SerIOConfig config;

    AJ_Printf("AJ_SerialTargetInit %s\n", ttyName);

    config.bitrate = bitRate;
    config.config = (const void *)ttyName;
    /* TODO: stop bits, etc... */

    return  AJ_SerialIOInit(&config);
}

/**
 * Interrupt handler for data arriving on the UART
 */
static void readBytesFromUart(uint8_t* data, uint32_t len)
{
    static uint8_t ReadingMsg = FALSE;

    // if there is data ready,
    AJ_Printf("readBytesFromUart: %d\n", len);
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
    //AJ_Printf("AJ_RX: %d\n", len);
    AJ_ASSERT(buf != NULL);
    rx_buf.data = buf;
    rx_buf.pos = buf;
    rx_buf.len = len;
}

void AJ_PauseRX()
{
    // Disable RX IRQ
}

void AJ_ResumeRX()
{
    // Enable RX IRQ
}

static void runTx()
{
    uint32_t len = tx_buf.len - (tx_buf.pos - tx_buf.data);

    if (len) {
        AJ_Printf("runTx: (%d)\n", len);
    }

    if (!tx_buf.data || !tx_buf.pos || !tx_buf.len || !len /*|| !aci_state.data_credit_available*/) {
        return;
    }
/*
    while (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) &&
           len && (aci_state.data_credit_available >= 1)) {
        uint32_t send_len = len > ACI_PIPE_TX_DATA_MAX_LEN ? ACI_PIPE_TX_DATA_MAX_LEN : len;
        int status;

        status = lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, tx_buf.pos, send_len);
        if (status) {
            aci_state.data_credit_available--;
            tx_buf.pos += send_len;
            len -= send_len;
        } else {
            break;
        }
    }
    AJ_Printf("Tx (credits: %d)\n", aci_state.data_credit_available);
*/
}

/* This function is requesting us to send data over our UART */
void __AJ_TX(uint8_t* buf, uint32_t len)
{
    AJ_Printf("__AJ_TX: %d\n", len);
    tx_buf.data = buf;
    tx_buf.pos = buf;
    tx_buf.len = len;

    runTx();
}

void AJ_TX(uint8_t* buf, uint32_t len)
{
    AJ_Printf("AJ_TX: %d\n", len);
    g_AJ_TX(buf, len); // should call the inner implementation
}

void AJ_PauseTX()
{
    // Disable TX IRQ
}

void AJ_ResumeTX()
{
    // Enable TX IRQ
}


AJ_Status AJ_SerialIOEnable(uint32_t direction, uint8_t enable)
{
    AJ_Printf("AJ_SerialIOEnable -->%d: %d\n", direction, enable);
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
    AJ_Printf("AJ_SetRxCB\n");
    RecvCB = rx_cb;
}

void AJ_SetTxCB(AJ_SerIOTxCompleteFunc tx_cb)
{
    AJ_Printf("AJ_SetTxCB\n");
    SendCB = tx_cb;
}

void AJ_SetTxSerialTransmit(AJ_SerialTxFunc tx_func)
{
    AJ_Printf("AJ_SetTxSerialTransmit\n");
    g_AJ_TX = tx_func;
}


AJ_Status AJ_SerialIOInit(AJ_SerIOConfig* config)
{
    AJ_Printf("AJ_SerialIOInit\n");

//    AJ_SetSioCheck(aci_loop);

    return AJ_OK;
}

AJ_Status AJ_SerialIOShutdown(void)
{
    AJ_Printf("AJ_SerialIOShutdown\n");
    return AJ_OK;
}

