/**
 * @file
 */
/******************************************************************************
 * Copyright (c) 2012, AllSeen Alliance. All rights reserved.
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

#include <assert.h>

#include "aj_target.h"
#include "aj_status.h"
#include "aj_bufio.h"

void AJ_IOBufInit(AJ_IOBuffer* ioBuf, uint8_t* buffer, uint32_t bufLen, uint8_t direction, void* context)
{
    ioBuf->bufStart = buffer;
    ioBuf->bufSize = bufLen;
    ioBuf->readPtr = buffer;
    ioBuf->writePtr = buffer;
    ioBuf->direction = direction;
    ioBuf->context = context;
}

void AJ_IOBufRebase(AJ_IOBuffer* ioBuf)
{
    int32_t unconsumed = AJ_IO_BUF_AVAIL(ioBuf);
    /*
     * Move any unconsumed data to the start of the I/O buffer
     */
    if (unconsumed) {
        memmove(ioBuf->bufStart, ioBuf->readPtr, unconsumed);
    }

    ioBuf->readPtr = ioBuf->bufStart;
    ioBuf->writePtr = ioBuf->bufStart + unconsumed;
}
