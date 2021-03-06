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
#define AJ_MODULE TARGET_UTIL
#include "aj_target.h"
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <byteswap.h>
#include <stdarg.h>
#include <arpa/inet.h>
#include <ajtcl/aj_debug.h>
#include <ajtcl/aj_util.h>

uint8_t dbgTARGET_UTIL = 0;

void AJ_Sleep(uint32_t ms)
{
    struct timespec waittime = { };
    waittime.tv_sec = ms / 1000;
    waittime.tv_nsec = (ms % 1000) * 1000000LL;

    // nanosleep returns the amount of time slept before being interrupted by a signal,
    // so loop until the full sleep is finished
    while (nanosleep(&waittime, &waittime) == -1) {
        continue;
    }

}

#ifndef NDEBUG
AJ_Status _AJ_GetDebugTime(AJ_Time* timer)
{
    static int useEpoch = -1;
    char* env;
    struct timespec now;
    AJ_Status status = AJ_ERR_RESOURCES;

    if (useEpoch == -1) {
        env = getenv("ER_DEBUG_EPOCH");
        useEpoch = env && (strcmp(env, "1") == 0);
    }

    if (useEpoch) {
        clock_gettime(CLOCK_REALTIME, &now);
        timer->seconds = now.tv_sec;
        timer->milliseconds = now.tv_nsec / 1000000;
        status = AJ_OK;
    }
    return status;
}
#endif

uint32_t AJ_GetElapsedTime(AJ_Time* timer, uint8_t cumulative)
{
    uint32_t elapsed;
    struct timespec now;

    clock_gettime(CLOCK_MONOTONIC, &now);

    elapsed = (1000 * (now.tv_sec - timer->seconds)) + ((now.tv_nsec / 1000000) - timer->milliseconds);
    if (!cumulative) {
        timer->seconds = now.tv_sec;
        timer->milliseconds = now.tv_nsec / 1000000;
    }
    return elapsed;
}
void AJ_InitTimer(AJ_Time* timer)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    timer->seconds = now.tv_sec;
    timer->milliseconds = now.tv_nsec / 1000000;

}

int32_t AJ_GetTimeDifference(AJ_Time* timerA, AJ_Time* timerB)
{
    int32_t diff;

    diff = (1000 * (timerA->seconds - timerB->seconds)) + (timerA->milliseconds - timerB->milliseconds);
    return diff;
}

void AJ_TimeAddOffset(AJ_Time* timerA, uint32_t msec)
{
    uint32_t msecNew;
    if (msec == -1) {
        timerA->seconds = -1;
        timerA->milliseconds = -1;
    } else {
        msecNew = (timerA->milliseconds + msec);
        timerA->seconds = timerA->seconds + (msecNew / 1000);
        timerA->milliseconds = msecNew % 1000;
    }
}


int8_t AJ_CompareTime(AJ_Time timerA, AJ_Time timerB)
{
    if (timerA.seconds == timerB.seconds) {
        if (timerA.milliseconds == timerB.milliseconds) {
            return 0;
        } else if (timerA.milliseconds > timerB.milliseconds) {
            return 1;
        } else {
            return -1;
        }
    } else if (timerA.seconds > timerB.seconds) {
        return 1;
    } else {
        return -1;
    }
}

uint64_t AJ_DecodeTime(char* der, const char* fmt)
{
    struct tm tm;
    if (!strptime(der, fmt, &tm)) {
        return 0;
    }
    return (uint64_t) timegm(&tm);
}

void* AJ_Malloc(size_t sz)
{
    return malloc(sz);
}
void* AJ_Realloc(void* ptr, size_t size)
{
    return realloc(ptr, size);
}

void AJ_Free(void* mem)
{
    if (mem) {
        free(mem);
    }
}

void AJ_MemZeroSecure(void* s, size_t n)
{
    volatile unsigned char* p = s;
    while (n--) *p++ = '\0';
    return;
}

/*
 * get a line of input from the the file pointer (most likely stdin).
 * This will capture the the num-1 characters or till a newline character is
 * entered.
 *
 * @param[out] str a pointer to a character array that will hold the user input
 * @param[in]  num the size of the character array 'str'
 * @param[in]  fp  the file pointer the sting will be read from. (most likely stdin)
 *
 * @return returns the same string as 'str' if there has been a read error a null
 *                 pointer will be returned and 'str' will remain unchanged.
 */
char*AJ_GetLine(char*str, size_t num, void*fp)
{
    char*p = fgets(str, num, fp);

    if (p != NULL) {
        size_t last = strlen(str) - 1;
        if (str[last] == '\n') {
            str[last] = '\0';
        }
    }
    return p;
}

static uint8_t ioThreadRunning = FALSE;
static char cmdline[1024];
static uint8_t consumed = TRUE;
static pthread_t threadId;

void* RunFunc(void* threadArg)
{
    while (ioThreadRunning) {
        if (consumed) {
            AJ_GetLine(cmdline, sizeof(cmdline), stdin);
            consumed = FALSE;
        }
        AJ_Sleep(1000);
    }
    return 0;
}

uint8_t AJ_StartReadFromStdIn()
{
    int ret = 0;
    if (!ioThreadRunning) {
        ret = pthread_create(&threadId, NULL, RunFunc, NULL);
        if (ret != 0) {
            AJ_ErrPrintf(("Error: fail to spin a thread for reading from stdin\n"));
        }
        ioThreadRunning = TRUE;
        return TRUE;
    }
    return FALSE;
}

char* AJ_GetCmdLine(char* buf, size_t num)
{
    if (!consumed) {
        strncpy(buf, cmdline, num);
        buf[num - 1] = '\0';
        consumed = TRUE;
        return buf;
    }
    return NULL;
}

uint8_t AJ_StopReadFromStdIn()
{
    void* exit_status;
    if (ioThreadRunning) {
        ioThreadRunning = FALSE;
        pthread_join(threadId, &exit_status);
        return TRUE;
    }
    return FALSE;
}

#ifndef NDEBUG

/*
 * This is not intended, nor required to be particularly efficient.  If you want
 * efficiency, turn of debugging.
 */
int _AJ_DbgEnabled(const char* module)
{
    char buffer[128];
    char* env;

    strcpy(buffer, "ER_DEBUG_ALL");
    env = getenv(buffer);
    if (env && strcmp(env, "1") == 0) {
        return TRUE;
    }

    strcpy(buffer, "ER_DEBUG_");
    strcat(buffer, module);
    env = getenv(buffer);
    if (env && strcmp(env, "1") == 0) {
        return TRUE;
    }

    return FALSE;
}

#endif

uint16_t AJ_ByteSwap16(uint16_t x)
{
    return bswap_16(x);
}

uint32_t AJ_ByteSwap32(uint32_t x)
{
    return bswap_32(x);
}

uint64_t AJ_ByteSwap64(uint64_t x)
{
    return bswap_64(x);
}

AJ_Status AJ_IntToString(int32_t val, char* buf, size_t buflen)
{
    AJ_Status status = AJ_OK;
    int c = snprintf(buf, buflen, "%d", val);
    if (c <= 0 || c > buflen) {
        status = AJ_ERR_RESOURCES;
    }
    return status;
}

AJ_Status AJ_InetToString(uint32_t addr, char* buf, size_t buflen)
{
    AJ_Status status = AJ_OK;
    int c = snprintf((char*)buf, buflen, "%u.%u.%u.%u", (addr & 0xFF000000) >> 24, (addr & 0x00FF0000) >> 16, (addr & 0x0000FF00) >> 8, (addr & 0x000000FF));
    if (c <= 0 || c > buflen) {
        status = AJ_ERR_RESOURCES;
    }
    return status;
}

static FILE* logFile = NULL;
static uint32_t logLim = 0;

int AJ_SetLogFile(const char* file, uint32_t maxLen)
{
    if (logFile) {
        fclose(logFile);
    }
    if (!file) {
        logFile = NULL;
    } else {
        logFile = fopen(file, "w+");
        if (!logFile) {
            return -1;
        }
        logLim = maxLen / 2;
    }
    return 0;
}

void AJ_Printf(const char* fmat, ...)
{
    va_list args;

    va_start(args, fmat);
    if (logFile) {
        vfprintf(logFile, fmat, args);
        if (logLim) {
            /*
             * Don't allow the log file to grow to more than 2 x logLim bytes
             */
            long pos = ftell(logFile);
            if (pos >= (2 * logLim)) {
                void* buf = malloc(logLim);
                if (buf) {
                    fseek(logFile, -logLim, SEEK_CUR);
                    fread(buf, logLim, 1, logFile);
                    fseek(logFile, 0, SEEK_SET);
                    ftruncate(fileno(logFile), 0);
                    fwrite(buf, logLim, 1, logFile);
                    free(buf);
                }
            }
        }
        fflush(logFile);
    } else {
        vprintf(fmat, args);
        fflush(stdout);
    }
    va_end(args);
}
