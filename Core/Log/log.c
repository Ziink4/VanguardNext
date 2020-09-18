//-------------------------------------------------------------------------
//  log.c
//-------------------------------------------------------------------------

#include "log.h"

#include <stdarg.h>

static const char BEGIN_FATAL[] = "\033[91mFATAL: ";
static const char BEGIN_ERROR[] =  "\033[31mERROR: ";
static const char BEGIN_WARNING[] = "\033[33mWARNING: ";
static const char BEGIN_INFO[] = "\033[32mINFO: ";
static const char BEGIN_DEBUG[] =  "DEBUG: ";
static const char END_LOG[] = "\n\033[0m";

void log_log(int level, const char *fmt, ...)
{
    if (level == LOG_FATAL){
        log_write(BEGIN_FATAL, sizeof(BEGIN_FATAL) - 1);
    }else if (level == LOG_ERROR){
        log_write(BEGIN_ERROR, sizeof(BEGIN_ERROR) - 1);
    }else if (level == LOG_WARNING){
        log_write(BEGIN_WARNING, sizeof(BEGIN_WARNING) - 1);
    }else if (level == LOG_INFO){
        log_write(BEGIN_INFO, sizeof(BEGIN_INFO) - 1);
    }else if (level == LOG_DEBUG){
        log_write(BEGIN_DEBUG, sizeof(BEGIN_DEBUG) - 1);
    }
 
    va_list args;
    va_start(args, fmt);
    log_vprintf(fmt, &args);
    va_end(args);

    log_write(END_LOG, sizeof(END_LOG) - 1);
}

static void print_memline(uint32_t addr, const char* buf)
{
    log_printf("    %.8X : %s\n", addr, buf);
}

void log_dumpmem(const void* mem, size_t len)
{
    static const char hex[] = "0123456789ABCDEF";
    char buf[16 * 2 + 4];
    const uint8_t* data = (const uint8_t*)mem;
    char* out = buf;
    uint32_t addr = (uint32_t)mem;
    for (int i = 0; i < len; i++){
        *out = hex[(data[i] & 0xf0) >> 4];
        out++;
        *out = hex[data[i] & 0x0f];
        out++;
        if (i % 4 == 3){
            *out = ' ';
            out++;
        }
        if (i % 16 == 15){
            *out = '\0';
            print_memline(addr, buf);
            out = buf;
            addr += 16;
        }
    }
    if (out != buf){
        *out = '\0';
        print_memline(addr, buf);
    }
}
