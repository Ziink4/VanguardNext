//-------------------------------------------------------------------------
//  log.h
//-------------------------------------------------------------------------

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LOG_FATAL      0
#define LOG_ERROR      1
#define LOG_WARNING    2
#define LOG_INFO       3
#define LOG_DEBUG      4

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_INFO
#endif

#ifdef DEBUG
#define LOG_ENABLE 1
#endif

#if LOG_ENABLE
#include <SEGGER_RTT.h>
#define log_init()                SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP)
#define log_write(buf, len)       SEGGER_RTT_Write(0, (buf), (len))
#define log_printf(...)           SEGGER_RTT_printf(0, __VA_ARGS__)
#define log_vprintf(fmt, va_list) SEGGER_RTT_vprintf(0, (fmt), (va_list))
void log_log(int level, const char* fmt, ...);
void log_dumpmem(const void* mem, size_t len);
#else
#define log_init()
#define log_write(...)
#define log_printf(...)
#define log_vprintf(...)
#define log_log(...)
#define log_dumpmem(...)
#endif

#if LOG_LEVEL >= LOG_FATAL
#define LOG_LOGF(...) log_log(LOG_FATAL, __VA_ARGS__)
#else
#define LOG_LOGF(...)
#endif

#if LOG_LEVEL >= LOG_ERROR
#define LOG_LOGE(...) log_log(LOG_ERROR, __VA_ARGS__)
#else
#define LOG_LOGE(...)
#endif

#if LOG_LEVEL >= LOG_WARNING
#define LOG_LOGW(...) log_log(LOG_WARNING, __VA_ARGS__)
#else
#define LOG_LOGW(...)
#endif

#if LOG_LEVEL >= LOG_INFO
#define LOG_LOGI(...) log_log(LOG_INFO, __VA_ARGS__)
#else
#define LOG_LOGI(...)
#endif

#if LOG_LEVEL >= LOG_DEBUG
#define LOG_LOGD(...) log_log(LOG_DEBUG, __VA_ARGS__)
#else
#define LOG_LOGD(...)
#endif

#ifdef __cplusplus
}
#endif
