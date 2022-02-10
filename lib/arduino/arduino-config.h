#pragma once

#include "sdkconfig.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef F_CPU
#if CONFIG_IDF_TARGET_ESP32 // ESP32/PICO-D4
#define F_CPU (CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ * 1000000U)
#elif CONFIG_IDF_TARGET_ESP32S2
#define F_CPU (CONFIG_ESP32S2_DEFAULT_CPU_FREQ_MHZ * 1000000U)
#endif
#endif

#if CONFIG_ARDUINO_ISR_IRAM
#define ARDUINO_ISR_ATTR IRAM_ATTR
#define ARDUINO_ISR_FLAG ESP_INTR_FLAG_IRAM
#else
#define ARDUINO_ISR_ATTR
#define ARDUINO_ISR_FLAG (0)
#endif

#ifndef CONFIG_ARDUINO_RUNNING_CORE
#define CONFIG_ARDUINO_RUNNING_CORE 1
#endif

#ifndef CONFIG_ARDUINO_EVENT_RUNNING_CORE
#define CONFIG_ARDUINO_EVENT_RUNNING_CORE 1
#endif

#ifndef ARDUINO_RUNNING_CORE
#define ARDUINO_RUNNING_CORE CONFIG_ARDUINO_RUNNING_CORE
#endif

#ifndef ARDUINO_EVENT_RUNNING_CORE
#define ARDUINO_EVENT_RUNNING_CORE CONFIG_ARDUINO_EVENT_RUNNING_CORE
#endif

#ifndef CONFIG_ARDUINO_UDP_RUNNING_CORE
#define CONFIG_ARDUINO_UDP_RUNNING_CORE 1
#endif

#ifndef CONFIG_ARDUINO_UDP_TASK_PRIORITY
#define CONFIG_ARDUINO_UDP_TASK_PRIORITY 3
#endif

#define ESP_REG(addr) *((volatile uint32_t *)(addr))
#define NOP() asm volatile ("nop")

#ifdef __cplusplus
}
#endif