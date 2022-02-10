//Based on esp32-hal.h
#ifndef ARDUINOLOOP_H_
#define ARDUINOLOOP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

#ifndef ARDUINO_LOOP_STACK_SIZE
#ifndef CONFIG_ARDUINO_LOOP_STACK_SIZE
#define ARDUINO_LOOP_STACK_SIZE 8192
#else
#define ARDUINO_LOOP_STACK_SIZE CONFIG_ARDUINO_LOOP_STACK_SIZE
#endif
#endif

//returns chip temperature in Celsius
float temperatureRead();

#if CONFIG_AUTOSTART_ARDUINO
//enable/disable WDT for Arduino's setup and loop functions
void enableLoopWDT();
void disableLoopWDT();
//feed WDT for the loop task
void feedLoopWDT();
#endif

//enable/disable WDT for the IDLE task on Core 0 (SYSTEM)
void enableCore0WDT();
void disableCore0WDT();
#ifndef CONFIG_FREERTOS_UNICORE
//enable/disable WDT for the IDLE task on Core 1 (Arduino)
void enableCore1WDT();
void disableCore1WDT();
#endif

//if xCoreID < 0 or CPU is unicore, it will use xTaskCreate, else xTaskCreatePinnedToCore
//allows to easily handle all possible situations without repetitive code
BaseType_t xTaskCreateUniversal( TaskFunction_t pxTaskCode,
                        const char * const pcName,
                        const uint32_t usStackDepth,
                        void * const pvParameters,
                        UBaseType_t uxPriority,
                        TaskHandle_t * const pxCreatedTask,
                        const BaseType_t xCoreID );

unsigned long micros();
unsigned long millis();
void delay(uint32_t);
void delayMicroseconds(uint32_t us);

#if !CONFIG_ESP32_PHY_AUTO_INIT
void arduino_phy_init();
#endif

#if !CONFIG_AUTOSTART_ARDUINO
void initArduino();
size_t getArduinoLoopTaskStackSize(void);
#endif

void StartArduino(void);
extern void loop(void);     //Must exist in user code
extern void setup(void);    //Must exist in user code
void yield(void);
#define optimistic_yield(u)

#ifdef __cplusplus
}
#endif

#endif // ARDUINOLOOP
