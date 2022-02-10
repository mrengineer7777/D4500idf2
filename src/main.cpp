#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "arduinoloop.h"
#include "HardwareSerial.h"
#include "Microframe.h"

extern "C" {
  #include "esp_task_wdt.h"
}

TaskHandle_t mainloopTaskHandle = NULL;

//-------------------------------Prototypes----------------------------------
void loop(void);
void loopTask(void *);
void setup(void);
void someTask(void *);
void main_HWini(void);
void main_ModuleStart(void);
//-------------------------------Prototypes----------------------------------

#if CONFIG_AUTOSTART_ARDUINO                                                        //This doesn't work because platformio.ini doesn't include arduino as framework, so it doesn't create ARDUINO config entries.
extern "C" void app_main() {
    StartArduino(); //Calls setup and creates a loopTask that calls loop()
}
#else
extern "C" void app_main() {
    initArduino();
    esp_task_wdt_init(WDT_TIMEOUT, true);                                           //Enable WDT
    xTaskCreatePinnedToCore(loopTask, "loopTask", getArduinoLoopTaskStackSize(), NULL, 1, &mainloopTaskHandle, ARDUINO_RUNNING_CORE);
    esp_task_wdt_add(mainloopTaskHandle);                                           //Monitor thread with WDT
}

//Loop
void loop(void) {
    esp_task_wdt_reset();
    delay(3000);
    Serial.println("LoopTask");
}

void loopTask(void *pvParameters) {
    setup();
    while(1) {
        loop();
    }
}

void setup(void) {
    Serial.begin(115200);
    Serial.println("Booting...");									                //USB won't be connected yet, so we will only see this on a reboot

    //Delay for debugging. Allows time for USB to connect to computer.
    delay(2000);
    esp_task_wdt_reset();
    delay(1000);
    
    main_HWini();
    main_ModuleStart();
}
#endif

void main_HWini()
{
}

void main_ModuleStart()
{
}