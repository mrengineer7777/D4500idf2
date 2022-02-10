#include "arduinoloop.h"

// #include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "esp_attr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_partition.h"
#include "esp32-hal-cpu.h"

#include "esp_timer.h"
#ifdef CONFIG_APP_ROLLBACK_ENABLE
#include "esp_ota_ops.h"
#endif //CONFIG_APP_ROLLBACK_ENABLE
#ifdef CONFIG_BT_ENABLED
#include "esp_bt.h"
#endif //CONFIG_BT_ENABLED
#include <sys/time.h>
#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/apb_ctrl_reg.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
//#include "HardwareSerial.h"
#include "esp32-hal-log.h"

#ifdef ESP_IDF_VERSION_MAJOR // IDF 4+
#if CONFIG_IDF_TARGET_ESP32 // ESP32/PICO-D4
#include "esp32/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/rtc.h"
#include "driver/temp_sensor.h"
#elif CONFIG_IDF_TARGET_ESP32C3
#include "esp32c3/rom/rtc.h"
#include "driver/temp_sensor.h"
#else 
#error Target CONFIG_IDF_TARGET is not supported
#endif
#else // ESP32 Before IDF 4.0
#include "rom/rtc.h"
#endif

TaskHandle_t loopTaskHandle = NULL;
bool loopTaskWDTEnabled = false;

//---Prototypes---
size_t getArduinoLoopTaskStackSize(void);
void enableLoopWDT();
void disableLoopWDT();
void initArduino(void);

//---Prototypes---

#if CONFIG_FREERTOS_UNICORE
void yieldIfNecessary(void){
    static uint64_t lastYield = 0;
    uint64_t now = millis();
    if((now - lastYield) > 2000) {
        lastYield = now;
        vTaskDelay(5); //delay 1 RTOS tick
    }
}
#endif

__attribute__((weak)) size_t getArduinoLoopTaskStackSize(void) {
    return ARDUINO_LOOP_STACK_SIZE;
}

void loopTask(void *pvParameters)
{
    while(1) {
#if CONFIG_FREERTOS_UNICORE
        yieldIfNecessary();
#endif
        if(loopTaskWDTEnabled){
            esp_task_wdt_reset();
        }
        loop();
        //if (serialEventRun) serialEventRun();
    }
}

void StartArduino(void) {
    initArduino();
    setup();
    xTaskCreatePinnedToCore(loopTask, "loopTask", getArduinoLoopTaskStackSize(), NULL, 1, &loopTaskHandle, ARDUINO_RUNNING_CORE);
}

void __yield()
{
    vPortYield();
}

void yield() __attribute__ ((weak, alias("__yield")));

#if CONFIG_AUTOSTART_ARDUINO
void enableLoopWDT(){
    if(loopTaskHandle != NULL){
        if(esp_task_wdt_add(loopTaskHandle) != ESP_OK){
            log_e("Failed to add loop task to WDT");
        } else {
            loopTaskWDTEnabled = true;
        }
    }
}

void disableLoopWDT(){
    if(loopTaskHandle != NULL && loopTaskWDTEnabled){
        loopTaskWDTEnabled = false;
        if(esp_task_wdt_delete(loopTaskHandle) != ESP_OK){
            log_e("Failed to remove loop task from WDT");
        }
    }
}

void feedLoopWDT(){
    esp_err_t err = esp_task_wdt_reset();
    if(err != ESP_OK){
        log_e("Failed to feed WDT! Error: %d", err);
    }
}
#endif

void enableCore0WDT(){
    TaskHandle_t idle_0 = xTaskGetIdleTaskHandleForCPU(0);
    if(idle_0 == NULL || esp_task_wdt_add(idle_0) != ESP_OK){
        log_e("Failed to add Core 0 IDLE task to WDT");
    }
}

void disableCore0WDT(){
    TaskHandle_t idle_0 = xTaskGetIdleTaskHandleForCPU(0);
    if(idle_0 == NULL || esp_task_wdt_delete(idle_0) != ESP_OK){
        log_e("Failed to remove Core 0 IDLE task from WDT");
    }
}

#ifndef CONFIG_FREERTOS_UNICORE
void enableCore1WDT(){
    TaskHandle_t idle_1 = xTaskGetIdleTaskHandleForCPU(1);
    if(idle_1 == NULL || esp_task_wdt_add(idle_1) != ESP_OK){
        log_e("Failed to add Core 1 IDLE task to WDT");
    }
}

void disableCore1WDT(){
    TaskHandle_t idle_1 = xTaskGetIdleTaskHandleForCPU(1);
    if(idle_1 == NULL || esp_task_wdt_delete(idle_1) != ESP_OK){
        log_e("Failed to remove Core 1 IDLE task from WDT");
    }
}
#endif

BaseType_t xTaskCreateUniversal(TaskFunction_t pxTaskCode,
                        const char * const pcName,
                        const uint32_t usStackDepth,
                        void * const pvParameters,
                        UBaseType_t uxPriority,
                        TaskHandle_t * const pxCreatedTask,
                        const BaseType_t xCoreID ){
#ifndef CONFIG_FREERTOS_UNICORE
    if(xCoreID >= 0 && xCoreID < 2) {
        return xTaskCreatePinnedToCore(pxTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask, xCoreID);
    } else {
#endif
    return xTaskCreate(pxTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask);
#ifndef CONFIG_FREERTOS_UNICORE
    }
#endif
}

unsigned long ARDUINO_ISR_ATTR micros()
{
    return (unsigned long) (esp_timer_get_time());
}

unsigned long ARDUINO_ISR_ATTR millis()
{
    return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

void delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void ARDUINO_ISR_ATTR delayMicroseconds(uint32_t us)
{
    uint64_t m = (uint64_t)esp_timer_get_time();
    if(us){
        uint64_t e = (m + us);
        if(m > e){ //overflow
            while((uint64_t)esp_timer_get_time() > e){
                NOP();
            }
        }
        while((uint64_t)esp_timer_get_time() < e){
            NOP();
        }
    }
}

void initVariant() __attribute__((weak));
void initVariant() {}

void init() __attribute__((weak));
void init() {}

bool verifyOta() __attribute__((weak));
bool verifyOta() { return true; }

#ifdef CONFIG_BT_ENABLED
//overwritten in esp32-hal-bt.c
bool btInUse() __attribute__((weak));
bool btInUse(){ return false; }
#endif

void initArduino()
{
#ifdef CONFIG_APP_ROLLBACK_ENABLE
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            if (verifyOta()) {
                esp_ota_mark_app_valid_cancel_rollback();
            } else {
                log_e("OTA verification failed! Start rollback to the previous version ...");
                esp_ota_mark_app_invalid_rollback_and_reboot();
            }
        }
    }
#endif
    //init proper ref tick value for PLL (uncomment if REF_TICK is different than 1MHz)
    //ESP_REG(APB_CTRL_PLL_TICK_CONF_REG) = APB_CLK_FREQ / REF_CLK_FREQ - 1;
#ifdef F_CPU
    setCpuFrequencyMhz(F_CPU/1000000);
#endif
#if CONFIG_SPIRAM_SUPPORT || CONFIG_SPIRAM
    psramInit();
#endif
    // Override global log level - NOTE this is set by initArduino()
    //esp_log_level_set("*", ESP_LOG_INFO);
    // To debug, use 'make menuconfig' to set default Log level to DEBUG, then uncomment:
    //esp_log_level_set("owb", ESP_LOG_DEBUG);                                      //Logging level can be adjusted by tag.
    //esp_log_level_set("ds18b20", ESP_LOG_DEBUG);
    ////ESP_LOGE("ARDUINO", "Err");//TEST
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_err_t err = nvs_flash_init();
    if(err == ESP_ERR_NVS_NO_FREE_PAGES){
        const esp_partition_t* partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
        if (partition != NULL) {
            err = esp_partition_erase_range(partition, 0, partition->size);
            if(!err){
               err = nvs_flash_init();
            } else {
               log_e("Failed to format the broken NVS partition!");
            }
        }
    }
    if(err) {
        log_e("Failed to initialize NVS! Error: %u", err);
    }
#ifdef CONFIG_BT_ENABLED
    if(!btInUse()){
        esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
    }
#endif
     init();
     initVariant();
}

//used by esp32-hal-log.h
const char * ARDUINO_ISR_ATTR pathToFileName(const char * path)
{
    size_t i = 0;
    size_t pos = 0;
    char * p = (char *)path;
    while(*p){
        i++;
        if(*p == '/' || *p == '\\'){
            pos = i;
        }
        p++;
    }
    return path+pos;
}

