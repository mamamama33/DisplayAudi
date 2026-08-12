#include <stdio.h>
#include"freertos/FreeRTOS.h"
#include"freertos/task.h"
#include <stdint.h>
#include"esp_sleep.h"
#include"Display.h"
#include"esp_log.h"

extern "C" {
    #include "PhysicalCan.h"    // Sadrži Can_Init()
    #include "TP2.0Protocol.h" // Sadrži TpInit()
    #include "KwpProtocol.h"   // Sadrži KwpInit()
    #include "DataAnaliser.h"   // Sadrži DataInit()
}

extern "C" void app_main(void) {

    vTaskDelay(pdMS_TO_TICKS(3000));
    Can_Init();
    vTaskDelay(pdMS_TO_TICKS(1000));
    TpInit();
    vTaskDelay(pdMS_TO_TICKS(100));
    KwpInit();
    vTaskDelay(pdMS_TO_TICKS(100));
    DataInit();
    vTaskDelay(pdMS_TO_TICKS(100));
    DisplayInit();
}