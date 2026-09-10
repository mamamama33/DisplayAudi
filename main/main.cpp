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
    
    vTaskDelay(pdMS_TO_TICKS(20));
    Can_Init();
    vTaskDelay(pdMS_TO_TICKS(20));
    TpInit();
    vTaskDelay(pdMS_TO_TICKS(20));
    KwpInit();
    vTaskDelay(pdMS_TO_TICKS(20));
    DataInit();
    vTaskDelay(pdMS_TO_TICKS(20));
    DisplayInit();
    vTaskDelay(pdMS_TO_TICKS(500)); // veci dilej kako bi se stabilizovao core 0 - gde je uzimanje podataka i lepljenje na ekran

}