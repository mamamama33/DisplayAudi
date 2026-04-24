#include <stdio.h>
#include"freertos/FreeRTOS.h"
#include"freertos/task.h"
#include <stdint.h>
#include"esp_sleep.h"
#include"TP2.0Protocol.h"
#include"PhysicalCan.h"
#include"KwpProtocol.h"
#include"DataAnaliser.h"
#include"Display.h"
#include"esp_log.h"
void app_main(void)
{
    Can_Init();
    vTaskDelay(pdMS_TO_TICKS(1000));
    TpInit();

    KwpInit();
    vTaskDelay(pdMS_TO_TICKS(300));

    DataInit();
    DisplayInit();
    

}
