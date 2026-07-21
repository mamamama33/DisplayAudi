#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "TP2.0Protocol.h"
#include "PhysicalCan.h"
#include "KwpProtocol.h"

static QueueHandle_t CanToKwpQueue;
static QueueHandle_t StalkButtonQueue;

uint8_t StalkGetter;


#define CAN_KWP_QUEUE_LEN  10
#define STALKBUTTON        20


/*
// maska za ID jeve 
//01101011111 35F (stalk buttons) 
//10101110101 575 (ignition) 
// Code: 00101010101 155 
// Mask: 11000101010 62A 
twai_mask_filter_config_t filter1 = { 
.id = 0x155, .mask = 0x62A, .is_ext = false, 
}; 
//01000000001 201 (engine diag) TO JE ZA KWP PROTOKOL 
//01000000111 207 (dash diag) 
//01100000000 300 (all diag) 
//11011000001 6c1 (dash) 
//11011000010 6c2 (navi, gw mode) 
//11011000011 6c3 (dash, alternative mode) 
// maska za ID jeve 200 201 300 301 
twai_mask_filter_config_t filter2 = { 
.id = 0x200, .mask = 0x7FC, .is_ext = false, 
};
*/

/* Getter za rucicu */
uint8_t GetStalkButton() {
    return StalkGetter;
}

/* CAN receive task */
void Can_Receive(void *pvParameters)
{
    twai_message_t msg;
    while (1)
    {
        // Blokira se i čeka poruku - nema procesorskog vremena dok ništa ne stiže
        if (twai_receive(&msg, portMAX_DELAY) == ESP_OK)
        {
            // Minimalan ispis, bez delay-a nakon toga!
            //printf("ID: 0x%03X DATA: ", (unsigned int)msg.identifier);
            //for(int i = 0; i < msg.data_length_code; i++) printf("%02X ", msg.data[i]);
            //printf("\n");

            if (msg.identifier == STALKBUTTONRXID) {
                xQueueSend(StalkButtonQueue, &msg, 0);
            }
            else if (msg.identifier == ENGINERXID || msg.identifier == ENGINERXID2) {
                xQueueSend(CanToKwpQueue, &msg, 0);
                //ESP_LOGE("Physical","");

            }
        }
        // NEMA vTaskDelay ovde!
    }
}

/* Stalk button task */
void StalkButton(void *pvParameters)
{
    twai_message_t msg;
    uint8_t buttons;
    static uint8_t lastButtons = 0;

    while (1)
    {
        if (xQueueReceive(StalkButtonQueue, &msg, portMAX_DELAY))
        {
            buttons = msg.data[1];
            //ESP_LOGW("stalk","Pritisnuto dugme");

            if (lastButtons != buttons)
            {
                if ((lastButtons & 0x20) && !(buttons & 0x20)) {
                    ESP_LOGI("STALK", "GORE");
                    StalkGetter=20;
                }
                else if ((lastButtons & 0x10) && !(buttons & 0x10)) {
                    ESP_LOGI("STALK", "DOLE");
                    StalkGetter=10;
                }
                else if ((lastButtons & 0x40) && !(buttons & 0x40)) {
                    ESP_LOGI("STALK", "OK/RESET");
                    StalkGetter=30;
                }

                lastButtons = buttons;
            }
        }

    }
}

/* KWP task */
void Kwp_Task(void *pvParameters)
{
    twai_message_t msg;

    while (1)
    {
        if (xQueueReceive(CanToKwpQueue, &msg, portMAX_DELAY))
        {
            //ESP_LOGI("Physical","Primljeno nesto od ECU ");
            VwTp_Receive(
                msg.identifier,
                msg.data_length_code,
                msg.data
            );
        }
    }
}

/* CAN write */
uint8_t CanWrite(uint16_t CanID, uint8_t len, uint8_t* Data)
{
    twai_message_t msg = {0};

    msg.identifier = CanID;
    msg.data_length_code = len;
    msg.extd = 0;
    msg.rtr = 0;

    for (int i = 0; i < len; i++) {
        msg.data[i] = Data[i];
    }

    if (twai_transmit(&msg, pdMS_TO_TICKS(15)) == ESP_OK) {
        //ESP_LOGE("Physical","Uspesno izvrsena funkcija twai_transmit");
        return 0;
    }
    /*printf("CAN TX [ID: 0x%03X] [LEN: %d] DATA: ", CanID, len);
    for(int i = 0; i < len; i++) {
            printf("%02X ", Data[i]);
    }*/
    return 1;
}

// CAN init 
void Can_Init()
{
    // Konfiguracija
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(17, 16, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();

    // Filter (možeš kasnije da optimizuješ)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    // Queue
    CanToKwpQueue   = xQueueCreate(CAN_KWP_QUEUE_LEN, sizeof(twai_message_t));
    StalkButtonQueue = xQueueCreate(STALKBUTTON, sizeof(twai_message_t));

    // Taskovi
    xTaskCreatePinnedToCore(Can_Receive, "CanRx", 2048, NULL, 7, NULL, 1);
    xTaskCreatePinnedToCore(Kwp_Task, "KwpTask", 4096, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(StalkButton, "StalkButton", 2048, NULL, 5, NULL, 1);
}
/*
void Can_Init()
{
    // 1. Generalna konfiguracija (Pinovi 17 i 16, 500kbps)
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(17, 16, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();

    2. FILTER LOGIKA:
       Želimo: 200, 201, 300, 301 (opseg 2xx i 3xx) i 35F.
       
       Podesićemo filter tako da proverava najbitnije bitove.
       Code:  0x200  (Binarno: 010 0000 0000)
       Mask:  0x400  (Binarno: 100 0000 0000) -> Ovo kaže: "Bit 10 mora biti 0" 
                                                 (svi tvoji ID-jevi počinju sa 0 na bitu 10)
                                                 
       Bolja varijanta za tvoje ID-jeve (200, 201, 300, 301, 35F):
       Svi su u rangu 0x200 do 0x3FF.
    
    
    twai_filter_config_t f_config;
    f_config.acceptance_code = (0x200 << 21); // Pomeramo za 21 bit jer je to standard za 11-bitne ID-jeve u ESP32
    f_config.acceptance_mask = ~(0x1FF << 21); // Maska koja dozvoljava promene u donjih 9 bita, ali fiksira gornje
    f_config.single_filter = true;

    // Instalacija drajvera
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    // Redovi poruka (Queues)
    CanToKwpQueue   = xQueueCreate(CAN_KWP_QUEUE_LEN, sizeof(twai_message_t));
    StalkButtonQueue = xQueueCreate(STALKBUTTON, sizeof(twai_message_t));

    // Taskovi
    xTaskCreatePinnedToCore(Can_Receive, "CanRx", 2048, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(Kwp_Task, "KwpTask", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(StalkButton, "StalkButton", 2048, NULL, 4, NULL, 0);
    
    vTaskDelay(pdMS_TO_TICKS(50));
}
*/