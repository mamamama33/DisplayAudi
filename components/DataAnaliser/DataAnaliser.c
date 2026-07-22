#include"DataAnaliser.h"
#include"KwpProtocol.h"
#include <stdatomic.h>
#include "esp_log.h"
#define correct 1
static uint8_t did=6;
uint8_t count1=0;
static DataState state=DATA_IDLE;
static TaskHandle_t taskHandle;
static Diag_CallbackType *callback;
uint8_t didbuffer[4u*3u];
uint8_t buttons=0;
atomic_bool DataReaded = ATOMIC_VAR_INIT(false);

typedef struct
{
    uint32_t timestamp;
    uint8_t data[3u];
}EngineDiag_ChannelType;


typedef struct
{
    uint8_t blocks[4];
    uint8_t id;

} Block;


Block Strane[][2] =
{
    // ================= PRVA STRANA =================
    {
        {
            .id = 6,
            .blocks = {
                0,5,2,5
                // 0 je brzina km/h a 2 je accel pedal pos
            }
        },

        {
            .id = 11,
            .blocks = {
                0,1,2,3
                // 0 je rpm, 1 je boost pressure actual,
                // 2 je specified, a duty cycle je 3
            }
        }
    },


    // ================= DRUGA STRANA =================
    {
        {
            .id = 7,
            .blocks = {
                0,5,2,3
                // 0 je fuel temp, 2 je intake air temp,
                // 3 je coolant temp - engine
            }
        },

        {
            .id = 62,
            .blocks = {
                5,1,2,5
                // 1 je coolant temp coolant,
                // 2 je ambient temp
            }
        }
    },


    // ================= TREĆA STRANA =================
    {
        {
            .id = 29,
            .blocks = {
                0,1,5,5
                // 0 je oil temp, 1 je oil level
            }
        },

        {
            .id = 15,
            .blocks = {
                5,1,2,5
                // 1 je engine torque,
                // 2 je fuel consumption
            }
        }
    },


    // ================= ČETVRTA STRANA =================
    {
        {
            .id = 63,
            .blocks = {
                0,5,2,5
                // 0 je refrigerant pressure,
                // 2 je cooling request koji vrv neću dobiti
            }
        },

        {
            .id = 64,
            .blocks = {
                0,1,2,5
                // 0 je coolant temp engine,
                // 1 je coolant temp cooler,
                // 2 je fan duty
            }
        }
    },


    // ================= PETA STRANA =================
    {
        {
            .id = 13,
            .blocks = {
                0,1,2,3
                // Koliko dizne bacaju je to sve
            }
        },

        {
            .id = 4,
            .blocks = {
                5,5,5,3
                // 3 je torsion value
            }
        }
    },


    // ================= ŠESTA STRANA =================
    {
        {
            .id = 10,
            .blocks = {
                0,5,5,5
                // MAF senzor
            }
        },

        {
            .id = 12,
            .blocks = {
                5,5,2,5
                // voltage
            }
        }
    }

};
uint8_t strana=5;
uint8_t TacanId=0;

/*PRVO SE SETUJE PA SE ONDA PREBACI U STANJE DA MOZE 
PREKO AUTOMATA DA SE POZOVE FUNKCIJA KwpRequest saljem
koji DID hocu da mi vrati,nakon toga Kwp_getDataFromEcu
sluzi da pokupim DID koji mi je ECU odgovorio
*/
uint8_t DidSetter(uint8_t Setdid)
{
    uint8_t retVal = DIAG_ERR;

    if (DATA_REQUEST == state)
    {
        vTaskSuspendAll(); // Critical section, interrupts enabled
        did = Setdid;
        state = DATA_REQUEST;
        retVal = DIAG_OK;
        xTaskResumeAll(); // End of critical section, interrupts enabled
    }
    return retVal;
}



static uint32_t zadnjeVremePrijema = 0;
static uint8_t counter;


uint8_t EngineDiag_GetChData(uint8_t * dataPtr,uint8_t* TrenutnaStrana)
{
    uint8_t retVal = DIAG_ERR;
    uint32_t sysTime = 0;
    uint8_t i = 0;
    uint8_t offset,b;
    if (atomic_load(&DataReaded) == true){
        vTaskSuspendAll();

            for (offset=0; offset < 4u; offset++)
            {
                      if(offset == Strane[strana][TacanId].blocks[offset]){

                        vTaskSuspendAll(); // Critical section, interrupts enabled
                        for (b=0; b < 3u; b++)
                        {
                            // Copy 3 bytes of data from buffer with offset
                            dataPtr[(offset*3)+b] = didbuffer[(offset*3)+b];

                        }
                        xTaskResumeAll(); // End of critical section, interrupts enabled

                      }
                      else{

                        for (b=0; b < 3u; b++)
                        {
                            // Copy 3 bytes of data from buffer with offset
                            dataPtr[(offset*3)+b] =0;

                        }

                      }
            }
        xTaskResumeAll(); 
        *TrenutnaStrana=strana;
        retVal=  DIAG_OK;
        atomic_store(&DataReaded, false);
    }
    else{
        //Ostalo bi diagerr ali radi lepseg izgleda
        retVal = DIAG_ERR;
    }

    return retVal;
}



//automat stanja
void DataCyclic(void *pvParameters){
    //Prosledjujem DID i dobijam podatke preko funckije-gettera -njegovog parametra  
    while(1){
        switch(state){
            case DATA_IDLE:
            if(SetDataDid==true)
                state=DATA_REQUEST;
            
            
            //buttons=GetStalkButton();        
            if(buttons!=0x00){
                strana++;
                if(strana>=6)
                strana=0;

            }
            if(TacanId>=2)
                TacanId=0;
            break;
            case DATA_REQUEST:
                //Prosledjujem mu koji DID  HOCU ili ti sta hocu da mi prikaze
                
                if(KwpRequest(Strane[strana][TacanId].id)==correct){
                state=DATA_READING;
                }

                break;

            /*Treba da napravim tajmer ako ne uspe da prekopira i sve to obradi dalje da nastavi sa upitom*/

            case DATA_READING:
                    /* kwp get data izvuce pa prosledi ovom */

                        if(Kwp_GetDataFromEcu(didbuffer)==correct){
                            atomic_store(&DataReaded, true);
                            state=DATA_IDLE;
                            TacanId++;    
                        }
                    
            break;

        }
        vTaskDelay(pdMS_TO_TICKS(10));

    }

}



/*KOD NJEGA IMA ECU ID AL TO MENI NE TREBA meni treba samo callback za handlovanje didova*/
uint8_t DataInit(){

    xTaskCreatePinnedToCore(DataCyclic, "Data", 2048u, NULL, 3, &taskHandle,1);
    vTaskDelay(60u / portTICK_PERIOD_MS);

    return 1;
}