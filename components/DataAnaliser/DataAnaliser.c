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
                // 0 je brzina km/h CASE 7 ,a 2 je accel pedal pos CASE 33
            }
        },

        {
            .id = 11,
            .blocks = {
                0,1,2,3
                // 0 je rpm CASE 1, 1 je boost pressure actual CASE 18,
                // 2 je specified CASE 18, a duty cycle je 3 CASE 23           SVE FORMULE OK
            }
        }
    },


    // ================= DRUGA STRANA =================
    {
        {
            .id = 7,
            .blocks = {
                0,5,2,3
                // 0 je fuel temp CASE 5, 2 je intake air temp,  SVI CASE 05 sto meni treba
                // 3 je coolant temp - engine
            }
        },
                                    //SVE CELA STRANA JE 05 CASE            SVE FORMULE OK
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
                // 0 je oil temp CASE 5, 1 je oil level CASE 102  NEMA FORMULA ZA 102 ali a=10 ,b = 00 tkd neko mnozenje jer u leru pise 00mm na snimku IMA CASE 13(mozda je taj)
            }
        },

        {
            .id = 15,
            .blocks = {
                5,1,2,5
                // 1 je engine torque CASE 52, formula OK 
                // 2 je fuel consumption CASE 35 formula ok
            }
        }
    },


    // ================= ČETVRTA STRANA =================
    {
        {
            .id = 63,
            .blocks = {
                0,1,5,5
                // 0 je refrigerant pressure, CASE 69 OK FORMULA
                // 1 je torque load CASE 52 kao engine torque 
            }
        },

        {
            .id = 64,
            .blocks = {
                0,1,2,5
                // 0 je coolant temp engine, CASE 5
                // 1 je coolant temp cooler, CASE 5 
                // 2 je fan duty             CASE 23 a=102 ,b= 25 FORMULE ok 
            }
        }
    },


    // ================= PETA STRANA =================
    {
        {
            .id = 13,
            .blocks = {
                0,1,2,3
                // Koliko dizne bacaju je to sve SVE SU CASE 51 OK FORMULA
            }
        },
                                                                //MOOGUCE NESTO NOVO DODATI
        {
            .id = 4,
            .blocks = {
                5,5,5,3
                // 3 je torsion value CASE 34 FORMULA OK
            }
        }
    },


    // ================= ŠESTA STRANA =================
    {
        {
            .id = 10,
            .blocks = {
                0,1,5,5
                // MAF senzor je 0 CASE 49 u formuli je mh/h a meni je mg/str, atmosferski pritisak 1 CASE 12
            }
        },
                                                                //SEM MAF FENZORA SVE FORMULE OK
        {
            .id = 12,
            .blocks = {
                5,1,2,5
                // PreGlow plug je 1 CASE 55 voltage je 2 CASE 6 TACAN 
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
                            dataPtr[(offset*3)+b] = 0;

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