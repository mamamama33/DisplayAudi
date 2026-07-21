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

atomic_bool DataReaded = ATOMIC_VAR_INIT(false);

typedef struct
{
    uint32_t timestamp;
    uint8_t data[3u];
}EngineDiag_ChannelType;




/*PRVO SE SETUJE PA SE ONDA PREBACI U STANJE DA MOZE 
PREKO AUTOMATA DA SE POZOVE FUNKCIJA KwpRequest saljem
koji DID hocu da mi vrati,nakon toga Kwp_getDataFromEcu
sluzi da pokupim DID koji mi je ECU odgovorio
*/
uint8_t DidSetter(uint8_t Setdid)
{
    static uint8_t lastch;
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

/*FUNKCIJA ZA DOBAVLJANJE podataka KOJA POZIVA DIDSETTER koji posle 
poziva KwpRequest koji od ECU trazi podatke*/
/*Znaci prvo ova funkcija pa tek onda ona druga*/
/*

uint8_t DataSetter(uint8_t did){

    static uint8_t lastch;
    uint8_t retVal = DIAG_ERR;
    
    if(did!=lasth){
        if (DIAG_OK == DidSetter(did))
        {
            lastch=did;
            retVal = DIAG_OK;
        }



    }
    else{
        retval=DIAG_OK
    }

    return retVal;
}
*/


uint8_t EngineDiag_GetChData(uint8_t * dataPtr)
{
    uint8_t retVal = DIAG_ERR;
    uint32_t sysTime = 0;
    uint8_t i = 0;

    if (atomic_load(&DataReaded) == true) {
        vTaskSuspendAll();
        for(i=0;i<12;i++){
            dataPtr[i]=didbuffer[i];
        }

        xTaskResumeAll(); 

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
                        
            break;
            case DATA_REQUEST:
                //Prosledjujem mu koji DID  HOCU ili ti sta hocu da mi prikaze
                if(KwpRequest(did)==correct){
                state=DATA_READING;

                }

                break;

            /*Treba da napravim tajmer ako ne uspe da prekopira i sve to obradi dalje da nastavi sa upitom*/

            case DATA_READING:
                    /* kwp get data izvuce pa prosledi ovom */

                        if(Kwp_GetDataFromEcu(didbuffer)==correct){
                            atomic_store(&DataReaded, true);
                            state=DATA_IDLE;    
                        }
                    
            break;

        }
        vTaskDelay(pdMS_TO_TICKS(10));

    }

}



/*KOD NJEGA IMA ECU ID AL TO MENI NE TREBA meni treba samo callback za handlovanje didova*/
uint8_t DataInit(){

    xTaskCreatePinnedToCore(DataCyclic, "Data", 2048u, NULL, 2, &taskHandle,1);
    vTaskDelay(60u / portTICK_PERIOD_MS);

    return 1;
}