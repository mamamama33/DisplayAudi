#include"DataAnaliser.h"
#include"KwpProtocol.h"
#include "esp_log.h"
#define correct 1
static uint8_t did=0;
uint8_t count1=0;
static DataState state=DATA_IDLE;
static TaskHandle_t taskHandle;
static Diag_CallbackType *callback;

typedef struct
{
    uint32_t timestamp;
    uint8_t data[3u];
}EngineDiag_ChannelType;

static EngineDiag_ChannelType channels[ENGINEDIAG_CH_MAX];
//Znaci ovo su koje didove zelim da trazim od ECU-a i koji su mi potrebni za prikaz na displeju
const uint8_t ChIdxToDid[ENGINEDIAG_CH_MAX] = 
{
    //DURMI1 je injection duration u vcds-u
    //qmi1 injection quantity u vcds-u
    /*--------PRVI BLOK U VCDS-U*/
    [ENGINEDIAG_CH_COOLANTTEMP1] = 1u,
    [ENGINEDIAG_CH_DURMI1] = 1u,
    [ENGINEDIAG_CH_QMI1] = 1u,
    [ENGINEDIAG_CH_ENGINESPEED1] = 1u,
    /*-------------------*/

    /*---------SESTI BLOK U VCDS-U---------*/
    [ENGINEDIAG_CH_VEHICLESPEED] = 2u,
    /*----------SEDMI BLOK U VCDS-U----------*/
    [ENGINEDIAG_CH_FUELTEMP] = 7u,
    [ENGINEDIAG_CH_IATTEMP7] = 7u,
    [ENGINEDIAG_CH_COOLANTTEMP7] = 7u,
    /*----------DESETI BLOK U VCDS-U----------*/
    [ENGINEDIAG_CH_MAF] = 10u,

    /*----------JEDANAESTI BLOK U VCDS-U----------*/
    [ENGINEDIAG_CH_BOOSTSPECIFIED]= 11u,
    [ENGINEDIAG_CH_BOOSTACTUAL] = 11u,
    [ENGINEDIAG_CHARGEPRESSURE] = 11u,
    /*----------DVANAESTI BLOK U VCDS-U----------*/
    [ENGINEDIAG_CH_VOLTAGE] = 12u,

    /*---------PETNAESTI BLOK U VCDS-U---------*/
    [ENGINEDIAG_CH_FUELCONSUMPTION] = 15u,
    /*---------DVADESETI BLOK U VCDS-U*/
    //TO JE ABS NEZ DA LI MI TREBA UOPSTE

    /*---------DVADESET I OSAM BLOK U VCDS-U---------*/
    [ENGINEDIAG_CH_ACCELERATION_PEDAL] = 28u,

    /*----------DVADESET I DEVET BLOK U VCDS-U----------*/
    [ENGINEDIAG_CH_OILTEMP] = 29u,
    [ENGINEDIAG_CH_OILLEVEL] = 29u,

    /*-----------SEZDESET I DRUGI BLOK U VCDS-U-----------*/
    [ENGINEDIAG_CH_ENGINETEMP62] = 62u,
    [ENGINEDIAG_CH_COOLERTEMP62] = 62u,
    [ENGINEDIAG_CH_AMBITEMP62] = 62u,
    [ENGINEDIAG_CH_IATTEMP62] = 62u,

    /*-----------SEZDESET I TRECI BLOK U VCDS-U-----------*/
    [ENGINEDIAG_CH_REFRIGERANTPRESSURE] = 63u,

    /*-----------SEZDESET I CETVRTI BLOK U VCDS-U-----------*/
    [ENGINEDIAG_CH_FANDUTYCYCLE] = 64u,

    [ENGINEDIAG_CH_EGTEMP74] = 74u,
    [ENGINEDIAG_CH_LAMBDA74] = 74u,
};
const uint8_t ChIdxToDidOffset[ENGINEDIAG_CH_MAX] = 
{
    /*--------PRVI BLOK U VCDS-U*/
    [ENGINEDIAG_CH_ENGINESPEED1] = 0u,
    [ENGINEDIAG_CH_QMI1] = 1u,
    [ENGINEDIAG_CH_DURMI1] = 2u,
    [ENGINEDIAG_CH_COOLANTTEMP1] = 3u,

    /*---------SESTI BLOK U VCDS-U---------*/
    [ENGINEDIAG_CH_VEHICLESPEED] = 0u,
    

    /*--------SEDMI BLOK U VCDS-U*/
    [ENGINEDIAG_CH_FUELTEMP] = 0u,
    //iNTAKE AIR TEMPERATURE 
    [ENGINEDIAG_CH_IATTEMP7] = 2u,
    [ENGINEDIAG_CH_COOLANTTEMP7] = 3u,
    /*----------DESETI BLOK U VCDS-U----------*/

    [ENGINEDIAG_CH_MAF] = 0u,

    /*----------JEDANAESTI BLOK U VCDS-U----------*/
    [ENGINEDIAG_CH_BOOSTSPECIFIED]= 1u,
    [ENGINEDIAG_CH_BOOSTACTUAL] = 2u,
    [ENGINEDIAG_CHARGEPRESSURE] = 3u,
    /*----------DVANAESTI BLOK U VCDS-U----------*/
    [ENGINEDIAG_CH_VOLTAGE] = 2u,

    /*---------PETNAESTI BLOK U VCDS-U---------*/

    [ENGINEDIAG_CH_FUELCONSUMPTION] = 2u,
    /*---------DVADESETI BLOK U VCDS-U*/
    //TO JE ABS NEZ DA LI MI TREBA UOPSTE

    /*---------DVADESET I OSAM BLOK U VCDS-U---------*/
    [ENGINEDIAG_CH_ACCELERATION_PEDAL] = 3u,

    /*----------DVADESET I DEVET BLOK U VCDS-U----------*/
    [ENGINEDIAG_CH_OILTEMP] = 0u,
    [ENGINEDIAG_CH_OILLEVEL] = 1u,
    /*--------- SEZDESET I DRUGI BLOK U VCDS-U---------*/
    [ENGINEDIAG_CH_ENGINETEMP62] = 0u,
    [ENGINEDIAG_CH_COOLERTEMP62] = 1u,
    [ENGINEDIAG_CH_AMBITEMP62] = 2u,
    [ENGINEDIAG_CH_IATTEMP62] = 3u,
    /*-----------SEZDESET I TRECI BLOK U VCDS-U-----------*/
    [ENGINEDIAG_CH_REFRIGERANTPRESSURE] = 0u,


    /*-----------SEZDESET I CETVRTI BLOK U VCDS-U-----------*/
    [ENGINEDIAG_CH_FANDUTYCYCLE] = 2u,

    [ENGINEDIAG_CH_EGTEMP74] = 1u,
    [ENGINEDIAG_CH_LAMBDA74] = 2u
};

/*PRVO SE SETUJE PA SE ONDA PREBACI U STANJE DA MOZE 
PREKO AUTOMATA DA SE POZOVE FUNKCIJA KwpRequest saljem
koji DID hocu da mi vrati,nakon toga Kwp_getDataFromEcu
sluzi da pokupim DID koji mi je ECU odgovorio
*/
uint8_t DidSetter(uint8_t Setdid)
{
    uint8_t retVal = DIAG_ERR;
    if (DATA_IDLE == state)
    {
        vTaskSuspendAll(); // Critical section, interrupts enabled
        did = Setdid;
        state = DATA_REQUEST;
        retVal = DIAG_OK;
        xTaskResumeAll(); // End of critical section, interrupts enabled
    }
    return retVal;
}



//Funkcija koja ce obradjivati DID-ove -Da desifruje sta je sta u poruci
/*buffer je niz od 4*3 bajtova od jednog bloka koje sam izvukao preko kwprequest */
/* PUNIM MOJU BAZU channels[ch].data[b] sa baferom */
static void EngineDiag_HandleDid(uint8_t * buffer, uint8_t actualDid)
{
    uint32_t timestamp = 0u;
    uint8_t ch,offset,b; 
    for (ch = 0;ch<ENGINEDIAG_CH_MAX;ch++)
    {
        // Search for a channel with this DID
        if (actualDid == ChIdxToDid[ch])
        {
            // a channel found
            for (offset=0; offset < 4u; offset++)
            {
                // Search for a matching MBW from all the 4 MWBs we received
                if (offset == ChIdxToDidOffset[ch])
                {
                    timestamp = xTaskGetTickCount();
                    vTaskSuspendAll(); // Critical section, interrupts enabled
                    for (b=0; b < 3u; b++)
                    {
                        // Copy 3 bytes of data from buffer with offset
                        channels[ch].data[b] = buffer[(offset * 3u)+b];
                        ESP_LOGE("Data","Uspeo je da upise u bafer od data");

                    }
                    channels[ch].timestamp = timestamp;
                    xTaskResumeAll(); // End of critical section, interrupts enabled
                }
            }
        }
    }
}


/*FUNKCIJA ZA DOBAVLJANJE podataka KOJA POZIVA DIDSETTER koji posle 
poziva KwpRequest koji od ECU trazi podatke*/
/*Znaci prvo ova funkcija pa tek onda ona druga*/
uint8_t EngineDiag_GetChData(const EngineDiag_ChannelIdType ch, uint8_t * dataPtr, uint32_t timeout)
{
    uint8_t retVal = DIAG_ERR;
    uint32_t sysTime = 0;
    uint8_t i = 0;
    if (ch < ENGINEDIAG_CH_MAX)
    {
        sysTime = xTaskGetTickCount();
        if ( channels[ch].timestamp > (sysTime - (timeout/ portTICK_PERIOD_MS)))
        {
            // Data is not too old
            vTaskSuspendAll(); // Critical section, interrupts enabled
            for (i=0; i<3u; i++)
            {
                dataPtr[i] = channels[ch].data[i];
                
                ESP_LOGE("Data","Displajov geter je pokupio podatke za case slucaj");

            }
            retVal = DIAG_OK;
            xTaskResumeAll(); // End of critical section, interrupts enabled
        }
        else 
        {
            //Data is too old or never received
            if (DIAG_OK == DidSetter(ChIdxToDid[ch]))
            {
                retVal = DIAG_PENDING;
            }
        }
    }
    return retVal;
}






//automat stanja
void DataCyclic(void *pvParameters){
    //Prosledjujem DID i dobijam podatke preko funckije-gettera -njegovog parametra  
    uint8_t didbuffer[4u*3u];
    while(1){
        vTaskDelay(pdMS_TO_TICKS(10));
        switch(state){
            case DATA_IDLE:
            if(SetDataDid==true)
                state=DATA_REQUEST;
                        
            break;
            case DATA_REQUEST:
                //Prosledjujem mu koji DID  HOCU ili ti sta hocu da mi prikaze
                if(KwpRequest(did)==correct){
                state=DATA_READED;
                //pov1=2;
               // ESP_LOGI("DataAnaliser","MOGU I USPEO SAM DA POSALJEM DID");
                }
                else{
                
                    //ESP_LOGE("DataAnaliser","Ne mogu da TRAZIM zahtev od ECU");
                    break;
                
                }

            /*Treba da napravim tajmer ako ne uspe da prekopira i sve to obradi dalje da nastavi sa upitom*/

            case DATA_READED:
                    /* kwp get data izvuce pa prosledi ovom */
                    if(xSemaphoreTake(GetData,pdMS_TO_TICKS(20))){

                        if(Kwp_GetDataFromEcu(didbuffer)==correct){
            
                            printf("bafera: %u",didbuffer[1]);
                            state=DATA_REQUEST;     
                        }

                    }
                    else{
                        //Semafor nije mogao vise da drzi
                    }

            
            break;
   
        }
    }

}



/*KOD NJEGA IMA ECU ID AL TO MENI NE TREBA meni treba samo callback za handlovanje didova*/
uint8_t DataInit(){

    xTaskCreatePinnedToCore(DataCyclic, "Data", 2048u, NULL, 3, &taskHandle,1);
    vTaskDelay(60u / portTICK_PERIOD_MS);

    return 1;
}