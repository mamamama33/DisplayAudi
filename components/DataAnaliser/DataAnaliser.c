#include"DataAnaliser.h"
#include"KwpProtocol.h"
#include "esp_log.h"
#define correct 1
static uint8_t did=0;
uint8_t count1=0,count2=0;
uint8_t pov1=0;
static DataState state=DATA_IDLE;
static TaskHandle_t taskHandle;
static Diag_CallbackType *callback;

typedef struct
{
    uint32_t timestamp;
    uint8_t data[3u];
}EngineDiag_ChannelType;

static EngineDiag_ChannelType channels[ENGINEDIAG_CH_MAX];

const uint8_t ChIdxToDid[ENGINEDIAG_CH_MAX] = 
{
    [ENGINEDIAG_CH_COOLANTTEMP1] = 1u,
    [ENGINEDIAG_CH_DURMI1] = 1u,
    [ENGINEDIAG_CH_QMI1] = 1u,
    [ENGINEDIAG_CH_ENGINESPEED1] = 1u,
    [ENGINEDIAG_CH_FUELTEMP] = 7u,
    [ENGINEDIAG_CH_IATTEMP7] = 7u,
    [ENGINEDIAG_CH_COOLANTTEMP7] = 7u,
    [ENGINEDIAG_CH_OILTEMP] = 29u,
    [ENGINEDIAG_CH_ENGINETEMP62] = 62u,
    [ENGINEDIAG_CH_COOLERTEMP62] = 62u,
    [ENGINEDIAG_CH_AMBITEMP62] = 62u,
    [ENGINEDIAG_CH_IATTEMP62] = 62u,
    [ENGINEDIAG_CH_EGTEMP67] = 67u,
    [ENGINEDIAG_CH_EGTEMP74] = 74u,
    [ENGINEDIAG_CH_LAMBDA74] = 74u,
};
const uint8_t ChIdxToDidOffset[ENGINEDIAG_CH_MAX] = 
{
    [ENGINEDIAG_CH_COOLANTTEMP1] = 3u,
    [ENGINEDIAG_CH_DURMI1] = 2u,
    [ENGINEDIAG_CH_QMI1] = 1u,
    [ENGINEDIAG_CH_ENGINESPEED1] = 0u,
    [ENGINEDIAG_CH_FUELTEMP] = 0u,
    [ENGINEDIAG_CH_IATTEMP7] = 2u,
    [ENGINEDIAG_CH_COOLANTTEMP7] = 3u,
    [ENGINEDIAG_CH_OILTEMP] = 0u,
    [ENGINEDIAG_CH_ENGINETEMP62] = 0u,
    [ENGINEDIAG_CH_COOLERTEMP62] = 1u,
    [ENGINEDIAG_CH_AMBITEMP62] = 2u,
    [ENGINEDIAG_CH_IATTEMP62] = 3u,
    [ENGINEDIAG_CH_EGTEMP67] = 0u,
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
        vTaskDelay(20u/portTICK_PERIOD_MS);
        switch(state){
            case DATA_IDLE:
            if(SetDataDid==true)
                state=DATA_REQUEST;
                        
            break;
            case DATA_REQUEST:
                //Prosledjujem mu koji DID  HOCU ili ti sta hocu da mi prikaze
                if(KwpRequest(did)==correct){
                state=DATA_WAITNG;
                pov1=2;
                ESP_LOGI("DataAnaliser","MOGU I USPEO SAM DA POSALJEM DID");
                }
                else{
                    //Cekamo da vidimo da li ce stici da se kwp izvrsi ili je stv greska
                    if(count1>20){
                       //Vracamo se na pocetak doslo je do greska u uspostavi veze KWP
                        //NE mogu da trazim jer njegov automat koci
                        ESP_LOGE("DataAnaliser","Ne mogu da TRAZIM zahtev od ECU");           
                        state=DATA_IDLE;
                        count1=0;
                    }
                    else{
                        count1++;
                    }
                }
            break;
            case DATA_WAITNG:
                //UZIMAM podatak uz proveru da li je dobar
                if(ReadyToGetData==true){
                    state=DATA_READED;
                }
            break;
            case DATA_READED:
                    if(Kwp_GetDataFromEcu(didbuffer)==correct){
                        EngineDiag_HandleDid(didbuffer,did);  
                        ESP_LOGI("DataAnaliser","Uspeo sam da dobijem neke podatke od ECU");           
                    }
                    else{
                        //Cekamo da vidimo da li ce stici da se kwp izvrsi ili je stv greska
                        if(count2>20){
                        //Vracamo se na pocetak doslo je do greska u uspostavi veze KWP
                            state=DATA_IDLE;
                            ESP_LOGE("DataAnaliser-ERROR","Ne mogu da DOBIJEM PODATKE OD ECU");          
                            count2=0;
                        }
                        else{
                            count2++;
                        }
                    }
            ESP_LOGI("Data","Uspesno zavrsena jedna sesija podatka");
            state=DATA_IDLE;
            break;
   
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }

}



/*KOD NJEGA IMA ECU ID AL TO MENI NE TREBA meni treba samo callback za handlovanje didova*/
uint8_t DataInit(){

    xTaskCreatePinnedToCore(DataCyclic, "Data", 2048u, NULL, 3, &taskHandle,1);
    vTaskDelay(60u / portTICK_PERIOD_MS);

    return 1;
}