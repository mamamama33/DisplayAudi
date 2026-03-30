#include"KwpProtocol.h"
#include "TP2.0Protocol.h"
#include"esp_log.h"

/*SID DEO*/
//  ID |||| SID|| DID||| DATA

//Ovo su SID-ovi koje ne koristim 
//NE TREBA 0x22  //readDataByCommonIdentifier (čitanje standardizovanih podataka zajedničkih više ECU-a),
//0x23 // readMemoryByAddress (direktno čitanje memorije ECU-a na zadatoj adresi),
//0x2C // inputOutputControlByLocalIdentifier (ručna kontrola izlaza: aktuatori, releji, ventili...),
//0x3E // testerPresent (sprečava timeout – održava sesiju aktivnom),
//0x83 // accessTimingParameter (čitanje ili podešavanje tajming parametara komunikacije).
//define STOPSESSION_SID 0x20  //stopDiagnosticSession (prekid aktivne dijagnostičke sesije i povratak u normalan režim),

//OVO su SID-ovi koje koristim 
#define STARTSESSION_SID 0x10u  //startDiagnosticSession (biranje Normal, Extended ili Programming sesije),
#define STARTROUTINE_SID 0x31u // startRoutine / stopRoutine (pokretanje ili zaustavljanje internih rutina – testovi, inicijalizacije),
#define READDATA_SID 0x21u  //readDataByLocalIdentifier (čitanje proizvođački-specifičnih podataka – lokalni ID),
#define POSITIVERESP_SID 0x40u //POBRTDA OD ECU-a pomeri za plus 0x40 SID
#define NEGATIVERESP_SID 0x7Fu //
#define READECUID_SID 0x1Au //VRV ALI SAMO VRV je biranje rezima -NORMAL
#define RESPONSPENDING_SID 0x78 // requestCorrectlyReceived – ResponsePending (ECU radi u pozadini i traži dodatno vreme). 

#define ecuid 0x01u
#define correct 1u
#define notcorrect 0u
KwpStage KwpStages=KWP_START; //handshake-session....

KwpStatus KwpStatuses=KWP_IDLE;   //IDLE ,,error...

static TaskHandle_t taskHandle;
static uint8_t        didBuffer[12u];
static uint8_t        dataId = 1u;
static uint8_t        ecuId = 1;
static uint8_t        retry = 0;
static uint8_t        active = 0;

//FUNKCIJE KOJE POZIVAM U CASE CU STAVITI KAO VOID 
static void KwpHandShake(uint8_t);
static void Kwp_StartSession(uint8_t );
static void Kwp_ReadEcuId(uint8_t);
static void Kwp_StartRoutine(uint8_t rid, uint16_t rEntOpt);
static void Kwp_ReadData(uint8_t);



/*ONE FUNCKIJE KOJE IMAJU POVRATNI TIP UINT8 ---KWP request data i kwp get data from ecu 


su funkcije koje se pozivaju iz DataAnaliser koda 


*/   /*

    KWP_IDLE,
    KWP_PROCESSING,
    KWP_ERROR,
    KWP_WAITING



*/ 
void KwpCyclic(void *pvParameters){
    
    static uint8_t timeout = 0;

    while(1){
        if(KwpStatuses==KWP_IDLE){
            vTaskDelay(pdMS_TO_TICKS(30));
            timeout=0;
            switch(KwpStages){

                case KWP_START:
                    if(correct==KwpStart(ecuid)){
                        KwpStages=KWP_START_HANDSHAKE;
                        ESP_LOGI("KWP","TpStart je prosao");
                        KwpStatuses=KWP_PROCESSING;

                    }
                    else{
                                if (retry > 10u)
                                {
                                    retry = 0;
                                    KwpStages = KWP_ERROR;
                                }
                                else 
                                {
                                    retry++;
                                }
                    }
                break;
                case KWP_START_HANDSHAKE:
                    ESP_LOGI("KWP-AUTOMAT","Poslat handshake");
                    
                    Kwp_StartSession(0x89u);
                break;
                /*NEZ ZASTO saljem zahtev za ECU ID KADA GA VEC IMAM*/
                case KWP_START_SESSION:
                    ESP_LOGI("KWP-AUTOMAT","Poslat zahtev za ECUID");
                    Kwp_ReadEcuId(0x9Bu);
                break;
                case KWP_READECUID:
                    ESP_LOGI("KWP-automat","Poslatu koju rutinu hocu-dijagnostika");
                    Kwp_StartRoutine(0xB8u, 0x0000u);

                break;
                case KWP_ROUTINE:
                    /*KADA AUTOMAT DODJE DOVDE MOZE DATAANALISER DA POZOVE KwpRequest znaci moze da posalje did pretoga moram setovati komunikaciju*/
                    /*KAO U VCDS-u da udjem u measuring blocks!!!!*/
                    KwpStages = KWP_READY; 
                break;
                /*Setujem dataID-neki kurac u DIDu*/    
                case KWP_READDID:
                    ESP_LOGI("KWP","Poslat KOJI DID HOCU I OVDE JE KRAJ");
                                 
                    Kwp_ReadData(dataId);
                break;
                    
                case KWP_CLOSE:
                    VwTp_Disconnect();
                /*ZAVRSEN JE JEDAN KRUG PODATAKA*/
                KwpStages=KWP_START;
                break;
                default:
                break;

            }
            /*SAMOOO POSTAVLJANJE SETOVANJE ZA DIAG */
        }
        /*Idalje se obradjuju podaci*/
        else if(KwpStatuses==KWP_PROCESSING){

           if (20u <= timeout){
                if (KWP_START == KwpStages)
                {
                    // Handle no response to connect timeout
                    KwpStatuses = KWP_IDLE;
                }
                else
                {
                    // No response from ECU
                   // KwpStages = KWP_CLOSE;
                    KwpStatuses = KWP_IDLE;
                }
                timeout=0;
            }
            else 
            {
                timeout++;
            }
        }

        /*Automat ulazi u cekanje kako bi mogao da primi poruku 
        iako smo primanje obradili u automatu TP2.0 sledeceg sloja
        */
        
        else if(KwpStatuses==KWP_WAITING){
            if (100u <= timeout)
            {
                // (Rx) timeout occured while waiting for response
                timeout=0;
                KwpStages = KWP_CLOSE;
                KwpStatuses = KWP_IDLE;
            }
            else 
            {
                timeout++;
            }
        }
        else 
        {
            // Error state
        }

            vTaskDelay(pdMS_TO_TICKS(50));

    }

}   


uint8_t KwpInit(){

    KwpStages =KWP_START;
    xTaskCreatePinnedToCore(KwpCyclic, "Data", 2048u, NULL, 3, &taskHandle,1);
    vTaskDelay(120u / portTICK_PERIOD_MS);
    return 1;
}
/*
uint8_t KwpDeInit(){

    if(KWP_IDLE==KwpStatuses){
        KwpStages=KWP_CLOSE;
        vTaskDelay(150u/portTICK_PERIOD_MS);
        


    }

}
*/
/*Funkcija koja samo did setuje a u cycle TP-a se stalno nesto salje koje zavisi
od uslova ova funkcije... ona nesto setuje pa to prodje u switch case uslovu
*/
uint8_t KwpRequest(uint8_t did){

    if((KWP_READY==KwpStages) && KWP_IDLE==KwpStatuses){
        vTaskSuspendAll();         
        KwpStages=KWP_READDID;
        dataId=did;
        xTaskResumeAll();
        return 1;
    }

    return 0;
}

uint8_t Kwp_GetConnectionState(void)
{
    uint8_t retVal = KWP_ERR;
    if (0 != active)
    {
        retVal = KWP_OK;
    }
    return retVal;
}

uint8_t Kwp_GetDataFromEcu(uint8_t * const dataPtr){

    uint8_t retVal=KWP_ERROR;
    uint8_t tmp;
    if ((KWP_READY == KwpStages) && (KWP_IDLE == KwpStatuses))
    {
        vTaskSuspendAll(); // Critical section, interrupts enabled
        for(tmp=0;tmp<sizeof(didBuffer);tmp++)
        {
            dataPtr[tmp] = didBuffer[tmp]; // copy data
        }
        retVal = KWP_OK;
        xTaskResumeAll(); // End of critical section, interrupts enabled
    }
    return retVal;

}



void Kwp_Receive(uint8_t * dataPtr,uint16_t len)
{

    uint8_t i = 0;
    if (KWP_WAITING == KwpStatuses)
    {
        if  (1u <= dataPtr[1])
        {
            vTaskSuspendAll(); // Critical section, interrupts enabled
            if (((POSITIVERESP_SID + KwpStages) == dataPtr[2]))
            {   
                ESP_LOGW("KWP","Znaci da je prihvaceno-POSITIVESID");

                // Positive response
                if ((KWP_READDID == KwpStages) && (dataId == dataPtr[3u]) && (sizeof(didBuffer) <= (len-4u)))
                {
                    
                    for (i=0;i<(sizeof(didBuffer) );i++)
                    {
                        didBuffer[i] = dataPtr[4u+i];
                    }
                    KwpStages = KWP_READY;
                    
                }
                KwpStatuses = KWP_IDLE;
            }
            else if ((NEGATIVERESP_SID == dataPtr[2u])&& (2u < dataPtr[1u]))
            {
                if ((KwpStages == dataPtr[3u]) && (RESPONSPENDING_SID == dataPtr[4u]))
                {
                    // Response pending, stay in KWP_WAITRESULT
                }
                else
                {
                    // Negative response
                    KwpStatuses = KWP_ERROR;
                }
            }
            else 
            {
                // Corrupt TP frame or unexpected response
                KwpStages = KWP_CLOSE;
                KwpStatuses = KWP_IDLE;
            }
            xTaskResumeAll(); // End of critical section, interrupts enabled
        }
    }
}

void Kwp_TxConfirmation(uint8_t result)
{
    if ((KWP_ERR != result) && (KWP_PROCESSING == KwpStatuses))
    {
        if (KWP_START == KwpStages)
        {
            KwpStages = KWP_START_HANDSHAKE;
            KwpStatuses = KWP_IDLE;
            active = 1;
        }
        else
        {
            KwpStatuses = KWP_WAITING;
        }
    }
    else if (KWP_ERR == result)
    {
        KwpStatuses = KWP_IDLE;
        retry = 0;
        KwpStages = KWP_START;
        active = 0;
    }
    else 
    {
        // Nothing to do
    }
}


static void Kwp_StartSession(uint8_t sessionId)
{
    uint8_t msg[4];
    msg[0] = 0x00; // Format byte
    msg[1] = 0x02; // Length
    msg[2] = STARTSESSION_SID; // SID
    msg[3] = sessionId;
    if (KWP_OK == Kwp_SendTp(msg))
    {
        ESP_LOGI("KWP","Uspem da posaljem session");
        KwpStatuses = KWP_PROCESSING;
        KwpStages = KWP_START_SESSION;
    }
    else{
        ESP_LOGE("KWP","NE USPEM da posaljem session");


    }
    vTaskDelay(30);
}

static void Kwp_ReadEcuId(uint8_t idOption)
{
    //ESP_LOGE("ULAZ U READECUID","POZVALO JE FUNKCCIJU!!");
    uint8_t msg[4];
    msg[0] = 0x00; // Format byte
    msg[1] = 0x02; // Length
    msg[2] = READECUID_SID; // SID
    msg[3] = idOption;
    if (KWP_OK == Kwp_SendTp(msg))
    {
        KwpStatuses = KWP_PROCESSING;
        KwpStages = KWP_READECUID;
        ESP_LOGI("KWP","Poslat zahtev za ECU");
    }
    else{
        ESP_LOGE("KWP","NIJE Poslat zahtev za ECU");


    }
}

static void Kwp_StartRoutine(uint8_t rid, uint16_t rEntOpt)
{
    uint8_t msg[6];
    msg[0] = 0x00; // Format byte
    msg[1] = 0x04; // Length
    msg[2] = STARTROUTINE_SID; // SID
    msg[3] = rid; // routine local id
    msg[4] = (uint8_t)rEntOpt; // RoutineEntryOption
    msg[5] = (uint8_t)((uint16_t)rEntOpt >> (uint16_t)8u); // RoutineEntryOption
    if (KWP_OK == Kwp_SendTp(msg))
    {
        KwpStatuses = KWP_PROCESSING;
        KwpStages = KWP_ROUTINE;
    
    }
}

static void Kwp_ReadData(uint8_t did)
{
    uint8_t msg[4];
    msg[0] = 0x00; // Format byte
    msg[1] = 0x02; // Length
    msg[2] = READDATA_SID; // SID
    msg[3] = did; // data local id
    if (KWP_OK == Kwp_SendTp(msg))
    {
        vTaskSuspendAll(); // Critical section, interrupts enabled
        KwpStatuses = KWP_PROCESSING;
        KwpStages = KWP_READDID;
        xTaskResumeAll(); // End of critical section, interrupts enabled
        ESP_LOGE("KWP","Poslat POSLEDNJI zahtev");

    }
}




