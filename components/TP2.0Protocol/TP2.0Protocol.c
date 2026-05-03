#include "TP2.0Protocol.h"
#include "KwpProtocol.h"
#include "PhysicalCan.h"

#include "esp_log.h"
static TaskHandle_t VwTpTaskHdl = NULL;
#define correct 1u
#define notcorrect 0u

SemaphoreHandle_t TpSendComplete = NULL;

/*Koristim static void kako bi zastitio da niko ne moze da ih poziva*/

static void VwTp_HandleTx(Tp_ChannelType * const chPtr);
static void VwTp_HandleTxTimeout(Tp_ChannelType * const chPtr);
static void VwTp_HandleRx(Tp_ChannelType * chPtr,uint8_t dlc,uint8_t * dataPtr);
static void VwTp_HandleRxTimeout(Tp_ChannelType * const chPtr);
static void VwTp_HandleConnect(Tp_ChannelType * chPtr,uint8_t * dataPtr);
static void VwTp_HandleCallbacks(Tp_ChannelType * const chPtr);
static void VwTp_sendTpParams(Tp_ChannelType * const chPtr, uint8_t response);
static void VwTp_sendClose(Tp_ChannelType * const chPtr);
static void VwTp_sendAck(Tp_ChannelType * const chPtr);
static void VwTp_sendBreak(Tp_ChannelType * const chPtr);

volatile uint8_t KwpStartSessionFlag = 0;

Tp_ChannelType TpEcu={.
    
    cfg={
        .rxId = 0x0u,
        .txId = 0x200u,
        .blockSize = 0x0Fu,
        .ackTimeout = 0x8Fu,
        .ips = 0x4Fu,
        .txConfirmation = Kwp_TxConfirmation, //POLJE STRUKTURE POZIVA FUNKCIJU !!!!! 
        .rxIndication = Kwp_Receive, //POLJE STRUKTURE POZIVA FUNKCIJU !!!!! 
    }
};



void Tp_Cyclic(void *pvParameters)
{
    uint8_t chId = 0;
    Tp_ChannelType * chPtr = &TpEcu;
    
    /*Ovo sam dodao kako bi mogao da prvo posaljem poruku-zahrev*/


    while(1)
    {

            VwTp_HandleTxTimeout(chPtr);
            VwTp_HandleTx(chPtr); // TODO: Requesting ACK after x transmitted frames 
            VwTp_HandleCallbacks(chPtr); // DA BI STALNO ODRADJIVAO CALLBACK FUNKCIJE 
            VwTp_HandleRxTimeout(chPtr);
            vTaskDelay(10u / portTICK_PERIOD_MS);

    }
}
/*NJU POZIVAM IZ KWP i u njoj punim bafer koji u cyclic stalno pozivam!!!*/
uint8_t VwTp_Send(uint8_t chId, uint8_t * buffer, uint16_t len)
{
    Tp_ChannelType * chPtr = &TpEcu;
    uint8_t retVal = VWTP_ERR;
    uint16_t i = 0;
    int k=0;
      //dodao  
    /*SETOOOVAO SAM OVO DA BI KWP automat radio i bez ovoga a ovo se setuje na prijemu*/

        vTaskSuspendAll(); // Critical section, interrupts enabled
        if ((chPtr->txState == VWTP_IDLE) && (sizeof(chPtr->txBuffer) >= len))
        {
            chPtr->txSize = len;
            chPtr->txOffset = 0u;
            for (i=0; i<len; i++)
            {
                chPtr->txBuffer[i] = buffer[i];
            }
            chPtr->txState = VWTP_WAIT;
            retVal = VWTP_OK;
            k=1;
        
        }
        /*Ovo sluzi za cekanje ali to cekanje mi spamuje stalno */
        else if ((chPtr->txState == VWTP_CONNECT))
        {
            // Try to open channel
            //chPtr->txFlags.params = VWTP_TPPARAMS_REQUEST;
            chPtr->rxState = VWTP_IDLE;
            retVal = VWTP_PENDING;
            //vTaskDelay(pdMS_TO_TICKS(20));
            //ESP_LOGE("TP2.0","Ovde ulazi ako se zaglaviio i to nije dobro");
            k=2;
        }
        else 
        {
            //ESP_LOGI("TP2.0","NE SETUJE VWTP_IDLE dobro");
            // Do nothing, return error
        }
        xTaskResumeAll(); // End of critical section, interrupts enabled
        
    return retVal;
}

void VwTp_Receive(uint16_t canId, uint8_t dlc, uint8_t * dataPtr)
{
    Tp_ChannelType * chPtr= NULL;
    
    if (canId == TpEcu.cfg.rxId) //EEE OVO JE KLJUCCCC!!!!!!!!!!!! ovde proverava od koga je poruka!!!
    {
        chPtr = &TpEcu;
        
    
    }
    else{

        ESP_LOGI("TP2.0","NIJE dobar ID od ECU");


    }
    
    if (NULL != chPtr)
    {
        /*Samo za prvi put samno za connect*/
        if ((chPtr->rxState == VWTP_CONNECT) )
        {
            VwTp_HandleConnect(chPtr,dataPtr);
            //ESP_LOGW("TP2.0","Prva poruka od ECU je dobra");
        }
        else 
        {
            VwTp_HandleRx(chPtr,dlc,dataPtr);//dataPtr dobija podataka od twai-OD CAN-bus-treba da se dekoduje prvi protokol a to je vwTP2.0

        }
    }
}

static void VwTp_HandleRxTimeout(Tp_ChannelType * const chPtr)
{
    if (VWTP_WAIT == chPtr->rxState)
    {
        if(20u <= chPtr->rxTimeout)
        {
            // Send break?
            chPtr->txFlags.brk = 1u;
            chPtr->rxState = VWTP_IDLE;
            chPtr->seqCntRx = chPtr->ackSeqCntRx;
            chPtr->rxTimeout = 0u;
        }
        else 
        {
            chPtr->rxTimeout++;
        }
    }
    else 
    {
        chPtr->rxTimeout = 0u;
    }
}


void VwTp_Disconnect()
{
    Tp_ChannelType * chPtr = NULL;
    chPtr = &TpEcu;
    VwTp_sendClose(chPtr);
    
}
/*TESTIRANJEM SAM DOBIO OVO OD ECU-A RX ID: 0x201 DATA: 0 D0 0 3 40 7 1 */
/*prolazi i ovo */
static void VwTp_HandleConnect(Tp_ChannelType * chPtr,uint8_t * dataPtr)
{

    chPtr=&TpEcu;

    // Connection response
    if ((0x00u == dataPtr[0]) && (0xD0u == dataPtr[1]) && (0u == (dataPtr[3] & 0x10u)) && (0u == (dataPtr[5] & 0x10u)))
    {
        vTaskSuspendAll(); // Critical section, interrupts enabled
        chPtr->cfg.rxId = (dataPtr[2] | (dataPtr[3]<<8u));
        chPtr->cfg.txId = (dataPtr[4] | (dataPtr[5]<<8u));
        chPtr->rxState = VWTP_IDLE;
        chPtr->txState = VWTP_IDLE;
        chPtr->txFlags.params = VWTP_TPPARAMS_REQUEST;
        xTaskResumeAll(); // End of critical section, interrupts enabled
    
    
        ESP_LOGW("TP2.0","Uspesno primljena prva povratna od ECU-setujem chptr-idle");
    }

}

//Funkcija koja prima chPtr koji je struktura ,dlc duzina ,dataPtr podatak CAN BUS komunikacije ali menja samo chPTR...
//Mislio da je koristio chPtr strukturu koja ima niz uint8_t rxbuffer da napuni bafer 
static void VwTp_HandleRx(Tp_ChannelType * chPtr,uint8_t dlc,uint8_t * dataPtr)
{
    //chPtr je oznacava ID-ko salje
    //ESP_LOGE("TP2.0","HandleRx se izvrsava");
    uint8_t i = 0;
    if ((0xA3u == dataPtr[0]) || (0xA0u == dataPtr[0]))
    {
        // Connection init or alive check
        if (chPtr->txState == VWTP_CONNECT)
        {
            chPtr->txState = VWTP_IDLE;
        }
        chPtr->txFlags.params = VWTP_TPPARAMS_RESPONSE;
    }
    else if (0xA1u == dataPtr[0])
    {
        // Connection initialized or response to alive check
        chPtr->txState = VWTP_FINISHED; // PUSTAM CALLBACK FUNKCIJE ZA KWP 
        KwpStartSessionFlag = 1;
    }
    else if (0x10u == (dataPtr[0] & 0xF0u))
    {
        vTaskSuspendAll(); // Critical section, interrupts enabled
        if ((dataPtr[0] & 0x0Fu) == ((chPtr->seqCntRx + 1u) & 0x0Fu))
        {
            // seq cnt OK
            chPtr->seqCntRx = (dataPtr[0] & 0xF);
            //last frame or single frame
            if (VWTP_IDLE == chPtr->rxState)
            {
                //single frame received
                chPtr->rxSize = (dlc-1);
                for(i=0;i<chPtr->rxSize;i++)
                {
                    chPtr->rxBuffer[i] = dataPtr[i+1];
                }
                #if (0 != CONFIG_VWTP_NAV_ROUTING)
                if (NULL != chPtr->cfg.appStatus)
                {
                    if (VWTP_OK != chPtr->cfg.appStatus())
                    {
                        chPtr->txFlags.ack = VWTP_TXTASK_ACK_NOTREADY; // RCRRP
                    }
                    else
                    {
                        chPtr->txFlags.ack = VWTP_TXTASK_ACK_READY; // Ack + rxIndication
                    }
                }
                else 
                #endif //CONFIG_VWTP_NAV_ROUTING
                {
                    chPtr->txFlags.ack = VWTP_TXTASK_ACK_READY; // Ack + rxIndication
                }
                chPtr->rxState = VWTP_ACK;
            }
            else if (VWTP_WAIT == chPtr->rxState )
            {
                // last frame received
                for(i=0;i<(dlc-1);i++)
                {
                    chPtr->rxBuffer[chPtr->rxSize + i] = dataPtr[i+1];
                }
                chPtr->rxSize += (dlc-1);
                #if (0 != CONFIG_VWTP_NAV_ROUTING)
                if (NULL != chPtr->cfg.appStatus)
                {
                    if (VWTP_OK != chPtr->cfg.appStatus())
                    {
                        chPtr->txFlags.ack = VWTP_TXTASK_ACK_NOTREADY; // RCRRP
                    }
                    else
                    {
                        chPtr->txFlags.ack = VWTP_TXTASK_ACK_READY; // Ack + rxIndication
                    }
                }
                else 
                #endif //CONFIG_VWTP_NAV_ROUTING
                {
                    chPtr->txFlags.ack = VWTP_TXTASK_ACK_READY; // Ack + rxIndication
                }
                chPtr->rxState = VWTP_ACK;
            }
            else 
            {
                // received single/last data frame while not expected
            
                ESP_LOGE("VWTP","Not expected s/l data frame, id %x, seq %x", chPtr->cfg.rxId, dataPtr[0]);
                VwTp_sendClose(chPtr);
            }
        }
        else 
        {
            // Wrong sequence number received
            // ESP_LOGE("VWTP","sl wrong seq num");
            if (VWTP_IDLE == chPtr->rxState)
            {
                chPtr->seqCntRx = ((dataPtr[0]-1u) & 0x0Fu); // resync
            }
        }
        xTaskResumeAll(); // End of critical section, interrupts enabled
    }

    else if ((0x20u == (dataPtr[0] & 0xF0u)) || (0u == (dataPtr[0] & 0xF0u)))
    {
        if ((VWTP_IDLE == chPtr->rxState) || (VWTP_WAIT == chPtr->rxState ))
        {
            vTaskSuspendAll(); // Critical section, interrupts enabled
            if ((dataPtr[0] & 0x0Fu) == ((chPtr->seqCntRx + 1u) & 0x0Fu))
            {
                chPtr->seqCntRx = (dataPtr[0] & 0x0Fu);
                chPtr->rxState = VWTP_WAIT; // more data expected
                for(i=0;i<(dlc-1);i++)
                {
                    chPtr->rxBuffer[chPtr->rxSize + i] = dataPtr[i+1];
                }
                chPtr->rxSize += (dlc-1);
                if (0 == (dataPtr[0] & 0xF0u))
                {
                    chPtr->txFlags.ack = VWTP_TXTASK_ACK_READY; // Ack + rxIndication
                }
            }
            else 
            {
                // Wrong sequence number received
                #ifdef VWTP_DET
                ESP_LOGE("VWTP","fc wrong seq num");
                #endif
            }
            xTaskResumeAll(); // End of critical section, interrupts enabled
        }
        else 
        {
            //Not expected first or consecutive data frame received
        
            ESP_LOGE("VWTP","Not expected f/c data frame, id %x, seq %x", chPtr->cfg.rxId, dataPtr[0]);
            VwTp_sendClose(chPtr);
        }
    }
    else if ((0xB0u == (dataPtr[0] & 0xF0u)) || (0x90u == (dataPtr[0] & 0xF0u)))
    {
        vTaskSuspendAll(); // Critical section, interrupts enabled
        if (VWTP_ACK == chPtr->txState)
        {
            chPtr->seqCntTx = (dataPtr[0] & 0x0Fu);
            chPtr->ackSeqCntTx = (dataPtr[0] & 0x0Fu);
            chPtr->txSize = 0;
            chPtr->txOffset = 0;
            chPtr->txState = VWTP_FINISHED; // Ovo smece sluzi da se pozove kwp_Receive iz 
            ESP_LOGE("TP2.0","Uspevam da se dog sa ECu");

        }
        else 
        {
            //received ack when not expected
            #ifdef VWTP_DET
            ESP_LOGE("VWTP","Not expected ACK");
            #endif
        }
        xTaskResumeAll(); // End of critical section, interrupts enabled
    }
    /*istekla je sesija zato se zatvarasc*/
    else if (0xA8u == dataPtr[0])
    {
        // Connection was terminated
        if (VWTP_CONNECT != chPtr->txState)
        {
            VwTp_sendClose(chPtr);
            KwpStartSessionFlag = 0;

            
        }
    }
    else if (0xA4u == dataPtr[0])
    {
    
        ESP_LOGI("VWTP","Break");
        if ((VWTP_WAIT == chPtr->txState) || (VWTP_ACK == chPtr->txState))
        {
            // Resend
            
            ESP_LOGI("VWTP","Resend");
            vTaskSuspendAll(); // Critical section, interrupts enabled
            chPtr->txOffset = 0;
            if (chPtr->txSize > 0)
            {
                chPtr->seqCntTx = chPtr->ackSeqCntTx;
                chPtr->txState = VWTP_WAIT;
            }
            else
            {
                chPtr->txState = VWTP_IDLE;
            }
            xTaskResumeAll(); // End of critical section, interrupts enabled
        }
    }
    else
    {
        // error case
        ESP_LOGW("TP2.0","OVDE GA ZATVARA-390");

        VwTp_sendClose(chPtr);
    }



}

static void VwTp_sendAck(Tp_ChannelType * const chPtr)
{
    uint8_t msg[1];
    uint8_t ackSeq;
    ackSeq = (chPtr->seqCntRx+1) & 0x0Fu;
    msg[0] = 0xB0u | ackSeq; // ack, ready
    if (CAN_OK == TPSENDMESSAGE(chPtr->cfg.txId,sizeof(msg),msg))
    {
        ESP_LOGE("TP2.0","ACK je poslat to je neophodno");
        vTaskSuspendAll(); // Critical section, interrupts enabled
        chPtr->ackSeqCntRx = chPtr->seqCntRx;
        chPtr->txFlags.ack = 0u;
        // reset states to receive the next msg
        chPtr->rxState = VWTP_FINISHED;
        xTaskResumeAll(); // End of critical section, interrupts enabled
    }
    
}

static void VwTp_sendClose(Tp_ChannelType * const chPtr)
{
    uint8_t tpClose[1] = {0xA8u};
    
    vTaskSuspendAll(); // Critical section, interrupts enabled
    chPtr->seqCntTx = 0;
    chPtr->seqCntRx = 0xFu;
    chPtr->rxSize = 0;
    chPtr->txSize = 0;
    chPtr->rxState = VWTP_IDLE;
    chPtr->txState = VWTP_CONNECT;
    chPtr->txFlags.ack = 0;
    xTaskResumeAll(); // End of critical section, interrupts enabled
    TPSENDMESSAGE(chPtr->cfg.txId,sizeof(tpClose),tpClose);
    // Reset diagnostics
    
    ESP_LOGE("TP2.0","Sendclose se salje jer je ECU tako rekao");
    vTaskSuspendAll(); // Critical section, interrupts enabled
    chPtr->rxState = VWTP_CONNECT;
    chPtr->txState = VWTP_CONNECT;
    chPtr->cfg.txId = 0x200u;
    chPtr->cfg.rxId = 0;
    xTaskResumeAll(); // End of critical section, interrupts enabled
    
    if (NULL != chPtr->cfg.txConfirmation)
    {
        chPtr->cfg.txConfirmation(VWTP_ERR);
    }
}
/*OVO SMECE MI SE STALNO SALJE!!!!*/
static void VwTp_sendTpParams(Tp_ChannelType * const chPtr, uint8_t response)
{
    uint8_t tpParams[6];
    
    if (response < 2)
    {
        tpParams[0u] = 0xA0u | response; // params resp
        tpParams[1u] = chPtr->cfg.blockSize; // msg num before ack
        tpParams[2u] = chPtr->cfg.ackTimeout; // timing
        tpParams[3u] = 0xFFu; // dummy
        tpParams[4u] = chPtr->cfg.ips; // timing2
        tpParams[5u] = 0xFFu; // dummy
        if (CAN_OK == TPSENDMESSAGE(chPtr->cfg.txId,sizeof(tpParams),tpParams))
        {
            chPtr->txFlags.params = 0;
            ESP_LOGE("TP2.0","TpParams poslat odma posle prvo ecu odgovora");
        }
    }
}

static void VwTp_sendBreak(Tp_ChannelType * const chPtr)
{
    uint8_t msg[1];
    
    msg[0] = 0xA4u; // Break
    if (CAN_OK == TPSENDMESSAGE(chPtr->cfg.txId,sizeof(msg),msg))
    {
        chPtr->txFlags.brk = 0;
         ESP_LOGE("TP2.0","break stalno salje");
    }
}

/* HANDLUJE callback za to da pozovem funkciju za kwp protokol da mu prosledim da on posle dekodoje */
static void VwTp_HandleCallbacks(Tp_ChannelType * const chPtr)
{
    
    if (chPtr->rxState == VWTP_FINISHED)
    {
        if (NULL != chPtr->cfg.rxIndication)
        {
            chPtr->cfg.rxIndication(chPtr->rxBuffer,chPtr->rxSize);
            ESP_LOGI("TP2.0","Ovo trigeruje kwpReceive");
        }
        vTaskSuspendAll(); // Critical section, interrupts enabled
        chPtr->rxSize = 0u;
        chPtr->rxState = VWTP_IDLE;
        xTaskResumeAll(); // End of critical section, interrupts enabled
    }
    if (chPtr->txState == VWTP_FINISHED)
    {
        if (NULL != chPtr->cfg.txConfirmation)
        {
            chPtr->cfg.txConfirmation(VWTP_OK);
        }
        chPtr->txState = VWTP_IDLE;
    }
}

/*Ja kad uradim handleconnect i to pozove se funkcija iz kwp-a StartSession pozove se TpSend U NJOJ SE SAMO SETUJE ----- sta se salje a onda ovde 
se stvarno posalje 


*/

static void VwTp_HandleTx(Tp_ChannelType * const chPtr)
{
    uint16_t tmp;
    uint8_t dlc;
    uint8_t msg[8];
    if ( VWTP_WAIT == chPtr->txState )
    {
        //ESP_LOGE("TP2.0","Treba ovde posle handsgake da se to ispegla");
        if ((chPtr->txSize < 8u) || (((chPtr->txSize)-(chPtr->txOffset)) < 8u))
        {
            dlc = ((chPtr->txSize)-(chPtr->txOffset))+1u;
            if (dlc != 1)
            {
                msg[0] = 0x10u | chPtr->seqCntTx; // single frame or last frame
                for (tmp=0;tmp<(dlc-1);tmp++)
                {
                    msg[tmp+1] = chPtr->txBuffer[tmp+(chPtr->txOffset)];
                }
                if (CAN_OK == TPSENDMESSAGE(chPtr->cfg.txId,dlc,msg))//CAN WRITE
                {
                    chPtr->txTimeout = 0;      // Resetuj tajmer za timeout
                    chPtr->txState = VWTP_ACK;
                    ESP_LOGE("TP2.0","Mora ovde da dodje da bi odgovorio na B1 iz HandleRx-ovo je pre prijema poruke ");
                    xSemaphoreGive(TpSendComplete);

                }
            }
            else
            {
                msg[0] = 0xA3; // alive check
                if (CAN_OK == TPSENDMESSAGE(chPtr->cfg.txId,dlc,msg))
                {
                    ESP_LOGE("TP2.0","handletx stalno salje");
                    chPtr->txState = VWTP_FINISHED;
                    xSemaphoreGive(TpSendComplete);

                }
            }
        }
        else
        {
            ESP_LOGE("TP2.0","Mora ovde da dodje da bi odgovorio na B1 ALI NE PRODJEs");

            dlc = 8;
            msg[0] = 0x20u | chPtr->seqCntTx; // consecutive frame
            for (tmp=0;tmp<7u;tmp++)
            {
                msg[tmp+1u] = chPtr->txBuffer[tmp+(chPtr->txOffset)];
            }
            if (CAN_OK == TPSENDMESSAGE(chPtr->cfg.txId,dlc,msg))
            {
                 ESP_LOGE("TP2.0","handletx stalno salje");
                chPtr->txOffset += 7u;
                if (chPtr->seqCntTx < 0xFu)
                {
                    chPtr->seqCntTx++;
                }
                else
                {
                    chPtr->seqCntTx = 0u;
                }
            }
        }
    }
    else if ((VWTP_IDLE == chPtr->txState) && (0 != chPtr->txFlags.ack))
    {
        #if (0 != CONFIG_VWTP_NAV_ROUTING)
        if (VWTP_TXTASK_ACK_READY == chPtr->txFlags.ack)
        {
            #endif //CONFIG_VWTP_NAV_ROUTING
            VwTp_sendAck(chPtr);
            #if (0 != CONFIG_VWTP_NAV_ROUTING)
        }
        else
        {
            VwTp_sendAckNotReady(chPtr);
        }
        #endif //CONFIG_VWTP_NAV_ROUTING
    }
    else if ((VWTP_IDLE == chPtr->txState) && (0 != chPtr->txFlags.brk))
    {
        VwTp_sendBreak(chPtr);
    }
    else if (((VWTP_IDLE == chPtr->txState) || (VWTP_CONNECT == chPtr->txState)) && (0 != chPtr->txFlags.params))
    {
        VwTp_sendTpParams(chPtr, (chPtr->txFlags.params & VWTP_TPPARAMS_MASK));
    }
    else
    {
        // No task to do
    }
}



uint8_t TpConnect(uint8_t EcuID){

    vTaskDelay(pdMS_TO_TICKS(40));
    uint8_t pov=0;
    uint8_t msg[7];
    Tp_ChannelType *EcuChannel=NULL;
    
    EcuChannel=&TpEcu;
    
    if(EcuChannel!=NULL){

        msg[0]=EcuID;
        msg[1] = 0xC0u; // connect
        msg[2] = 0x00u; // rxId (lsb)
        msg[3] = 0x10u; // invalid rxId (msb)
        msg[4] = 0x00u; // txId (lsb)
        msg[5] = 0x03u; // valid txId (msb) 0x300
        msg[6] = 0x01u; // app = kwp2000
        if(correct == TPSENDMESSAGE(EcuChannel->cfg.txId,sizeof(msg),msg)){
            pov=correct;
            EcuChannel->cfg.rxId= (0x200u | EcuID); 
            ESP_LOGI("TP","USPESNO POSLAT CONNECT NA CAN ");
        }

    }

    return pov;
}

static void VwTp_HandleTxTimeout(Tp_ChannelType * const chPtr){

    uint8_t ackCfg = 0;
    if (VWTP_ACK == chPtr->txState)
    {
        ESP_LOGI("TP2.0","Pa onda ovde");
        if (0x80 == (chPtr->cfg.ackTimeout & 0xC0u))
        {
            // multiplier: 10 ms
            ackCfg = (chPtr->cfg.ackTimeout & 0x3Fu); // we are already in 10ms scale
        }
        else if (0x40 == (chPtr->cfg.ackTimeout & 0xC0u))
        {
            // multiplier: 1 ms
            ackCfg = (chPtr->cfg.ackTimeout & 0x3Fu)/10u; // we are in 10ms scale
        }
        else if (0xC0 == (chPtr->cfg.ackTimeout & 0xC0u))
        {
            // multiplier: 100 ms
            ackCfg = (chPtr->cfg.ackTimeout & 0x3Fu) * 10u; // we are in 10ms scale
        }
        if (chPtr->txTimeout >= ackCfg)
        {
            // timeout, resend
            #ifdef VWTP_DET
            ESP_LOGI("VWTP","Timeout, resend: id: %x , seq: %x",chPtr->cfg.txId,chPtr->seqCntTx);
            #endif
            vTaskSuspendAll(); // Critical section, interrupts enabled
            chPtr->txOffset = 0;
            if (chPtr->txSize > 0)
            {
                chPtr->seqCntTx = chPtr->ackSeqCntTx; // set back the seq cntr
                chPtr->txState = VWTP_WAIT;
            }
            else
            {
                chPtr->txState = VWTP_IDLE;
            }
            chPtr->txTimeout = 0;
            xTaskResumeAll(); // End of critical section, interrupts enabled
            
        }
        else
        {
            // waiting for ACK
            chPtr->txTimeout++;
        }
    }
    else
    {
        // Normal communication
        chPtr->txTimeout = 0;
    }

}


void TpInit(){
    TpEcu.txState = VWTP_CONNECT;
    TpEcu.rxState = VWTP_CONNECT;
    TpEcu.seqCntRx = 0xFu;
    TpSendComplete = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(Tp_Cyclic, "VwTp", 4096u, NULL, 6, &VwTpTaskHdl,1);
    vTaskDelay(pdMS_TO_TICKS(120));
}

