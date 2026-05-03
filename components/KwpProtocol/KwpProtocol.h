#ifndef KWP_PROTOCOL_H
#define KWP_PROTOCOL_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdint.h>


#define KwpStart(ecuid) (TpConnect(ecuid))
#define Kwp_DisconnectTp()   (VwTp_Disconnect(1u))
#define Kwp_SendTp(msg)   (VwTp_Send(1u, msg, sizeof(msg)))

#define KWP_OK  (0u)
#define KWP_ERR (1u)



typedef enum{    
    KWP_IDLE,
    KWP_PROCESSING,
    KWP_ERROR,
    KWP_WAITING
}KwpStatus;

//Opisuje stanje kwp protokola
typedef enum{
    KWP_START,
    KWP_START_HANDSHAKE,// USPOSTAVA VEZE
    KWP_START_SESSION, //
    KWP_READECUID, //proveravam da li je to moj EDC16u1 ECU
    KWP_ROUTINE,
    KWP_READDID,
    KWP_READY,
    KWP_CLOSE
}KwpStage;

uint8_t KwpInit();
uint8_t Kwp_GetConnectionState(void);
void Kwp_TxConfirmation(uint8_t );

void Kwp_TxConfirmation(uint8_t);
void Kwp_Receive(uint8_t *,uint16_t );
uint8_t KwpRequest(uint8_t); //SETER DID-a
uint8_t Kwp_GetDataFromEcu(uint8_t*);//GETER

#endif // KWP_PROTOCOL_H