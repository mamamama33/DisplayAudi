
#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#define VWTP_RXBUFFERSIZE 256u
#define VWTP_TXBUFFERSIZE 256u
#define TPSENDMESSAGE(id,len,dataPtr) (CanWrite(id,len,dataPtr))

#define VWTP_OK         ((uint8_t)0u)
#define VWTP_ERR        ((uint8_t)1u)
#define VWTP_PENDING    ((uint8_t)2u)

#define CAN_OK (uint8_t) 0u
#define CAN_ERR     ((uint8_t)1)


#define VWTP_TPPARAMS_MASK       (1u)
#define VWTP_TPPARAMS_REQUEST    (2u)
#define VWTP_TPPARAMS_RESPONSE   (3u)

#define VWTP_TXTASK_ACK_READY    (1u)
#define VWTP_TXTASK_ACK_NOTREADY (2u)

typedef enum
{
    VWTP_CONNECT = 0u,
    VWTP_IDLE,
    VWTP_WAIT,
    VWTP_ACK,
    VWTP_FINISHED
}VwTp_StatesType;

typedef enum
{
    VWTP_DIAG = 0u,
    VWTP_NONDIAG
}VwTp_ModeType;

typedef enum
{
    VWTP_APP_READY = 0u,
    VWTP_APP_BUSY
}VwTp_AppState;

typedef struct
{
    uint8_t ack : 2; // 1 = ack, ready 2 = ack, not ready
    uint8_t params :2; // 2 = req, 3 = resp
    uint8_t brk :1;
} VwTp_TxTasks;

typedef struct 
{
    uint16_t rxId; // canID for reception
    uint16_t txId; // canID for sending frames
    uint8_t blockSize; // sent frames before ack
    uint8_t ackTimeout; // time until wait for ack
    uint8_t ips; // inter-packet-space, time between two TP frames
    void (*rxIndication)(uint8_t *data,uint16_t len); //callback: Data rx
    void (*txConfirmation)(uint8_t result); //callback: Data sent
}Tp_ChannelCfgType;

typedef struct
{
    uint8_t seqCntTx;
    uint8_t ackSeqCntTx; // last acknowledged tx seq cnt
    uint8_t seqCntRx;
    uint8_t ackSeqCntRx; // last acknowledged rx seq cnt
    uint8_t rxBuffer[VWTP_RXBUFFERSIZE];
    uint8_t txBuffer[VWTP_TXBUFFERSIZE];
    uint8_t txTimeout;
    uint8_t rxTimeout;
    uint16_t rxSize;
    uint16_t txSize;
    uint16_t txOffset;
    VwTp_StatesType txState;
    VwTp_StatesType rxState;
    VwTp_TxTasks txFlags;
    Tp_ChannelCfgType cfg;
}Tp_ChannelType;

extern Tp_ChannelType TpEcu; // Samo kažemo "postoji negde"
void VwTp_Receive(uint16_t,uint8_t ,uint8_t*);
uint8_t TpConnect(uint8_t);
uint8_t VwTp_Send(uint8_t , uint8_t * , uint16_t );
void VwTp_Disconnect();
extern volatile uint8_t KwpStartSessionFlag;
extern SemaphoreHandle_t TpSendComplete;
void TpInit();