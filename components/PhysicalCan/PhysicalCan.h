#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>

#define STALKBUTTONRXID 0X35F

#define ENGINERXID2 0x300
#define ENGINERXID 0x201

uint8_t GetStalkButton();
void Can_Init();
void PhysicalCanInit();
uint8_t CanWrite(uint16_t,uint8_t,uint8_t*);

