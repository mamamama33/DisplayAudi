
#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static void Dis_CreateStrings(uint8_t *);
void DisCyclic(void *);
uint8_t Dis_DecodeFrame(uint8_t *);
void DisplayInit();
#define RETOK 0
#define RETERR 1
#define RETWAIT 2
