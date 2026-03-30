
#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static void Dis_CreateStrings(uint8_t *);
void DisCyclic(void *);
uint8_t Dis_DecodeFrame(uint8_t *);
void DisplayInit();

