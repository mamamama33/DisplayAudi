
#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <AnimatedGIF.h>


static void Dis_CreateStrings(uint8_t *);
void DisCyclic(void *);

#ifdef __cplusplus
extern "C" {
#endif

// Deklaracija funkcije za inicijalizaciju
float Dis_DecodeFrame(uint8_t *);

#ifdef __cplusplus
}
#endif

void DisplayInit();
#define RETOK 0
#define RETERR 1
#define RETWAIT 2



// Deklaracije funkcija
void setupBacklight();
void updateAutoBacklight();
void GIFDraw(GIFDRAW *pDraw);
void playStartupGIF(int repeatCount);
void drawDashboardLayout();
void redrawSpeedBlock();
void updateRpmBar(int val);
void updateSpeed(int spd);
void updateBoost(float act, float spec);
void updatePedal(int pedal);

