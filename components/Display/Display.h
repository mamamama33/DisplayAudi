
#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../AnimatedGIF/src/AnimatedGIF.h"

static void Dis_CreateStrings(uint8_t *);
void DisCyclic(void *);

#ifdef __cplusplus
extern "C" {
#endif

// Deklaracija funkcije za inicijalizaciju
uint8_t Dis_DecodeFrame(uint8_t *);

#ifdef __cplusplus
}
#endif

void DisplayInit();
#define RETOK 0
#define RETERR 1
#define RETWAIT 2


// -------------------------------------------------------------------------
// BOJE I PALETA
// -------------------------------------------------------------------------
#define TFT_BG_DARK   0x1082  // Grafit tamno siva
#define TFT_GRID_LINE 0x2104  // Suptilna razdvajajuća linija
//#define TFT_DARKGREY  0x39E7  // Okviri i neaktivni blokovi
#define TFT_LIGHTRED  0xF800  

// -------------------------------------------------------------------------
// PIN KONFIGURACIJA I PWM
// -------------------------------------------------------------------------
const int BL_PIN  = 21;  
const int LDR_PIN = 34;  

const uint32_t PWM_FREQ = 2000;  
const uint8_t  PWM_RES  = 8;     

const int MIN_BRIGHTNESS = 20;   
const int MAX_BRIGHTNESS = 255;  

float filteredBrightness = 255.0; 

// -------------------------------------------------------------------------
// DIMENZIJE EKRANA I GIF-A
// -------------------------------------------------------------------------
const int SCREEN_W = 320;
const int SCREEN_H = 480;

const int GIF_W = 400;
const int GIF_H = 300;

int16_t gif_xpos = 0;
int16_t gif_ypos = 0;

uint32_t updateTime = 0; 

// DIZEL PARAMETRI I GRANICE OBRTAJA
const int MAX_RPM           = 4500; 
const int SPORT_RPM_LIMIT   = 2800; 
const int REDLINE_RPM_LIMIT = 3500; 

// BOOST OPSEG (1.0 bar do 3.0 bar)
const float BOOST_MIN = 1.0;
const float BOOST_MAX = 3.0;

int rpm_val        = 0;
int speed_val      = 0;
float boost_spec   = 1.0; 
float boost_act    = 1.0; 
int pedal_val      = 0; 
int d = 0;

int currentSpeedBgMode = -1; 
uint32_t flashTimer = 0;
bool flashState = false;

// -------------------------------------------------------------------------
// EKRANSKI LAYOUT
// -------------------------------------------------------------------------
const int METER_X = 15;                
const int METER_W = SCREEN_W - 30;    

// 1. LOGO & RPM ZONA
const int RPM_Y = 18;
const int RPM_H = 140;                             
const int RPM_BAR_X = METER_X + 10;        
const int RPM_BAR_Y_BOTTOM = RPM_Y + 130;          
const int MIN_BLOCK_H = 15;                        
const int MAX_BLOCK_H = 70;                        
const int BLOCK_W   = 6;                 
const int NUM_BLOCKS = 27;                
const float BLOCK_STEP = 10.0;            

// 2. SPEED ZONA
const int SPEED_Y = 165;                           
const int SPEED_H = 115;

// 3. BOOST ZONA
const int BOOST_Y = 288;                           
const int BOOST_H = 120;

const int BOOST_NUM_BLOCKS = 18;
const int BOOST_BLOCK_W    = 9;
const int BOOST_BLOCK_H    = 16;
const float BOOST_STEP     = 12.0;

const int REQ_BAR_Y = BOOST_Y + 38;
const int ACT_BAR_Y = BOOST_Y + 76; 

// 4. PEDAL ZONA
const int PEDAL_Y = 415;                           
const int PEDAL_H = 55;

const int PEDAL_BAR_X = METER_X + 15;
const int PEDAL_BAR_Y = PEDAL_Y + 25;
const int PEDAL_BAR_W = METER_W - 30; 
const int PEDAL_BAR_H = 18;

uint16_t lineBuffer[GIF_W];

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