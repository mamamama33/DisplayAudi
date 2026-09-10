#include "Display.h"

#include "esp_log.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "audi_gif.h" 

extern "C" {
#include "DataAnaliser.h"
#include "PhysicalCan.h"
}

// =========================================================================
// DEFINICIJE BOJA (Modernizovana paleta)
// =========================================================================
#define COLOR_BG          0x0821  // Veoma tamno plava / mat crna pozadina
#define COLOR_TEXT_LIGHT  0xFFFF  // Beli primarni tekst
#define COLOR_TEXT_DIM    0x7BEF  // Svetlo siva za oznake
#define COLOR_BAR_BG      0x18E3  // Tamno siva/plava pozadina prazne trake

#define TFT_BG_DARK       0x1082
#define TFT_GRID_LINE     0x2104

#ifdef TFT_DARKGREY
  #undef TFT_DARKGREY
#endif
#define TFT_DARKGREY      0x39E7
#define TFT_LIGHTRED      0xF800

// =========================================================================
// HARDVERSKE DEFINICIJE I PWM
// =========================================================================
#define BL_PIN            GPIO_NUM_21
#define LDR_PIN           GPIO_NUM_34
#define PWM_FREQ          2000
#define MIN_BRIGHT        20
#define MAX_BRIGHT        255

// =========================================================================
// GLOBALNE DIMENZIJE EKRANA (320x480)
// =========================================================================
#define SCREEN_W          320
#define SCREEN_H          480

// =========================================================================
// STRANA 2: TEMPERATURE LAYOUT (320x480)
// =========================================================================
#define NUM_ROWS          5
#define START_Y           38      // Y koordinata prvog reda
#define ROW_SPACING       86      // Vertikalni razmak između redova
#define PADDING_X         15      // Odstojanje od leve/desne ivice
#define BAR_W             (SCREEN_W - (PADDING_X * 2)) // Širina trake (290px)
#define BAR_H             18      // Debljina trake punjenja

// --- INICIJALIZACIJA OBJEKATA ---
TFT_eSPI tft = TFT_eSPI(); 
TFT_eSprite sprSpeed = TFT_eSprite(&tft); 
AnimatedGIF gif;

const int GIF_W = 400, GIF_H = 300;
int16_t gif_xpos = 0, gif_ypos = 0;

const int MAX_RPM = 4500, SPORT_RPM_LIMIT = 2800, REDLINE_RPM_LIMIT = 3500;
const float BOOST_MIN = 1.0, BOOST_MAX = 3.0;

int rpm_val = 0, speed_val = 0, pedal_val = 0, d = 0;
float boost_spec = 1.0, boost_act = 1.0, filteredBrightness = 255.0;
int currentSpeedBgMode = -1;

// Ekranske pozicije za Stranicu 1 (Dashboard)
const int METER_X = 15, METER_W = SCREEN_W - 30;
const int RPM_Y = 18, RPM_BAR_X = METER_X + 10, RPM_BAR_Y_BOTTOM = RPM_Y + 130;
const int MIN_BLOCK_H = 15, MAX_BLOCK_H = 70, BLOCK_W = 6, NUM_BLOCKS = 27;
const float BLOCK_STEP = 10.0;
const int SPEED_Y = 165, SPEED_H = 115, BOOST_Y = 288, BOOST_H = 120;
const int BOOST_NUM_BLOCKS = 18, BOOST_BLOCK_W = 9, BOOST_BLOCK_H = 16;
const float BOOST_STEP = 12.0;
const int REQ_BAR_Y = BOOST_Y + 38, ACT_BAR_Y = BOOST_Y + 76;
const int PEDAL_Y = 415, PEDAL_BAR_X = METER_X + 15, PEDAL_BAR_Y = PEDAL_Y + 25;
const int PEDAL_BAR_W = METER_W - 30, PEDAL_BAR_H = 18;

uint16_t lineBuffer[GIF_W];

static TaskHandle_t disTaskHandle;

uint8_t data[12];
uint8_t block[3];
uint8_t Ispravnost; 
uint8_t Trenutnastrana2 = 0;

// Forward deklaracije
void updateSpeed(int spd);

// =========================================================================
// DEKODIRANJE CAN PA KETA / DIAGNOSTIKE
// =========================================================================
float Dis_DecodeFrame(uint8_t *frameData)
{
    float f = 0;

    switch (frameData[0])
    {
      case 0:
        return 0;
        break;
      
      case 1: // 0.2*a*b rpm
        f = frameData[1] * frameData[2];
        f /= 5;
        break;

      case 2: // a*0.002*b %
      case 3: // 0.002*a*b Deg
        f = frameData[1] * frameData[2];
        f *= 0.002;
        break;

      case 4: // abs(b-127)*0.01*a
        f = abs(frameData[2] - 127);
        f *= frameData[1];
        f /= 100;
        break;
                
      case 5: // a*(b-100)*0.1 °C
        f = frameData[1] * (frameData[2] - 100);
        f /= 10;
        break;
      
      case 6:  // 0.001*a*b V
      case 12: // 0.001*a*b Ohm
      case 21: // 0.001*a*b V
      case 22: // 0.001*a*b ms
      case 24: // 0.001*a*b A
        f = frameData[1] * frameData[2];
        f *= 0.001;
        break;
      
      case 7: // 0.01*a*b km/h
        f = frameData[1] * frameData[2];
        f /= 100;
        break;
        
      case 8: // 0.1*a*b
        f = frameData[1] * frameData[2];
        f *= 0.1;
        break;

      case 9: // (b-127)*0.02*a Deg
        f = (frameData[2] - 127.0) * frameData[1];
        f *= 0.02;
        break;
        
      case 11: // 0.0001*a*(b-128)+1
        f = frameData[1] * (frameData[2] - 128.0);
        f *= 0.0001;
        f += 1;
        break;

      case 13: // (b-127)*0.001*a mm
        f = (frameData[2] - 127.0) * frameData[1];
        f *= 0.001;
        break;

      case 14: // 0.005*a*b bar
        f = frameData[2] * frameData[1];
        f *= 0.005;
        break;
        
      case 15: // 0.01*a*b ms
      case 19: // a*b*0.01 l
        f = frameData[1] * frameData[2];
        f *= 0.01;
        break;
        
      case 18: // 0.04*a*b mbar
        f = frameData[1] * frameData[2];
        f /= 25;
        break;
        
      case 20: // a*(b-128)/128 %
        f = frameData[1] * (frameData[2] - 128);
        f /= 128.0;
        break;
        
      case 23: // b/256*a %
        f = frameData[2] / 256.0;
        f *= frameData[1];
        break;
        
      case 25: // (b*1.421)+(a/182) g/s
        f = (frameData[1] / 182.0) + (1.421 * frameData[2]);
        break;

      case 26: // b-a C
      case 28:
        f = frameData[2] - frameData[1];
        break;
       
      case 27: // abs(b-128)*0.01*a 
        f = (frameData[2] - 128.0);
        if (f < 0) {
          f = f * (-1);
        }
        f *= frameData[1];
        f *= 0.01;
        break;

      case 30: // b/12*a Deg k/w
        f = frameData[2] / 12.0;
        f *= frameData[1];
        break;

      case 31: // b/2560*a °C
        f = frameData[2] / 2560.0;
        f *= frameData[1];
        break;
        
      case 33: // 100*b/a %
        if (frameData[1] == 0) {
          f = 100 * frameData[2];
        } else {
          f = (100 * frameData[2]) / frameData[1];
        }
        break;
      
      case 34: // (b-128)*0.01*a kW
        f = (frameData[2] - 128.0) * frameData[1];
        f *= 0.01;
        break;
        
      case 35: // 0.01*a*b l/h
        f = frameData[2] * frameData[1];
        f *= 0.01;
        break;        

      case 38: // (b-128)*0.001*a Deg k/w
        f = (frameData[2] - 128.0);
        f *= frameData[1];
        f *= 0.001;
        break;
              
      case 39: // b/256*a mg/h
        f = frameData[2] / 256.0;
        f *= frameData[1];
        break;

      case 43: // b*0.1+(25.5*a) V
        f = frameData[2] * 0.1;
        f += 25.5 * frameData[1];
        break;

      case 45: // 0.1*a*b/100     
        f = frameData[2] * frameData[1];
        f /= 1000.0;
        break;

      case 46: // (a*b-3200)*0.0027 Deg k/w
        f = frameData[2] * frameData[1];
        f -= 3200.0;
        f *= 0.0027;
        break;

      case 47: // (b-128)*a ms
        f = (frameData[2] - 128) * frameData[1];
        break;

      case 49: // (b/4)*a*0.1 mg/h
        f = (frameData[2] / 4.0);
        f *= frameData[1];
        f *= 0.1;
        break;
      
      case 50: // (b-128)/(0.01*a) mbar
        f = (frameData[2] - 128.0);
        f /= 0.01;
        if (frameData[1] != 0) {
          f /= frameData[1];
        }
        break;

      case 51: // ((b-128)/255)*a mg/h
        f = frameData[2] - 128.0;
        f /= 255.0;
        f *= frameData[1];
        break;

      case 52: // b*0.02*a-a Nm
        f = frameData[1] * frameData[2];
        f /= 50;
        f -= frameData[1];
        break;

      case 53: // (b-128)*1.4222+0.006*a g/s
        f = (frameData[2] - 128.0) * 1.4222;
        f += frameData[1] * 0.006;
        break;

      case 48:      
      case 54: // a*256+b Count
        f = frameData[1] * 256;
        f += frameData[2];
        break;

      case 55: // a*b/200 s
        f = frameData[1] * frameData[2];
        f /= 200.0;
        break;

      case 56: // a*256+b WSC
        f = 256 * frameData[1] + frameData[2];
        break;

      case 59: // (a*256+b)/32768
        f = 256.0 * frameData[1] + frameData[2];
        f = 32768.0;
        break;

      case 60: // (a*256+b)*0.01 sec
        f = 256.0 * frameData[1] + frameData[2];
        f *= 0.01;
        break;

      case 61: // (b-128)/a
        f = frameData[2] - 128.0;
        if (frameData[1] != 0) {
          f /= frameData[1];
        }
        break;

      case 62: // 0.256*a*b S
        f = frameData[1] * frameData[2];
        f *= 0.256;
        break;

      case 64: // a+b Ohm
        f = frameData[1] + frameData[2];
        break;

      case 65: // 0.01*a*(b-127) mm
        f = 0.01 * frameData[1];
        f *= frameData[2] - 127.0;
        break;

      case 66: // (a*b)/511.12 V
        f = frameData[1] * frameData[2];
        f /= 511.12;
        break;

      case 67: // (640*a)+b*2.5 Deg
        f = 640.0 * frameData[1];
        f += 2.5 * frameData[2];
        break;

      case 68: // (256*a+b)/7.365 deg/s
        f = 256.0 * frameData[1] + frameData[2];
        f /= 7.365;
        break;

      case 69: // (256*a +b)*0.3254 Bar
        f = 256.0 * frameData[1] + frameData[2];
        f *= 0.3254;
        break;

      case 70: // (256*a +b)*0.192 m/s^2
        f = 256.0 * frameData[1] + frameData[2];
        f *= 0.192;
        break;

      default:
        return 0;
        break;
    }

    return f;
}

// =========================================================================
// KONTROLA OSVETLJENJA (PWM + LDR)
// =========================================================================
void setupBacklight() {
    gpio_set_direction(LDR_PIN, GPIO_MODE_INPUT);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = BL_PIN,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = MAX_BRIGHT,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

void updateAutoBacklight() {
    int ldrRaw = analogRead(LDR_PIN); 
    int targetBrightness = map(ldrRaw, 0, 4095, MIN_BRIGHT, MAX_BRIGHT);
    targetBrightness = constrain(targetBrightness, MIN_BRIGHT, MAX_BRIGHT);
    filteredBrightness += (targetBrightness - filteredBrightness) * 0.03f;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)filteredBrightness);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// =========================================================================
// GIF ANIMACIJA
// =========================================================================
void GIFDraw(GIFDRAW *pDraw) {
    uint8_t *s = pDraw->pPixels;
    uint16_t *palette = pDraw->pPalette;
    int iWidth = pDraw->iWidth;
    int y = pDraw->iY + pDraw->y;
    int screenY = gif_ypos + y;

    if (screenY < 0 || screenY >= SCREEN_H) return;

    for (int x = 0; x < iWidth; x++) {
        uint8_t c = *s++;
        lineBuffer[x] = (c == pDraw->ucTransparent || c == 0xFF) ? TFT_BLACK : palette[c];
    }

    int drawX = gif_xpos + pDraw->iX;
    int drawW = iWidth;
    int bufferOffset = 0;

    if (drawX < 0) { bufferOffset = -drawX; drawW += drawX; drawX = 0; }
    if (drawX + drawW > SCREEN_W) { drawW = SCREEN_W - drawX; }

    if (drawW > 0) {
        tft.pushImage(drawX, screenY, drawW, 1, &lineBuffer[bufferOffset]);
    }
}

void playStartupGIF() {
    tft.fillScreen(TFT_BLACK);
    gif_xpos = (SCREEN_W - GIF_W) / 2; 
    gif_ypos = (SCREEN_H - GIF_H) / 2; 

    while(true){
       if (gif.open((uint8_t *)audi_gif, sizeof(audi_gif), GIFDraw)) {
            tft.startWrite(); // The TFT chip select is locked low
            while (gif.playFrame(false,NULL)) {
              yield();
            }
            gif.close();
            tft.endWrite();
            break;
        }
    }
    /*for (int loopCount = 0; loopCount < repeatCount; loopCount++) {
        if (gif.open((uint8_t *)audi_gif, sizeof(audi_gif), GIFDraw)) {
            tft.startWrite(); // The TFT chip select is locked low
            while (gif.playFrame(false,NULL)) {
              yield();
            }
            gif.close();
            tft.endWrite();
        }
    }
      */
}

// =========================================================================
// STRANA 2: MODERNI PRIKAZ TEMPERATURA (FULL SCREEN 320x480)
// =========================================================================

struct SensorConfig {
    const char* label;
    float minV;
    float maxV;
    float warnV;
    float critV;
};

const SensorConfig sensorConfigs[5] = {
    { "FUEL TEMP",       0.0f,  90.0f, 60.0f, 75.0f },
    { "AMBIENT TEMP",  -20.0f,  50.0f, 35.0f, 45.0f },
    { "COOLANT (RAD)",   0.0f, 120.0f, 90.0f, 105.0f },
    { "COOLANT (ENG)",   0.0f, 120.0f, 95.0f, 110.0f },
    { "INTAKE AIR",    -10.0f,  80.0f, 50.0f, 65.0f }
};

void drawTelemetryRow(uint8_t row, float val, bool isPulsing = false) {
    if (row >= 5) return;

    int yBase = START_Y + (row * ROW_SPACING);
    const SensorConfig& cfg = sensorConfigs[row];

    // 1. Naziv senzora (Gore levo)
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.fillRect(PADDING_X, yBase, 160, 20, COLOR_BG); 
    tft.drawString(cfg.label, PADDING_X, yBase, 2);

    // 2. Vrednost temperature (Gore desno, moderan prikaz)
    tft.setTextDatum(TR_DATUM);
    char buf[12];
    snprintf(buf, sizeof(buf), "%5.1f", val);
    
    tft.fillRect(SCREEN_W - PADDING_X - 100, yBase, 80, 26, COLOR_BG);
    
    if(val >= cfg.critV) {
        tft.setTextColor(TFT_RED, COLOR_BG);
    } else {
        tft.setTextColor(COLOR_TEXT_LIGHT, COLOR_BG);
    }
    
    tft.drawString(buf, SCREEN_W - PADDING_X - 18, yBase, 4);

    // Oznaka za jedinicu "°C"
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft.drawString("C", SCREEN_W - PADDING_X - 12, yBase + 8, 2);

    // 3. Puna progresivna traka
    int barY = yBase + 30;
    
    float minV = cfg.minV;
    float maxV = (cfg.maxV <= minV) ? minV + 1.0f : cfg.maxV;
    
    float clamped = constrain(val, minV, maxV);
    float pct = (clamped - minV) / (maxV - minV);
    
    int fillW = round(pct * BAR_W);

    uint16_t barColor = TFT_GREEN;
    if (val >= cfg.critV) {
        barColor = TFT_RED;
    } else if (val >= cfg.warnV) {
        barColor = TFT_YELLOW;
    } else if (val < 0.0f) {
        barColor = TFT_CYAN;
    }

    // Pozadinski kanal
    tft.fillRoundRect(PADDING_X, barY, BAR_W, BAR_H, 4, COLOR_BAR_BG);
    
    // Popunjena traka
    if (fillW > 0) {
        tft.fillRoundRect(PADDING_X, barY, fillW, BAR_H, 4, barColor);
    }
}

void drawTelemetryPage(float fTemp = 0.0f, float aTemp = 0.0f, float cTemp = 0.0f, float eTemp = 0.0f, float iTemp = 0.0f) {
    tft.fillScreen(COLOR_BG);
    
    // Modernizovan naslov
    tft.setTextColor(0x5AEB, COLOR_BG);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("T E M P E R A T U R E S", SCREEN_W / 2, 8, 2);

    // Linija razdvajanja
    tft.drawFastHLine(PADDING_X, 26, BAR_W, COLOR_BAR_BG);

    float currentValues[5] = { fTemp, aTemp, cTemp, eTemp, iTemp };

    for (uint8_t row = 0; row < 5; row++) {
        drawTelemetryRow(row, currentValues[row]);
    }
}

void updateFuelTemp(float temp, bool isPulsing = false) { drawTelemetryRow(0, temp, isPulsing); }
void updateAmbientTemp(float temp, bool isPulsing = false) { drawTelemetryRow(1, temp, isPulsing); }
void updateCoolantTemp(float temp, bool isPulsing = false) { drawTelemetryRow(2, temp, isPulsing); }
void updateCoolantTempEngine(float temp, bool isPulsing = false) { drawTelemetryRow(3, temp, isPulsing); }
void updateIntakeAir(float temp, bool isPulsing = false) { drawTelemetryRow(4, temp, isPulsing); }

// =========================================================================
// STRANA 1: DASHBOARD (RPM / SPEED / BOOST / PEDAL)
// =========================================================================
uint16_t getSpeedBgColor() {
    return (currentSpeedBgMode == 2) ? TFT_LIGHTRED : TFT_BG_DARK; 
}

void redrawSpeedBlock() { 
    updateSpeed(speed_val); 
}

void drawDashboardLayout() {
    tft.fillScreen(TFT_BG_DARK);
    tft.setTextColor(TFT_SILVER, TFT_BG_DARK);
    tft.drawCentreString("A U D I   S P O R T", SCREEN_W / 2, 4, 1);
    tft.drawCentreString("RPM", METER_X + METER_W / 2, RPM_Y, 2);

    for (int i = 0; i < NUM_BLOCKS; i++) {
        int bx = RPM_BAR_X + (int)(i * BLOCK_STEP);
        int block_h = map(i, 0, NUM_BLOCKS - 1, MIN_BLOCK_H, MAX_BLOCK_H);
        tft.drawRoundRect(bx, RPM_BAR_Y_BOTTOM - block_h, BLOCK_W, block_h, 1, TFT_DARKGREY);
    }

    tft.drawFastHLine(10, SPEED_Y - 5, SCREEN_W - 20, TFT_GRID_LINE);
    redrawSpeedBlock();
    tft.drawFastHLine(10, BOOST_Y - 5, SCREEN_W - 20, TFT_GRID_LINE);

    tft.setTextColor(TFT_SILVER, TFT_BG_DARK);
    tft.drawCentreString("BOOST", METER_X + METER_W / 2, BOOST_Y + 2, 2);
    tft.drawString("REQ", METER_X + 5, REQ_BAR_Y + 2, 2);
    tft.drawString("ACT", METER_X + 5, ACT_BAR_Y + 2, 2);

    tft.drawFastHLine(10, PEDAL_Y - 5, SCREEN_W - 20, TFT_GRID_LINE);
    tft.drawString("ACCEL PEDAL", METER_X + 12, PEDAL_Y + 4, 2);
    tft.drawRect(PEDAL_BAR_X, PEDAL_BAR_Y, PEDAL_BAR_W, PEDAL_BAR_H, TFT_DARKGREY);
}

void updateSpeed(int spd) {
    if (spd < 0) spd = 0;
    if (spd > 299) spd = 299;

    char buf[8]; 
    itoa(spd, buf, 10);

    uint16_t bg = getSpeedBgColor();
    uint16_t spdColor = (currentSpeedBgMode == 2 || currentSpeedBgMode == 3) ? 
                        (currentSpeedBgMode == 2 ? TFT_BLACK : TFT_LIGHTRED) : TFT_WHITE;

    sprSpeed.fillSprite(bg);
    sprSpeed.setTextSize(1); 
    sprSpeed.setTextColor(spdColor, bg);
    sprSpeed.setTextDatum(MC_DATUM); 
    sprSpeed.drawString(buf, METER_W / 2, 42, 8); 
    sprSpeed.pushSprite(METER_X, SPEED_Y); 
}

void updateRpmBar(int val) {
    if (val < 0) val = 0;
    if (val > MAX_RPM) val = MAX_RPM;

    int active_blocks = map(val, 0, MAX_RPM, 0, NUM_BLOCKS);

    for (int i = 0; i < NUM_BLOCKS; i++) {
        int bx = RPM_BAR_X + (int)(i * BLOCK_STEP);
        int block_h = map(i, 0, NUM_BLOCKS - 1, MIN_BLOCK_H, MAX_BLOCK_H);
        int by = RPM_BAR_Y_BOTTOM - block_h;
        uint16_t block_color = (i >= 20) ? TFT_LIGHTRED : ((i >= 11) ? TFT_YELLOW : TFT_GREEN);

        if (i < active_blocks) {
            tft.fillRoundRect(bx, by, BLOCK_W, block_h, 1, block_color);
        } else {
            tft.fillRoundRect(bx, by, BLOCK_W, block_h, 1, TFT_BG_DARK);
            tft.drawRoundRect(bx, by, BLOCK_W, block_h, 1, TFT_DARKGREY);
        }
    }

    char buf[8]; 
    itoa(val, buf, 10);
    tft.fillRect(METER_X + (METER_W / 2) - 45, RPM_Y + 16, 90, 26, TFT_BG_DARK);
    tft.setTextColor(val >= REDLINE_RPM_LIMIT ? TFT_LIGHTRED : (val >= SPORT_RPM_LIMIT ? TFT_YELLOW : TFT_WHITE), TFT_BG_DARK);
    tft.drawCentreString(buf, METER_X + METER_W / 2, RPM_Y + 16, 4); 
}

void updateBoostReq(float spec) {
    int spec_blocks = map((int)(spec * 100), 100, 300, 0, BOOST_NUM_BLOCKS);
    int startX = METER_X + 50;
    bool pulseState = (millis() % 160) < 80;

    for (int i = 0; i < BOOST_NUM_BLOCKS; i++) {
        int bx = startX + (int)(i * BOOST_STEP);
        uint16_t col = (i >= 12) ? TFT_LIGHTRED : ((i >= 7) ? TFT_YELLOW : TFT_GREEN);

        if (i < spec_blocks) {
            tft.fillRect(bx, REQ_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H, (i == spec_blocks - 1 && pulseState) ? TFT_WHITE : col);
        } else {
            tft.fillRect(bx, REQ_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H, TFT_BG_DARK);
            tft.drawRect(bx, REQ_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H, TFT_DARKGREY);
        }
    }

    char buf_spec[6];
    dtostrf(spec, 3, 1, buf_spec); 
    tft.fillRect(startX + (BOOST_NUM_BLOCKS * BOOST_STEP) + 5, REQ_BAR_Y, 35, 16, TFT_BG_DARK);
    tft.drawRightString(buf_spec, METER_X + METER_W, REQ_BAR_Y, 2);
}

void updateBoostAct(float act) {
    int act_blocks = map((int)(act * 100), 100, 300, 0, BOOST_NUM_BLOCKS);
    int startX = METER_X + 50;
    bool pulseState = (millis() % 160) < 80;

    for (int i = 0; i < BOOST_NUM_BLOCKS; i++) {
        int bx = startX + (int)(i * BOOST_STEP);
        uint16_t col = (i >= 12) ? TFT_LIGHTRED : ((i >= 7) ? TFT_YELLOW : TFT_GREEN);

        if (i < act_blocks) {
            tft.fillRect(bx, ACT_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H, (i == act_blocks - 1 && pulseState) ? TFT_WHITE : col);
        } else {
            tft.fillRect(bx, ACT_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H, TFT_BG_DARK);
            tft.drawRect(bx, ACT_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H, TFT_DARKGREY);
        }
    }

    char buf_act[6];
    dtostrf(act, 3, 1, buf_act);
    tft.fillRect(startX + (BOOST_NUM_BLOCKS * BOOST_STEP) + 5, ACT_BAR_Y, 35, 16, TFT_BG_DARK);
    tft.drawRightString(buf_act, METER_X + METER_W, ACT_BAR_Y, 2);
}

void updatePedal(int pedal) {
    if (pedal < 0) pedal = 0;
    if (pedal > 100) pedal = 100;

    int fill_w = map(pedal, 0, 100, 0, PEDAL_BAR_W - 2);
    uint16_t pedal_color = (pedal > 80) ? TFT_LIGHTRED : ((pedal > 40) ? TFT_YELLOW : TFT_GREEN);

    if (fill_w > 0) {
        tft.fillRect(PEDAL_BAR_X + 1, PEDAL_BAR_Y + 1, fill_w, PEDAL_BAR_H - 2, pedal_color);
    }
    if (PEDAL_BAR_W - 2 - fill_w > 0) {
        tft.fillRect(PEDAL_BAR_X + 1 + fill_w, PEDAL_BAR_Y + 1, PEDAL_BAR_W - 2 - fill_w, PEDAL_BAR_H - 2, TFT_BG_DARK);
    }

    char buf[8]; 
    sprintf(buf, "%d%%", pedal);
    tft.fillRect(PEDAL_BAR_X + PEDAL_BAR_W - 45, PEDAL_Y + 4, 45, 16, TFT_BG_DARK);
    tft.setTextColor(pedal_color, TFT_BG_DARK);
    tft.drawRightString(buf, PEDAL_BAR_X + PEDAL_BAR_W, PEDAL_Y + 4, 2);
}

// =========================================================================
// GLAVNA DISPLAY SWITCHER LOGIKA
// =========================================================================
void Display(uint8_t firstframe, uint8_t offset, float rez) {
  updateAutoBacklight();

  switch (Trenutnastrana2) {
    case 0: // Strana 1: Dashboard
      if (firstframe == 1) updateRpmBar(rez);
      else if (firstframe == 7) updateSpeed(rez);
      else if (firstframe == 33) updatePedal(rez);
      else if (firstframe == 18 && offset == 1) updateBoostAct(rez);
      else if (firstframe == 18 && offset == 2) updateBoostReq(rez);
    break;

    case 1: // Strana 2: Temperature
      if (offset == 0) updateFuelTemp(rez);
      else if (offset == 1) updateAmbientTemp(rez);
      else if (offset == 2) updateCoolantTemp(rez);
      else if (offset == 3) updateCoolantTempEngine(rez);
      else if (offset == 4) updateIntakeAir(rez);
    break;

    case 2:
    case 3:
    case 4:
    case 5:
    break;
  }
}

int i = 0;
int o = 0;
float Rezultat;

void DisCyclic(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(100));
    while (1) {
          vTaskDelay(pdMS_TO_TICKS(10));
          uint8_t offset, b;

          //i = (i + 1) % 50;
          //o = (o + 1) % 5;
          //Display(5, o, i);
          //Mozda dodati gif animaciju kada se promeni strana trenutna
     


          if (EngineDiag_GetChData(data, &Trenutnastrana2) == RETOK) {
              for (offset = 0; offset < 4u; offset++) {
                        vTaskSuspendAll();
                        for (b = 0; b < 3u; b++) {
                            block[b] = data[(offset * 3) + b];
                        }
                        xTaskResumeAll();

                        Rezultat = Dis_DecodeFrame(block);
              }
            if(Trenutnastrana2==1){

            drawTelemetryPage();
            }

              if (Rezultat != 0) {
                Display(block[0], offset, Rezultat);
              }
              else{
                ESP_LOGI("DIS","Rezultat je 0");
              }
          }
    }
}

void DisplayInit() {
    setupBacklight();

    tft.begin();
    tft.initDMA();

    tft.setRotation(0);

    tft.fillScreen(COLOR_BG);

    sprSpeed.createSprite(METER_W, SPEED_H);

    gif.begin(GIF_PALETTE_RGB565_BE);
    playStartupGIF();

    tft.fillScreen(COLOR_BG);
    
    // Inicijalno iscrtavanje stranice sa temperaturama
    
    //drawTelemetryPage();
    drawDashboardLayout();
    xTaskCreatePinnedToCore(DisCyclic, "Dis", 2048u, NULL, 3, &disTaskHandle, 0);

}