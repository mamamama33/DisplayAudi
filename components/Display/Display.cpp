#include "Display.h"
#include "esp_log.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "audi_gif.h" 
#include "driver/touch_pad.h"
#include "carbonbg.h"
#include "icons.h"
extern "C" {
#include "DataAnaliser.h"
#include "PhysicalCan.h"
}

// =========================================================================
// MODERNE BOJE
// =========================================================================
#define ACCENT_BLUE    0x5D9F
#define ACCENT_SKY     0x4C9F
#define ACCENT_CYAN    0x07FF
#define ACCENT_AMBER   0xFDC0
#define ACCENT_RED     0xF9C7
#define ACCENT_LIME    0x4FE8
#define ACCENT_DARK    0x10A3
#define TRANSPARENT_COLOR  0x0000

#define COLOR_BG          0x0821
#define COLOR_TEXT_LIGHT  0xFFFF
#define COLOR_TEXT_DIM    0x7BEF
#define COLOR_BAR_BG      0x18E3

#define TFT_BG_DARK       0x0821
#define TFT_GRID_LINE     0x2104

#ifdef TFT_DARKGREY
  #undef TFT_DARKGREY
#endif
#define TFT_DARKGREY      0x39E7
#define TFT_LIGHTRED      0xF800

// =========================================================================
// HARDVERSKE DEFINICIJE
// =========================================================================
#define BL_PIN            GPIO_NUM_15
#define LDR_PIN           GPIO_NUM_32
#define LDR_ADC_CHANNEL   ADC1_CHANNEL_4 

#define WARNINGLIGHT      GPIO_NUM_36
#define WARNINGLIGHT2     GPIO_NUM_39
#define WARNINGLIGHT3     GPIO_NUM_34
#define WARNINGLIGHT4     GPIO_NUM_35
#define PWM_FREQ          2000
#define MIN_BRIGHT        50
#define MAX_BRIGHT        255

#define LDR_ADC_MIN       1700
#define LDR_ADC_MAX       4600

#define SCREEN_W          320
#define SCREEN_H          480

#define NUM_ROWS          5
#define START_Y           38
#define ROW_SPACING       86
#define PADDING_X         15
#define BAR_W             (SCREEN_W - (PADDING_X * 2))
#define BAR_H             18

#define ICON_SIZE         32
#define ICON_POS_X        (SCREEN_W - PADDING_X - ICON_SIZE)
#define ICON_POS_Y_OFFSET 50

TFT_eSPI tft = TFT_eSPI(); 
TFT_eSprite sprSpeed = TFT_eSprite(&tft); 
TFT_eSprite sprInj[4] = {
    TFT_eSprite(&tft),
    TFT_eSprite(&tft),
    TFT_eSprite(&tft),
    TFT_eSprite(&tft)
};
AnimatedGIF gif;

// Dimenzije injection metera
#define INJ_METER_R      60
#define INJ_METER_SIZE   (INJ_METER_R * 2 + 10)

const int GIF_W = 400, GIF_H = 300;
int16_t gif_xpos = 0, gif_ypos = 0;

const int MAX_RPM = 4500, SPORT_RPM_LIMIT = 2800, REDLINE_RPM_LIMIT = 3500;
const float BOOST_MIN = 1.0, BOOST_MAX = 3.0;

int rpm_val = 0, speed_val = 0, pedal_val = 0, d = 0;
float boost_spec = 1.0, boost_act = 1.0, filteredBrightness = 255.0;
int currentSpeedBgMode = -1;

const int METER_X = 15, METER_W = SCREEN_W - 30;
const int RPM_Y = 18, RPM_BAR_X = METER_X + 10, RPM_BAR_Y_BOTTOM = RPM_Y + 130;
const int MIN_BLOCK_H = 15, MAX_BLOCK_H = 70, BLOCK_W = 6, NUM_BLOCKS = 27;
const float BLOCK_STEP = 10.0;
const int SPEED_Y = 165, SPEED_H = 115, BOOST_Y = 288, BOOST_H = 120;
const int BOOST_NUM_BLOCKS = 18, BOOST_BLOCK_W = 9, BOOST_BLOCK_H = 15;
const float BOOST_STEP = 12.0;
const int REQ_BAR_Y = BOOST_Y + 38, ACT_BAR_Y = BOOST_Y + 76;
const int PEDAL_Y = 415, PEDAL_BAR_X = METER_X + 5, PEDAL_BAR_Y = PEDAL_Y + 25;
const int PEDAL_BAR_W = METER_W - 10, PEDAL_BAR_H = 26;

uint16_t lineBuffer[GIF_W];

static TaskHandle_t disTaskHandle;

uint8_t data[12];
uint8_t block[3];
uint8_t Ispravnost; 
uint8_t Trenutnastrana2 = 0;
uint8_t TrenutniId = 0;
uint16_t touch_val = 0;
bool changed = false;
uint8_t brojac = 0;

// --- VREDNOSTI ZA NOVE STRANICE ---
float refrigerantPressure = 40;
float coolantTempGlobal = 100;
float coolantTempGlobalIN = 100;
float fanDuty = 54;

float injectionDuration[4] = {3, 5, 2, 1};

float mafValue = 3;
float voltageValue = 14;

float vehicleSpeed = 0.0f;

// --- 0-100 TIMER ---
enum RunState {
    RUN_WAIT_STATIONARY,
    RUN_ARMED,
    RUN_MEASURING,
    RUN_RESULT
};

RunState runState = RUN_WAIT_STATIONARY;
uint32_t runStartTime = 0;
uint32_t runStillSince = 0;
uint32_t runResultShownAt = 0;

float time60 = 0.0f;
float time80 = 0.0f;
float time100 = 0.0f;

float bestTime100 = 0.0f;
float lastTime100 = 0.0f;

float historyTimes[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
int historyIndex = 0;


// Forward deklaracije
void updateSpeed(int spd);
void updateRpmBar(int val);
void updateBoostReq(int spec);
void updateBoostAct(int act);
void updatePedal(int pedal);
void update0to100(float speed_kmh);
void drawCoolingStatic();
void drawCoolingDynamic();
void drawInjectionPage();
void drawMafStatic();
void drawMafDynamic();
void drawTimerStatic();
void drawTimerDynamic();

// =========================================================================
// SPRITE POMOĆNA
// =========================================================================
void fillSpriteWithCarbon(TFT_eSprite* spr, int x, int y, int w, int h) {
    const uint16_t *bg = carbon_bg;
    uint16_t *spr_ptr = (uint16_t *)spr->getPointer();
    
    for (int row = 0; row < h; row++) {
        const uint16_t *src = bg + (y + row) * SCREEN_W + x;
        for (int col = 0; col < w; col++) {
            uint16_t c = src[col];
            spr_ptr[row * w + col] = (c << 8) | (c >> 8);
        }
    }
}

// =========================================================================
// CARBON POZADINA
// =========================================================================
static bool carbon_active = false;

void redrawCarbonRegion(int x, int y, int w, int h) {
    if (w <= 0 || h <= 0) return;
    
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > SCREEN_W) w = SCREEN_W - x;
    if (y + h > SCREEN_H) h = SCREEN_H - y;
    if (w <= 0 || h <= 0) return;
    
    const uint16_t *bg = carbon_bg;
    
    tft.setSwapBytes(true);
    tft.startWrite();
    tft.setAddrWindow(x, y, w, h);
    for (int row = 0; row < h; row++) {
        const uint16_t *src = bg + (y + row) * SCREEN_W + x;
        tft.pushColors((uint16_t *)src, w, true);
    }
    tft.endWrite();
    tft.setSwapBytes(false);
}

void smartClear(int x, int y, int w, int h) {
    if (carbon_active) {
        redrawCarbonRegion(x, y, w, h);
    } else {
        tft.fillRect(x, y, w, h, COLOR_BG);
    }
}

// =========================================================================
// KONTROLA OSVETLJENJA
// =========================================================================
void setupBacklight() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LDR_ADC_CHANNEL, ADC_ATTEN_DB_11);

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
        .hpoint         = 0,
        .flags          = { .output_invert = 0 }
    };
    ledc_channel_config(&ledc_channel);

    touch_pad_init();
    touch_pad_config(TOUCH_PAD_NUM8, 0);
}

// =========================================================================
// DEKODIRANJE CAN PAKETA
// =========================================================================
float Dis_DecodeFrame(uint8_t *frameData)
{
    float f = 0;
    switch (frameData[0])
    {
      case 0: return 0;
      case 1: f = frameData[1] * frameData[2]; f /= 5.0f; break;
      case 2:
      case 3: f = frameData[1] * frameData[2]; f *= 0.002f; break;
      case 4: f = abs(frameData[2] - 127); f *= frameData[1]; f /= 100.0f; break;
      case 5: f = frameData[1] * (frameData[2] - 100); f /= 10.0f; break;
      case 6:
      case 12:
      case 21:
      case 22:
      case 24: f = frameData[1] * frameData[2]; f *= 0.001f; break;
      case 7: f = frameData[1] * frameData[2]; f /= 100.0f; break;
      case 8: f = frameData[1] * frameData[2]; f *= 0.1f; break;
      case 9: f = (frameData[2] - 127.0f) * frameData[1]; f *= 0.02f; break;
      case 11: f = frameData[1] * (frameData[2] - 128.0f); f *= 0.0001f; f += 1.0f; break;
      case 13: f = (frameData[2] - 127.0f) * frameData[1]; f *= 0.001f; break;
      case 14: f = frameData[2] * frameData[1]; f *= 0.005f; break;
      case 15:
      case 19: f = frameData[1] * frameData[2]; f *= 0.01f; break;
      case 18: f = frameData[1] * frameData[2]; f /= 25.0f; break;
      case 20: f = frameData[1] * (frameData[2] - 128.0f); f /= 128.0f; break;
      case 23: f = frameData[2] / 256.0f; f *= frameData[1]; break;
      case 25: f = (frameData[1] / 182.0f) + (1.421f * frameData[2]); break;
      case 26:
      case 28: f = (float)frameData[2] - frameData[1]; break;
      case 27: f = (frameData[2] - 128.0f); if (f < 0) f *= -1.0f; f *= frameData[1]; f *= 0.01f; break;
      case 30: f = frameData[2] / 12.0f; f *= frameData[1]; break;
      case 31: f = frameData[2] / 2560.0f; f *= frameData[1]; break;
      case 33: if (frameData[1] == 0) f = 100.0f * frameData[2]; else f = (100.0f * frameData[2]) / frameData[1]; break;
      case 34: f = (frameData[2] - 128.0f) * frameData[1]; f *= 0.01f; break;
      case 35: f = frameData[2] * frameData[1]; f *= 0.01f; break;
      case 38: f = (frameData[2] - 128.0f); f *= frameData[1]; f *= 0.001f; break;
      case 39: f = frameData[2] / 256.0f; f *= frameData[1]; break;
      case 43: f = frameData[2] * 0.1f; f += 25.5f * frameData[1]; break;
      case 45: f = frameData[2] * frameData[1]; f /= 1000.0f; break;
      case 46: f = frameData[2] * frameData[1]; f -= 3200.0f; f *= 0.0027f; break;
      case 47: f = (frameData[2] - 128.0f) * frameData[1]; break;
      case 49: f = (frameData[2] / 4.0f); f *= frameData[1]; f *= 0.1f; break;
      case 50: f = (frameData[2] - 128.0f); f /= 0.01f; if (frameData[1] != 0) f /= frameData[1]; break;
      case 51: f = frameData[2] - 128.0f; f /= 255.0f; f *= frameData[1]; break;
      case 52: f = frameData[1] * frameData[2]; f /= 50.0f; f -= frameData[1]; break;
      case 53: f = (frameData[2] - 128.0f) * 1.4222f; f += frameData[1] * 0.006f; break;
      case 48:
      case 54: f = frameData[1] * 256.0f; f += frameData[2]; break;
      case 55: f = frameData[1] * frameData[2]; f /= 200.0f; break;
      case 56: f = 256.0f * frameData[1] + frameData[2]; break;
      case 59: f = 256.0 * frameData[1] + frameData[2]; f = 32768.0; break;
      case 60: f = 256.0f * frameData[1] + frameData[2]; f *= 0.01f; break;
      case 61: f = frameData[2] - 128.0f; if (frameData[1] != 0) f /= frameData[1]; break;
      case 62: f = frameData[1] * frameData[2]; f *= 0.256f; break;
      case 64: f = (float)frameData[1] + frameData[2]; break;
      case 65: f = 0.01f * frameData[1]; f *= frameData[2] - 127.0f; break;
      case 66: f = frameData[1] * frameData[2]; f /= 511.12f; break;
      case 67: f = 640.0f * frameData[1]; f += 2.5f * frameData[2]; break;
      case 68: f = 256.0f * frameData[1] + frameData[2]; f /= 7.365f; break;
      case 69: f = 256.0f * frameData[1] + frameData[2]; f *= 0.3254f; break;
      case 70: f = 256.0f * frameData[1] + frameData[2]; f *= 0.192f; break;
      default: return 0;
    }
    return f;
}

void updateAutoBacklight() {
    int ldrRaw = analogRead(LDR_PIN); 
    ldrRaw = constrain(ldrRaw, LDR_ADC_MIN, LDR_ADC_MAX);

    int targetBrightness = map(ldrRaw, LDR_ADC_MIN, LDR_ADC_MAX, MIN_BRIGHT, MAX_BRIGHT);
    filteredBrightness += (targetBrightness - filteredBrightness) * 0.02f;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)filteredBrightness);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    touch_pad_read(TOUCH_PAD_NUM8, &touch_val);
    if(touch_val < 200) {
        changed = true;
        brojac++;
        if(brojac >= 7) {
            brojac = 0;
        }
        SetStrana(brojac);
    }

    vTaskDelay(pdMS_TO_TICKS(10)); 
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
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 255);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    while(true){
       if (gif.open((uint8_t *)audi_gif, sizeof(audi_gif), GIFDraw)) {
            tft.startWrite();
            while (gif.playFrame(false,NULL)) {
              yield();
            }
            gif.close();
            tft.endWrite();
            break;
        }
    }
}

// =========================================================================
// RAINBOW
// =========================================================================
uint16_t rainbow(byte value) {
    byte red = 0, green = 0, blue = 0;
    byte quadrant = value / 32;

    if (quadrant == 0) { blue = 31; green = 2 * (value % 32); red = 0; }
    if (quadrant == 1) { blue = 31 - (value % 32); green = 63; red = 0; }
    if (quadrant == 2) { blue = 0; green = 63; red = value % 32; }
    if (quadrant == 3) { blue = 0; green = 63 - 2 * (value % 32); red = 31; }
    return (red << 11) + (green << 5) + blue;
}

// =========================================================================
// RING METER
// =========================================================================
int ringMeter(float value, float vmin, float vmax, int x, int y, int r, 
              const char *units, byte scheme, uint8_t decimals) {
    
    x += r; y += r;
    int w = r / 3;
    int angle = 150;
    long vScaled = map((long)(value * 10), (long)(vmin * 10), (long)(vmax * 10), -angle, angle);
    int v = (int)vScaled;
    byte seg = 3;
    byte inc = 6;
    
    int colour = ACCENT_CYAN;
    
    for (int i = -angle + inc/2; i < angle - inc/2; i += inc) {
        float sx = cos((i - 90) * 0.0174532925);
        float sy = sin((i - 90) * 0.0174532925);
        uint16_t x0 = sx * (r - w) + x;
        uint16_t y0 = sy * (r - w) + y;
        uint16_t x1 = sx * r + x;
        uint16_t y1 = sy * r + y;

        float sx2 = cos((i + seg - 90) * 0.0174532925);
        float sy2 = sin((i + seg - 90) * 0.0174532925);
        int x2 = sx2 * (r - w) + x;
        int y2 = sy2 * (r - w) + y;
        int x3 = sx2 * r + x;
        int y3 = sy2 * r + y;

        if (i < v) {
            switch (scheme) {
                case 0: colour = ACCENT_RED; break;
                case 1: colour = ACCENT_LIME; break;
                case 2: colour = ACCENT_BLUE; break;
                case 3: colour = rainbow(map(i, -angle, angle, 0, 127)); break;
                case 4: colour = rainbow(map(i, -angle, angle, 70, 127)); break;
                default: colour = ACCENT_CYAN; break;
            }
            tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
            tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);
        }
    }
    
    char buf[10];
    if (decimals == 0) snprintf(buf, sizeof(buf), "%d", (int)round(value));
    else snprintf(buf, sizeof(buf), "%.1f", value);
    
    tft.setTextColor(ACCENT_CYAN);
    tft.setTextDatum(MC_DATUM);
    if (r > 60) tft.drawString(buf, x, y - 5, 4);
    else tft.drawString(buf, x, y, 2);
    
    tft.setTextColor(ACCENT_SKY);
    if (r > 60) tft.drawString(units, x, y + 20, 2);
    else tft.drawString(units, x, y + 12, 1);
    
    return x + r;
}

// =========================================================================
// TELEMETRY STRUKTURA
// =========================================================================
struct TelemetryItem {
    const char* label;      
    const char* unit;       
    float val;              
    float minV;             
    float maxV;             
    float warnV;            
    float critV;            
    uint8_t decimals;       
    const uint16_t* icon;   
};

TelemetryItem tempItems[5] = {
    { "FUEL TEMP",      "C", 0.0f,   0.0f,  90.0f, 60.0f, 75.0f, 1, fuelTemp },
    { "AMBIENT TEMP",   "C", 0.0f, -20.0f,  50.0f, 35.0f, 45.0f, 1, Ambient_Intake_temp2 },
    { "COOLANT (RAD)",  "C", 0.0f,   0.0f, 120.0f, 90.0f, 105.0f, 1, engineTemp },
    { "INTAKE AIR",     "C", 0.0f, -10.0f,  80.0f, 50.0f, 65.0f, 1, Ambient_Intake_temp2 },
    { "COOLANT (ENG)",  "C", 0.0f,   0.0f, 120.0f, 95.0f, 110.0f, 1, engineTemp }
};

TelemetryItem engineItems[4] = {
    { "OIL TEMP",       "C",  0.0f,  0.0f, 150.0f, 110.0f, 125.0f, 1, OilTemp },
    { "OIL LEVEL",      "%",  0.0f,  0.0f, 100.0f,  20.0f,  10.0f, 0, oil },
    { "FUEL CONS",      "L",  0.0f,  0.0f,  25.0f,  15.0f,  20.0f, 1, fuelTemp },
    { "ENG TORQUE",     "Nm", 0.0f,  0.0f, 500.0f, 400.0f, 460.0f, 0, engineTorque }
};

// =========================================================================
// STATIČKE PROMENLJIVE
// =========================================================================
static int lastFillW[NUM_ROWS] = {-1, -1, -1, -1, -1};
static uint16_t lastBarColor[NUM_ROWS] = {0, 0, 0, 0, 0};
static bool barFirstDraw[NUM_ROWS] = {true, true, true, true, true};

void resetBarState() {
    for (int i = 0; i < NUM_ROWS; i++) {
        lastFillW[i] = -1;
        lastBarColor[i] = 0;
        barFirstDraw[i] = true;
    }
}

// =========================================================================
// IKONICA
// =========================================================================
void drawIconTransparent(int x, int y, int size, const uint16_t* icon) {
    if (icon == NULL) return;
    
    for (int r = 0; r < size; r++) {
        for (int c = 0; c < size; c++) {
            uint16_t pixel = pgm_read_word(&icon[r * size + c]);
            if (pixel != TRANSPARENT_COLOR) {
                tft.drawPixel(x + c, y + r, pixel);
            }
        }
    }
}

// =========================================================================
// TELEMETRY ROW STATIC + DYNAMIC
// =========================================================================
void drawTelemetryRowStatic(uint8_t row, TelemetryItem& item) {
    if (row >= NUM_ROWS) return;
    int yBase = START_Y + (row * ROW_SPACING);
    
    if (item.icon != NULL) {
        int iconX = ICON_POS_X;
        int iconY = yBase + ICON_POS_Y_OFFSET;
        if (iconY + ICON_SIZE > yBase + ROW_SPACING - 2) iconY = yBase + ROW_SPACING - ICON_SIZE - 2;
        smartClear(iconX, iconY, ICON_SIZE, ICON_SIZE);
        drawIconTransparent(iconX, iconY, ICON_SIZE, item.icon);
    }
    
    smartClear(PADDING_X, yBase - 2, 170, 22);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(ACCENT_BLUE);
    tft.drawString(item.label, PADDING_X, yBase, 2);
    
    smartClear(SCREEN_W - PADDING_X - 40, yBase + 4, 40, 22);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(ACCENT_SKY);
    tft.drawString(item.unit, SCREEN_W - PADDING_X - 12, yBase + 6, 2);
}
void drawTelemetryRowDynamic(uint8_t row, TelemetryItem& item) {
    if (row >= NUM_ROWS) return;
    
    static uint32_t lastUpdateTime[NUM_ROWS] = {0};
    uint32_t now = millis();
    if (now - lastUpdateTime[row] < 50) return;
    lastUpdateTime[row] = now;
    
    static float lastVal[NUM_ROWS] = {0};
    static bool firstDrawVal[NUM_ROWS] = {true};
    
    if (!firstDrawVal[row]) {
        float diff = fabs(item.val - lastVal[row]);
        if (diff < 0.2f) return;
    }
    firstDrawVal[row] = false;
    lastVal[row] = item.val;
    
    int yBase = START_Y + (row * ROW_SPACING);
    
    char buf[12];
    if (item.decimals == 0) snprintf(buf, sizeof(buf), "%d", (int)round(item.val));
    else snprintf(buf, sizeof(buf), "%.1f", item.val);
    
    uint16_t textColor = ACCENT_CYAN;
    uint16_t barColor  = ACCENT_LIME;
    
    if (item.val >= item.critV) { textColor = ACCENT_RED; barColor = ACCENT_RED; }
    else if (item.val >= item.warnV) { textColor = ACCENT_AMBER; barColor = ACCENT_AMBER; }
    else if (item.val < item.minV) { barColor = ACCENT_BLUE; }
    
    int valX = SCREEN_W - PADDING_X - 22 - 90;
    int valY = yBase - 6;
    smartClear(valX, valY, 90, 32);
    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(textColor);
    tft.drawString(buf, SCREEN_W - PADDING_X - 22, yBase, 4);
    
    int barY = yBase + 30;
    
    float minV = item.minV;
    float maxV = (item.maxV <= minV) ? minV + 1.0f : item.maxV;
    float clamped = constrain(item.val, minV, maxV);
    float pct = (clamped - minV) / (maxV - minV);
    
    // ← PROMENJENO: fillW se računa za unutrašnjost (BAR_W - 4)
    int fillW = round(pct * (BAR_W - 4));
    
    int prevFillW = lastFillW[row];
    uint16_t prevColor = lastBarColor[row];
    
    if (barFirstDraw[row]) {
        smartClear(PADDING_X, barY, BAR_W, BAR_H);
        tft.drawRoundRect(PADDING_X, barY, BAR_W, BAR_H, 4, ACCENT_DARK);
        tft.drawRoundRect(PADDING_X + 1, barY + 1, BAR_W - 2, BAR_H - 2, 4, ACCENT_BLUE);
        if (fillW > 0) tft.fillRoundRect(PADDING_X + 2, barY + 2, fillW, BAR_H - 4, 3, barColor);
        //                                                ↑          ↑       ↑      ↑
        //                                            offset 2   offset 2    visina
        lastFillW[row] = fillW;
        lastBarColor[row] = barColor;
        barFirstDraw[row] = false;
        return;
    }
    
    if (prevFillW == fillW && prevColor == barColor) return;
    
    if (fillW > prevFillW && prevColor == barColor) {
        // Dodaj samo dodatak (od prevFillW do fillW)
        tft.fillRect(PADDING_X + 2 + prevFillW, barY + 2, fillW - prevFillW, BAR_H - 4, barColor);
    }
    else if (fillW < prevFillW && prevColor == barColor) {
        // Obriši višak
        smartClear(PADDING_X + 2 + fillW, barY + 2, prevFillW - fillW, BAR_H - 4);
        
        // Ponovo nacrtaj okvir
        tft.drawRoundRect(PADDING_X, barY, BAR_W, BAR_H, 4, ACCENT_DARK);
        tft.drawRoundRect(PADDING_X + 1, barY + 1, BAR_W - 2, BAR_H - 2, 4, ACCENT_BLUE);
        
        // Nacrtaj fill
        if (fillW > 0) tft.fillRoundRect(PADDING_X + 2, barY + 2, fillW, BAR_H - 4, 3, barColor);
    }
    else {
        // Boja se promenila — sve ispočetka
        smartClear(PADDING_X, barY, BAR_W, BAR_H);
        tft.drawRoundRect(PADDING_X, barY, BAR_W, BAR_H, 4, ACCENT_DARK);
        tft.drawRoundRect(PADDING_X + 1, barY + 1, BAR_W - 2, BAR_H - 2, 4, ACCENT_BLUE);
        if (fillW > 0) tft.fillRoundRect(PADDING_X + 2, barY + 2, fillW, BAR_H - 4, 3, barColor);
    }
    
    lastFillW[row] = fillW;
    lastBarColor[row] = barColor;
}

// =========================================================================
// STRANICE 1, 2 (TEMPERATURES, ENGINE)
// =========================================================================
void drawTelemetryPageGeneric(const char* title, TelemetryItem items[], uint8_t count) {
    carbon_active = true;
    resetBarState();
    redrawCarbonRegion(0, 0, SCREEN_W, SCREEN_H);
    
    tft.setTextColor(ACCENT_BLUE);
    tft.setTextDatum(TC_DATUM);
    tft.drawString(title, SCREEN_W / 2, 8, 2);
    tft.drawFastHLine(PADDING_X, 26, BAR_W, ACCENT_DARK);

    uint8_t rowsToDraw = (count > NUM_ROWS) ? NUM_ROWS : count;
    for (uint8_t row = 0; row < rowsToDraw; row++) {
        drawTelemetryRowStatic(row, items[row]);
    }
}

void drawTelemetryPage() { drawTelemetryPageGeneric("T E M P E R A T U R E S", tempItems, 5); }
void showEngineStatusPage() { drawTelemetryPageGeneric("E N G I N E  &  F U E L", engineItems, 4); }

void updateFuelTemp(float temp, bool isPulsing = false)          { tempItems[0].val = temp; drawTelemetryRowDynamic(0, tempItems[0]); }
void updateAmbientTemp(float temp, bool isPulsing = false)       { tempItems[1].val = temp; drawTelemetryRowDynamic(1, tempItems[1]); }
void updateCoolantTemp(float temp, bool isPulsing = false)       { tempItems[2].val = temp; drawTelemetryRowDynamic(2, tempItems[2]); }
void updateIntakeAir(float temp, bool isPulsing = false)         { tempItems[3].val = temp; drawTelemetryRowDynamic(3, tempItems[3]); }
void updateCoolantTempEngine(float temp, bool isPulsing = false) { tempItems[4].val = temp; drawTelemetryRowDynamic(4, tempItems[4]); }

void updateOilTemp(float temp)      { engineItems[0].val = temp; drawTelemetryRowDynamic(0, engineItems[0]); }
void updateOilLevel(float level)    { engineItems[1].val = level; drawTelemetryRowDynamic(1, engineItems[1]); }
void updateFuelCons(float cons)     { engineItems[2].val = cons; drawTelemetryRowDynamic(2, engineItems[2]); }
void updateEngineTorque(float torque) { engineItems[3].val = torque; drawTelemetryRowDynamic(3, engineItems[3]); }

// =========================================================================
// STRANICA 3: COOLING — STATIČKI DEO (samo kad se uđe na stranicu)
// =========================================================================
void drawCoolingStatic() {
    carbon_active = true;
    redrawCarbonRegion(0, 0, SCREEN_W, SCREEN_H);
    
    tft.setTextColor(ACCENT_BLUE);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("C O O L I N G", SCREEN_W / 2, 8, 2);
    
    int r = 55;
    int startX = 30;
    int yTop = 55;
    int r2=70;
    int yCoolant = yTop + r2*4;

    tft.setTextColor(ACCENT_SKY);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("REFRIG", startX + r, yTop + r*2 + 2, 2);
    tft.drawString("COOLANTOUT", startX + r*2 + 30 + r, yTop + r*2 + 2, 2);
    
    // ← COOLANT IN label (gore — gde je bio FAN DUTY)
    tft.setTextColor(ACCENT_SKY);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("COOLANTIN", SCREEN_W/2, yCoolant+25, 2);
    
    // ← FAN DUTY label (dole — gde je bio COOLANT ENG)
    tft.setTextColor(ACCENT_BLUE);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("FAN DUTY",15, yCoolant+80,2);
}

// DINAMIČKI DEO (svaki CAN frame, samo regioni)
void drawCoolingDynamic() {
    if (brojac != 3) return;
    
    // Rate limiting — max 20 FPS
    static uint32_t lastDraw = 0;
    uint32_t now = millis();
    if (now - lastDraw < 50) return;
    lastDraw = now;
    
    carbon_active = true;
    int r = 55;
    int startX = 30;
    int yTop = 55;
    int r2 = 70;
    int yCoolant = yTop + r2*2 + 25;
    
    // REFRIG
    smartClear(startX, yTop, r*2, r*2);
    ringMeter(refrigerantPressure, 0, 30, startX, yTop, r, "bar", 3, 1);
    
    // COOLANTOUT
    smartClear(startX + r*2 + 30, yTop, r*2, r*2);
    ringMeter(coolantTempGlobal, -20, 120, startX + r*2 + 30, yTop, r, "C", 3, 0);
    
    // COOLANTIN
    smartClear((SCREEN_W - r2*2)/2, yCoolant, r2*2, r2*2);
    ringMeter(coolantTempGlobalIN, -20, 120, (SCREEN_W - r2*2)/2, yCoolant, r2, "C", 3, 0);
    
    // FAN DUTY
    int barY = 445;
    int fanFill = map((int)fanDuty, 0, 100, 0, BAR_W - 2);
    
    smartClear(PADDING_X, barY, BAR_W, BAR_H + 2);
    tft.drawRect(PADDING_X, barY, BAR_W, BAR_H, ACCENT_DARK);
    if (fanFill > 0) {
        uint16_t fanColor = (fanDuty > 80) ? ACCENT_RED : ((fanDuty > 40) ? ACCENT_AMBER : ACCENT_LIME);
        tft.fillRect(PADDING_X + 1, barY + 1, fanFill, BAR_H - 2, fanColor);
    }
    
    // FAN DUTY vrednost
    char buf[10];
    snprintf(buf, sizeof(buf), "%d%%", (int)fanDuty);
    smartClear(SCREEN_W - PADDING_X - 50, barY - 25, 50, 22);
    tft.setTextColor(ACCENT_CYAN);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(buf, SCREEN_W - PADDING_X, barY - 25, 2);
}

// =========================================================================
// STRANICA 4: INJECTION
// =========================================================================
static int injOldValue[4] = {-999, -999, -999, -999};

void drawInjectionMeter(int idx, float value, float vmin, float vmax, 
                        int screenX, int screenY, 
                        const char *label, const char *units) {
    
    TFT_eSprite* spr = &sprInj[idx];
    int r = INJ_METER_R;
    int size = INJ_METER_SIZE;
    
    int cx = size / 2;
    int cy = size / 2;
    
    fillSpriteWithCarbon(spr, screenX, screenY, size, size);
    
    int angle = 150;
    
    // Spoljni luk (sivi)
    for (int i = -angle; i <= angle; i += 3) {
        float rad = (i - 90) * 0.0174532925;
        float sx = cos(rad);
        float sy = sin(rad);
        
        int x0 = (int)(sx * r + cx + 0.5f);
        int y0 = (int)(sy * r + cy + 0.5f);
        int x1 = (int)(sx * (r - 4) + cx + 0.5f);
        int y1 = (int)(sy * (r - 4) + cy + 0.5f);
        
        spr->drawLine(x0, y0, x1, y1, ACCENT_DARK);
    }
    
    // Glavne oznake (6 podela)
    for (int i = 0; i <= 6; i++) {
        int a = -angle + (2 * angle * i / 6);
        float rad = (a - 90) * 0.0174532925;
        float sx = cos(rad);
        float sy = sin(rad);
        
        int x0 = (int)(sx * r + cx + 0.5f);
        int y0 = (int)(sy * r + cy + 0.5f);
        int x1 = (int)(sx * (r - 12) + cx + 0.5f);
        int y1 = (int)(sy * (r - 12) + cy + 0.5f);
        
        spr->drawLine(x0, y0, x1, y1, ACCENT_BLUE);
    }
    
    if (value < vmin) value = vmin;
    if (value > vmax) value = vmax;
    
    float pct = (value - vmin) / (vmax - vmin);
    int needleAngle = -angle + (2 * angle * pct);
    float needleRad = (needleAngle - 90) * 0.0174532925;
    
    int nx = (int)(cos(needleRad) * (r - 15) + cx + 0.5f);
    int ny = (int)(sin(needleRad) * (r - 15) + cy + 0.5f);
    
    // Iglа - debela
    spr->drawLine(cx - 2, cy, nx - 2, ny, ACCENT_RED);
    spr->drawLine(cx - 1, cy, nx - 1, ny, ACCENT_RED);
    spr->drawLine(cx, cy, nx, ny, ACCENT_RED);
    spr->drawLine(cx + 1, cy, nx + 1, ny, ACCENT_RED);
    spr->drawLine(cx + 2, cy, nx + 2, ny, ACCENT_RED);
    
    // Centar igle
    spr->fillCircle(cx, cy, 6, ACCENT_RED);
    spr->fillCircle(cx, cy, 3, ACCENT_DARK);
    
    // Vrednost u centru - veći font
    char buf[10];
    snprintf(buf, sizeof(buf), "%.2f", value);
    
    spr->setTextColor(ACCENT_CYAN);
    spr->setTextDatum(MC_DATUM);
    spr->drawString(buf, cx, cy + 28, 4);   // ← font 4 (bilo 2)
    
    // Jedinica
    spr->setTextColor(ACCENT_SKY);
    spr->drawString(units, cx, cy + 48, 2);   // ← font 2 (bilo 1)
    
    spr->pushSprite(screenX, screenY);
    
    // Label iznad metera
    tft.setTextColor(ACCENT_BLUE);
    tft.setTextDatum(TC_DATUM);
    tft.drawString(label, screenX + size/2, screenY - 15, 4);   // ← font 4 (bilo 2)
    
    injOldValue[idx] = (int)value;
}

void drawInjectionPage() {
    if (brojac != 4) return;
    
    carbon_active = true;
    
    int size = INJ_METER_SIZE;   // 130
    int gapX = 20;                // razmak između kolona
    int gapY = 30;                // razmak između redova
    
    // Ukupna širina 2 metera + razmak
    int totalW = 2 * size + gapX;   // 130*2 + 20 = 280
    
    // Centriraj horizontalno
    int startX = (SCREEN_W - totalW) / 2;   // (320 - 280) / 2 = 20
    
    // Centriraj vertikalno (ukupna visina 2 metera + razmak)
    int totalH = 2 * size + gapY;   // 130*2 + 30 = 290
    int startY = (SCREEN_H - totalH) / 2 - 15;   // (480 - 290) / 2 - 15 = 80
    
    drawInjectionMeter(0, injectionDuration[0], -5, 5, startX, startY, "INJ 1", "ms");
    drawInjectionMeter(1, injectionDuration[1], -5, 5, startX + size + gapX, startY, "INJ 2", "ms");
    drawInjectionMeter(2, injectionDuration[2], -5, 5, startX, startY + size + gapY, "INJ 3", "ms");
    drawInjectionMeter(3, injectionDuration[3], -5, 5, startX + size + gapX, startY + size + gapY, "INJ 4", "ms");
}
void drawInjectionStatic() {
    carbon_active = true;
    redrawCarbonRegion(0, 0, SCREEN_W, SCREEN_H);
    
    tft.setTextColor(ACCENT_BLUE);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("I N J E C T I O N", SCREEN_W / 2, 8, 2);
}

// =========================================================================
// STRANICA 5: MAF & VOLTAGE
// =========================================================================
void drawMafStatic() {
    carbon_active = true;
    redrawCarbonRegion(0, 0, SCREEN_W, SCREEN_H);
    
    tft.setTextColor(ACCENT_BLUE);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("M A F   &   V O L T A G E", SCREEN_W / 2, 8, 2);
    
    int r = 88;
    int centerX = SCREEN_W / 2;
    
    tft.setTextColor(ACCENT_SKY);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("MAF", centerX, 410-(r*2)-25, 2); //15 je bilp
    tft.drawString("VOLTAGE", centerX,410, 2); //nilo 235
}

void drawMafDynamic() {
    if (brojac != 5) return;
    
    carbon_active = true;
    int r = 88;
    int centerX = SCREEN_W / 2;
    
    smartClear(centerX - r, 35, r*2, r*2);
    ringMeter(mafValue, 0, 50, centerX - r, 35, r, "g/s", 3, 2);
    
    smartClear(centerX - r, 235, r*2, r*2);
    ringMeter(voltageValue, 0, 16, centerX - r, 235, r, "V", 3, 2);
}

// =========================================================================
// STRANICA 6: 0-100 TIMER
// =========================================================================
void update0to100(float speed_kmh) {
    uint32_t now = millis();
    
    switch (runState) {
        case RUN_WAIT_STATIONARY:
            if (speed_kmh < 1.0f) {
                if (runStillSince == 0) runStillSince = now;
                if (now - runStillSince > 2000) runState = RUN_ARMED;
            } else {
                runStillSince = 0;
            }
            break;
            
        case RUN_ARMED:
            if (speed_kmh > 2.0f) {
                runStartTime = now;
                time60 = 0.0f;
                time80 = 0.0f;
                time100 = 0.0f;
                runState = RUN_MEASURING;
                drawTimerDynamic();
            }
            break;
            
        case RUN_MEASURING:
            {
                float t = (now - runStartTime) / 1000.0f;
                if (time60 == 0.0f && speed_kmh >= 60.0f) time60 = t;
                if (time80 == 0.0f && speed_kmh >= 80.0f) time80 = t;
                if (speed_kmh >= 100.0f) {
                    time100 = t;
                    lastTime100 = t;
                    if (bestTime100 == 0.0f || t < bestTime100) bestTime100 = t;
                    historyTimes[historyIndex] = t;
                    historyIndex = (historyIndex + 1) % 5;
                    runState = RUN_RESULT;
                    runResultShownAt = now;
                    drawTimerDynamic();
                }
                if (t > 30.0f) {
                    runState = RUN_WAIT_STATIONARY;
                    runStillSince = 0;
                    drawTimerDynamic();
                }
            }
            break;
            
        case RUN_RESULT:
            if (now - runResultShownAt > 8000) {
                runState = RUN_WAIT_STATIONARY;
                runStillSince = 0;
                drawTimerDynamic();
            }
            break;
    }
}

void drawTimerStatic() {
    carbon_active = true;
    redrawCarbonRegion(0, 0, SCREEN_W, SCREEN_H);
    
    tft.setTextColor(ACCENT_BLUE);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("0 - 1 0 0  T I M E R", SCREEN_W / 2, 8, 2);
    
    tft.drawFastHLine(PADDING_X, 140, BAR_W, ACCENT_DARK);
    
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(ACCENT_BLUE);
    tft.drawString("0-60:", PADDING_X, 160, 2);
    tft.drawString("0-80:", PADDING_X, 195, 2);
    tft.drawString("0-100:", PADDING_X, 230, 2);
    
    tft.drawFastHLine(PADDING_X, 275, BAR_W, ACCENT_DARK);
    
    tft.setTextColor(ACCENT_BLUE);
    tft.drawString("BEST:", PADDING_X, 295, 2);
    tft.drawString("LAST:", PADDING_X, 325, 2);
    
    tft.drawFastHLine(PADDING_X, 360, BAR_W, ACCENT_DARK);
    
    tft.setTextColor(ACCENT_SKY);
    tft.drawString("HISTORY:", PADDING_X, 375, 2);
}

void drawTimerDynamic() {
    if (brojac != 6) return;
    
    carbon_active = true;
    
    // Status
    const char* statusStr = "WAIT";
    uint16_t statusColor = ACCENT_SKY;
    switch (runState) {
        case RUN_WAIT_STATIONARY: statusStr = "WAIT STATIONARY"; statusColor = ACCENT_SKY; break;
        case RUN_ARMED: statusStr = "ARMED"; statusColor = ACCENT_LIME; break;
        case RUN_MEASURING: statusStr = "MEASURING"; statusColor = ACCENT_AMBER; break;
        case RUN_RESULT: statusStr = "RESULT"; statusColor = ACCENT_CYAN; break;
    }
    
    smartClear(0, 45, SCREEN_W, 30);
    tft.setTextColor(statusColor);
    tft.setTextDatum(TC_DATUM);
    tft.drawString(statusStr, SCREEN_W / 2, 50, 4);
    
    // Brzina
    char buf[20];
    snprintf(buf, sizeof(buf), "%.0f km/h", vehicleSpeed);
    smartClear(0, 90, SCREEN_W, 30);
    tft.setTextColor(ACCENT_CYAN);
    tft.drawString(buf, SCREEN_W / 2, 95, 4);
    
    // Vremena
    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(ACCENT_CYAN);
    
    char tb[20];
    smartClear(SCREEN_W - PADDING_X - 80, 155, 80, 25);
    if (time60 > 0) snprintf(tb, sizeof(tb), "%.2f s", time60); else snprintf(tb, sizeof(tb), "--");
    tft.drawString(tb, SCREEN_W - PADDING_X, 160, 2);
    
    smartClear(SCREEN_W - PADDING_X - 80, 190, 80, 25);
    if (time80 > 0) snprintf(tb, sizeof(tb), "%.2f s", time80); else snprintf(tb, sizeof(tb), "--");
    tft.drawString(tb, SCREEN_W - PADDING_X, 195, 2);
    
    smartClear(SCREEN_W - PADDING_X - 80, 225, 80, 25);
    if (time100 > 0) snprintf(tb, sizeof(tb), "%.2f s", time100); else snprintf(tb, sizeof(tb), "--");
    tft.drawString(tb, SCREEN_W - PADDING_X, 230, 2);
    
    // Best
    tft.setTextColor(ACCENT_LIME);
    smartClear(SCREEN_W - PADDING_X - 80, 290, 80, 25);
    if (bestTime100 > 0) snprintf(tb, sizeof(tb), "%.2f s", bestTime100); else snprintf(tb, sizeof(tb), "--");
    tft.drawString(tb, SCREEN_W - PADDING_X, 295, 2);
    
    // Last
    tft.setTextColor(ACCENT_CYAN);
    smartClear(SCREEN_W - PADDING_X - 80, 320, 80, 25);
    if (lastTime100 > 0) snprintf(tb, sizeof(tb), "%.2f s", lastTime100); else snprintf(tb, sizeof(tb), "--");
    tft.drawString(tb, SCREEN_W - PADDING_X, 325, 2);
    
    // History
    char hist[80] = "";
    for (int i = 0; i < 5; i++) {
        if (historyTimes[i] > 0) {
            char h[16];
            snprintf(h, sizeof(h), "%.2f ", historyTimes[i]);
            strcat(hist, h);
        }
    }
    smartClear(PADDING_X, 400, SCREEN_W - 2*PADDING_X, 30);
    tft.setTextColor(ACCENT_CYAN);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(hist, PADDING_X, 405, 2);
}

// =========================================================================
// DASHBOARD (STRANICA 0)
// =========================================================================
uint16_t getSpeedBgColor() {
    return (currentSpeedBgMode == 2) ? TFT_LIGHTRED : COLOR_BG; 
}

void redrawSpeedBlock() { updateSpeed(speed_val); }

void drawDashboardLayout() {
    carbon_active = true;
    redrawCarbonRegion(0, 0, SCREEN_W, SCREEN_H);
    
    tft.setTextDatum(TL_DATUM); 
    tft.setTextFont(1); 
    tft.setTextColor(TFT_SILVER);
    tft.drawCentreString("A U D I   S P O R T", SCREEN_W / 2, 4, 1);
    
    tft.setTextFont(2);
    tft.drawCentreString("RPM", METER_X + METER_W / 2, RPM_Y, 2);

    for (int i = 0; i < NUM_BLOCKS; i++) {
        int bx = RPM_BAR_X + (int)(i * BLOCK_STEP);
        int block_h = map(i, 0, NUM_BLOCKS - 1, MIN_BLOCK_H, MAX_BLOCK_H);
        tft.drawRoundRect(bx, RPM_BAR_Y_BOTTOM - block_h, BLOCK_W, block_h, 1, TFT_DARKGREY);
    }

    tft.drawFastHLine(10, SPEED_Y - 5, SCREEN_W - 20, TFT_GRID_LINE);
    redrawSpeedBlock();
    tft.drawFastHLine(10, BOOST_Y - 5, SCREEN_W - 20, TFT_GRID_LINE);

    tft.setTextColor(TFT_SILVER);
    tft.drawCentreString("BOOST", METER_X + METER_W / 2, BOOST_Y + 2, 2);
    tft.drawString("REQ", METER_X + 5, REQ_BAR_Y + 2, 2);
    tft.drawString("ACT", METER_X + 5, ACT_BAR_Y + 2, 2);

    tft.drawFastHLine(10, PEDAL_Y - 5, SCREEN_W - 20, TFT_GRID_LINE);
    tft.drawString("ACCEL PEDAL", METER_X + 5, PEDAL_Y + 4, 2);
    tft.drawRect(METER_X + 5, PEDAL_BAR_Y, PEDAL_BAR_W, PEDAL_BAR_H, TFT_DARKGREY);
}
void updateSpeed(int spd) {
    if (spd < 0) spd = 0;
    if (spd > 299) spd = 299;
    speed_val = spd;

    char buf[8]; 
    itoa(spd, buf, 10);

    if (carbon_active) {
        const uint16_t *bg = carbon_bg;
        uint16_t *spr_ptr = (uint16_t *)sprSpeed.getPointer();
        for (int row = 0; row < SPEED_H; row++) {
            const uint16_t *src = bg + (SPEED_Y + row) * SCREEN_W + METER_X;
            for (int col = 0; col < METER_W; col++) {
                uint16_t c = src[col];
                spr_ptr[row * METER_W + col] = (c << 8) | (c >> 8);
            }
        }
    } else {
        sprSpeed.fillSprite(COLOR_BG);
    }

    sprSpeed.setTextSize(1); 
    
    // Broj brzine — BELO
    sprSpeed.setTextColor(TFT_WHITE);
    sprSpeed.setTextDatum(MC_DATUM); 
    sprSpeed.drawString(buf, METER_W / 2, 35, 8);
    
    // "km/h" — BELO
    sprSpeed.setTextColor(TFT_WHITE);
    sprSpeed.setTextDatum(MC_DATUM);
    sprSpeed.drawString("km/h", METER_W / 2, 95, 2);
    
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
            smartClear(bx, by, BLOCK_W, block_h);
            tft.drawRoundRect(bx, by, BLOCK_W, block_h, 1, TFT_DARKGREY);
        }
    }

    char buf[8]; 
    itoa(val, buf, 10);
    smartClear(METER_X + (METER_W / 2) - 45, RPM_Y + 16, 90, 26);
    tft.setTextColor(val >= REDLINE_RPM_LIMIT ? TFT_LIGHTRED : (val >= SPORT_RPM_LIMIT ? TFT_YELLOW : TFT_WHITE));
    tft.setTextDatum(TC_DATUM);
    tft.drawCentreString(buf, METER_X + METER_W / 2, RPM_Y + 16, 4); 
}

void updateBoostReq(int spec) {
    int spec_blocks = map(spec, 0, 3000, 0, BOOST_NUM_BLOCKS);
    int startX = METER_X + 38;
    bool pulseState = (millis() % 160) < 80;

    for (int i = 0; i < BOOST_NUM_BLOCKS; i++) {
        int bx = startX + (int)(i * BOOST_STEP);
        uint16_t col = (i >= 12) ? TFT_LIGHTRED : ((i >= 7) ? TFT_YELLOW : TFT_GREEN);

        if (i < spec_blocks) {
            tft.fillRect(bx, REQ_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H, (i == spec_blocks - 1 && pulseState) ? TFT_WHITE : col);
        } else {
            smartClear(bx, REQ_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H);
            tft.drawRect(bx, REQ_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H, TFT_DARKGREY);
        }
    }

    char buf_spec[6];
    snprintf(buf_spec, sizeof(buf_spec), "%d", (int)spec);   // "2000"
    smartClear(startX + (BOOST_NUM_BLOCKS * BOOST_STEP) + 5, REQ_BAR_Y, 35, 16);
    tft.setTextColor(TFT_SILVER);
    tft.setTextDatum(TR_DATUM);
    tft.drawRightString(buf_spec, METER_X + METER_W, REQ_BAR_Y, 2);
}

void updateBoostAct(int act) {
    int act_blocks = map(act, 0, 3000, 0, BOOST_NUM_BLOCKS);
    int startX = METER_X + 38;
    bool pulseState = (millis() % 160) < 80;

    for (int i = 0; i < BOOST_NUM_BLOCKS; i++) {
        int bx = startX + (int)(i * BOOST_STEP);
        uint16_t col = (i >= 12) ? TFT_LIGHTRED : ((i >= 7) ? TFT_YELLOW : TFT_GREEN);

        if (i < act_blocks) {
            tft.fillRect(bx, ACT_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H, (i == act_blocks - 1 && pulseState) ? TFT_WHITE : col);
        } else {
            smartClear(bx, ACT_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H);
            tft.drawRect(bx, ACT_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H, TFT_DARKGREY);
        }
    }

    char buf_act[6];
    snprintf(buf_act, sizeof(buf_act), "%d", (int)act);   // "2000"
    smartClear(startX + (BOOST_NUM_BLOCKS * BOOST_STEP) + 5, ACT_BAR_Y, 35, 16);
    tft.setTextColor(TFT_SILVER);
    tft.setTextDatum(TR_DATUM);
    tft.drawRightString(buf_act, METER_X + METER_W, ACT_BAR_Y, 2);
}

void updatePedal(int pedal) {
    if (pedal < 0) pedal = 0;
    if (pedal > 100) pedal = 100;

    int fill_w = map(pedal, 0, 100, 0, PEDAL_BAR_W - 2);
    uint16_t pedal_color = (pedal > 80) ? TFT_LIGHTRED : ((pedal > 40) ? TFT_YELLOW : TFT_GREEN);

    if (fill_w > 0) {
        tft.fillRect(PEDAL_BAR_X+1, PEDAL_BAR_Y + 1, fill_w, PEDAL_BAR_H - 2, pedal_color);
    }
    if (PEDAL_BAR_W - 2 - fill_w > 0) {
        smartClear(PEDAL_BAR_X + 1 + fill_w, PEDAL_BAR_Y + 1, PEDAL_BAR_W - 2 - fill_w, PEDAL_BAR_H - 2);
    }

    char buf[8]; 
    sprintf(buf, "%d%%", pedal);
    smartClear(PEDAL_BAR_X + PEDAL_BAR_W - 45, PEDAL_Y + 4, 45, 16);
    tft.setTextColor(pedal_color);
    tft.setTextDatum(TR_DATUM);
    tft.drawRightString(buf, PEDAL_BAR_X + PEDAL_BAR_W, PEDAL_Y + 4, 2);
}
    //tft.drawRect(METER_X + 5, PEDAL_BAR_Y, PEDAL_BAR_W, PEDAL_BAR_H, TFT_DARKGREY);


// =========================================================================
// SWEEP
// =========================================================================
void runGaugeSweep(uint8_t page) {
    const int steps = 12;
    
    if (page == 0) {
        for (int i = 0; i <= steps; i++) {
            float ratio = (float)i / steps;
            updateRpmBar((int)(ratio * MAX_RPM));
            updateSpeed((int)(ratio * 280));
            updateBoostReq(1000.0f + ratio * 2000.0f);   // 1000 → 3000 (mbar)
            updateBoostAct(1000.0f + ratio * 2000.0f);
            updatePedal((int)(ratio * 100));
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        for (int i = steps; i >= 0; i--) {
            float ratio = (float)i / steps;
            updateRpmBar((int)(ratio * MAX_RPM));
            updateSpeed((int)(ratio * 280));
            updateBoostReq(1000.0f + ratio * 2000.0f);
            updateBoostAct(1000.0f + ratio * 2000.0f);
            updatePedal((int)(ratio * 100));
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } 
    else if (page == 1) {
        for (int i = 0; i <= steps; i++) {
            float ratio = (float)i / steps;
            updateFuelTemp(tempItems[0].minV + ratio * (tempItems[0].maxV - tempItems[0].minV));
            updateAmbientTemp(tempItems[1].minV + ratio * (tempItems[1].maxV - tempItems[1].minV));
            updateCoolantTemp(tempItems[2].minV + ratio * (tempItems[2].maxV - tempItems[2].minV));
            updateIntakeAir(tempItems[3].minV + ratio * (tempItems[3].maxV - tempItems[3].minV));
            updateCoolantTempEngine(tempItems[4].minV + ratio * (tempItems[4].maxV - tempItems[4].minV));
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        for (int i = steps; i >= 0; i--) {
            float ratio = (float)i / steps;
            updateFuelTemp(tempItems[0].minV + ratio * (tempItems[0].maxV - tempItems[0].minV));
            updateAmbientTemp(tempItems[1].minV + ratio * (tempItems[1].maxV - tempItems[1].minV));
            updateCoolantTemp(tempItems[2].minV + ratio * (tempItems[2].maxV - tempItems[2].minV));
            updateIntakeAir(tempItems[3].minV + ratio * (tempItems[3].maxV - tempItems[3].minV));
            updateCoolantTempEngine(tempItems[4].minV + ratio * (tempItems[4].maxV - tempItems[4].minV));
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        for (int i = 0; i < 5; i++) {
        tempItems[i].val = 0.0f;
        }
        resetBarState();
        for (int i = 0; i < 5; i++) {
            drawTelemetryRowDynamic(i, tempItems[i]);
        }
    }
    else if (page == 2) {
        for (int i = 0; i <= steps; i++) {
            float ratio = (float)i / steps;
            updateOilTemp(engineItems[0].minV + ratio * (engineItems[0].maxV - engineItems[0].minV));
            updateOilLevel(engineItems[1].minV + ratio * (engineItems[1].maxV - engineItems[1].minV));
            updateFuelCons(engineItems[2].minV + ratio * (engineItems[2].maxV - engineItems[2].minV));
            updateEngineTorque(engineItems[3].minV + ratio * (engineItems[3].maxV - engineItems[3].minV));
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        for (int i = steps; i >= 0; i--) {
            float ratio = (float)i / steps;
            updateOilTemp(engineItems[0].minV + ratio * (engineItems[0].maxV - engineItems[0].minV));
            updateOilLevel(engineItems[1].minV + ratio * (engineItems[1].maxV - engineItems[1].minV));
            updateFuelCons(engineItems[2].minV + ratio * (engineItems[2].maxV - engineItems[2].minV));
            updateEngineTorque(engineItems[3].minV + ratio * (engineItems[3].maxV - engineItems[3].minV));
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        for (int i = 0; i < 4; i++) {
            engineItems[i].val = 0.0f;
        }
        resetBarState();
        for (int i = 0; i < 4; i++) {
            drawTelemetryRowDynamic(i, engineItems[i]);
        }
    }
    else if (page == 3) {
        // COOLING sweep
        for (int i = 0; i <= steps; i++) {
            float ratio = (float)i / steps;
            refrigerantPressure = ratio * 30.0f;
            coolantTempGlobal = -20.0f + ratio * 140.0f;    // -20 → 120
            coolantTempGlobalIN = -20.0f + ratio * 140.0f;
            fanDuty = ratio * 100.0f;
            drawCoolingDynamic();
            vTaskDelay(pdMS_TO_TICKS(4));
        }
        for (int i = steps; i >= 0; i--) {
            float ratio = (float)i / steps;
            refrigerantPressure = ratio * 30.0f;
            coolantTempGlobal = -20.0f + ratio * 140.0f;
            coolantTempGlobalIN = -20.0f + ratio * 140.0f;
            fanDuty = ratio * 100.0f;
            drawCoolingDynamic();
            vTaskDelay(pdMS_TO_TICKS(4));
        }
        // reset na 0 posle sweep-a
        refrigerantPressure = 0.0f;
        coolantTempGlobal = 0.0f;
        coolantTempGlobalIN = 0.0f;
        fanDuty = 0.0f;
        drawCoolingDynamic();
    }
    else if (page == 4) {
        // INJECTION sweep
        for (int i = 0; i <= steps; i++) {
            float ratio = (float)i / steps;
            for (int j = 0; j < 4; j++) {
                injectionDuration[j] = ratio * 5.0f;
            }
            drawInjectionPage();
            vTaskDelay(pdMS_TO_TICKS(4));
        }
        for (int i = steps; i >= 0; i--) {
            float ratio = (float)i / steps;
            for (int j = 0; j < 4; j++) {
                injectionDuration[j] = ratio * 5.0f;
            }
            drawInjectionPage();
            vTaskDelay(pdMS_TO_TICKS(4));
        }
        // reset na 0
        for (int j = 0; j < 4; j++) {
            injectionDuration[j] = 0.0f;
        }
        drawInjectionPage();
    }
    else if (page == 5) {
        // MAF & VOLTAGE sweep
        for (int i = 0; i <= steps; i++) {
            float ratio = (float)i / steps;
            mafValue = ratio * 50.0f;
            voltageValue = ratio * 16.0f;
            drawMafDynamic();
            vTaskDelay(pdMS_TO_TICKS(4));
        }
        for (int i = steps; i >= 0; i--) {
            float ratio = (float)i / steps;
            mafValue = ratio * 50.0f;
            voltageValue = ratio * 16.0f;
            drawMafDynamic();
            vTaskDelay(pdMS_TO_TICKS(4));
        }
        // reset
        mafValue = 0.0f;
        voltageValue = 0.0f;
        drawMafDynamic();
    }
    else if (page == 6) {
        // TIMER sweep - samo brzina
        for (int i = 0; i <= steps; i++) {
            float ratio = (float)i / steps;
            vehicleSpeed = ratio * 100.0f;
            drawTimerDynamic();
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        for (int i = steps; i >= 0; i--) {
            float ratio = (float)i / steps;
            vehicleSpeed = ratio * 100.0f;
            drawTimerDynamic();
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        // reset
        vehicleSpeed = 0.0f;
        drawTimerDynamic();
    }
}

// =========================================================================
// DISPLAY SWITCHER — sve dinamičke funkcije
// =========================================================================
void Display(uint8_t firstframe, uint8_t offset, float rez, uint8_t id=0) {


    switch (Trenutnastrana2) {
      case 0:
        if (firstframe == 1) updateRpmBar(rez);
        else if (firstframe == 7) { updateSpeed(rez); vehicleSpeed = rez; }
        else if (firstframe == 33) updatePedal(rez);
        else if (firstframe == 18 && offset == 1) updateBoostAct(rez);
        else if (firstframe == 18 && offset == 2) updateBoostReq(rez);
      break;
      case 1:
        if (offset == 0 && id == 0) updateFuelTemp(rez);
        else if (offset == 1 && id == 1) updateCoolantTemp(rez);
        else if (offset == 2 && id == 1) updateAmbientTemp(rez);
        else if (offset == 3 && id == 0) updateCoolantTempEngine(rez);
        else if (offset == 2 && id == 0) updateIntakeAir(rez);
      break;

      case 2:
        if (offset == 0 && id == 1) updateOilTemp(rez);
        else if (offset == 1 && id == 0) updateOilLevel(rez);
        else if (offset == 2 && id == 0) updateFuelCons(rez);
        else if (offset == 1 && id == 1) updateEngineTorque(rez);
      break;

      case 3:   // COOLING - samo dinamički deo
        drawCoolingDynamic();

        if (offset == 0 && id == 1) { refrigerantPressure = rez; drawCoolingDynamic(); }
        else if (offset == 1) { coolantTempGlobal = rez; drawCoolingDynamic(); }
        else if (offset == 2) { fanDuty = rez; drawCoolingDynamic(); }
        else if (offset == 0 && id == 0) { coolantTempGlobalIN = rez; drawCoolingDynamic(); }
      break;

      case 4:   // INJECTION - sprite
        drawInjectionPage();
        if (offset < 4) {
            injectionDuration[offset] = rez;
            if (brojac == 4) drawInjectionPage();
        }
      break;

      case 5:   // MAF & VOLTAGE - samo dinamički deo
      drawMafDynamic();
        if (offset == 0) { mafValue = rez; drawMafDynamic(); }
        else if (offset == 2) { voltageValue = rez; drawMafDynamic(); }
      break;

      case 6:   // 0-100 TIMER
        if (firstframe == 7) { vehicleSpeed = rez; update0to100(rez); }
      break;
    }
}

int i = 0;
int o = 0;
float Rezultat;

// =========================================================================
// GLAVNI TASK
// =========================================================================
void DisCyclic(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(100));
    while (1) {
          vTaskDelay(pdMS_TO_TICKS(10));
          uint8_t offset, b;
          updateAutoBacklight();
          if(changed == true){
            switch(brojac){
              case 0:
                drawDashboardLayout();
                runGaugeSweep(0);
                break;
              case 1:
                drawTelemetryPage();
                runGaugeSweep(1);
                break;
              case 2:
                showEngineStatusPage();
                runGaugeSweep(2);
                break;
              case 3:
                drawCoolingStatic();
                drawCoolingDynamic();
                runGaugeSweep(3);
                break;
              case 4:
                drawInjectionStatic();
                drawInjectionPage();
                runGaugeSweep(4);
                break;
              case 5:
                drawMafStatic();
                drawMafDynamic();
                runGaugeSweep(5);
                break;
              case 6:
                drawTimerStatic();
                drawTimerDynamic();
                runGaugeSweep(6);
                break;
            }              
            changed = false;
          }

          if (EngineDiag_GetChData(data, &Trenutnastrana2) == RETOK) {
            for (offset = 0; offset < 4u; offset++) {
                        vTaskSuspendAll();
                        for (b = 0; b < 3u; b++) {
                            block[b] = data[(offset * 3) + b];
                        }
                        xTaskResumeAll();
                        Rezultat = Dis_DecodeFrame(block);
                        
                        if (Trenutnastrana2 >= 1 && Trenutnastrana2 <= 6) {
                            if (Rezultat != 0)
                                Display(block[0], offset, Rezultat, TrenutniId);
                        }
                        else {
                            Display(block[0], offset, Rezultat);
                        }
            }
          }
    }
}

// =========================================================================
// INIT
// =========================================================================
void DisplayInit() {
    setupBacklight();
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    
    tft.begin();
    tft.initDMA();
    tft.setRotation(0);
    tft.setSwapBytes(false);

    tft.fillScreen(COLOR_BG);

    sprSpeed.createSprite(METER_W, SPEED_H);
    sprSpeed.setColorDepth(16);

    for (int i = 0; i < 4; i++) {
        sprInj[i].createSprite(INJ_METER_SIZE, INJ_METER_SIZE);
        sprInj[i].setColorDepth(16);
    }
    
    //gif.begin(GIF_PALETTE_RGB565_BE);
    //playStartupGIF();

    drawDashboardLayout();
    runGaugeSweep(0);
    
    xTaskCreatePinnedToCore(DisCyclic, "Dis", 4096u, NULL, 3, &disTaskHandle, 0);
}