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
#include "vag_codes.h"
extern "C" {
#include "DataAnaliser.h"
#include "PhysicalCan.h"
#include "KwpProtocol.h"
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
#define LDR_PIN           GPIO_NUM_14
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


// Boje za meni (ako već nemaš, koristi postojeće ACCENT_*)
#define MENU_BG          0x0821   // Tamna pozadina
#define MENU_TITLE       0x5D9F   // Plavi naslov
#define MENU_ITEM        0x7BEF   // Siva opcija
#define MENU_ITEM_SEL    0x07FF   // Cyan — selektovana opcija
#define MENU_ITEM_BOX    0x10A3   // Okvir opcije
#define MENU_ITEM_BOX_SEL 0x5D9F  // Okvir selektovane
#define MENU_FOOTER      0x4C9F   // Footer tekst

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
uint8_t TrenutniID=3;
uint16_t touch_val = 0;
uint16_t button_val = 0;

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



void drawMenuItem(int index, const char* text, bool selected) {
    int itemY = 140 + (index * 80);   // 3 opcije: y = 140, 220, 300
    int itemH = 60;
    int itemX = 20;
    int itemW = SCREEN_W - 40;        // 280 px
    
    int arrowY = itemY + itemH / 2;   // Y centar opcije (gde su trouglići)
    
    // =========================================================
    // 1. OBRIŠI REGION TROUGLIĆA (uvek, pre crtanja)
    // =========================================================
    // Levi trouglić — region od 0 do itemX (gde se crta levi trouglić)
    // Trouglić je širok 8 px (itemX - 12 do itemX - 4), visok 16 px
    // Da bi bili sigurni, brišemo malo veći region
    tft.fillRect(0, arrowY - 12, itemX, 24, MENU_BG);
    
    // Desni trouglić — region od itemX + itemW do SCREEN_W
    tft.fillRect(itemX + itemW, arrowY - 12, SCREEN_W - (itemX + itemW), 24, MENU_BG);
    
    // =========================================================
    // 2. NACRTAJ OKVIR OPCIJE
    // =========================================================
    uint16_t boxColor = selected ? MENU_ITEM_BOX_SEL : MENU_ITEM_BOX;
    tft.drawRoundRect(itemX, itemY, itemW, itemH, 8, boxColor);
    
    // Ako je selektovana — dodaj deblji okvir
    if (selected) {
        tft.drawRoundRect(itemX + 1, itemY + 1, itemW - 2, itemH - 2, 8, boxColor);
        tft.drawRoundRect(itemX + 2, itemY + 2, itemW - 4, itemH - 4, 8, boxColor);
    }
    
    // =========================================================
    // 3. NACRTAJ TEKST OPCIJE (centrirano)
    // =========================================================
    uint16_t textColor = selected ? MENU_ITEM_SEL : MENU_ITEM;
    tft.setTextColor(textColor);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(text, itemX + itemW / 2, itemY + itemH / 2, 4);
    
    // =========================================================
    // 4. AKO JE SELEKTOVANA — NACRTAJ TROUGLIĆE
    // =========================================================
    if (selected) {
        // Leva strelica ▶
        tft.fillTriangle(itemX - 12, arrowY,
                         itemX - 4, arrowY - 8,
                         itemX - 4, arrowY + 8,
                         MENU_ITEM_SEL);
        
        // Desna strelica ◀
        tft.fillTriangle(itemX + itemW + 12, arrowY,
                         itemX + itemW + 4, arrowY - 8,
                         itemX + itemW + 4, arrowY + 8,
                         MENU_ITEM_SEL);
    }
}
// =========================================================================
// Pomoćna — nacrtaj ceo meni (statički deo — jednom)
// =========================================================================
void drawMenuLayout() {
    // Pozadina
    tft.fillScreen(MENU_BG);
    
    // Naslov
    tft.setTextColor(MENU_TITLE);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("A U D I   S P O R T", SCREEN_W / 2, 30, 4);
    
    // Linija ispod naslova
    tft.drawFastHLine(20, 70, SCREEN_W - 40, MENU_ITEM_BOX);
    
    // Podnaslov
    tft.setTextColor(MENU_ITEM);
    tft.drawString("M A I N   M E N U", SCREEN_W / 2, 90, 2);
    
    // Footer — uputstvo
    tft.setTextColor(MENU_FOOTER);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("PRESS BUTTON TO SELECT", SCREEN_W / 2, SCREEN_H - 40, 2);
    
    // Verzija
    tft.setTextColor(MENU_ITEM_BOX);
    tft.drawString("v1.0", SCREEN_W / 2, SCREEN_H - 20, 1);
}

// =========================================================================
// GLAVNA MENI FUNKCIJA
// Blokira dok korisnik ne izabere EXIT
// Vraća: izabranu opciju (0 = Diagnostics, 1 = About, 2 = Exit)
// =========================================================================


void ReDrawMenu(){
    int selectedItem = 0;    // 0 = Diagnostics, 1 = About, 2 = Exit
    const int NUM_ITEMS = 3;
    
    const char* itemNames[3] = {
        "DIAGNOSTICS",
        "ABOUT CAR",
        "EXIT"
    };
    // Nacrtaj sve opcije (inicijalno — prva selektovana)
    for (int i = 0; i < NUM_ITEMS; i++) {
        drawMenuItem(i, itemNames[i], (i == selectedItem));
    }

}

const char* nadjiOpisGreske(uint16_t trazeniKod) {
    int low = 0;
    int high = VAG_DATABASE_SIZE - 1;

    while (low <= high) {
        int mid = low + (high - low) / 2;

        // Čitanje hexCode vrednosti direktno iz PROGMEM-a (Flash memorije)
        uint16_t trenutniKod = pgm_read_word(&(VAG_DIAG_DATABASE[mid].hexCode));

        if (trenutniKod == trazeniKod) {
            return VAG_DIAG_DATABASE[mid].description; // Pronađeno!
        }

        if (trenutniKod < trazeniKod) {
            low = mid + 1;
        } else {
            high = mid - 1;
        }
    }

    return "Greška nije pronađena u bazi.";
}

// Pomoćna funkcija ako ti sa CAN bus-a stiže HEX kao string (npr. "4C67" ili "4c67")
const char* nadjiOpisIzStringa(const char* hexString) {
    // Pretvara string "4C67" u uint16_t broj (0x4C67)
    uint16_t kod = (uint16_t) strtol(hexString, NULL, 16);
    return nadjiOpisGreske(kod);
}


uint16_t MenuUpDown;
uint16_t MenuOk;
void MainMenu(){
    // Stanje menija
    int selectedItem = 0;    // 0 = Diagnostics, 1 = About, 2 = Exit
    const int NUM_ITEMS = 3;
    
    const char* itemNames[3] = {
        "DIAGNOSTICS",
        "ABOUT CAR",
        "EXIT"
    };
    
    // Nacrtaj statički deo menija (jednom)
    drawMenuLayout();
    
    ReDrawMenu();
    // Petlja — čeka korisnika
    while (true) {
        // Čitaj dugme sa stalka
        //uint8_t buttons = GetStalkButton();

        touch_pad_read(TOUCH_PAD_NUM9, &MenuOk);
        touch_pad_read(TOUCH_PAD_NUM8, &MenuUpDown);
        /*
        if (MenuUpDown <200) {
            int oldSelected = selectedItem;
            
            // Pomeri selekciju gore (ciklično)
            selectedItem--;
            if (selectedItem < 0) selectedItem = NUM_ITEMS - 1;
            
            // Nacrtaj SAMO staru (kao ne-selektovanu) i novu (kao selektovanu)
            drawMenuItem(oldSelected, itemNames[oldSelected], false);
            drawMenuItem(selectedItem, itemNames[selectedItem], true);
            
            vTaskDelay(pdMS_TO_TICKS(200));   // debounce
        }*/
        
        if (MenuUpDown <200) {
            int oldSelected = selectedItem;
            
            // Pomeri selekciju dole (ciklično)
            selectedItem++;
            if (selectedItem >= NUM_ITEMS) selectedItem = 0;
            
            // Nacrtaj SAMO staru i novu
            drawMenuItem(oldSelected, itemNames[oldSelected], false);
            drawMenuItem(selectedItem, itemNames[selectedItem], true);
            
            vTaskDelay(pdMS_TO_TICKS(200));   // debounce
        }
        
        // =========================================================
        // POTVRDA (0x20 = tvoj kod za OK, prilagodi)
        // =========================================================
        else if (MenuOk<200) {
            vTaskDelay(pdMS_TO_TICKS(200));
            
            if (selectedItem == 0) {
                tft.fillScreen(MENU_BG);
                // Naslov
                tft.setTextColor(MENU_TITLE);
                tft.setTextDatum(TC_DATUM);
                tft.drawString("OTVORIO DIJAGNOSTIKU", SCREEN_W / 2, 30, 4);
                //Trazim zahtev za dijagnostiku 
                ChangeMode=true;
                //pustim da izvrti da pokupi dobru vrednost numofdtcbytes
                vTaskDelay(pdMS_TO_TICKS(2000));
                uint8_t highbyteDTC ;
                uint8_t lowbyteDTC ;
                uint8_t statusbyteDTC;
                uint8_t DTCData [numofDTCBytes];
                char linijaIspisa[128];
            
                GetDTC_Data(DTCData);

                   if(numofDTCBytes >1){

                        uint8_t numberofDTC=numofDTCBytes/3;
                        uint8_t DTC[numberofDTC][3];
                        char dtcBuffer[10];
                        int offset;
                        DTCData[0]=0x4C;
                        DTCData[1]=0X67;
                        DTCData[2]=0X00;
                        numberofDTC=1;
                        for(offset=0;offset<numberofDTC;offset++){
                            for(int k=0;k<3;k++){
                                DTC[offset][k]=DTCData[(offset*3)+k];
                            }
                            highbyteDTC=DTC[offset][0];
                            lowbyteDTC=DTC[offset][1];
                            statusbyteDTC=DTC[offset][2];                       
                            snprintf(dtcBuffer, sizeof(dtcBuffer),"%02X%02X", highbyteDTC, lowbyteDTC);
                            //tft.drawString("Broje gresaka je:" +(char)numberofDTC,45, 120, 4);
                            tft.setTextDatum(MC_DATUM);

                            tft.setTextColor(TFT_WHITE, TFT_BLACK); 

                            const char* opis = nadjiOpisIzStringa(dtcBuffer);

                            // Sastavljanje kompletnog teksta za prikaz
                            snprintf(linijaIspisa, sizeof(linijaIspisa), "Greska broj :%d pod oznakom %s je opisa :%s", offset + 1, dtcBuffer, opis);

                            // Ispis sastavljenog stringa na ekran
                            tft.drawString(linijaIspisa, 30, 80 + (50 * offset), 4);
                            tft.setTextDatum(TL_DATUM);
                        }
                   }
                   else{
                    tft.drawString("Nema gresaka u memorije engine modula",20,80 , 4);

                   }
                   while(1){
                    touch_pad_read(TOUCH_PAD_NUM9, &MenuOk);
                    if(MenuOk<200){
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(1000));
                   }

            }
            else if (selectedItem == 1) {
                tft.fillScreen(MENU_BG);
                // Naslov
                tft.setTextColor(MENU_TITLE);
                tft.setTextDatum(TC_DATUM);
                tft.drawString("O AUTU", SCREEN_W / 2, 170, 4);
                vTaskDelay(pdMS_TO_TICKS(2000));
                touch_pad_read(TOUCH_PAD_NUM9, &MenuOk);
                if(MenuOk<200){
                    break;
                }
            }
            else if (selectedItem == 2) {
                // EXIT — izađi iz menija
                // return 2;
                break;
            }
        }

        
        vTaskDelay(pdMS_TO_TICKS(20));   // mala pauza
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
    touch_pad_config(TOUCH_PAD_NUM9, 0);

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


int warninglogox = 10;
int warninglogoy = 100;

int OpenDoorlogox =0;
int OpenDoorlogoy = 140;

void updateAutoBacklight() {
    int ldrRaw = analogRead(LDR_PIN); 
    ldrRaw = constrain(ldrRaw, LDR_ADC_MIN, LDR_ADC_MAX);

    int targetBrightness = map(ldrRaw, LDR_ADC_MIN, LDR_ADC_MAX, MIN_BRIGHT, MAX_BRIGHT);
    filteredBrightness += (targetBrightness - filteredBrightness) * 0.05f;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)filteredBrightness);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    touch_pad_read(TOUCH_PAD_NUM8, &touch_val);
    touch_pad_read(TOUCH_PAD_NUM9, &button_val);

    if(button_val<200){

        MainMenu();
        changed=true;
    }
    if(touch_val < 200) {
        changed = true;
        brojac++;
        if(brojac >= 7) {
            brojac = 0;
        }
        SetStrana(brojac);
    }
    int warningLight=analogRead(WARNINGLIGHT); // nivo ulja zuta za ulje 
    int warningLight2=analogRead(WARNINGLIGHT2); //temperatura/pritisak ulja
    int warningLight3=analogRead(WARNINGLIGHT3); //temperatura rashladne tecnosti  
    int warningLight4=analogRead(WARNINGLIGHT4); //rezerva
    //obicno je vrednost  2300
    
    if(warningLight>3000){
        tft.fillScreen(COLOR_BG);
        tft.setSwapBytes(true);   // ← PRE
        //tft.pushImage(warninglogox, warninglogoy, LOWOIL2_WIDTH, LOWOIL2_HEIGHT, (uint16_t*)lowoil2, false);
        tft.pushImage(OpenDoorlogox, OpenDoorlogoy, OTVORENASUVOZACEVA2_WIDTH, OTVORENASUVOZACEVA2_HEIGHT, (uint16_t*)OtvorenaSuvozaceva2, false);
        tft.setSwapBytes(false);  // ← POSLE        
        vTaskDelay(pdMS_TO_TICKS(4000));
        changed=true;
    }
    /*
    if(warningLight2>3000){
        tft.fillScreen(COLOR_BG);
        tft.setSwapBytes(true);   // ← PRE
        tft.pushImage(warninglogox, warninglogoy, LOWOIL2_WIDTH, LOWOIL2_HEIGHT, (uint16_t*)pressureoil2, false);
        tft.setSwapBytes(false);  // ← POSLE        
        vTaskDelay(pdMS_TO_TICKS(4000));
        changed=true;
    }
    if(warningLight3>3000){
        tft.fillScreen(COLOR_BG);
        tft.setSwapBytes(true);   // ← PRE
        tft.pushImage(warninglogox, warninglogoy, LOWOIL2_WIDTH, LOWOIL2_HEIGHT, (uint16_t*)coolant2, false);
        tft.setSwapBytes(false);  // ← POSLE        
        vTaskDelay(pdMS_TO_TICKS(4000));
        changed=true;
    }
    if(warningLight4>3000){
        tft.fillScreen(COLOR_BG);
        tft.setSwapBytes(true);   // ← PRE
        tft.pushImage(warninglogox, warninglogoy, LOWOIL2_WIDTH, LOWOIL2_HEIGHT, (uint16_t*)fuel2, false);
        tft.setSwapBytes(false);  // ← POSLE        
        vTaskDelay(pdMS_TO_TICKS(4000));
        changed=true;

    }
    */

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
    
    // Ograničavamo vrednost unutar min/max opsega
    if (value < vmin) value = vmin;
    if (value > vmax) value = vmax;

    int center_x = x + r; 
    int center_y = y + r;
    int w = r / 3;
    int angle = 150;
    
    long vScaled = map((long)(value * 100), (long)(vmin * 100), (long)(vmax * 100), -angle, angle);
    int v = (int)vScaled;
    byte seg = 3;
    byte inc = 6;
    
    int colour = ACCENT_CYAN;
    
    // 1. ISCRTAVANJE I AŽURIRANJE SEGMENATA PRSTENA
    for (int i = -angle + inc/2; i < angle - inc/2; i += inc) {
        float sx = cos((i - 90) * 0.0174532925);
        float sy = sin((i - 90) * 0.0174532925);
        uint16_t x0 = sx * (r - w) + center_x;
        uint16_t y0 = sy * (r - w) + center_y;
        uint16_t x1 = sx * r + center_x;
        uint16_t y1 = sy * r + center_y;

        float sx2 = cos((i + seg - 90) * 0.0174532925);
        float sy2 = sin((i + seg - 90) * 0.0174532925);
        int x2 = sx2 * (r - w) + center_x;
        int y2 = sy2 * (r - w) + center_y;
        int x3 = sx2 * r + center_x;
        int y3 = sy2 * r + center_y;

        if (i < v) {
            // AKTIVNI SEGMENTI (ispunjeni svetlom bojom zavisno od scheme)
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
        } else {
            // NEAKTIVNI SEGMENTI (OVO VRAĆA DUGONAJAVNE/SIVE KVADRATE KADA VREDNOST PADNE)
            // Zadržava vidljivu strukturu prstena umesto da je obriše u crno
            uint16_t inactiveColor = TFT_DARKGREY; // Ako imate definisanu posebnu boju (npr. ACCENT_DARK_GRAY), ubacite je ovde
            tft.fillTriangle(x0, y0, x1, y1, x2, y2, inactiveColor);
            tft.fillTriangle(x1, y1, x2, y2, x3, y3, inactiveColor);
        }
    }
    
    // 2. PRIPREMA TEKSTA
    char buf[10];
    if (decimals == 0) snprintf(buf, sizeof(buf), "%d", (int)round(value));
    else snprintf(buf, sizeof(buf), "%.1f", value);

    // 3. BRISANJE SAMO MESTA GDE SE ISPISUJU BROJEVI I JEDINICE
    int cleanWidth = (r * 8) / 10;          
    int cleanHeight = (r > 60) ? 42 : 26;   

    smartClear(center_x - (cleanWidth / 2), center_y - (cleanHeight / 2), cleanWidth, cleanHeight);

    // 4. ISPIS NOVIH VREDNOSTI PREKO VRAĆENE POZADINE
    tft.setTextDatum(MC_DATUM);
    if (r > 60) {
        tft.setTextColor(ACCENT_CYAN);
        tft.drawString(buf, center_x, center_y - 5, 4);
        tft.setTextColor(ACCENT_SKY);
        tft.drawString(units, center_x, center_y + 18, 2);
    } else {
        tft.setTextColor(ACCENT_CYAN);
        tft.drawString(buf, center_x, center_y - 2, 2);
        tft.setTextColor(ACCENT_SKY);
        tft.drawString(units, center_x, center_y + 10, 1);
    }
    
    return center_x + r;
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
    int r = 55;
    int startX = 30;
    int yTop = 55;
    int r2=70;
    int yCoolant = yTop + r2*4;

void drawCoolingStatic() {
    carbon_active = true;
    redrawCarbonRegion(0, 0, SCREEN_W, SCREEN_H);
    
    tft.setTextColor(ACCENT_BLUE);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("C O O L I N G", SCREEN_W / 2, 8, 2);
    
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

// Statike za praćenje promena
static float last_refrigerantPressure = -999.0f;
static float last_coolantTempGlobal = -999.0f;
static float last_coolantTempGlobalIN = -999.0f;
static int   last_fanDuty = -1;

void UpdateCoolingRefrig(){
  
    carbon_active = true;
    // NEMA VIŠE smartClear() OVDE!
    ringMeter(refrigerantPressure, 0, 30, startX, yTop, r, "bar", 3, 1);
}

void UpdateCoolingCoolantOut(){
    carbon_active = true;
    // NEMA VIŠE smartClear() OVDE!
    ringMeter(coolantTempGlobal, -20, 120, startX + r*2 + 30, yTop, r, "C", 3, 0);
}

void UpdateCoolingCoolantIn(){

    carbon_active = true;
    // NEMA VIŠE smartClear() OVDE!
    ringMeter(coolantTempGlobalIN, -20, 120, (SCREEN_W - r2*2)/2, yCoolant-r2*2+30, r2, "C", 3, 0);
}

void UpdateFanDuty(){
    int currentFan = (int)fanDuty;
    if (currentFan == last_fanDuty) return; // Nema promene, preskoči
    last_fanDuty = currentFan;

    carbon_active = true;
    int barY = 445;
    int fanFill = map(currentFan, 0, 100, 0, BAR_W - 2);
    
    // Za progres bar brišemo samo unutrašnjost bara ili pravimo inkrementalni fill
    tft.drawRect(PADDING_X, barY, BAR_W, BAR_H, ACCENT_DARK);
    
    uint16_t fanColor = (currentFan > 80) ? ACCENT_RED : ((currentFan > 40) ? ACCENT_AMBER : ACCENT_LIME);
    
    // Unutrašnjost popunjavamo u dve operacije (aktivno + neaktivno)
    if (fanFill > 0) {
        tft.fillRect(PADDING_X + 1, barY + 1, fanFill, BAR_H - 2, fanColor);
    }
    if (BAR_W - 2 - fanFill > 0) {
        tft.fillRect(PADDING_X + 1 + fanFill, barY + 1, BAR_W - 2 - fanFill, BAR_H - 2, COLOR_BG);
    }
    
    // FAN DUTY tekst osvežavanje sa Paddingom
    char buf[10];
    snprintf(buf, sizeof(buf), "%d%%", currentFan);
    tft.setTextColor(ACCENT_CYAN, COLOR_BG);
    tft.setTextDatum(TR_DATUM);
    tft.setTextPadding(60); // Briše stare cifre samostalno u pozadinskoj boji
    tft.drawString(buf, SCREEN_W - PADDING_X, barY - 25, 2);
    tft.setTextPadding(0);
}

// =========================================================================
// STRANICA 4: INJECTION
// =========================================================================
static int injOldValue[4] = {-999, -999, -999, -999};
TFT_eSprite sprInjBg = TFT_eSprite(&tft);       // 130×130 pozadina
TFT_eSprite sprInjSmall[4] = {                   // 50×50 foreground
    TFT_eSprite(&tft), TFT_eSprite(&tft),
    TFT_eSprite(&tft), TFT_eSprite(&tft)
};
// Globalno — dodaj gde su ostale globalne promenljive
bool injResetLabels = false;

void drawInjectionMeter(int idx, float value, float vmin, float vmax, 
                        int screenX, int screenY, 
                        const char *label, const char *units) {
    
    int r = INJ_METER_R;
    int size = INJ_METER_SIZE;
    int cx = size / 2;
    int cy = size / 2;
    
    static bool initialized[4] = {false, false, false, false};
    static int lastNx[4] = {-1, -1, -1, -1};
    static int lastNy[4] = {-1, -1, -1, -1};
    static float lastVal[4] = {0, 0, 0, 0};
    
    // ============================================================
    // RESET kad se uđe na stranicu (postavi injResetLabels = true u DisCyclic)
    // ============================================================
    if (injResetLabels) {
        for (int i = 0; i < 4; i++) {
            initialized[i] = false;
            lastNx[i] = -1;
            lastNy[i] = -1;
            lastVal[i] = 0;
        }
        injResetLabels = false;
    }
    
    // ============================================================
    // 1. PRVI PUT — nacrtaj carbon pozadinu + luk + oznake + label
    // ============================================================
    if (!initialized[idx]) {
        // Carbon pozadina
        redrawCarbonRegion(screenX, screenY, size, size);
        
        // Spoljni luk
        int angle = 150;
        for (int i = -angle; i <= angle; i += 6) {
            float rad = (i - 90) * 0.0174532925;
            float sx = cos(rad);
            float sy = sin(rad);
            int x0 = (int)(sx * r + cx + 0.5f);
            int y0 = (int)(sy * r + cy + 0.5f);
            int x1 = (int)(sx * (r - 4) + cx + 0.5f);
            int y1 = (int)(sy * (r - 4) + cy + 0.5f);
            tft.drawLine(screenX + x0, screenY + y0, screenX + x1, screenY + y1, ACCENT_DARK);
        }
        
        // Glavne oznake
        for (int i = 0; i <= 6; i++) {
            int a = -angle + (2 * angle * i / 6);
            float rad = (a - 90) * 0.0174532925;
            float sx = cos(rad);
            float sy = sin(rad);
            int x0 = (int)(sx * r + cx + 0.5f);
            int y0 = (int)(sy * r + cy + 0.5f);
            int x1 = (int)(sx * (r - 12) + cx + 0.5f);
            int y1 = (int)(sy * (r - 12) + cy + 0.5f);
            tft.drawLine(screenX + x0, screenY + y0, screenX + x1, screenY + y1, ACCENT_BLUE);
        }
        
        // Label iznad metera (nacrtaj SAMO u init)
        // Obriši region iznad (vrati carbon) pa nacrtaj label
        redrawCarbonRegion(screenX, screenY - 20, size, 20);
        tft.setTextColor(ACCENT_BLUE);
        tft.setTextDatum(TC_DATUM);
        tft.drawString(label, screenX + size/2, screenY - 15, 4);
        
        initialized[idx] = true;
        lastNx[idx] = -1;
        lastNy[idx] = -1;
        lastVal[idx] = 0;
    }
    
    // ============================================================
    // 2. OGRANIČI VREDNOST
    // ============================================================
    if (value < vmin) value = vmin;
    if (value > vmax) value = vmax;
    
    // ============================================================
    // 3. IZRAČUNAJ NOVU POZICIJU IGLE
    // ============================================================
    float pct = (value - vmin) / (vmax - vmin);
    int angle = 150;
    int needleAngle = -angle + (2 * angle * pct);
    float needleRad = (needleAngle - 90) * 0.0174532925;
    
    int needleLen = r - 18;
    int nx = (int)(cos(needleRad) * needleLen + cx + 0.5f);
    int ny = (int)(sin(needleRad) * needleLen + cy + 0.5f);
    
    // ============================================================
    // 4. AKO SE NIJE PROMENILO — IZAĐI
    // ============================================================
    if (lastNx[idx] == nx && lastNy[idx] == ny && lastVal[idx] == value) {
        return;
    }
    lastVal[idx] = value;
    
    // ============================================================
    // 5. OBRISI STARU IGLU — 3 linije u COLOR_BG
    // ============================================================
    if (lastNx[idx] >= 0 && lastNy[idx] >= 0) {
        tft.drawLine(screenX + cx - 1, screenY + cy, 
                     screenX + lastNx[idx] - 1, screenY + lastNy[idx], COLOR_BG);
        tft.drawLine(screenX + cx, screenY + cy, 
                     screenX + lastNx[idx], screenY + lastNy[idx], COLOR_BG);
        tft.drawLine(screenX + cx + 1, screenY + cy, 
                     screenX + lastNx[idx] + 1, screenY + lastNy[idx], COLOR_BG);
        
        tft.fillCircle(screenX + cx, screenY + cy, 5, COLOR_BG);
    }
    
    // ============================================================
    // 6. OBRISI STARU VREDNOST — C ARBON pozadina (ne COLOR_BG!)
    // ============================================================
    // Umesto fillRect(COLOR_BG), vrati carbon za region teksta
    redrawCarbonRegion(screenX + cx - 28, screenY + cy + 10, 56, 55);    
    // ============================================================
    // 7. NACRTAJ NOVU IGLU — 3 debele linije
    // ============================================================
    tft.drawLine(screenX + cx - 1, screenY + cy, 
                 screenX + nx - 1, screenY + ny, ACCENT_RED);
    tft.drawLine(screenX + cx, screenY + cy, 
                 screenX + nx, screenY + ny, ACCENT_RED);
    tft.drawLine(screenX + cx + 1, screenY + cy, 
                 screenX + nx + 1, screenY + ny, ACCENT_RED);
    
    // Centar igle
    tft.fillCircle(screenX + cx, screenY + cy, 5, ACCENT_RED);
    tft.fillCircle(screenX + cx, screenY + cy, 3, ACCENT_DARK);
    
    // ============================================================
    // 8. NACRTAJ NOVU VREDNOST (preko carbon-a)
    // ============================================================
    char buf[10];
    snprintf(buf, sizeof(buf), "%.2f", value);
    tft.setTextColor(ACCENT_CYAN);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(buf, screenX + cx, screenY + cy + 28, 4);
    
    tft.setTextColor(ACCENT_SKY);
    tft.drawString(units, screenX + cx, screenY + cy + 48, 2);
    
    // ============================================================
    // 9. ZAPAMTI POZICIJU
    // ============================================================
    lastNx[idx] = nx;
    lastNy[idx] = ny;
}
void UpdateInjectionMeter() {
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
void drawInjectionPage() {
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
    
    //smartClear(centerX - r, 35, r*2, r*2);
    ringMeter(mafValue, 0, 1200, centerX - r, 35, r, "mg/stroke", 3, 2);
    
    //smartClear(centerX - r, 235, r*2, r*2);
    ringMeter(voltageValue, 0, 16, centerX - r, 235, r, "V", 3, 2);
}

// =========================================================================
// STRANICA 6: 0-100 TIMER
// =========================================================================
bool ChangedSpeed=true;

void update0to100(float speed_kmh) {
    uint32_t now = millis();
    
    switch (runState) {
        case RUN_WAIT_STATIONARY:
            if (speed_kmh < 1.0f) {
                if (runStillSince == 0) runStillSince = now;
                if (now - runStillSince > 2000) {
                    runState = RUN_ARMED;
                    ChangedSpeed=true;
                }
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
                ChangedSpeed=true;

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
                    ChangedSpeed=true;
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
                ChangedSpeed=true;
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

    if(ChangedSpeed==true){

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
    ChangedSpeed=false;
    
    }

  // Brzina
    char buf[20];
    snprintf(buf, sizeof(buf), "%.0f km/h", vehicleSpeed);
    smartClear(0, 90, SCREEN_W, 30);
    tft.setTextColor(ACCENT_CYAN);
    tft.drawString(buf, SCREEN_W / 2-35, 95, 4);
    
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
            vTaskDelay(pdMS_TO_TICKS(6));
        }
        for (int i = steps; i >= 0; i--) {
            float ratio = (float)i / steps;
            updateRpmBar((int)(ratio * MAX_RPM));
            updateSpeed((int)(ratio * 280));
            updateBoostReq(1000.0f + ratio * 2000.0f);
            updateBoostAct(1000.0f + ratio * 2000.0f);
            updatePedal((int)(ratio * 100));
            vTaskDelay(pdMS_TO_TICKS(6));
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
        updateFuelTemp(0);
        updateAmbientTemp(0);
        updateCoolantTemp(0);
        updateIntakeAir(0);
        updateCoolantTempEngine(0);
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

        updateOilTemp(0);
        updateOilLevel(0);
        updateFuelCons(0);
        updateEngineTorque(0);
    
    }
    else if (page == 3) {
        // COOLING sweep
        for (int i = 0; i <= steps; i++) {
            float ratio = (float)i / steps;
            refrigerantPressure = ratio * 30.0f;
            coolantTempGlobal = -20.0f + ratio * 140.0f;    // -20 → 120
            coolantTempGlobalIN = -20.0f + ratio * 140.0f;
            fanDuty = ratio * 100.0f;
            UpdateCoolingCoolantIn();
            UpdateCoolingCoolantOut();
            UpdateFanDuty();
            UpdateCoolingRefrig();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        for (int i = steps; i >= 0; i--) {
            float ratio = (float)i / steps;
            refrigerantPressure = ratio * 30.0f;
            coolantTempGlobal = -20.0f + ratio * 140.0f;
            coolantTempGlobalIN = -20.0f + ratio * 140.0f;
            fanDuty = ratio * 100.0f;
            UpdateCoolingCoolantIn();
            UpdateCoolingCoolantOut();
            UpdateFanDuty();
            UpdateCoolingRefrig();            
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        // reset na 0 posle sweep-a
        refrigerantPressure = 0.0f;
        coolantTempGlobal = 0.0f;
        coolantTempGlobalIN = 0.0f;
        fanDuty = 0.0f;
        UpdateCoolingCoolantIn();
        UpdateCoolingCoolantOut();
        UpdateFanDuty();
        UpdateCoolingRefrig();  
    }
    else if (page == 4) {
        // INJECTION sweep
        for (int i = 0; i <= steps; i++) {
            float ratio = (float)i / steps;
            for (int j = 0; j < 4; j++) {
                injectionDuration[j] = ratio * 5.0f;
            }
            UpdateInjectionMeter();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        for (int i = steps; i >= 0; i--) {
            float ratio = (float)i / steps;
            for (int j = 0; j < 4; j++) {
                injectionDuration[j] = ratio * 5.0f;
            }
            UpdateInjectionMeter();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        // reset na 0
        for (int j = 0; j < 4; j++) {
            injectionDuration[j] = 0.0f;
        }
        UpdateInjectionMeter();
    }
    else if (page == 5) {
        // MAF & VOLTAGE sweep
        for (int i = 0; i <= steps; i++) {
            float ratio = (float)i / steps;
            mafValue = ratio * 50.0f;
            voltageValue = ratio * 16.0f;
            drawMafDynamic();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        for (int i = steps; i >= 0; i--) {
            float ratio = (float)i / steps;
            mafValue = ratio * 50.0f;
            voltageValue = ratio * 16.0f;
            drawMafDynamic();
            vTaskDelay(pdMS_TO_TICKS(10));
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
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        for (int i = steps; i >= 0; i--) {
            float ratio = (float)i / steps;
            vehicleSpeed = ratio * 100.0f;
            drawTimerDynamic();
            vTaskDelay(pdMS_TO_TICKS(10));
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
        //printf("id je %d",id);
        if (offset == 0 && id == 0) updateFuelTemp(rez);
        else if (offset == 1 && id == 1) updateCoolantTemp(rez);
        else if (offset == 2 && id == 1) updateAmbientTemp(rez);
        else if (offset == 3 && id == 0) updateCoolantTempEngine(rez);
        else if (offset == 2 && id == 0) updateIntakeAir(rez);
      break;
        //Moram sam izracunati oil level case bio 102 njega nema 
        //Mogu dodati torque request
      case 2:
        if (offset == 0 && id == 0) updateOilTemp(rez);
        else if (offset == 1 && id == 0) updateOilLevel(rez);
        else if (offset == 2) updateFuelCons(rez);
        else if (offset == 1 && id == 1) updateEngineTorque(rez);
      break;

      case 3:   // COOLING - samo dinamički de
        if (offset == 0 && id == 0) { refrigerantPressure = rez; UpdateCoolingRefrig(); }
        else if (offset == 0 && id==1) { coolantTempGlobal = rez; UpdateCoolingCoolantOut(); }
        else if (offset == 2) { fanDuty = rez; UpdateFanDuty(); }
        else if (offset == 1 && id == 1) { coolantTempGlobalIN = rez; UpdateCoolingCoolantIn(); }
      break;
        //Dodao bih torsionu vrednost jer mogu u tim blokovima mi je 
      case 4:   // INJECTION - sprite
        if (offset==0)
            injectionDuration[0] = rez;
        else if (offset==1)
            injectionDuration[1] = rez;
        else if (offset==2)
            injectionDuration[2] = rez;
        else if (offset==3)
            injectionDuration[3] = rez;
        UpdateInjectionMeter();
        
      break;

      case 5:   // MAF & VOLTAGE - samo dinamički deo
      drawMafDynamic();
        if (offset == 0) { mafValue = rez; drawMafDynamic(); }
        else if (offset == 2) { voltageValue = rez; drawMafDynamic(); }
      break;

      case 6:   // 0-100 TIMER
        if (firstframe == 7) {update0to100(rez); }
      break;
    }
}

int i = 0;
int o = 0;
float Rezultat;
void drawAudiLogo(int cx, int cy, int r, uint16_t color) {
    int offset = r * 1.3;
    tft.drawCircle(cx - offset * 3 / 2, cy, r, color);
    tft.drawCircle(cx - offset / 2, cy, r, color);
    tft.drawCircle(cx + offset / 2, cy, r, color);
    tft.drawCircle(cx + offset * 3 / 2, cy, r, color);
}
// =========================================================================
// FADE OUT - zatamni ekran + prikaži Audi logo
// =========================================================================
int GetBrightness(){

    int ldrRaw = analogRead(LDR_PIN); 
    ldrRaw = constrain(ldrRaw, LDR_ADC_MIN, LDR_ADC_MAX);
    int targetBrightness = map(ldrRaw, LDR_ADC_MIN, LDR_ADC_MAX, MIN_BRIGHT, MAX_BRIGHT);
    return targetBrightness;
}


void fadeOut() {
    // 1. Zatamni ekran postepeno
    int MAXbrightness=GetBrightness();
    for (int i = 0; i <= 75; i++) {
        int brightness = MAXbrightness - (i * MAXbrightness / 75);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    // 2. Sad je ekran potpuno crn - očisti TFT i nacrtaj Audi logo
    tft.fillScreen(COLOR_BG);
    

   // Audi logo kao slika
    int logoX = (SCREEN_W - AUDI_LOGO_WIDTH) / 2;
    int logoY = (SCREEN_H - AUDI_LOGO_HEIGHT) / 2 - 30;

    tft.setSwapBytes(true);   // za BE format (isti kao carbon_bg)
    tft.pushImage(logoX, logoY, AUDI_LOGO_WIDTH, AUDI_LOGO_HEIGHT,(uint16_t*)audi_logo, false);   // ← DODAJ fals    tft.setSwapBytes(false);
    // 3. Posvetli da se vidi logo
    for (int i = 0; i <= 75; i++) {
        int brightness = (i * MAXbrightness / 75);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    // 4. Kratka pauza da se vidi logo
    vTaskDelay(pdMS_TO_TICKS(640));
    
    // 5. Ponovo zatamni
    for (int i = 0; i <= 75; i++) {
        int brightness = MAXbrightness - (i * MAXbrightness / 75);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    // 6. Sad je ekran crn i backlight je ugašen — spremno za crtanje nove stranice
}

// =========================================================================
// FADE IN - posvetli ekran (posle crtanja nove stranice)
// =========================================================================
void fadeIn() {
    int MAXbrightness=GetBrightness();
    for (int i = 0; i <= 75; i++) {
        int brightness = (i * MAXbrightness / 75);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

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
            fadeOut();
            switch(brojac){
              case 0:
                drawDashboardLayout();
                fadeIn();              
                runGaugeSweep(0);
                break;
              case 1:
                drawTelemetryPage();
                fadeIn();              
                runGaugeSweep(1);
                break;
              case 2:
                showEngineStatusPage();
                fadeIn();              
                runGaugeSweep(2);
                break;
              case 3:
                drawCoolingStatic();
                fadeIn();              
                runGaugeSweep(3);
                break;
              case 4:
                drawInjectionPage();
                fadeIn();              
                injResetLabels = true;     // ← DODAJ OVO
                runGaugeSweep(4);
                break;
              case 5:
                drawMafStatic();
                fadeIn();              
                //drawMafDynamic();
                runGaugeSweep(5);
                break;
              case 6:
                drawTimerStatic();
                fadeIn();              
                //drawTimerDynamic();
                runGaugeSweep(6);
                break;
            }
            changed = false;
          }
//IZMENE
          if (EngineDiag_GetChData(data, &Trenutnastrana2,&TrenutniID) == RETOK) {
            for (offset = 0; offset < 4u; offset++) {
                        vTaskSuspendAll();
                        for (b = 0; b < 3u; b++) {
                            block[b] = data[(offset * 3) + b];
                        }
                        xTaskResumeAll();
                        Rezultat = Dis_DecodeFrame(block);
                        
                        if (Trenutnastrana2 >= 1 && Trenutnastrana2 <= 6) {
                            if (Rezultat != 0)
                                Display(block[0], offset, Rezultat, TrenutniID);
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
    
   // gif.begin(GIF_PALETTE_RGB565_BE);
    //playStartupGIF();

    drawDashboardLayout();
    runGaugeSweep(0);
    
    xTaskCreatePinnedToCore(DisCyclic, "Dis", 4096u, NULL, 3, &disTaskHandle, 0);
}