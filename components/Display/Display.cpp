#include"Display.h"
#include"DataAnaliser.h"
#include "PhysicalCan.h"
#include "esp_log.h"


#include <SPI.h>
#include <TFT_eSPI.h>
#include "audi_gif.h" 

TFT_eSPI tft = TFT_eSPI(); 
TFT_eSprite sprSpeed = TFT_eSprite(&tft); 
AnimatedGIF gif;

// -------------------------------------------------------------------------


static TaskHandle_t disTaskHandle;

uint8_t data[12];
uint8_t block[3];
uint8_t Ispravnost; 
uint8_t Trenutnastrana2=0;

char BufferforDis[1024];

/*UZimam stalno poslednji blok i uzimam 1 karakter koji ne treba da uzmem*/
void DisCyclic(void *pvParameters){
    while(1){
          vTaskDelay(pdMS_TO_TICKS(10));
          uint8_t offset,b;
          //geter trenutne strane
          //Zato sto imam 2 sida u svakom clanu

            if(EngineDiag_GetChData(data,&Trenutnastrana2)==RETOK){
            


              for (offset=0; offset < 4u; offset++)
              {
                        vTaskSuspendAll(); // Critical section, interrupts enabled
                        for (b=0; b < 3u; b++)
                        {
                            // Copy 3 bytes of data from buffer with offset
                            block[b] = data[(offset*3)+b];

                        }
                        xTaskResumeAll(); // End of critical section, interrupts enabled

                        Dis_DecodeFrame(block);
                        //printf("%u \n",block[0]);


              }
              //Svaki put kad izracunam blok ja cu ga ispisati na ekran- JEDAN PODATAK
              Display(block,offset);

  
            }

                updateAutoBacklight();

                updateRpmBar(66);
                updateSpeed(33);
                updateBoost(31, 12);
                updatePedal(112);


    }

}
// U ovoj funkciji zelim da ispisujem promenljive vrednosti kao sto su rpm,boost,km/h,acc pedal...
uint8_t Display(uint8_t *DataForDisplay,uint8_t offset){

  switch(Trenutnastrana2){
    //Promena vrednosti rpm,boost,km/h i acc pedal
    case 0:
      if()



    break;

    case 1:



    break;

    case 2:

    break;


    case 3:


    break;


    case 4:


    break;

    case 5:



    break;

  }







}



uint8_t Dis_DecodeFrame(uint8_t *frameData)
{
  //char * p_saved = p;

  char * p;
  p=BufferforDis;
  uint8_t len;
  uint16_t val_u16;
  int16_t val_s16;
  float f;
  int  j;
  
    switch (frameData[0])
    {
    
      case 1: //0.2*a*b rpm
        val_u16 = frameData[1] * frameData[2];
        val_u16 /= 5;
        p += sprintf(p, "%d", val_u16);
        break;

      case 2: //a*0.002*b  	%
      case 3: //0.002*a*b  	Deg
        f = frameData[1] * frameData[2];
        f *= 0.002;
        p += sprintf(p, "%.1f", f);
        break;

      case 4: // abs(b-127)*0.01*a  	"ATDC" if Value >127, else"BTDC"
        val_u16 = abs(frameData[2] - 127);
        val_u16 *= frameData[1];
        val_u16 /= 100;
        p += sprintf(p, "%d", val_u16);
        break;
                
      case 5: // a*(b-100)*0.1 �C
        val_s16 = frameData[1] * (frameData[2] - 100);
        val_s16 /= 10;
        p += sprintf(p, "%- 3d C", val_s16);
        break;
      
      case 6:  //0.001*a*b  	V
      case 12: //0.001*a*b  	Ohm
      case 21: //0.001*a*b  	V
      case 22: //0.001*a*b  	ms
      case 24: //0.001*a*b  	A
        f = frameData[1] * frameData[2];
        f *= 0.001;
        p += sprintf(p, "%.2f", f);
        break;
      
      case 7: // 0.01*a*b km/h
        val_u16 = frameData[1] * frameData[2];
        val_u16 /= 100;
        p += sprintf(p, "%d", val_u16);
        break;
        
      case 8: //0.1*a*b  	(no units)
        f = frameData[1] * frameData[2];
        f *= 0.1;
        p += sprintf(p, "%.1f", f);
        break;

      case 9: //(b-127)*0.02*a  	Deg
        f = (frameData[2] - 127.0) * frameData[1];
        f *= 0.02;
        p += sprintf(p, "%.1f", f);
        break;
        
      case 10:
        if (0 == frameData[2])
        {
          p += sprintf(p, "COLD");
        }
        else
        {
          p += sprintf(p, "WARM");
        }
        break;
        
      case 11: //0.0001*a*(b-128)+1  -
        f = frameData[1] * (frameData[2] - 128.0);
        f *= 0.0001;
        f += 1;
        p += sprintf(p, "%.1f", f);
        break;

      case 13: //(b-127)*0.001*a  	mm
        f = (frameData[2] - 127.0) * frameData[1];
        f *= 0.001;
        p += sprintf(p, "%.1f", f);
        break;

      case 14: //0.005*a*b  	bar
        f = frameData[2] * frameData[1];
        f *= 0.005;
        p += sprintf(p, "%.1f", f);
        break;
        
      case 15: //0.01*a*b ms
      case 19: //a*b*0.01  	l
        f = frameData[1] * frameData[2];
        f *= 0.01;
        p += sprintf(p, "%.2f", f);
        break;
        
      case 16: //bitvalue
        for (j = 128; j > 0; j = j>>1)
        {
          if (frameData[1] & j)
          {
            *p++ = (frameData[2] & j)?'1':'0';
          }
          else
          {
            *p++ = 'X';
          }
        }
        break;
        
      case 17: //chr(a) + chr(b) 
        p += sprintf(p, "%c%c", frameData[1], frameData[2]);
        break;
        
      case 18:  //0.04*a*b  mbar
        val_u16 = frameData[1] * frameData[2];
        val_u16 /= 25;
        p += sprintf(p, "%d", val_u16);
        break;
        
      case 20:  //	a*(b-128)/128  	%
        f = frameData[1] * (frameData[2] - 128);
        f /= 128.0;
        p += sprintf(p, "%.2f", f);
        break;
        
      case 23: //23  	EGR Valve, Duty Cycle / Inj. Timing ???  	   	b/256*a  	% FAN DUTY cycle
        f = frameData[2] / 256.0;
        f *= frameData[1];
        p += sprintf(p, "%.1f", f);
        break;
        
      case 25: //  	(b*1.421)+(a/182)  	g/s
        f = (frameData[1] / 182.0) + (1.421 * frameData[2]);
        p += sprintf(p, "%.2f", f);
        break;

      case 26: //b-a  	C
      case 28:
        val_s16 = frameData[2] - frameData[1];
        p += sprintf(p, "%d", val_s16);
        break;
       
      case 27: //abs(b-128)*0.01*a 
        f = (frameData[2] - 128.0);
        if (f < 0)
        {
          f = f * (-1);
        }
        f *= frameData[1];
        f *= 0.01;
        p += sprintf(p, "%.1f", f);
        break;

      case 30: //b/12*a  	Deg k/w
        f = frameData[2] / 12.0;
        f *= frameData[1];
        p += sprintf(p, "%.1f", f);
        break;

      case 31: //b/2560*a  	�C
        f = frameData[2] / 2560.0;
        f *= frameData[1];
        p += sprintf(p, "%.1f", f);
        break;
        
      case 33: // 100*b/a  (if a==0 then 100*b)  	%
        if (frameData[1] == 0)
        {
          val_u16 = 100 * frameData[2];
        }
        else
        {
          val_u16 = (100 * frameData[2])/frameData[1];
        }
        p += sprintf(p, "%d", val_u16);
        break;
      
      case 34: // (b-128)*0.01*a  	kW
        f = (frameData[2] - 128.0) * frameData[1];
        f *= 0.01;
        p += sprintf(p, "%.1f", f); 
        break;
        
      case 35: // 0.01*a*b  	l/h
        f = frameData[2] * frameData[1];
        f *= 0.01;
        p += sprintf(p, "%.1f", f); 
        break;        

      case 37:
        switch (frameData[2])
        {
          case 0x00:
            p += sprintf(p, "-");
            break;

          case 0x02:
            p += sprintf(p, "ADP OK"); 
            break;
                      
          case 0x05:
            p += sprintf(p, "Idle"); 
            break;
            
          case 0x06:
            p += sprintf(p, "Partial thr"); 
            break;
            
          case 0x07:
            p += sprintf(p, "WOT"); 
            break;

          case 0x08:
            p += sprintf(p, "Enrichment"); 
            break;

          case 0x09:
            p += sprintf(p, "Deceleration"); 
            break;

          case 0x0E:
            p += sprintf(p, "A/C low"); 
            break;

          case 0x10:
            p += sprintf(p, "Compr. OFF"); 
            break;

          case 0xD6:
            p += sprintf(p, "Htg. S1 0xD6"); 
            break;

          case 0xD7:
            p += sprintf(p, "Htg. S1 0xD7"); 
            break;

          case 0xD9:
            p += sprintf(p, "Htg. S2 0xD9"); 
            break;

          case 0xEB:
            p += sprintf(p, "Test OFF"); 
            break;

          default:
            p += sprintf(p, "0x%02x", frameData[2]); 
        }
        break;

      case 38: //(b-128)*0.001*a  	Deg k/w
        f = (frameData[2] - 128.0);
        f *= frameData[1];
        f *= 0.001;
        p += sprintf(p, "%.1f", f); 
        break;
              
      case 39: //b/256*a  mg/h
        f = frameData[2] / 256.0;
        f *= frameData[1];
        p += sprintf(p, "%.1f", f); 
        break;

      case 43: // 	b*0.1+(25.5*a)  	V
        f = frameData[2] * 0.1;
        f += 25.5 * frameData[1];
        p += sprintf(p, "%.2f", f); 
        break;
              
      case 44: //a : b  	h:m
        p += sprintf(p, "%d:%d", frameData[1], frameData[2]);
        break;        

      case 45: //0.1*a*b/100  	 
        f = frameData[2] * frameData[1];
        f /= 1000.0;
        p += sprintf(p, "%.3f", f); 
        break;

      case 46: //(a*b-3200)*0.0027  	Deg k/w
        f = frameData[2] * frameData[1];
        f -= 3200.0;
        f *= 0.0027;
        p += sprintf(p, "%.1f", f); 
        break;

      case 47: //(b-128)*a  	ms
        val_s16 = (frameData[2] - 128) * frameData[1];
        p += sprintf(p, "%d", val_s16); 
        break;

      case 49: //(b/4)*a*0,1  mg/h
        f = (frameData[2] / 4.0);
        f *= frameData[1];
        f *= 0.1;
        p += sprintf(p, "%.1f", f); 
        break;
      
      case 50: // (b-128)/(0.01*a), if a==0 (b-128)/0.01  	mbar
        f = (frameData[2] - 128.0);
        f /= 0.01;
        if (frameData[1] != 0)
        {
          f /= frameData[1];
        }
        p += sprintf(p, "%.1f", f); 
        break;

      case 51: //((b-128)/255)*a  	mg/h
        f = frameData[2] - 128.0;
        f /= 255.0;
        f *= frameData[1];
        p += sprintf(p, "%.1f", f); 
        break;

      case 52: // b*0.02*a-a  	Nm
        val_u16 = frameData[1] * frameData[2];
        val_u16 /= 50;
        val_u16 -= frameData[1];
        p += sprintf(p, "%d", val_u16);
        break;

      case 53: // 53  	Luftdurchflu� Luftmassenmesser (???)  	   	(b-128)*1.4222+0.006*a  	g/s
        f = (frameData[2] - 128.0) * 1.4222;
        f += frameData[1] * 0.006;
        p += sprintf(p, "%.1f", f); 
        break;

      case 48: //b+a*255  	-      
      case 54: // 	a*256+b  	Count
        val_u16 = frameData[1] * 256;
        val_u16 += frameData[2];
        p += sprintf(p, "%d", val_u16);
        break;

      case 55: //a*b/200  	s
        f = frameData[1] * frameData[2];
        f /= 200.0;
        p += sprintf(p, "%.1f", f);
        break;

      case 56: //a*256+b  	WSC
        val_u16 = 256 * frameData[1] + frameData[2];
        p += sprintf(p, "%d", val_u16);
        break;

      case 59: //(a*256+b)/32768  	-
        f = 256.0 * frameData[1] + frameData[2];
        f = 32768.0;
        p += sprintf(p, "%.2f", f);
        break;

      case 60: //(a*256+b)*0.01  	sec
        f = 256.0 * frameData[1] + frameData[2];
        f *= 0.01;
        p += sprintf(p, "%.1f", f);
        break;

      case 61: // (b-128)/a, if a==0 (b-128)  -
        f = frameData[2] - 128.0;
        if (frameData[1] != 0)
        {
          f /= frameData[1];
        }
        p += sprintf(p, "%.2f", f); 
        break;

      case 62: //0.256*a*b  	S
        f = frameData[1] * frameData[2];
        f *= 0.256;
        p += sprintf(p, "%.1f", f);
        break;
        
      case 63: //chr(a) + chr(b) + "?"  -
        p += sprintf(p, "%c%c?", frameData[1], frameData[2]);
        break;

      case 64: //a+b  	Ohm
        val_u16 = frameData[1] + frameData[2];
        p += sprintf(p, "%d", val_u16);
        break;

      case 65: //0.01*a*(b-127)  	mm
        f = 0.01 * frameData[1];
        f *= frameData[2] - 127.0;
        p += sprintf(p, "%.2f", f);
        break;

      case 66: //(a*b)/511.12  	V
        f = frameData[1] * frameData[2];
        f /= 511.12;
        p += sprintf(p, "%.2f", f);
        break;

      case 67: //(640*a)+b*2.5  	Deg
        f = 640.0 * frameData[1];
        f += 2.5 * frameData[2];
        p += sprintf(p, "%.1f", f);
        break;

      case 68: //(256*a+b)/7.365  	deg/s
        f = 256.0 * frameData[1] + frameData[2];
        f /= 7.365;
        p += sprintf(p, "%.2f", f);
        break;

      case 69: //(256*a +b)*0.3254  	Bar
        f = 256.0 * frameData[1] + frameData[2];
        f *= 0.3254;
        p += sprintf(p, "%.2f", f);
        break;

      case 70: //(256*a +b)*0.192  	m/s^2
        f = 256.0 * frameData[1] + frameData[2];
        f *= 0.192;
        p += sprintf(p, "%.2f", f);
        break;

      default:
        p += sprintf(p, "---" );
        return 1;
        break;
    }
   
    return 0;
}


// =========================================================================
// KONTROLA OSVETLJENJA (PWM + LDR)
// =========================================================================
void setupBacklight() {
  pinMode(LDR_PIN, INPUT);
  ledcAttachChannel(BL_PIN, PWM_FREQ, PWM_RES, 0);
  ledcWrite(BL_PIN, MAX_BRIGHTNESS);
}

void updateAutoBacklight() {
  int ldrRaw = analogRead(LDR_PIN); 
  int targetBrightness = map(ldrRaw, 0, 4095, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
  targetBrightness = constrain(targetBrightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS);

  filteredBrightness += (targetBrightness - filteredBrightness) * 0.03;
  ledcWrite(BL_PIN, (int)filteredBrightness);
}

// =========================================================================
// GIF LOGIKA
// =========================================================================
void playStartupGIF(int repeatCount) {
  tft.fillScreen(TFT_BLACK);

  gif_xpos = (SCREEN_W - GIF_W) / 2; 
  gif_ypos = (SCREEN_H - GIF_H) / 2; 

  for (int loopCount = 0; loopCount < repeatCount; loopCount++) {
    if (gif.open((uint8_t *)audi_gif, sizeof(audi_gif), GIFDraw)) {
      int delayMs = 0;
      while (gif.playFrame(false, &delayMs)) {
        if (delayMs > 0) vTaskDelay(pdMS_TO_TICKS(delayMs));
        else vTaskDelay(pdMS_TO_TICKS(35)); 
        yield();
      }
      gif.close();
    }
  }
}

void GIFDraw(GIFDRAW *pDraw) {
  uint8_t *s = pDraw->pPixels;
  uint16_t *palette = pDraw->pPalette;
  int iWidth = pDraw->iWidth;

  int y = pDraw->iY + pDraw->y;
  int screenY = gif_ypos + y;

  if (screenY < 0 || screenY >= SCREEN_H) return;

  for (int x = 0; x < iWidth; x++) {
    uint8_t c = *s++;
    if (c == pDraw->ucTransparent || c == 0xFF) {
      lineBuffer[x] = TFT_BLACK; 
    } else {
      lineBuffer[x] = palette[c];
    }
  }

  int drawX = gif_xpos + pDraw->iX;
  int drawW = iWidth;
  int bufferOffset = 0;

  if (drawX < 0) {
    bufferOffset = -drawX;
    drawW += drawX;
    drawX = 0;
  }

  if (drawX + drawW > SCREEN_W) {
    drawW = SCREEN_W - drawX;
  }

  if (drawW > 0) {
    tft.pushImage(drawX, screenY, drawW, 1, &lineBuffer[bufferOffset]);
  }
}

// =========================================================================
// LAYOUT & RENDERING
// =========================================================================
uint16_t getSpeedBgColor() {
  if (currentSpeedBgMode == 2) return TFT_LIGHTRED;
  return TFT_BG_DARK; 
}

void drawDashboardLayout() {
  // MALI LOGO
  tft.setTextColor(TFT_SILVER, TFT_BG_DARK);
  tft.drawCentreString("A U D I   S P O R T", SCREEN_W / 2, 4, 1);

  // 1. RPM Zona
  tft.setTextColor(TFT_SILVER, TFT_BG_DARK);
  tft.drawCentreString("RPM", METER_X + METER_W / 2, RPM_Y, 2);

  for (int i = 0; i < NUM_BLOCKS; i++) {
    int bx = RPM_BAR_X + (int)(i * BLOCK_STEP);
    int block_h = map(i, 0, NUM_BLOCKS - 1, MIN_BLOCK_H, MAX_BLOCK_H);
    int by = RPM_BAR_Y_BOTTOM - block_h;
    tft.drawRoundRect(bx, by, BLOCK_W, block_h, 1, TFT_DARKGREY);
  }

  // Linija 1
  tft.drawFastHLine(10, SPEED_Y - 5, SCREEN_W - 20, TFT_GRID_LINE);

  // 2. SPEED Zona
  redrawSpeedBlock();

  // Linija 2
  tft.drawFastHLine(10, BOOST_Y - 5, SCREEN_W - 20, TFT_GRID_LINE);

  // 3. BOOST Zona
  tft.setTextColor(TFT_SILVER, TFT_BG_DARK);
  tft.drawCentreString("BOOST", METER_X + METER_W / 2, BOOST_Y + 2, 2);

  tft.drawString("REQ", METER_X + 5, REQ_BAR_Y + 2, 2);
  tft.drawString("ACT", METER_X + 5, ACT_BAR_Y + 2, 2);

  // Linija 3
  tft.drawFastHLine(10, PEDAL_Y - 5, SCREEN_W - 20, TFT_GRID_LINE);

  // 4. ACC PEDAL Zona
  tft.setTextColor(TFT_SILVER, TFT_BG_DARK);
  tft.drawString("ACCEL PEDAL", METER_X + 12, PEDAL_Y + 4, 2);

  tft.drawRect(PEDAL_BAR_X, PEDAL_BAR_Y, PEDAL_BAR_W, PEDAL_BAR_H, TFT_DARKGREY);
}

void redrawSpeedBlock() {
  updateSpeed(speed_val);
}

void updateSpeed(int spd) {
  if (spd < 0) spd = 0;
  if (spd > 299) spd = 299;

  char buf[8];
  itoa(spd, buf, 10);

  uint16_t bg = getSpeedBgColor();
  uint16_t txtColor;
  uint16_t spdColor;

  if (currentSpeedBgMode == 2) {
    txtColor = TFT_BLACK;
    spdColor = TFT_BLACK;
  } else if (currentSpeedBgMode == 3) {
    txtColor = TFT_LIGHTRED;
    spdColor = TFT_LIGHTRED;
  } else {
    txtColor = TFT_SILVER;
    spdColor = TFT_WHITE;
  }

  int sprW = METER_W;
  int sprH = SPEED_H;

  sprSpeed.fillSprite(bg);

  // PUNE VELIČINE BRZINA (Koristi pravi velikački Font 8)
  sprSpeed.setTextSize(1);                     
  sprSpeed.setTextColor(spdColor, bg);
  sprSpeed.setTextDatum(MC_DATUM);          
  sprSpeed.drawString(buf, sprW / 2, 42, 8); 

  // Oznaka "km/h"
  sprSpeed.setTextColor(txtColor, bg);
  sprSpeed.setTextDatum(BC_DATUM);
  sprSpeed.drawString("km/h", sprW / 2, sprH - 2, 2);

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
    
    uint16_t block_color = TFT_GREEN;
    if (i >= 11 && i < 20) block_color = TFT_YELLOW; 
    else if (i >= 20)      block_color = TFT_LIGHTRED;    

    if (i < active_blocks) {
      tft.fillRoundRect(bx, by, BLOCK_W, block_h, 1, block_color);
    } else {
      tft.fillRoundRect(bx, by, BLOCK_W, block_h, 1, TFT_BG_DARK);
      tft.drawRoundRect(bx, by, BLOCK_W, block_h, 1, TFT_DARKGREY);
    }
  }

  uint16_t text_color = TFT_WHITE;
  if (val >= SPORT_RPM_LIMIT && val < REDLINE_RPM_LIMIT) {
    text_color = TFT_YELLOW;
  } else if (val >= REDLINE_RPM_LIMIT) {
    text_color = TFT_LIGHTRED;
  }

  char buf[8];
  itoa(val, buf, 10);
  
  // SMANJEN RPM FONT (Font 4 - umerena i čitljiva veličina)
  tft.fillRect(METER_X + (METER_W / 2) - 45, RPM_Y + 16, 90, 26, TFT_BG_DARK);
  tft.setTextColor(text_color, TFT_BG_DARK);
  tft.setTextSize(1);
  tft.drawCentreString(buf, METER_X + METER_W / 2, RPM_Y + 16, 4); 
}

void updateBoost(float act, float spec) {
  if (act < BOOST_MIN) act = BOOST_MIN;
  if (act > BOOST_MAX) act = BOOST_MAX;
  if (spec < BOOST_MIN) spec = BOOST_MIN;
  if (spec > BOOST_MAX) spec = BOOST_MAX;

  int spec_blocks = map((int)(spec * 100), 100, 300, 0, BOOST_NUM_BLOCKS);
  int act_blocks  = map((int)(act * 100),  100, 300, 0, BOOST_NUM_BLOCKS);

  int startX = METER_X + 50;
  bool pulseState = ((esp_timer_get_time() / 1000ULL) % 160) < 80; // ZAJEDNIČKI TAJMER ZA PULSIRANJE

  // 1. REQ BAR (S PULSIRANJEM POSLEDNJEG BLOKA)
  for (int i = 0; i < BOOST_NUM_BLOCKS; i++) {
    int bx = startX + (int)(i * BOOST_STEP);
    uint16_t col = (i >= 12) ? TFT_LIGHTRED : ((i >= 7) ? TFT_YELLOW : TFT_GREEN);

    if (i < spec_blocks) {
      if (i == spec_blocks - 1 && spec > 1.15 && pulseState) {
        tft.fillRect(bx, REQ_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H, TFT_WHITE);
      } else {
        tft.fillRect(bx, REQ_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H, col);
      }
    } else {
      tft.fillRect(bx, REQ_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H, TFT_BG_DARK);
      tft.drawRect(bx, REQ_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H, TFT_DARKGREY);
    }
  }

  // 2. ACT BAR (S PULSIRANJEM POSLEDNJEG BLOKA)
  for (int i = 0; i < BOOST_NUM_BLOCKS; i++) {
    int bx = startX + (int)(i * BOOST_STEP);
    uint16_t col = (i >= 12) ? TFT_LIGHTRED : ((i >= 7) ? TFT_YELLOW : TFT_GREEN);

    if (i < act_blocks) {
      if (i == act_blocks - 1 && act > 1.15 && pulseState) {
        tft.fillRect(bx, ACT_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H, TFT_WHITE);
      } else {
        tft.fillRect(bx, ACT_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H, col);
      }
    } else {
      tft.fillRect(bx, ACT_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H, TFT_BG_DARK);
      tft.drawRect(bx, ACT_BAR_Y, BOOST_BLOCK_W, BOOST_BLOCK_H, TFT_DARKGREY);
    }
  }

  // NUMERIČKI ISPIS
  char buf_spec[6], buf_act[6];
  dtostrf(spec, 3, 1, buf_spec);
  dtostrf(act,  3, 1, buf_act);

  tft.fillRect(startX + (BOOST_NUM_BLOCKS * BOOST_STEP) + 5, REQ_BAR_Y, 35, 16, TFT_BG_DARK);
  tft.setTextColor(TFT_SILVER, TFT_BG_DARK);
  tft.drawRightString(buf_spec, METER_X + METER_W, REQ_BAR_Y, 2);

  tft.fillRect(startX + (BOOST_NUM_BLOCKS * BOOST_STEP) + 5, ACT_BAR_Y, 35, 16, TFT_BG_DARK);
  tft.setTextColor(TFT_WHITE, TFT_BG_DARK);
  tft.drawRightString(buf_act, METER_X + METER_W, ACT_BAR_Y, 2);
}

void updatePedal(int pedal) {
  if (pedal < 0) pedal = 0;
  if (pedal > 100) pedal = 100;

  int fill_w = map(pedal, 0, 100, 0, PEDAL_BAR_W - 2);

  uint16_t pedal_color = TFT_GREEN;
  if (pedal > 40 && pedal <= 80) pedal_color = TFT_YELLOW;
  else if (pedal > 80)          pedal_color = TFT_LIGHTRED;

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



void DisplayInit(){
  setupBacklight();
  
  tft.begin();
  tft.setRotation(0); 
  tft.fillScreen(TFT_BG_DARK);

  sprSpeed.createSprite(METER_W, SPEED_H);

  gif.begin(GIF_PALETTE_RGB565_BE);
  playStartupGIF(1); 

  tft.fillScreen(TFT_BG_DARK);
  drawDashboardLayout();
  xTaskCreatePinnedToCore(DisCyclic,"Dis",2048u,NULL,3,&disTaskHandle,0);
  vTaskDelay(pdMS_TO_TICKS(50));
}