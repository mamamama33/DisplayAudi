#include"Display.h"
#include"DataAnaliser.h"
#include "PhysicalCan.h"
#include "esp_log.h"

static TaskHandle_t disTaskHandle;

uint8_t buttons=0; //0 znaci ni jedno dugmo se nije kliknulo
uint8_t strana=0;

uint8_t data[12];
uint8_t block[3];
uint8_t Ispravnost; 

char BufferforDis[1024];
typedef struct
{
    uint8_t blocks[4];
    uint8_t id;

} Block;


Block Strane[][2] =
{
    // ================= PRVA STRANA =================
    {
        {
            .id = 6,
            .blocks = {
                0,5,2,5
                // 0 je brzina km/h a 2 je accel pedal pos
            }
        },

        {
            .id = 11,
            .blocks = {
                0,1,2,3
                // 0 je rpm, 1 je boost pressure actual,
                // 2 je specified, a duty cycle je 3
            }
        }
    },


    // ================= DRUGA STRANA =================
    {
        {
            .id = 7,
            .blocks = {
                0,5,2,3
                // 0 je fuel temp, 2 je intake air temp,
                // 3 je coolant temp - engine
            }
        },

        {
            .id = 62,
            .blocks = {
                5,1,2,5
                // 1 je coolant temp coolant,
                // 2 je ambient temp
            }
        }
    },


    // ================= TREĆA STRANA =================
    {
        {
            .id = 29,
            .blocks = {
                0,1,5,5
                // 0 je oil temp, 1 je oil level
            }
        },

        {
            .id = 15,
            .blocks = {
                5,1,2,5
                // 1 je engine torque,
                // 2 je fuel consumption
            }
        }
    },


    // ================= ČETVRTA STRANA =================
    {
        {
            .id = 63,
            .blocks = {
                0,5,2,5
                // 0 je refrigerant pressure,
                // 2 je cooling request koji vrv neću dobiti
            }
        },

        {
            .id = 64,
            .blocks = {
                0,1,2,5
                // 0 je coolant temp engine,
                // 1 je coolant temp cooler,
                // 2 je fan duty
            }
        }
    },


    // ================= PETA STRANA =================
    {
        {
            .id = 13,
            .blocks = {
                0,1,2,3
                // Koliko dizne bacaju je to sve
            }
        },

        {
            .id = 4,
            .blocks = {
                5,5,5,3
                // 3 je torsion value
            }
        }
    },


    // ================= ŠESTA STRANA =================
    {
        {
            .id = 10,
            .blocks = {
                0,5,5,5
                // MAF senzor
            }
        },

        {
            .id = 12,
            .blocks = {
                5,5,2,5
                // voltage
            }
        }
    }

};

/*UZimam stalno poslednji blok i uzimam 1 karakter koji ne treba da uzmem*/
void DisCyclic(void *pvParameters){
    while(1){
          vTaskDelay(pdMS_TO_TICKS(5));
          uint8_t offset,b;
          buttons=GetStalkButton();
          /*Logika za stalkgetter*/
                
          //ch oznacava koju stranu 
          if(buttons!=0x00){
            strana++;
            if(strana>=6)
              strana=0;

          }
          //Zato sto imam 2 sida u svakom clanu
          for(int i =1;i<3;i++){
            
            if(DataSetter(Strane[strana][i].id)){



              
            }
            if(EngineDiag_GetChData(Strane[strana][i].id,data)==RETOK){
              
              for (offset=0; offset < 4u; offset++)
              {
                      if(offset == Strane[strana][i].blocks[offset]){

                        vTaskSuspendAll(); // Critical section, interrupts enabled
                        for (b=0; b < 3u; b++)
                        {
                            // Copy 3 bytes of data from buffer with offset
                            block[b] = data[(offset*3)+b];

                        }
                        xTaskResumeAll(); // End of critical section, interrupts enabled

                        Dis_DecodeFrame(block);

                      }

              }
            }


          }

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
        
      case 23: //23  	EGR Valve, Duty Cycle / Inj. Timing ???  	   	b/256*a  	%
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


void DisplayInit(){

    xTaskCreatePinnedToCore(DisCyclic,"Dis",2048u,NULL,3,&disTaskHandle,0);
    vTaskDelay(pdMS_TO_TICKS(50));
}