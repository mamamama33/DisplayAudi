#define USER_SETUP_INFO "ST7796_ESP32"

#define ST7796_DRIVER

#define TFT_WIDTH  320
#define TFT_HEIGHT 480

// Standardni SPI pinovi za ESP32
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS    5
#define TFT_DC    2
#define TFT_RST   4
#define TFT_MISO 19

// Ako ne koristiš kontrolu osvetljenja preko GPIO-a, ostavi zakomentarisano:
//#define TFT_BL   32
//#define TFT_BACKLIGHT_ON HIGH

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF
#define SMOOTH_FONT

#define SPI_FREQUENCY       80000000
#define SPI_TOUCH_FREQUENCY  2500000
