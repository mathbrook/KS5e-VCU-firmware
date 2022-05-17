#include <WS2812Serial.h>
const int numled=3;
byte drawingMemory[numled*3];         //  3 bytes per LED
DMAMEM byte displayMemory[numled*12]; // 12 bytes per LED
WS2812Serial leds(3, displayMemory, drawingMemory, 17,WS2812_GRB);
int colorMax=6;
#define RED    0xFF0000
#define GREEN  0x00FF00
#define BLUE   0x0000FF
#define YELLOW 0xFFFF00
#define PINK   0x10EF0078
#define ORANGE 0xE05800
#define WHITE  0xFF000000
#define BLACK  0x0
#define BRIGHTNESS 255
uint32_t colorList[]={RED,ORANGE,YELLOW,GREEN,BLUE,PINK,WHITE,BLACK};
void setPixels(uint8_t color[3]){
    for(int i=0;i<numled;i++){
    leds.setPixel(i, colorList[color[i]]);
    leds.show();
    }
};
uint8_t enableLights[]={0,7,7};
uint8_t waitingRtdLights[]={2,7,7};
uint8_t rtdLights[]={3,7,7};