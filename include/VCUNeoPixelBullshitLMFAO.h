#include <WS2812Serial.h>
#include "parameters.hpp"
const int numled=3+6;
const int numledOnDash=6;
byte drawingMemory[numled*3];         //  3 bytes per LED
DMAMEM byte displayMemory[numled*12]; // 12 bytes per LED
byte DashdrawingMemory[numledOnDash*3];         //  3 bytes per LED
DMAMEM byte DashdisplayMemory[numledOnDash*12]; // 12 bytes per LED
WS2812Serial leds(numled, displayMemory, drawingMemory, 17,WS2812_GRB);
WS2812Serial DashLeds(numledOnDash, DashdisplayMemory, DashdrawingMemory, 24,WS2812_GRB);
uint32_t colorList[]={RED,ORANGE,YELLOW,GREEN,BLUE,PINK,WHITE,BLACK};

int colorMax=6;

void setPixels(uint8_t color[3]){
    for(int i=0;i<numled;i++){
    leds.setPixel(i, colorList[color[i]]);
    leds.show();
    }
};
void DashLedscolorWipe(int color){
    for(int i=0;i<leds.numPixels();i++){
        leds.setPixel(i,color);
        leds.show();
    }
}
uint8_t enableLights[]={0,7,7};
uint8_t waitingRtdLights[]={2,7,7};
uint8_t rtdLights[]={3,7,7};