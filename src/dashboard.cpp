
#include "dashboard.hpp"
void Dashboard::init_dashboard()
{
    if (!DashDisplay.begin())
    {
        Serial.println("L dash");
    };
    DashDisplay.setBrightness(15);

    DashDisplay.print(TORQUE_1);
    DashDisplay.writeDisplay();
    
    leds->begin();
    leds->setBrightness(BRIGHTNESS);

    int microsec = 200000 / leds->numPixels();
    colorWipe(RED, microsec);
    colorWipe(ORANGE, microsec);
    colorWipe(YELLOW, microsec);
    colorWipe(GREEN, microsec);
    colorWipe(BLUE, microsec);
    colorWipe(0x8F00FF,microsec);
    colorWipe(PINK, microsec);
    colorWipe(0xFFFFFF, microsec);
    colorWipe(GREEN, microsec);
}

void Dashboard::set_dashboard_led_color(int color){
    led_color_ = color;
}

void Dashboard::refresh_dash(int voltage)
{
    DashDisplay.begin();
    DashDisplay.clear();
    DashDisplay.print(voltage, DEC);
    DashDisplay.writeDisplay();
    DashLedscolorWipe();
}

void Dashboard::DashLedscolorWipe()
{
    for (int i = 0; i < leds->numPixels(); i++)
    {
        leds->setPixel(i, led_color_);
        leds->show();
    }
}
void Dashboard::DashLedsBrightness()
{
    uint8_t currbright = leds->getBrightness();
    if((currbright+5)>255){
        ledsdirection=false;
    }else if(currbright<=30){
        ledsdirection=true;
    }
    uint8_t newbright;
    if(ledsdirection){
        newbright = currbright + 25;
    }else{
        newbright=currbright-25;
    }
    leds->setBrightness(newbright);

}
void Dashboard::colorWipe(int color, int wait_us) {
  for (int i=0; i < leds->numPixels(); i++) {
    leds->setPixel(i, color);
    leds->show();
    delayMicroseconds(wait_us);
  }
}
uint8_t enableLights[] = {0, 7, 7};
uint8_t waitingRtdLights[] = {2, 7, 7};
uint8_t rtdLights[] = {3, 7, 7};