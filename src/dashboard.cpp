
#include "dashboard.hpp"
void Dashboard::init_dashboard()
{
    if (!DashDisplay.begin())
    {
        Serial.println("L dash");
    };
    DashDisplay.print("COPE");
    DashDisplay.writeDisplay();
    
    leds->begin();
    leds->setBrightness(BRIGHTNESS);

    set_dashboard_led_color(WHITE);
    DashLedscolorWipe();
    delay(500);
    set_dashboard_led_color(PINK);
    DashLedscolorWipe();
    delay(500);
    set_dashboard_led_color(GREEN);
    DashLedscolorWipe();


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
uint8_t enableLights[] = {0, 7, 7};
uint8_t waitingRtdLights[] = {2, 7, 7};
uint8_t rtdLights[] = {3, 7, 7};