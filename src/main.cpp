/*Testing live rinehart torque control with MCP3204 and teensy 4.1*/

#include <KS2e.h>
/******DEFINES*******/
// #define STARTUP 0
// #define TRACTIVE_SYSTEM_NOT_ACTIVE 1
// #define TRACTIVE_SYSTEM_ACTIVE 2
// #define ENABLING_INVERTER 3
// #define WAITING_RTD 4
// #define RTD 5
// #define mcFaulted 6
// #define rebootMC 7
#define HT_DEBUG_EN
unsigned long pchgAliveTimer = 0;
unsigned long now = 0;
int testState;
bool inverterState, rtdButtonPressed = false;
bool precharge_success = false;
bool rtdFlag = false;
// torque limits
// placeholder pedal position values
uint8_t defaultInverterCmd[] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t MC_internalState[8], MC_voltageInfo[8], MC_faultState[8], MC_motorPosInfo[8];
Metro timer_debug_pedals = Metro(1000);
Metro pchgMsgTimer = Metro(100);
Metro miscDebugTimer = Metro(1000);
Metro timer_inverter_enable = Metro(2000); // Timeout failed inverter enable
Metro timer_motor_controller_send = Metro(50);
Metro timer_coloumb_count_send = Metro(1000);
Metro timer_can_update = Metro(100);
Metro timer_sensor_can_update = Metro(5);
Metro timer_restart_inverter = Metro(500, 1); // Allow the MCU to restart the inverter
Metro timer_status_send = Metro(100);
Metro timer_watchdog_timer = Metro(500);
Metro updatePixelsTimer = Metro(200);
Metro pm100speedInspection = Metro(500);
MCU_status mcu_status{};
PM100Info::MCU_pedal_readings VCUPedalReadings{};
// GPIOs
// const int rtdButtonPin=33,TorqueControl=34,LaunchControl=35,InverterRelay=14;
int pixelColor;
bool inverter_restart = false;
#define DEBUG 1
// objects

uint8_t state = 0;                                            // basic state machine state

uint8_t enableSmallTorque[] = {0xD2, 0x04, 0, 0, 1, 1, 0, 0}; // The message to enable the motor with small torque
uint8_t maxTorque;
uint8_t PixelColorz[] = {7, 7, 7, 7, 7, 7};

/*****PROTOTYPES*****/
void idle();
void writeEnableFWDNoTorque();
void writeEnableSmallTorque();
void readBroadcast();


void setup()
{
    mcu_status.set_max_torque(0); // no torque on startup
    mcu_status.set_torque_mode(0);
    Serial.begin(115200);
    pinMode(RTDbutton, INPUT_PULLUP);
    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, LOW);
    pinMode(TORQUEMODE, INPUT);
    pinMode(LAUNCHCONTROL, INPUT_PULLUP);
    pinMode(MC_RELAY, OUTPUT);
    pinMode(WSFL, INPUT_PULLUP);
    pinMode(WSFR, INPUT_PULLUP);
    if (!dac.begin())
    {
        Serial.println("L dac");
    };
    dac.setVoltage(PUMP_SPEED, false);
    if (!DashDisplay.begin())
    {
        Serial.println("L dash");
    };
    DashDisplay.print("yEET");
    DashDisplay.writeDisplay();
    leds.begin();
    leds.setBrightness(BRIGHTNESS);


    digitalWrite(MC_RELAY, HIGH);
    mcu_status.set_inverter_powered(true);
    keepInverterAlive(0);
    delay(500);
    
    mcu_status.set_max_torque(TORQUE_2);
    DashLedscolorWipe(WHITE);
    delay(500);
    DashLedscolorWipe(PINK);
    delay(500);
    DashLedscolorWipe(GREEN);
}

void loop()
{
    readBroadcast();
    if (pm100speedInspection.check())
    {
        DashDisplay.begin();
        DashDisplay.clear();
        pm100Speed.print();
        DashDisplay.print(pm100Voltage.get_dc_bus_voltage(), DEC);
        DashDisplay.writeDisplay();
        // pm100Faults.print();
        pm100temp1.print();
        pm100temp2.print();
        pm100temp3.print();
    }
    if (updatePixelsTimer.check())
    { // 5hz
        DashLedscolorWipe(pixelColor);
    }

    
    state_machine();

    // TODO move these 
    if (timer_sensor_can_update.check())
    {
        CAN_message_t tx_msg;
        // Update the pedal readings to send over CAN
        VCUPedalReadings.set_accelerator_pedal_1(accel1);
        VCUPedalReadings.set_accelerator_pedal_2(accel2);
        VCUPedalReadings.set_brake_transducer_1(brake1);
        VCUPedalReadings.set_brake_transducer_2(brake1);

        // Send Main Control Unit pedal reading message
        VCUPedalReadings.write(tx_msg.buf);
        tx_msg.id = ID_VCU_PEDAL_READINGS;
        tx_msg.len = sizeof(VCUPedalReadings);
        DaqCAN.write(tx_msg);
    }
    if (timer_can_update.check())
    {
        // Send Main Control Unit status message
        CAN_message_t tx_msg;
        mcu_status.write(tx_msg.buf);
        tx_msg.id = ID_VCU_STATUS;
        tx_msg.len = sizeof(mcu_status);
        DaqCAN.write(tx_msg);
    }


}


// TODO finish moving these
void readBroadcast()
{
    char lineBuffer[200];
    CAN_message_t rxMsg;
    if (AccumulatorCAN.read(rxMsg))
    {
        if (rxMsg.id == 0x69) // accumulator
        {
            pchgAliveTimer = millis();
            pchgState = rxMsg.buf[0];
            int accVoltage = rxMsg.buf[1] + (rxMsg.buf[2] * 100);
            int tsVoltage = rxMsg.buf[3] + (rxMsg.buf[4] * 100);
            sprintf(lineBuffer, "precharging: state: %d ACV: %dv TSV: %dv\n", pchgState, accVoltage, tsVoltage);
            // Serial.print(lineBuffer);
        }
        if (rxMsg.id == ID_BMS_INFO) // accumulator
        {
            memcpy(BMS_packInfo, rxMsg.buf, sizeof(BMS_packInfo));
        }

        
        if (pchgState == 2)
        {
            precharge_success = true;
        }
        else if ((now - pchgAliveTimer >= 100) || pchgState == 0 || pchgState == 1 || pchgState == 3)
        {
            precharge_success = false;
        }
        // Serial.println("");
        DaqCAN.write(rxMsg);
    }
}









