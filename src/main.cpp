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
int pchgState;
bool inverterState, rtdButtonPressed = false;
bool precharge_success = false;
bool rtdFlag = false;
// torque limits
// placeholder pedal position values
uint8_t defaultInverterCmd[] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t MC_internalState[8], MC_voltageInfo[8], MC_faultState[8], MC_motorPosInfo[8], BMS_packInfo[8];
Metro timer_debug_raw_torque = Metro(100);
Metro timer_debug_pedals = Metro(1000);
Metro pchgMsgTimer = Metro(100);
Metro miscDebugTimer = Metro(1000);
Metro timer_inverter_enable = Metro(2000); // Timeout failed inverter enable
Metro timer_motor_controller_send = Metro(50);
Metro timer_coloumb_count_send = Metro(1000);
Metro timer_ready_sound = Metro(1000); // Time to play RTD sound
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

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CAN;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> DaqCAN;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> AccumulatorCAN;
uint8_t state = 0;                                            // basic state machine state

uint8_t enableSmallTorque[] = {0xD2, 0x04, 0, 0, 1, 1, 0, 0}; // The message to enable the motor with small torque
uint8_t maxTorque;
uint8_t PixelColorz[] = {7, 7, 7, 7, 7, 7};

/*****PROTOTYPES*****/
void state_machine();
void writeControldisableWithZeros();
void writeEnableNoTorque();
void idle();
void writeEnableFWDNoTorque();
void writeEnableSmallTorque();
void doStartup();
void readBroadcast();
int calculate_torque();
void sendPrechargeStartMsg();
void keepInverterAlive(bool enable);
int figureOutMCStuff();
int getmcBusVoltage();
int getmcMotorRPM();
void tryToClearMcFault();
void forceMCdischarge(); // unused, sketchy
void read_pedal_values();
void set_state(MCU_STATE new_state);
void check_TS_active();
void check_inverter_disabled();
void reset_inverter();

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
    set_state(MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
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

    read_pedal_values();
    if (timer_restart_inverter.check() && inverter_restart)
    {
        inverter_restart = false;
        digitalWrite(MC_RELAY, HIGH);
        mcu_status.set_inverter_powered(true);
    }
    state_machine();
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

void readBroadcast()
{
    char lineBuffer[200];
    CAN_message_t rxMsg;
    if (CAN.read(rxMsg) || AccumulatorCAN.read(rxMsg))
    {
        if (rxMsg.id == ID_MC_INTERNAL_STATES)
        {
            pm100State.load(rxMsg.buf);
            memcpy(MC_internalState, rxMsg.buf, sizeof(MC_internalState));
        }
        if (rxMsg.id == ID_MC_FAULT_CODES)
        {
            memcpy(MC_faultState, rxMsg.buf, sizeof(MC_faultState));
        }
        if (rxMsg.id == ID_MC_VOLTAGE_INFORMATION)
        {
            pm100Voltage.load(rxMsg.buf);
            memcpy(MC_voltageInfo, rxMsg.buf, sizeof(MC_voltageInfo));
        }
        if (rxMsg.id == ID_MC_MOTOR_POSITION_INFORMATION)
        {
            pm100Speed.load(rxMsg.buf);
            memcpy(MC_motorPosInfo, rxMsg.buf, sizeof(MC_motorPosInfo));
        }
        if (rxMsg.id == ID_MC_TEMPERATURES_1)
        {
            pm100temp1.load(rxMsg.buf);
        }
        if (rxMsg.id == ID_MC_TEMPERATURES_2)
        {
            pm100temp2.load(rxMsg.buf);
        }
        if (rxMsg.id == ID_MC_TEMPERATURES_3)
        {
            pm100temp3.load(rxMsg.buf);
        }
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

/* Handle changes in state */
void set_state(MCU_STATE new_state)
{
    if (mcu_status.get_state() == new_state)
    {
        return;
    }

    // exit logic
    switch (mcu_status.get_state())
    {
    case MCU_STATE::STARTUP:
        break;
    case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
        tryToClearMcFault();
        break;
    case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE:
        break;
    case MCU_STATE::ENABLING_INVERTER:
        break;
    case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND:
        // make dashboard stop buzzer
        mcu_status.set_activate_buzzer(false);
        digitalWrite(BUZZER, LOW);
        break;
    case MCU_STATE::READY_TO_DRIVE:
        break;
    }

    mcu_status.set_state(new_state);

    // entry logic
    switch (new_state)
    {
    case MCU_STATE::STARTUP:
        break;
    case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
    {
        uint8_t TSNA[] = {3, 3, 3, 3, 3, 3};
        memcpy(PixelColorz, TSNA, sizeof(PixelColorz));
        pixelColor = GREEN;
        break;
    }
    case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE:
        pixelColor = RED;
        {
            uint8_t TSA[] = {0, 0, 0, 0, 0, 0};
            memcpy(PixelColorz, TSA, sizeof(PixelColorz));
        }
        break;
    case MCU_STATE::ENABLING_INVERTER:
    {
        {
            uint8_t ENINV[] = {5, 5, 5, 5, 5, 5};
            memcpy(PixelColorz, ENINV, sizeof(PixelColorz));
        }
        pixelColor = YELLOW;
        tryToClearMcFault();
        doStartup();
        Serial.println("MCU Sent enable command");
        timer_inverter_enable.reset();
        break;
    }
    case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND:
        // make dashboard sound buzzer
        {
            uint8_t ENINV[] = {2, 1, 2, 1, 2, 1};
            memcpy(PixelColorz, ENINV, sizeof(PixelColorz));
        }
        pixelColor = ORANGE;
        mcu_status.set_activate_buzzer(true);
        digitalWrite(BUZZER, HIGH);
        timer_ready_sound.reset();
        Serial.println("RTDS enabled");
        break;
    case MCU_STATE::READY_TO_DRIVE:
        pixelColor = PINK;
        {
            uint8_t RTD[] = {3, 3, 3, 3, 3, 3};
            memcpy(PixelColorz, RTD, sizeof(PixelColorz));
        }
        Serial.println("Ready to drive");
        break;
    }
}

int calculate_torque()
{
    int calculated_torque = 0;
    const int max_torque = mcu_status.get_max_torque() * 10;
    int torque1 = map(round(accel1), START_ACCELERATOR_PEDAL_1, END_ACCELERATOR_PEDAL_1, 0, max_torque);
    int torque2 = map(round(accel2), START_ACCELERATOR_PEDAL_2, END_ACCELERATOR_PEDAL_2, 0, max_torque);
    // #if DEBUG
    //   Serial.print("max torque: ");
    //   Serial.println(max_torque);
    //   Serial.print("torque1: ");
    //   Serial.println(torque1);
    //   Serial.print("torque2: ");
    //   Serial.println(torque2);
    // #endif

    // torque values are greater than the max possible value, set them to max
    if (torque1 > max_torque)
    {
        torque1 = max_torque;
    }
    if (torque2 > max_torque)
    {
        torque2 = max_torque;
    }
    // compare torques to check for accelerator implausibility
    calculated_torque = (torque1 + torque2) / 2;

    if (calculated_torque > max_torque)
    {
        calculated_torque = max_torque;
    }
    if (calculated_torque < 0)
    {
        calculated_torque = 0;
    }
    //#if DEBUG
    if (timer_debug_raw_torque.check())
    {
        // Serial.print("TORQUE REQUEST DELTA PERCENT: "); // Print the % difference between the 2 accelerator sensor requests
        // Serial.println(abs(torque1 - torque2) / (double) max_torque * 100);
        // Serial.print("MCU RAW TORQUE: ");
        // Serial.println(calculated_torque);
        // Serial.print("TORQUE 1: ");
        // Serial.println(torque1);
        // Serial.print("TORQUE 2: ");
        // Serial.println(torque2);
        // Serial.print("Accel 1: ");
        // Serial.println(accel1);
        // Serial.print("Accel 2: ");
        // Serial.println(accel2);
        // Serial.print("Brake1 : ");
        // Serial.println(brake1);
    }
    if (abs(pm100Speed.get_motor_speed()) <= 50)
    {
        if (calculated_torque >= 600)
        {
            calculated_torque = 600; // ideally limit torque at low RPMs, see how high this number can be raised
        }
    }
    //#endif
    return calculated_torque;
}
inline void read_pedal_values()
{
    /* Filter ADC readings */
    accel1 = ALPHA * accel1 + (1 - ALPHA) * ADC.read_adc(ADC_ACCEL_1_CHANNEL);
    accel2 = ALPHA * accel2 + (1 - ALPHA) * ADC.read_adc(ADC_ACCEL_2_CHANNEL);
    brake1 = ALPHA * brake1 + (1 - ALPHA) * ADC.read_adc(ADC_BRAKE_1_CHANNEL);
    // we dont have 2 brake sensors so commented out
    //  filtered_brake2_reading = ALPHA * filtered_brake2_reading + (1 - ALPHA) * ADC.read_adc(ADC_BRAKE_2_CHANNEL);

#if DEBUG
    if (timer_debug_pedals.check())
    {
        Serial.print("ACCEL 1: ");
        Serial.println(accel1);
        Serial.print("ACCEL 2: ");
        Serial.println(accel2);
        Serial.print("BRAKE 1: ");
        Serial.println(brake1);
    }
#endif

    // only uses front brake pedal
    mcu_status.set_brake_pedal_active(brake1 >= BRAKE_ACTIVE);
}

/* Shared state functinality */

// if TS is below HV threshold, return to Tractive System Not Active
inline void check_TS_active()
{
    if ((getmcBusVoltage() < MIN_HV_VOLTAGE))
    {
#if DEBUG
        Serial.println("Setting state to TS Not Active, because TS is below HV threshold");
#endif
        set_state(MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
    }
}
// if the inverter becomes disabled, return to Tractive system active
inline void check_inverter_disabled()
{
    if (!pm100State.get_inverter_enable_state())
    {
#if DEBUG
        Serial.println("Setting state to TS Active because inverter is disabled");
#endif
        set_state(MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
    }
}

int figureOutMCStuff()
{
    int mcState;
    mcState = MC_internalState[0];
    return mcState;
}
int getmcBusVoltage()
{
    int busVoltage = MC_voltageInfo[0] + MC_voltageInfo[1] * 256;
    return busVoltage;
}
int getmcMotorRPM()
{
    int emraxSpeed = MC_motorPosInfo[2] + MC_motorPosInfo[3] * 256;
    if (emraxSpeed > 65000)
    {
        emraxSpeed = 0;
    }
    return emraxSpeed;
}


void reset_inverter()
{
    inverter_restart = true;
    digitalWrite(MC_RELAY, LOW);
    timer_restart_inverter.reset();
    mcu_status.set_inverter_powered(false);

#if DEBUG
    Serial.println("INVERTER RESET");
#endif
}