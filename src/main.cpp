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
unsigned long pchgAliveTimer=0;
unsigned long now=0;
int testState;
int pchgState;
bool inverterState,rtdButtonPressed=false;
bool precharge_success=false;
bool rtdFlag=false;
//torque limits
//placeholder pedal position values
uint8_t defaultInverterCmd[]={0,0,0,0,0,0,0,0};
uint8_t MC_internalState[8], MC_voltageInfo[8], MC_faultState[8], MC_motorPosInfo[8];
Metro timer_debug_raw_torque=Metro(10);
Metro timer_debug_pedals=Metro(1000);
Metro pchgMsgTimer=Metro(100);
Metro miscDebugTimer=Metro(1000);
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
elapsedMillis dischargeCountdown;
PM100Info::MC_internal_states pm100State{};
PM100Info::MC_motor_position_information pm100Speed{};
PM100Info::MC_voltage_information pm100Voltage{};
PM100Info::MC_temperatures_1 pm100temp1{};
MCU_status mcu_status{};
//GPIOs
const int rtdButtonPin=33,TorqueControl=34,LaunchControl=35,InverterRelay=14;
const int maxState = 7;
int lastState;
int rinehartState;
int lastRinehartState;
int pixelColor;
bool inverter_restart = false;
#define DEBUG 1
//objects
#define NUM_TX_MAILBOXES 2
#define NUM_RX_MAILBOXES 6
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CAN;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> DaqCAN;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> AccumulatorCAN;
uint8_t state = 0;                                      //basic state machine state
uint8_t disableWithZeros[] = {0, 0, 0, 0, 0, 0, 0, 0};  //The message to disable the controller/cancel lockout
uint8_t enableNoTorque[] = {0, 0, 0, 0, 1, 1, 0, 0};    //The message to enable the motor with zero torque
uint8_t enableSmallTorque[] = {0xD2, 0x04, 0, 0, 1, 1, 0, 0}; //The message to enable the motor with small torque
uint8_t maxTorque;
uint8_t PixelColorz[]={7,7,7,7,7,7};

/*****PROTOTYPES*****/
void state_machine();
void writeControldisableWithZeros();
void writeEnableNoTorque();
void idle();
void writeEnableFWDNoTorque();
void writeEnableSmallTorque();
void doStartup();
void readBroadcast();
void blinkLED();
int calculate_torque();
void sendPrechargeStartMsg();
void keepInverterAlive(bool enable);
int figureOutMCStuff();
int getmcBusVoltage();
int getmcMotorRPM();
void tryToClearMcFault();
void forceMCdischarge();
void testStateMachine();
void read_pedal_values();
void set_state(MCU_STATE new_state);
void check_TS_active();
void check_inverter_disabled();
void reset_inverter();
void setup()
{
    mcu_status.set_max_torque(0); //no torque on startup
    mcu_status.set_torque_mode(0);
    Serial.begin(115200);
    pinMode(RTDbutton,INPUT_PULLUP);
    pinMode(BUZZER,OUTPUT);digitalWrite(BUZZER,LOW);
    pinMode(TORQUEMODE, INPUT_PULLUP);
    pinMode(LAUNCHCONTROL, INPUT_PULLUP);
    pinMode(MC_RELAY, OUTPUT);
    pinMode(WSFL,INPUT_PULLUP);pinMode(WSFR,INPUT_PULLUP);
    if(!dac.begin()){Serial.println("L dac");};
    dac.setVoltage(PUMP_SPEED,false);
    if(!DashDisplay.begin()){Serial.println("L dash");};
    leds.begin();
    leds.setBrightness(BRIGHTNESS);
    DashLeds.begin();
    DashLeds.setBrightness(BRIGHTNESS);
    CAN.begin();
    CAN.setBaudRate(500000);
    DaqCAN.begin();
    DaqCAN.setBaudRate(500000);
    AccumulatorCAN.begin();
    AccumulatorCAN.setBaudRate(500000);
    CAN.setMaxMB(NUM_TX_MAILBOXES+NUM_RX_MAILBOXES);
    for (int i = 0; i<NUM_RX_MAILBOXES; i++){
        CAN.setMB((FLEXCAN_MAILBOX)i,RX,STD);
    }
    for (int i = NUM_RX_MAILBOXES; i<(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++){
        CAN.setMB((FLEXCAN_MAILBOX)i,TX,STD);
    }
    CAN.setMBFilter(REJECT_ALL);
    CAN.setMBFilter(MB0,0x69); //precharge circuit id
    CAN.setMBFilter(MB1,ID_MC_VOLTAGE_INFORMATION);
    CAN.setMBFilter(MB2,ID_MC_FAULT_CODES);
    CAN.setMBFilter(MB3, ID_MC_INTERNAL_STATES);
    CAN.setMBFilter(MB4,ID_MC_MOTOR_POSITION_INFORMATION);
    CAN.setMBFilter(MB5,ID_MC_TEMPERATURES_1,ID_MC_TEMPERATURES_2,ID_MC_TEMPERATURES_3);
    CAN.mailboxStatus();
    testState=0;
    digitalWrite(MC_RELAY,HIGH);
    mcu_status.set_inverter_powered(true);
    keepInverterAlive(0);
    delay(500);
    set_state(MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
    mcu_status.set_max_torque(TORQUE_2);
    setPixels(enableLights);
    delay(100);
    setPixels(waitingRtdLights);
    delay(100);
    setPixels(rtdLights);
    DashLedscolorWipe(GREEN);
}
void loop()
{
    

    readBroadcast();
    if(updatePixelsTimer.check()){ //5hz 
        // Serial.println(getmcBusVoltage());
        //setPixels(PixelColorz);
        DashDisplay.print(pm100Voltage.get_dc_bus_voltage(),DEC);
        DashDisplay.writeDisplay();
        DashLedscolorWipe(pixelColor);
        if(digitalRead(TORQUEMODE)==LOW){
            mcu_status.set_max_torque(TORQUE_1);
            Serial.println("TOrque is set low");
        }
        if(digitalRead(TORQUEMODE)==HIGH){
            Serial.println("Troque is set high");
            mcu_status.set_max_torque(TORQUE_2);
        }
        Serial.print("rtd status: ");
        Serial.println(digitalRead(RTDbutton));
        }
        
        // pm100Voltage.print();
        // pm100State.print();
        // pm100Speed.print();
    read_pedal_values();
    if (timer_restart_inverter.check() && inverter_restart) {
        inverter_restart = false;
        digitalWrite(MC_RELAY, HIGH);
        mcu_status.set_inverter_powered(true);
    }
    state_machine();
}
void readBroadcast()
{   
    char lineBuffer[50];
    CAN_message_t rxMsg;
    if (CAN.read(rxMsg) || AccumulatorCAN.read(rxMsg))
    {   
        if (rxMsg.id == ID_MC_INTERNAL_STATES)
        {
            pm100State.load(rxMsg.buf);
            memcpy(MC_internalState,rxMsg.buf,sizeof(MC_internalState));
        }
        if (rxMsg.id == ID_MC_FAULT_CODES)
        {
            memcpy(MC_faultState,rxMsg.buf,sizeof(MC_faultState));
        }
        if (rxMsg.id == ID_MC_VOLTAGE_INFORMATION)
        {
            pm100Voltage.load(rxMsg.buf);
            memcpy(MC_voltageInfo,rxMsg.buf,sizeof(MC_voltageInfo));
        }
         if (rxMsg.id == ID_MC_MOTOR_POSITION_INFORMATION)
        {
            pm100Speed.load(rxMsg.buf);
            memcpy(MC_motorPosInfo,rxMsg.buf,sizeof(MC_motorPosInfo));
        }
        if (rxMsg.id== ID_MC_TEMPERATURES_1){
            pm100temp1.load(rxMsg.buf);
        }
        if (rxMsg.id == 0x69){
            pchgAliveTimer=millis();
            pchgState=rxMsg.buf[0];
            int accVoltage=rxMsg.buf[1]+(rxMsg.buf[2]*100);
            int tsVoltage=rxMsg.buf[3]+(rxMsg.buf[4]*100);
            sprintf(lineBuffer, "precharging: state: %d ACV: %dv TSV: %dv\n",pchgState,accVoltage,tsVoltage);
            //Serial.print(lineBuffer);
        }
        if(pchgState==2){precharge_success=true;}else if((now-pchgAliveTimer>=100) || pchgState==0 ||pchgState==1||pchgState==3){precharge_success=false;}
        // Serial.println("");
        
    }
}
void writeControldisableWithZeros()
{
    CAN_message_t ctrlMsg;
    ctrlMsg.len = 8;
    ctrlMsg.id = 0xC0; //OUR CONTROLLER
    memcpy(ctrlMsg.buf, disableWithZeros, sizeof(ctrlMsg.buf));
    if (CAN.write(ctrlMsg) > 0)
    {
        Serial.println("****DISABLE****");
    }
}
void writeEnableNoTorque()
{
    CAN_message_t ctrlMsg;
    ctrlMsg.len = 8;
    ctrlMsg.id = 0xC0; //OUR CONTROLLER
    memcpy(ctrlMsg.buf, enableNoTorque, sizeof(ctrlMsg.buf));
    CAN.write(ctrlMsg);
    Serial.println("----ENABLE----");
}
void doStartup()
{
    writeEnableNoTorque();
    writeControldisableWithZeros();
    writeEnableNoTorque();
}
inline void state_machine() {
    switch (mcu_status.get_state()) {
        case MCU_STATE::STARTUP: break;
        case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
            keepInverterAlive(0);
            #if DEBUG
            if(miscDebugTimer.check()){
            Serial.println("TS NOT ACTIVE");
            }
            if(pchgMsgTimer.check()){
                sendPrechargeStartMsg();
            }
            #endif
            // if TS is above HV threshold, move to Tractive System Active
            if ((pm100Voltage.get_dc_bus_voltage() >= MIN_HV_VOLTAGE)/* && precharge_success==true*/) {
                #if DEBUG
                Serial.println("Setting state to TS Active from TS Not Active");
                #endif
                set_state(MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
            }
            break;

        case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE:
            check_TS_active();
            keepInverterAlive(0);

            // if start button has been pressed and brake pedal is held down, transition to the next state
            if (digitalRead(RTDbutton)==0 && mcu_status.get_brake_pedal_active()) {
                #if DEBUG
                Serial.println("Setting state to Enabling Inverter");
                #endif
                set_state(MCU_STATE::ENABLING_INVERTER);
            }
            break;

        case MCU_STATE::ENABLING_INVERTER:
            check_TS_active();
            keepInverterAlive(1);

            // inverter enabling timed out
            if (timer_inverter_enable.check()) {
                #if DEBUG
                Serial.println("Setting state to TS Active from Enabling Inverter");
                #endif
                set_state(MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
            }
            // motor controller indicates that inverter has enabled within timeout period
            if (pm100State.get_inverter_enable_state()) {
                #if DEBUG
                Serial.println("Setting state to Waiting Ready to Drive Sound");
                #endif
                set_state(MCU_STATE::WAITING_READY_TO_DRIVE_SOUND);
            }
            break;

        case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND:
            check_TS_active();
            check_inverter_disabled();
            keepInverterAlive(1);

            // if the ready to drive sound has been playing for long enough, move to ready to drive mode
            if (timer_ready_sound.check()) {
                #if DEBUG
                Serial.println("Setting state to Ready to Drive");
                #endif
                set_state(MCU_STATE::READY_TO_DRIVE);
            }
            break;

        case MCU_STATE::READY_TO_DRIVE:
            check_TS_active();
            check_inverter_disabled();
            
            //update_coulomb_count();
            if (timer_motor_controller_send.check()) {
                // FSAE EV.5.5
                // FSAE T.4.2.10
                int calculated_torque = 0;
                calculated_torque=calculate_torque();
                if (accel1 < MIN_ACCELERATOR_PEDAL_1 || accel1 > MAX_ACCELERATOR_PEDAL_1) {
                    mcu_status.set_no_accel_implausability(false);
                    #if DEBUG
                    Serial.println("T.4.2.10 1");
                    Serial.println(accel1);
                    #endif
                }
                else if (accel2 < MIN_ACCELERATOR_PEDAL_2 ||accel2 > MAX_ACCELERATOR_PEDAL_2) {
                    mcu_status.set_no_accel_implausability(false);
                    #if DEBUG
                    Serial.println("T.4.2.10 2");
                    Serial.println(accel2);
                    #endif
                }
                // check that the pedals are reading within 10% of each other
                // sum of the two readings should be within 10% of the average travel
                // T.4.2.4
                else if ((accel1 - (4096 - accel2)) >
                         (END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1 + START_ACCELERATOR_PEDAL_2 - END_ACCELERATOR_PEDAL_2)/20 ){
                    #if DEBUG
                    Serial.println("T.4.2.4");
                    Serial.printf("computed - %f\n", accel1 - (4096 - accel2));
                    Serial.printf("standard - %d\n", (END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1 + START_ACCELERATOR_PEDAL_2 - END_ACCELERATOR_PEDAL_2)/20);
                    #endif
                    mcu_status.set_no_accel_implausability(false);
                }
                else{
                    #if DEBUG
                    Serial.println("T.4.2.4");
                    Serial.printf("computed - %f\n", accel1 - (4096 - accel2));
                    Serial.printf("standard - %d\n", (END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1 + START_ACCELERATOR_PEDAL_2 - END_ACCELERATOR_PEDAL_2)/20);
                    #endif
                    mcu_status.set_no_accel_implausability(true);
                }

                // BSE check
                // EV.5.6
                // FSAE T.4.3.4
                if (brake1 < 200 || brake1 > 2000) {
                    mcu_status.set_no_brake_implausability(false);
                }
                else{
                    mcu_status.set_no_brake_implausability(true);
                }

                // FSAE EV.5.7
                // APPS/Brake Pedal Plausability Check
                if  (
                        (
                            (accel1 > ((END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1)/4 + START_ACCELERATOR_PEDAL_1))
                            ||
                            (accel2 > ((END_ACCELERATOR_PEDAL_2 - START_ACCELERATOR_PEDAL_2)/4 + START_ACCELERATOR_PEDAL_2))
                        )
                        && mcu_status.get_brake_pedal_active()
                    )
                {
                    mcu_status.set_no_accel_brake_implausability(false);
                }
                else if
                (
                    (accel1 < ((END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1)/20 + START_ACCELERATOR_PEDAL_1))
                    &&
                    (accel2 < ((END_ACCELERATOR_PEDAL_2 - START_ACCELERATOR_PEDAL_2)/20 + START_ACCELERATOR_PEDAL_2))
                )
                {
                    mcu_status.set_no_accel_brake_implausability(true);
                }
                // mcu_status.set_no_accel_brake_implausability(true);
                calculated_torque = 0;

                if (
                    mcu_status.get_no_brake_implausability() &&
                    mcu_status.get_no_accel_implausability() &&
                    mcu_status.get_no_accel_brake_implausability()
                    ) {
                    calculated_torque = calculate_torque();
                } else {
                  Serial.println("not calculating torque");
                  Serial.printf("no brake implausibility: %d\n", mcu_status.get_no_brake_implausability());
                  Serial.printf("no accel implausibility: %d\n", mcu_status.get_no_accel_implausability());
                  Serial.printf("no accel brake implausibility: %d\n", mcu_status.get_no_accel_brake_implausability());
                }
                // Implausibility exists, command 0 torque

                #if DEBUG
                // if (timer_debug_torque.check()) {
                   /* Serial.print("MCU REQUESTED TORQUE: ");
                    Serial.println(calculated_torque);
                    Serial.print("MCU NO IMPLAUS ACCEL: ");
                    Serial.println(mcu_status.get_no_accel_implausability());
                    Serial.print("MCU NO IMPLAUS BRAKE: ");
                    Serial.println(mcu_status.get_no_brake_implausability());
                    Serial.print("MCU NO IMPLAUS ACCEL BRAKE: ");
                    Serial.println(mcu_status.get_no_accel_brake_implausability());*/
                   /* Serial.printf("ssok: %d\n", mcu_status.get_software_is_ok());
                    Serial.printf("bms: %d\n", mcu_status.get_bms_ok_high());
                    Serial.printf("imd: %d\n", mcu_status.get_imd_ok_high());*/
                //}
                #endif
                uint8_t torquePart1 = calculated_torque % 256;
                uint8_t torquePart2 = calculated_torque/256;
                uint8_t angularVelocity1=0,angularVelocity2=0;
                bool emraxDirection = true; //forward
                bool inverterEnable = true; //go brrr
                // if((brake1>=22000)||calculated_torque==0|| !mcu_status.get_no_accel_brake_implausability()){
                //     torquePart1=0x0C;
                //     torquePart2=0xFE; //50nm regen
                // }
                uint8_t torqueCommand[]={torquePart1,torquePart2,angularVelocity1,angularVelocity2,emraxDirection,inverterEnable,0,0};
                CAN_message_t ctrlMsg;
                ctrlMsg.len=8;
                ctrlMsg.id=ID_MC_COMMAND_MESSAGE;
                memcpy(ctrlMsg.buf, torqueCommand, sizeof(ctrlMsg.buf));
                CAN.write(ctrlMsg);
            }
            break;
    }
}
/* Handle changes in state */
void set_state(MCU_STATE new_state) {
    if (mcu_status.get_state() == new_state) {
        return;
    }

    // exit logic
    switch(mcu_status.get_state()){
        case MCU_STATE::STARTUP: break;
        case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE: tryToClearMcFault(); break;
        case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE: break;
        case MCU_STATE::ENABLING_INVERTER: break;
        case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND:
            // make dashboard stop buzzer
            mcu_status.set_activate_buzzer(false);
           digitalWrite(BUZZER,LOW);
            break;
        case MCU_STATE::READY_TO_DRIVE: break;
    }

    mcu_status.set_state(new_state);

    // entry logic
    switch (new_state) {
        case MCU_STATE::STARTUP: break;
        case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE: 
            {uint8_t TSNA[] ={3,3,3,3,3,3};
            memcpy(PixelColorz,TSNA,sizeof(PixelColorz));
            pixelColor=GREEN;
            break;}
        case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE:
            pixelColor=RED;
            {uint8_t TSA[] ={0,0,0,0,0,0};
            memcpy(PixelColorz,TSA,sizeof(PixelColorz));}
            break;
        case MCU_STATE::ENABLING_INVERTER: {
{            uint8_t ENINV[] ={5,5,5,5,5,5};
            memcpy(PixelColorz,ENINV,sizeof(PixelColorz));}
            pixelColor=YELLOW;
            doStartup();
            Serial.println("MCU Sent enable command");
            timer_inverter_enable.reset();
            break;
        }
        case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND:
            // make dashboard sound buzzer
            {uint8_t ENINV[] ={2,1,2,1,2,1};
            memcpy(PixelColorz,ENINV,sizeof(PixelColorz));}
            pixelColor=ORANGE;
            mcu_status.set_activate_buzzer(true);
            digitalWrite(BUZZER,HIGH);
            timer_ready_sound.reset();
            Serial.println("RTDS enabled");
            break;
        case MCU_STATE::READY_TO_DRIVE:
        pixelColor=PINK;
            {uint8_t RTD[] ={3,3,3,3,3,3};
            memcpy(PixelColorz,RTD,sizeof(PixelColorz));}
            Serial.println("Ready to drive");
            break;
    }
}


int calculate_torque() {
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
    if (torque1 > max_torque) {
        torque1 = max_torque;
    }
    if (torque2 > max_torque) {
        torque2 = max_torque;
    }
    // compare torques to check for accelerator implausibility
    calculated_torque = (torque1 + torque2) / 2;

    if (calculated_torque > max_torque) {
        calculated_torque = max_torque;
    }
    if (calculated_torque < 0) {
        calculated_torque = 0;
    }
    //#if DEBUG
    if (timer_debug_raw_torque.check()) {
        Serial.print("TORQUE REQUEST DELTA PERCENT: "); // Print the % difference between the 2 accelerator sensor requests
        Serial.println(abs(torque1 - torque2) / (double) max_torque * 100);
        Serial.print("MCU RAW TORQUE: ");
        Serial.println(calculated_torque);
        Serial.print("TORQUE 1: ");
        Serial.println(torque1);
        Serial.print("TORQUE 2: ");
        Serial.println(torque2);
        Serial.print("Accel 1: ");
        Serial.println(accel1);
        Serial.print("Accel 2: ");
        Serial.println(accel2);
        Serial.print("Brake1 : ");
        Serial.println(brake1);
    }
    //#endif
     return calculated_torque;
}
inline void read_pedal_values() {
    /* Filter ADC readings */
    accel1 = ALPHA * accel1 + (1 - ALPHA) * ADC.read_adc(ADC_ACCEL_1_CHANNEL);
    accel2 = ALPHA * accel2 + (1 - ALPHA) * ADC.read_adc(ADC_ACCEL_2_CHANNEL);
    brake1 = ALPHA * brake1 + (1 - ALPHA) * ADC.read_adc(ADC_BRAKE_1_CHANNEL);
    //we dont have 2 brake sensors so commented out
    // filtered_brake2_reading = ALPHA * filtered_brake2_reading + (1 - ALPHA) * ADC.read_adc(ADC_BRAKE_2_CHANNEL);

    #if DEBUG
if (timer_debug_pedals.check()) {
   Serial.print("ACCEL 1: "); Serial.println(accel1);
   Serial.print("ACCEL 2: "); Serial.println(accel2);
   Serial.print("BRAKE 1: "); Serial.println(brake1);
}
  //  Serial.print("BRAKE 2: "); Serial.println(filtered_brake2_reading);
    #endif

    // only uses front brake pedal
    mcu_status.set_brake_pedal_active(brake1 >= BRAKE_ACTIVE);

    /* Print values for debugging */
    /*#if DEBUG
    if (timer_debug.check()) {
        Serial.print("MCU PEDAL ACCEL 1: ");
        Serial.println(mcu_pedal_readings.get_accelerator_pedal_1());
        Serial.print("MCU PEDAL ACCEL 2: ");
        Serial.println(mcu_pedal_readings.get_accelerator_pedal_2());
        Serial.print("MCU PEDAL BRAKE: ");
        Serial.println(mcu_pedal_readings.get_brake_transducer_1());
        Serial.print("MCU BRAKE ACT: ");
        Serial.println(mcu_status.get_brake_pedal_active());
        Serial.print("MCU STATE: ");
        Serial.println(mcu_status.get_state());
    }
    #endif*/
}

void sendPrechargeStartMsg(){
    uint8_t prechargeCmd[]={0,0,0,0,0,0,0,0};
      CAN_message_t ctrlMsg;
      ctrlMsg.len=8;
      ctrlMsg.id=420;

      // MC_command_message mc_command_message(0,0,1,1,1,0);
      
      // mc_command_message.set_torque_command(calculated_torque);
      // mc_command_message.set_discharge_enable(true);
      // mc_command_message.set_inverter_enable(true);
      memcpy(ctrlMsg.buf, prechargeCmd, sizeof(ctrlMsg.buf));
      AccumulatorCAN.write(ctrlMsg);
}
/* Shared state functinality */

// if TS is below HV threshold, return to Tractive System Not Active
inline void check_TS_active() {
    if ((getmcBusVoltage() < MIN_HV_VOLTAGE)) {
        #if DEBUG
        Serial.println("Setting state to TS Not Active, because TS is below HV threshold");
        #endif
        set_state(MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
    }
}
// if the inverter becomes disabled, return to Tractive system active
inline void check_inverter_disabled() {
    if (!pm100State.get_inverter_enable_state()) {
        #if DEBUG
        Serial.println("Setting state to TS Active because inverter is disabled");
        #endif
        set_state(MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
    }
}
void keepInverterAlive(bool enable){ //do u want the MC on or not?
    if(mcControlTimer.check()){
    CAN_message_t ctrlMsg;
      ctrlMsg.len=8;
      ctrlMsg.id=ID_MC_COMMAND_MESSAGE;
      uint8_t heartbeatMsg[]={0,0,0,0,1,enable,0,0};
      memcpy(ctrlMsg.buf, heartbeatMsg, sizeof(ctrlMsg.buf));
      CAN.write(ctrlMsg);
    }
}
int figureOutMCStuff(){
    int mcState;
    mcState=MC_internalState[0];
    return mcState;
}
int getmcBusVoltage(){
    int busVoltage=MC_voltageInfo[0]+MC_voltageInfo[1]*256;
    return busVoltage;
}
int getmcMotorRPM(){
    int emraxSpeed=MC_motorPosInfo[2]+MC_motorPosInfo[3]*256;
    if(emraxSpeed>65000){
        emraxSpeed=0;
    }
    return emraxSpeed;
}
void  tryToClearMcFault(){
    if(mcControlTimer.check()){
    CAN_message_t ctrlMsg;
      ctrlMsg.len=8;
      ctrlMsg.id=ID_MC_READ_WRITE_PARAMETER_COMMAND;
      uint8_t clearFaultMsg[]={20,0,1,0,0,0,0,0};
      memcpy(ctrlMsg.buf, clearFaultMsg, sizeof(ctrlMsg.buf));
      CAN.write(ctrlMsg);
}
}
void forceMCdischarge(){
    dischargeCountdown=0;
    while(dischargeCountdown<=100){
        if(mcControlTimer.check()==1){
            CAN_message_t ctrlMsg;
        ctrlMsg.len=8;
        ctrlMsg.id=ID_MC_COMMAND_MESSAGE;
        uint8_t dischgMsg[]={0,0,0,0,1,0b0000010,0,0};//bit one?
        memcpy(ctrlMsg.buf, dischgMsg, sizeof(ctrlMsg.buf));
        CAN.write(ctrlMsg);
            //writeEnableNoTorque();
            }
        } 
        for(int i=0;i<=10;i++){
            writeControldisableWithZeros();
        }
}
void reset_inverter() {
    inverter_restart = true;
    digitalWrite(MC_RELAY, LOW);
    timer_restart_inverter.reset();
    mcu_status.set_inverter_powered(false);

    #if DEBUG
    Serial.println("INVERTER RESET");
    #endif
}