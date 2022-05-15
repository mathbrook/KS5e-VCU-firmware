/*Testing live rinehart torque control with MCP3204 and teensy 4.1*/

#include <KS2e.h>
#include <Bounce2.h>
/******DEFINES*******/
#define STARTUP 0
#define TRACTIVE_SYSTEM_NOT_ACTIVE 1
#define TRACTIVE_SYSTEM_ACTIVE 2
#define ENABLING_INVERTER 3
#define WAITING_RTD 4
#define RTD 5
#define mcFaulted 6
#define rebootMC 7
unsigned long pchgAliveTimer=0;
unsigned long now=0;
int testState;
int pchgState;
bool inverterState,rtdButtonPressed=false;
bool precharge_success=false;
bool rtdFlag=false;
//torque limits
#define TORQUE_1 100
#define TORQUE_2 160
#define TORQUE_3 240
//placeholder pedal position values
float accel1{},accel2{},brake1{},brake2{};
uint8_t defaultInverterCmd[]={0,0,0,0,0,0,0,0};
uint8_t MC_internalState[8], MC_voltageInfo[8], MC_faultState[8], MC_motorPosInfo[8];
MCU_status mcu_status{};
Metro timer_debug_raw_torque=Metro(100);
Metro pchgMsgTimer=Metro(100);
Metro miscDebugTimer=Metro(1000);
PM100Info::MC_internal_states pm100info;
//GPIOs
const int rtdButtonPin=33,TorqueControl=34,LaunchControl=35,InverterRelay=14;
Bounce2::Button rtdButton=Bounce2::Button();
const int maxState = 7;
int lastState;
int rinehartState;
int lastRinehartState;
//#define DEBUG 0
elapsedMillis dischargeCountdown;
elapsedMillis RTDcountDown;
elapsedMillis RTDbuzzer; const int RTDPin=15; unsigned long RTD_interval=200;
//objects
#define NUM_TX_MAILBOXES 2
#define NUM_RX_MAILBOXES 6
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CAN;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CANtwo;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CANthree;
uint8_t state = 0;                                      //basic state machine state
uint8_t disableWithZeros[] = {0, 0, 0, 0, 0, 0, 0, 0};  //The message to disable the controller/cancel lockout
uint8_t enableNoTorque[] = {0, 0, 0, 0, 1, 1, 0, 0};    //The message to enable the motor with zero torque
uint8_t enableSmallTorque[] = {0xD2, 0x04, 0, 0, 1, 1, 0, 0}; //The message to enable the motor with small torque
uint8_t maxTorque;
/*
 * CAN ID definitions
 */
#define ID_MC_TEMPERATURES_1 0xA0
#define ID_MC_TEMPERATURES_2 0xA1
#define ID_MC_TEMPERATURES_3 0xA2
#define ID_MC_ANALOG_INPUTS_VOLTAGES 0xA3
#define ID_MC_DIGITAL_INPUT_STATUS 0xA4
#define ID_MC_MOTOR_POSITION_INFORMATION 0xA5
#define ID_MC_CURRENT_INFORMATION 0xA6
#define ID_MC_VOLTAGE_INFORMATION 0xA7
#define ID_MC_FLUX_INFORMATION 0xA8
#define ID_MC_INTERNAL_VOLTAGES 0xA9
#define ID_MC_INTERNAL_STATES 0xAA
#define ID_MC_FAULT_CODES 0xAB
#define ID_MC_TORQUE_TIMER_INFORMATION 0xAC
#define ID_MC_MODULATION_INDEX_FLUX_WEAKENING_OUTPUT_INFORMATION 0xAD
#define ID_MC_FIRMWARE_INFORMATION 0xAE
#define ID_MC_DIAGNOSTIC_DATA 0xAF
#define ID_MC_COMMAND_MESSAGE 0xC0
#define ID_MC_READ_WRITE_PARAMETER_COMMAND 0xC1
#define ID_MC_READ_WRITE_PARAMETER_RESPONSE 0xC2

/*****PROTOTYPES*****/
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
int mcBusVoltage();
int mcMotorRPM();
void tryToClearMcFault();
void forceMCdischarge();


static CAN_message_t tx_msg;
void setup()
{
    Serial.begin(115200);
//     ads.setGain(GAIN_TWOTHIRDS);
//   if (!ads.begin()) {
//     Serial.println("Failed to initialize ADS.");
//     while(1);
//   }
    // ADC.init(10,1000000);
    pinMode(rtdButtonPin,INPUT_PULLUP);
    pinMode(RTDPin,OUTPUT);pinMode(12,OUTPUT); digitalWrite(12,LOW); digitalWrite(RTDPin,LOW);
    pinMode(TorqueControl, INPUT_PULLUP);
    pinMode(LaunchControl, INPUT_PULLUP);
    pinMode(InverterRelay, OUTPUT); digitalWrite(InverterRelay,HIGH);
    // pinMode(LED_BUILTIN, OUTPUT);
    // digitalWrite(LED_BUILTIN, LOW);
    rtdButton.attach(rtdButtonPin,INPUT_PULLUP);
    rtdButton.interval(25);
    rtdButton.setPressedState(LOW);
    CAN.begin();
    CAN.setBaudRate(500000);
    CANtwo.begin();
    CANtwo.setBaudRate(250000);
    CANthree.begin();
    CANthree.setBaudRate(500000);
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
    CAN.mailboxStatus();
    testState=STARTUP;
    mcu_status.set_max_torque(TORQUE_3);
}
void testStateMachine(int state);
void loop()
{
    Serial.println("asdf");
//     // if(miscDebugTimer.check()){
//     //     Serial.print("APBS Status: ");
//     //     Serial.println(mcu_status.get_no_accel_brake_implausability());
//     // }
//     now=millis();
//     if(lastState!=testState){Serial.println(testState);lastState=testState;}
//     // Serial.println(testState);
//     float idk=now-pchgAliveTimer;
//     if(idk>=100){precharge_success=false;
//     }
//     // Serial.println(idk);
//     //Serial.println(precharge_success);
//     // if((mcControlTimer.check()==1)&&testState!=RTD){
//     //     keepInverterAlive();
//     // }
//   testStateMachine(testState);
//     //end doing state machine
//   readBroadcast();
//   rinehartState=figureOutMCStuff();
//   if((rinehartState!=lastRinehartState)||timer_debug_raw_torque.check()){
//       Serial.print("VCU State: ");
//       Serial.println(testState);
//     lastRinehartState=rinehartState;
//   switch(rinehartState){
//         case(0): Serial.println("VSM Start State");break;
//         case(4): Serial.println("VSM wait state");break;
//         case(5): Serial.println("VSM Ready State");break;
//         case(6): Serial.println("Motor Running State");break;
//        case(7): Serial.println("Blink fault State");break;
//     }
//     Serial.print("Motor rpm: ");
//         Serial.println(mcMotorRPM());
//         Serial.print("Bus Voltage(x10): ");
//         Serial.println(mcBusVoltage());   
//     }
    // if(timer_debug_raw_torque.check()){
    //     Serial.print("Motor rpm(x10): ");
    //     Serial.println(mcMotorRPM());
    //     Serial.print("Bus Voltage(x10): ");
    //     Serial.println(mcBusVoltage());
    // }
    calculate_torque();
    delay(1000);
        // uint16_t adsReading=ADC.read_adc(0);
        // Serial.print(0); Serial.print(": ");
        // Serial.println(ADC.read_adc(0));
        // delay(100);
}
void readBroadcast()
{   
    char lineBuffer[50];
    CAN_message_t rxMsg;
    if (CAN.read(rxMsg))
    {   
        
        // Serial.print("  ID: 0x");
        // Serial.print(rxMsg.id, HEX);
        // Serial.print(" DATA: ");
        // for (uint8_t i = 0; i < 8; i++)
        // {
        //     Serial.print(rxMsg.buf[i],HEX);
        //     Serial.print(" ");
        // }
        if (rxMsg.id == ID_MC_INTERNAL_STATES)
        {
            memcpy(MC_internalState,rxMsg.buf,sizeof(MC_internalState));
           // Serial.print("<< Internal States");
        }
        if (rxMsg.id == ID_MC_FAULT_CODES)
        {
            memcpy(MC_faultState,rxMsg.buf,sizeof(MC_faultState));
        }
        if (rxMsg.id == ID_MC_VOLTAGE_INFORMATION)
        {
            memcpy(MC_voltageInfo,rxMsg.buf,sizeof(MC_voltageInfo));
        }
         if (rxMsg.id == ID_MC_MOTOR_POSITION_INFORMATION)
        {
            memcpy(MC_motorPosInfo,rxMsg.buf,sizeof(MC_motorPosInfo));
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
        blinkLED();
    }
    // else
    // {
    //     Serial.print("NO CAN state: ");
    //     Serial.println(state);
    //    delay(50);
    // }
}
void writeEnableNoTorque()
{
    CAN_message_t ctrlMsg;
    ctrlMsg.len = 8;
    ctrlMsg.id = 0xC0; //OUR CONTROLLER
    memcpy(ctrlMsg.buf, enableNoTorque, sizeof(ctrlMsg.buf));
    CAN.write(ctrlMsg);
    Serial.println("----ENABLE----");
    blinkLED();
}
void doStartup()
{
    writeEnableNoTorque();
    writeControldisableWithZeros();
    writeEnableNoTorque();
}
void blinkLED()
{
    // digitalWrite(LED_BUILTIN, LOW);
    // digitalWrite(LED_BUILTIN, HIGH);
    // // delay(30);
    // digitalWrite(LED_BUILTIN, LOW);
    // delay(30);
}
void testStateMachine(int state){
  switch(state){
    case(STARTUP):
    writeControldisableWithZeros();
    tryToClearMcFault();
    if(rinehartState!=7){
    testState=TRACTIVE_SYSTEM_NOT_ACTIVE;}
    //what to do here?
    break;
    case(TRACTIVE_SYSTEM_NOT_ACTIVE):
    //what to do here? probably skip
    // if(mcControlTimer.check()==1){
    //     writeControldisableWithZeros();
    // }
    keepInverterAlive(0);
    if(pchgMsgTimer.check()==1){sendPrechargeStartMsg();}
    if(precharge_success==true && mcBusVoltage()>=600){
    testState=TRACTIVE_SYSTEM_ACTIVE;
    }
    break;
    case(TRACTIVE_SYSTEM_ACTIVE):
    testState=ENABLING_INVERTER;
    //here?
    break;
    case(ENABLING_INVERTER):
    if(mcControlTimer.check()){
    doStartup();}
    if(rinehartState==5 || rinehartState==6){
        testState=WAITING_RTD;
    }
    break;
    case(WAITING_RTD):
    // if(mcControlTimer.check()==1){
    //     writeEnableNoTorque();
    // }
    RTDcountDown=0;
    {keepInverterAlive(0);
    int waitingToRtdTorque=calculate_torque();
      rtdFlag=false;
    #ifdef DEBUG
    if(miscDebugTimer.check()){
        Serial.print("Brake pedal active: "); 
        Serial.println(mcu_status.get_brake_pedal_active());
        Serial.print("RTD button pressed(0 is pressed): ");
        Serial.println(digitalRead(rtdButtonPin));
    }
    #endif
    if(mcu_status.get_brake_pedal_active() && digitalRead(rtdButtonPin)==LOW && (waitingToRtdTorque)<=0){ testState=RTD;}
    else if(mcBusVoltage()<=600 || (now-pchgAliveTimer>=1000)){
        #ifdef DEBUG
        Serial.println("==STATE TRANSITION TO TS NOT ACTIVE ==")
        Serial.print("Bus V: "); Serial.println(mcBusVoltage()); 
        Serial.print("Precharge board state: ");Serial.println(pchgState); 
        Serial.print("Precharge timeout: "); Serial.println(now-pchgAliveTimer); 
        #endif
        forceMCdischarge();
        testState=TRACTIVE_SYSTEM_NOT_ACTIVE;}
        }
    break;
    case(RTD):
    if(precharge_success==false){
        forceMCdischarge();
        testState=STARTUP;
        }
    if(RTDcountDown<=250){
        digitalWrite(RTDPin,HIGH);
    }else{digitalWrite(RTDPin,LOW);rtdFlag=true;}
    if(mcControlTimer.check()==1){
      //implement plausibility checks on the test bench at some point
      int calculated_torque = calculate_torque(); //plaus checks are built in to the torque calculate function rn
      uint8_t torquePart1 = calculated_torque % 256;
      uint8_t torquePart2 = calculated_torque/256;
      uint8_t angularVelocity1=0,angularVelocity2=0;
      bool emraxDirection = true; //forward
      bool inverterEnable = true; //go brrr
      if((brake1>=22000)||calculated_torque==0|| !mcu_status.get_no_accel_brake_implausability()){
          torquePart1=0x0C;
          torquePart2=0xFE; //50nm regen
      }
      uint8_t torqueCommand[]={torquePart1,torquePart2,angularVelocity1,angularVelocity2,emraxDirection,inverterEnable,0,0};
      CAN_message_t ctrlMsg;
      ctrlMsg.len=8;
      ctrlMsg.id=ID_MC_COMMAND_MESSAGE;
      memcpy(ctrlMsg.buf, torqueCommand, sizeof(ctrlMsg.buf));
      CAN.write(ctrlMsg);
    }
    break;
    case(mcFaulted):
    tryToClearMcFault();
    if(rinehartState!=7){
        testState=STARTUP;
    }//else if(rinehartState==7){testState=rebootMC;}
    break;
    // case(rebootMC):
  }

}

int calculate_torque() {
     int calculated_torque = 0;
      accel1 = ALPHA * accel1 + (1 - ALPHA) * ADC.read_adc(0);
    accel2 = ALPHA * accel2 + (1 - ALPHA) * ADC.read_adc(1);
    brake1 = ALPHA * brake1 + (1 - ALPHA) * ADC.read_adc(2);
    //filtered_brake2_reading = ALPHA * filtered_brake2_reading + (1 - ALPHA) * ADC.read_adc(ADC_BRAKE_2_CHANNEL);

    //  accel1=ads.readADC_SingleEnded(0); 
    // accel2=ads.readADC_SingleEnded(2); 
    // brake1=ads.readADC_SingleEnded(3);
    if (brake1>BRAKE_ACTIVE){mcu_status.set_brake_pedal_active(true);}else if(brake1<=BRAKE_ACTIVE){mcu_status.set_brake_pedal_active(false);}
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
    
                // FSAE EV.5.7
                // APPS/Brake Pedal Plausability Check
                if  (
                        (
                            (accel1 > ((END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1)/4 + START_ACCELERATOR_PEDAL_1))
                            ||
                            (accel2 < ((END_ACCELERATOR_PEDAL_2 - START_ACCELERATOR_PEDAL_2)/4 + START_ACCELERATOR_PEDAL_2))
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
                    (accel2 > ((END_ACCELERATOR_PEDAL_2 - START_ACCELERATOR_PEDAL_2)/20 + START_ACCELERATOR_PEDAL_2))
                )
                {
                    mcu_status.set_no_accel_brake_implausability(true);
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
      CAN.write(ctrlMsg);
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
int mcBusVoltage(){
    int busVoltage=MC_voltageInfo[0]+MC_voltageInfo[1]*256;
    return busVoltage;
}
int mcMotorRPM(){
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