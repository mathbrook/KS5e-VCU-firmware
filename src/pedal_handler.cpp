#include "pedal_handler.hpp"
#include "state_machine.hpp"
Metro debugPrint = Metro(100);

// initializes pedal's ADC
void PedalHandler::init_pedal_handler()
{
    // TODO don't htink you actually have to init analog GPIOs on teensy
    // for(int i; i < sizeof(analog_init_list)/sizeof(int);i++){
    //     pinMode(analog_init_list[i],INPUT);
    // }
    pedal_ADC = ADC_SPI(CS_ADC, DEFAULT_SPI_SPEED);
    pid_->setTimeStep(PID_TIMESTEP);
    wsfl_->begin(WSFL);
    wsfr_->begin(WSFR);
    pid_->setBangBang(double(BANGBANG_RANGE));
}
void PedalHandler::run_pedals()
{
    this->apps1.sensor_run();
    this->apps2.sensor_run();
    this->bse1.sensor_run();
}

/**
 * @brief calculate torque to be commanded based on accel pedal position
 * 
 * @param motor_speed 
 * @param max_torque 
 * @return int16_t 
 */
int16_t PedalHandler::calculate_torque(int16_t &motor_speed, int &max_torque)
{
    int calculated_torque = 0;

    int torque1 = static_cast<int>(this->apps1.getTravelRatio() * max_torque);
    int torque2 = static_cast<int>(this->apps2.getTravelRatio() * max_torque);
    // torque values are greater than the max possible value, set them to max
    if (torque1 > max_torque)
    {
        torque1 = max_torque;
    }
    if (torque2 > max_torque)
    {
        torque2 = max_torque;
    }
    // Use average of the two APPS torque results to calculate the torque request

    calculated_torque = (torque1 + torque2) / 2; // TODO un-cheese this

    if (calculated_torque > max_torque)
    {
        calculated_torque = max_torque;
    }
    if (calculated_torque < 0)
    {
        calculated_torque = 0;
    }

    return calculated_torque;
}

/**
 * @brief calculate the regen to be commanded based on brake pedal position
 * 
 * @param motor_speed 
 * @param max_regen_torque 
 * @return int16_t 
 */
int16_t PedalHandler::calculate_regen(int16_t &motor_speed, int16_t max_regen_torque)
{
    int16_t calculated_regen_torque = 0;

    const int16_t regen_torque_maximum = REGEN_NM * -10;
    calculated_regen_torque = this->bse1.getTravelRatio() * regen_torque_maximum;
    // Smooth regen torque so it doesnt yeet driveline
    // TODO find out what this limits the rate of change to
    smoothed_regen_torque = 0.8 * smoothed_regen_torque + (1-0.8) * calculated_regen_torque;
    #if DEBUG
    Serial.printf("Calculated regen: %d smoothed regen: %d",calculated_regen_torque,smoothed_regen_torque);
    #endif
    return smoothed_regen_torque;
}
/**
 * @brief read and update all pedal values
 * 
 * @return true if brake active
 * @return false if not active
 */
bool PedalHandler::read_pedal_values()
{
    /* Filter ADC readings */

    // exponential smoothing https://chat.openai.com/share/cf98f2ea-d87c-4d25-a365-e398ffebf968
    // TODO - evaluate this ALPHA value and if maybe we should decrease it
    pedal_ADC.update_readings(FILTERING_ALPHA_10HZ);
    accel1_ = pedal_ADC.get_reading(ADC_ACCEL_1_CHANNEL);
    accel2_ = pedal_ADC.get_reading(ADC_ACCEL_2_CHANNEL);
    brake1_ = pedal_ADC.get_reading(ADC_BRAKE_1_CHANNEL);
    steering_angle_ = pedal_ADC.get_reading(ADC_STEERING_CHANNEL);
    // accel1_ =
    //     ALPHA * accel1_ + (1 - ALPHA) * pedal_ADC.read_adc(ADC_ACCEL_1_CHANNEL);
    // accel2_ =
    //     ALPHA * accel2_ + (1 - ALPHA) * pedal_ADC.read_adc(ADC_ACCEL_2_CHANNEL);
    // brake1_ =
    //     ALPHA * brake1_ + (1 - ALPHA) * pedal_ADC.read_adc(ADC_BRAKE_1_CHANNEL);

    // steering_angle_ =
    //     ALPHA * steering_angle_ + (1 - ALPHA) * pedal_ADC.read_adc(ADC_STEERING_CHANNEL);
#if DEBUG
    if (debugPrint.check())
    {
        Serial.print("APPS1 :");
        Serial.println(accel1_);
        Serial.print("APPS2 :");
        Serial.println(accel2_);
        Serial.print("BRAKE :");
        Serial.println(brake1_);
        Serial.print("STEERING_ANGLE :");
        Serial.println(steering_angle_);
    }
#endif

    VCUPedalReadings.set_accelerator_pedal_1(accel1_);
    VCUPedalReadings.set_accelerator_pedal_2(accel2_);
    VCUPedalReadings.set_brake_transducer_1(brake1_);
    VCUPedalReadings.set_brake_transducer_2(steering_angle_);

    brake_is_active_ = (brake1_ >= BRAKE_ACTIVE);
    return brake_is_active_;
}

/**
 * @brief send pedal readings over CAN, with timers built in to function
 * 
 */
void PedalHandler::send_readings()
{
    if (pedal_out_20hz->check())
    {
        // Send Main Control Unit pedal reading message @ 20hz
        CAN_message_t tx_msg;
        VCUPedalReadings.write(tx_msg.buf);
        tx_msg.id = ID_VCU_PEDAL_READINGS;
        tx_msg.len = sizeof(VCUPedalReadings);

        // Send wheel speed readings @ 20hz
        CAN_message_t tx_msg2;
        tx_msg2.id = ID_VCU_WS_READINGS;
        tx_msg2.len = 8;
        int16_t rpm_wsfl = (int16_t)(wsfl_t.current_rpm);
        int16_t rpm_wsfr = (int16_t)(wsfr_t.current_rpm);
        int16_t rpm_buf[]={rpm_wsfl,rpm_wsfr};
        memcpy(&tx_msg2.buf[0], &rpm_buf, sizeof(rpm_buf));
        WriteCANToInverter(tx_msg);
        WriteCANToInverter(tx_msg2);
    }
    if (pedal_out_1hz->check())
    {
        // Send miscellaneous board analog readings @ 1hz
        CAN_message_t tx_msg3;
        tx_msg3.id = ID_VCU_BOARD_ANALOG_READS_ONE;
        memcpy(&tx_msg3.buf[0], &glv_current_, sizeof(glv_current_));
        memcpy(&tx_msg3.buf[2], &glv_voltage_, sizeof(glv_voltage_));
        memcpy(&tx_msg3.buf[4], &bspd_voltage_, sizeof(bspd_voltage_));
        memcpy(&tx_msg3.buf[6], &vcc_voltage_, sizeof(vcc_voltage_));

        CAN_message_t tx_msg4;
        tx_msg4.id = ID_VCU_BOARD_ANALOG_READS_TWO;
        memcpy(&tx_msg4.buf[0], &sdc_voltage_, sizeof(sdc_voltage_));
        memcpy(&tx_msg4.buf[2], &sdc_current_, sizeof(sdc_current_));
        memcpy(&tx_msg4.buf[4], &analog_input_nine_voltage_, sizeof(analog_input_nine_voltage_));
        memcpy(&tx_msg4.buf[6], &analog_input_ten_voltage_, sizeof(analog_input_ten_voltage_));
        WriteCANToInverter(tx_msg3);
        WriteCANToInverter(tx_msg4);
    }
}

/**
 * @brief do rules required pedal plausibility checks
 * 
 * @param accel_is_plausible 
 * @param brake_is_plausible 
 * @param accel_and_brake_plausible 
 * @param impl_occ 
 */
void PedalHandler::verify_pedals(
    bool &accel_is_plausible, bool &brake_is_plausible,
    bool &accel_and_brake_plausible, bool &impl_occ)
{
    int max_torque = TORQUE_1 * 10;
    int torque1 =static_cast<int>(this->apps1.getTravelRatio() * max_torque);
    int torque2 =static_cast<int>(this->apps2.getTravelRatio() * max_torque);

    float torqSum = abs(torque1 - torque2);
    float torqAvg = (torque1 + torque2) / 2;
    float torqDiff = torqSum / torqAvg;

    if (accel1_ < MIN_ACCELERATOR_PEDAL_1 ||
        accel1_ > MAX_ACCELERATOR_PEDAL_1)
    {
        accel_is_plausible = false;
    }
    else if (accel2_ < MIN_ACCELERATOR_PEDAL_2 ||
             accel2_ > MAX_ACCELERATOR_PEDAL_2)
    {
        accel_is_plausible = false;
    }
    // check that the pedals are reading within 10% of each other TODO re-enabled 6/10/23, why was it commented out in the first place? how did we fix it before??
    // sum of the two readings should be within 10% of the average travel
    // T.4.2.4
    else if (torqDiff * 100 > APPS_ALLOWABLE_TRAVEL_DEVIATION)
    {
        accel_is_plausible = false;
    }
    else
    {
        accel_is_plausible = true;
    }
    // Serial.printf("Torque 1: %d Torque 2: %d Torque Sum: %f Torque Average %f Torque Difference: %f\n", torque1, torque2, torqSum, torqAvg, torqDiff);

    // BSE check
    // EV.5.6
    // FSAE T.4.3.4
    if (brake1_ < MIN_BRAKE_PEDAL || brake1_ > MAX_BRAKE_PEDAL)
    {
        brake_is_plausible = false;
    }
    else
    {
        brake_is_plausible = true;
    }

    // FSAE EV.5.7
    // APPS/Brake Pedal Plausability Check
    if (((accel1_ > ((END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1) / 4 +
                     START_ACCELERATOR_PEDAL_1)) ||
         (accel2_ > ((END_ACCELERATOR_PEDAL_2 - START_ACCELERATOR_PEDAL_2) / 4 +
                     START_ACCELERATOR_PEDAL_2))) &&
        brake_is_active_)
    {

        accel_and_brake_plausible = false;
    }
    else if ((accel1_ <
              ((END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1) / 20 +
               START_ACCELERATOR_PEDAL_1)) &&
             (accel2_ <
              ((END_ACCELERATOR_PEDAL_2 - START_ACCELERATOR_PEDAL_2) / 20 +
               START_ACCELERATOR_PEDAL_2)))
    {
        accel_and_brake_plausible = true;
        implausibility_occured_ = false; // only here do we want to reset this flag
    }
    else
    {
        accel_and_brake_plausible = true;
    }

    if ((!accel_and_brake_plausible) || (!brake_is_plausible) ||
        (!accel_is_plausible))
    {
        implausibility_occured_ = true;
    }

    impl_occ = implausibility_occured_;
#if DEBUG
    // Serial.printf("Implaus occured: %d accel plaus: %d brake plaus: %d accel and brake plaus: %d\n", implausibility_occured_, accel_is_plausible, brake_is_plausible, accel_and_brake_plausible);
#endif
}

// idgaf anything below (all wheel speed)
double PedalHandler::get_wsfr() { return wsfr_t.current_rpm; }
double PedalHandler::get_wsfl() { return wsfl_t.current_rpm; }
/**
 * @brief update wheel speed readings
 * 
 * @param current_time_millis 
 * @param ws 
 * @param freq 
 */
void PedalHandler::update_wheelspeed(unsigned long current_time_millis, wheelspeeds_t *ws, FreqMeasureMulti *freq){
    if ((current_time_millis - ws->current_rpm_change_time) > RPM_TIMEOUT) 
    { 
        ws->current_rpm = 0;
    }
    if (freq->available()){
        // average several reading together
        ws->sum = ws->sum + freq->read();
        ws->count = ws->count + 1;
        ws->current_rpm_change_time = millis();
        if (ws->count > 1)
        {
            float testRpm = freq->countToFrequency(ws->sum / ws->count) * 60 / WHEELSPEED_TOOTH_COUNT;
            ws->current_rpm = testRpm;

            /*if ( testRpm - prev_rpm < 1)
            {
              current_rpm = testRpm;
              prev_rpm = testRpm;
            }*/

            ws->sum = 0;
            ws->count = 0;
            // prev_rpm = testRpm;
        }

    }
}

void PedalHandler::ws_run()
{
    update_wheelspeed(millis(),&wsfl_t, wsfl_);
    update_wheelspeed(millis(),&wsfr_t, wsfr_);
}


// Returns true if BSPD is ok, false if not
bool PedalHandler::get_board_sensor_readings()
{
    glv_current_ = FILTERING_ALPHA_10HZ * glv_current_ + (1 - FILTERING_ALPHA_10HZ) * analogRead(GLV_ISENSE);
    glv_voltage_ = FILTERING_ALPHA_10HZ * glv_voltage_ + (1 - FILTERING_ALPHA_10HZ) * analogRead(GLV_VSENSE);
    bspd_voltage_ = FILTERING_ALPHA_10HZ * bspd_voltage_ + (1 - FILTERING_ALPHA_10HZ) * analogRead(BSPDSENSE);
    sdc_voltage_ = FILTERING_ALPHA_10HZ * sdc_voltage_ + (1 - FILTERING_ALPHA_10HZ) * analogRead(SDCVSENSE);
    sdc_current_ = FILTERING_ALPHA_10HZ * sdc_current_ + (1 - FILTERING_ALPHA_10HZ) * analogRead(SDCISENSE);
    vcc_voltage_ = FILTERING_ALPHA_10HZ * vcc_voltage_ + (1 - FILTERING_ALPHA_10HZ) * analogRead(_5V_VSENSE);
    analog_input_nine_voltage_ = FILTERING_ALPHA_10HZ * analog_input_nine_voltage_ + (1 - FILTERING_ALPHA_10HZ) * analogRead(A9);
    analog_input_ten_voltage_ = FILTERING_ALPHA_10HZ * analog_input_ten_voltage_ + (1 - FILTERING_ALPHA_10HZ) * analogRead(A10);
    return (bspd_voltage_ > BSPD_OK_HIGH_THRESHOLD);
}

// Use this to manipulate ADC values with a fake one
void PedalHandler::read_pedal_values_debug(uint16_t value)
{
    /* Filter ADC readings */

    // exponential smoothing https://chat.openai.com/share/cf98f2ea-d87c-4d25-a365-e398ffebf968
    // TODO - evaluate this FILTERING_ALPHA_10HZ value and if maybe we should decrease it
    accel1_ =
        FILTERING_ALPHA_10HZ * accel1_ + (1 - FILTERING_ALPHA_10HZ) * value;
    accel2_ =
        FILTERING_ALPHA_10HZ * accel2_ + (1 - FILTERING_ALPHA_10HZ) * value;
    brake1_ =
        FILTERING_ALPHA_10HZ * brake1_ + (1 - FILTERING_ALPHA_10HZ) * value;

    steering_angle_ =
        FILTERING_ALPHA_10HZ * steering_angle_ + (1 - FILTERING_ALPHA_10HZ) * value;

    if (debugPrint.check())
    {
#if DEBUG
        // Serial.print("APPS1 :");
        // Serial.println(accel1_);
        // Serial.print("APPS2 :");
        // Serial.println(accel2_);
        // Serial.print("BRAKE :");
        // Serial.println(brake1_);
        // Serial.print("STEERING_ANGLE :");
        // Serial.println(steering_angle_);
        // this->apps1.printValues();
        // this->apps2.printValues();
        // this->bse1.printValues();
        Serial.printf("%d\t%d\t%f\n",value,accel1_,this->apps1.getTravelRatio()*100);
        // Serial.printf("apps1: %f %f\n",this->apps1.getVoltage(),this->apps1.getTravelRatio());
        // Serial.printf("apps2: %f %f\n",this->apps2.getVoltage(),this->apps2.getTravelRatio());
        // Serial.printf("bse1: %f %f\n",this->bse1.getVoltage(),this->bse1.getTravelRatio());
#endif
    }
}