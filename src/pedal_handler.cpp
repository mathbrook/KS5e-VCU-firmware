#include "pedal_handler.hpp"

// initializes pedal's ADC
void PedalHandler::init_pedal_handler()
{
    pedal_ADC = ADC_SPI(DEFAULT_SPI_CS, DEFAULT_SPI_SPEED);
}

int PedalHandler::calculate_torque(int16_t &motor_speed, int &max_torque)
{
    int calculated_torque = 0;
    int torque1 = map(round(accel1_), START_ACCELERATOR_PEDAL_1, END_ACCELERATOR_PEDAL_1, 0, max_torque);
    int torque2 = map(round(accel2_), START_ACCELERATOR_PEDAL_2, END_ACCELERATOR_PEDAL_2, 0, max_torque);

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
    if (timer_debug_raw_torque->check())
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
        // Serial.println(accel1_);
        // Serial.print("Accel 2: ");
        // Serial.println(accel2_);
        // Serial.print("Brake1_ : ");
        // Serial.println(brake1_);
    }
    if (abs(motor_speed) <= 50)
    {
        if (calculated_torque >= 600)
        {
            calculated_torque = 600; // ideally limit torque at low RPMs, see how high this number can be raised
        }
    }
    //#endif
    return calculated_torque;
}

// returns true if the brake pedal is active
bool PedalHandler::read_pedal_values()
{
    /* Filter ADC readings */
    float raw_brake = pedal_ADC.read_adc(ADC_BRAKE_1_CHANNEL);
    float raw_accel1 = pedal_ADC.read_adc(ADC_ACCEL_1_CHANNEL);
    float raw_accel2 = pedal_ADC.read_adc(ADC_ACCEL_2_CHANNEL);

    // accel1_ = ALPHA * accel1_ + (1 - ALPHA) * raw_accel1;
    // accel2_ = ALPHA * accel2_ + (1 - ALPHA) * raw_accel2;
    // brake1_ = ALPHA * brake1_ + (1 - ALPHA) * raw_brake;
    accel1_ = raw_accel1;
    accel2_ = raw_accel2;
    brake1_ = raw_brake;


#if DEBUG
    // Serial.println("reading pedal vals");
    // Serial.printf("val %f\n", brake1_);
    // Serial.printf("raw val %f\n", raw_brake);
    
    // if (timer_debug_raw_torque->check())
    // {
        // Serial.print("ACCEL 1: ");
        // Serial.println(accel1_);
        // Serial.print("ACCEL 2: ");
        // Serial.println(accel2_);
        // Serial.print("BRAKE 1: ");
        // Serial.println(brake1_);
  //  }
#endif
    VCUPedalReadings.set_accelerator_pedal_1(accel1_);
    VCUPedalReadings.set_accelerator_pedal_2(accel2_);
    VCUPedalReadings.set_brake_transducer_1(brake1_);
    VCUPedalReadings.set_brake_transducer_2(brake1_);
    CAN_message_t tx_msg;

    // Send Main Control Unit pedal reading message
    VCUPedalReadings.write(tx_msg.buf);
    tx_msg.id = ID_VCU_PEDAL_READINGS;
    tx_msg.len = sizeof(VCUPedalReadings);

    // write out the actual accel command over CAN
    if (pedal_out->check())
    {
        WriteToDaqCAN(tx_msg);
    }
    // only uses front brake pedal
    brake_is_active_ = (brake1_ >= BRAKE_ACTIVE);
    return brake_is_active_;
}

void PedalHandler::verify_pedals(bool &accel_is_plausible, bool &brake_is_plausible, bool &accel_and_brake_plausible)
{

    if (accel1_ < MIN_ACCELERATOR_PEDAL_1 || accel1_ > MAX_ACCELERATOR_PEDAL_1)
    {
        accel_is_plausible = false;
#if DEBUG
        // Serial.println("T.4.2.10 1");
        // Serial.println(accel1_);
#endif
    }
    else if (accel2_ < MIN_ACCELERATOR_PEDAL_2 || accel2_ > MAX_ACCELERATOR_PEDAL_2)
    {
        accel_is_plausible = false;
#if DEBUG
        // Serial.println("T.4.2.10 2");
        // Serial.println(accel2_);
#endif
    }
    // check that the pedals are reading within 10% of each other
    // sum of the two readings should be within 10% of the average travel
    // T.4.2.4
    else if ((accel1_ - (4096 - accel2_)) >
             (END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1 + START_ACCELERATOR_PEDAL_2 - END_ACCELERATOR_PEDAL_2) / 20)
    {
#if DEBUG
        // Serial.println("T.4.2.4");
        // Serial.printf("computed - %f\n", accel1_ - (4096 - accel2_));
        // Serial.printf("standard - %d\n", (END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1 + START_ACCELERATOR_PEDAL_2 - END_ACCELERATOR_PEDAL_2) / 20);
#endif
        accel_is_plausible = false;
    }
    else
    {
#if DEBUG
        // Serial.println("T.4.2.4");
        // Serial.printf("computed - %f\n", accel1_ - (4096 - accel2_));
        // Serial.printf("standard - %d\n", (END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1 + START_ACCELERATOR_PEDAL_2 - END_ACCELERATOR_PEDAL_2) / 20);
#endif
        // mcu_status.set_no_accel_implausability(true);
        accel_is_plausible = true;
    }

    // BSE check
    // EV.5.6
    // FSAE T.4.3.4
    if (brake1_ < 200 || brake1_ > 4000)
    {
        brake_is_plausible = false;
    }
    else
    {
        brake_is_plausible = true;
    }

    // FSAE EV.5.7
    // APPS/Brake Pedal Plausability Check
    if (
        (
            (accel1_ > ((END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1) / 4 + START_ACCELERATOR_PEDAL_1)) ||
            (accel2_ > ((END_ACCELERATOR_PEDAL_2 - START_ACCELERATOR_PEDAL_2) / 4 + START_ACCELERATOR_PEDAL_2))) &&
        brake_is_active_)
    {
        Serial.println("dawg, dont press em both");
        accel_and_brake_plausible = false;
    }
    else if (
        (accel1_ < ((END_ACCELERATOR_PEDAL_1 - START_ACCELERATOR_PEDAL_1) / 20 + START_ACCELERATOR_PEDAL_1)) &&
        (accel2_ < ((END_ACCELERATOR_PEDAL_2 - START_ACCELERATOR_PEDAL_2) / 20 + START_ACCELERATOR_PEDAL_2)))
    {
        accel_and_brake_plausible = true;
    }else
    {
        accel_and_brake_plausible=true;
    }

}