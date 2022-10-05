#include "state_machine.hpp"
//#define ACCDEBUG
// initializes the mcu status and pedal handler
void StateMachine::init_state_machine(MCU_status &mcu_status)
{
  dash_->init_dashboard();                                      // do this before setting state
  set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE); // this is where it is failing rn
  pedals->init_pedal_handler();
}

/* Handle changes in state */
void StateMachine::set_state(MCU_status &mcu_status, MCU_STATE new_state)
{
  if (mcu_status.get_state() == new_state)
  {
    return;
  }

  // exit logic
  switch (mcu_status.get_state())
  {
  case MCU_STATE::STARTUP:
  {
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
  {
    pm100->tryToClearMcFault();
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE:
  {
    accumulator->resetPchgState(); // dealing with sus behavior, precharge timed out but would stay "ready"
    break;
  }
  case MCU_STATE::ENABLING_INVERTER:
  {
    break;
  }
  case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND:
  {
    // make dashboard stop buzzer
    mcu_status.set_activate_buzzer(false);
    digitalWrite(BUZZER, LOW);
    break;
  }
  case MCU_STATE::READY_TO_DRIVE:
  {
    accumulator->resetPchgState();
    break;
  }
  }

  mcu_status.set_state(new_state);

  // entry logic
  // TODO unfuck the other pixel setting
  switch (new_state)
  {
  case MCU_STATE::STARTUP:
  {
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
  {
    pm100->forceMCdischarge();
    dash_->set_dashboard_led_color(GREEN); // this is causing problems rn
    dash_->DashLedscolorWipe();
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE:
  {
    Serial.println("going into tractive sustem active");
    dash_->set_dashboard_led_color(RED);
    dash_->DashLedscolorWipe();
    break;
  }
  case MCU_STATE::ENABLING_INVERTER:
  {
    Serial.println("going into enabling inverter");
    dash_->set_dashboard_led_color(YELLOW);
    dash_->DashLedscolorWipe();
    pm100->tryToClearMcFault();
    pm100->doStartup();
    break;
  }
  case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND:
  {
    // make dashboard sound buzzer
    dash_->set_dashboard_led_color(ORANGE);
    dash_->DashLedscolorWipe();
    mcu_status.set_activate_buzzer(true);
    digitalWrite(BUZZER, HIGH);
    timer_ready_sound->reset();
#if DEBUG
    Serial.println("RTDS enabled");
#endif
    break;
  case MCU_STATE::READY_TO_DRIVE:
    dash_->set_dashboard_led_color(PINK);
    dash_->DashLedscolorWipe();
#if DEBUG
    Serial.println("Ready to drive");
#endif
    break;
  }
  }
}

void StateMachine::handle_state_machine(MCU_status &mcu_status)
{
  // things that are done every loop go here:
  pm100->updateInverterCAN();
  accumulator->updateAccumulatorCAN();
  mcu_status.set_brake_pedal_active(pedals->read_pedal_values());
  switch (mcu_status.get_state())
  {
  case MCU_STATE::STARTUP:
  {
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
  {
    //delete this later, testing when not in TS mode
    uint8_t max_t = mcu_status.get_max_torque();
    int max_t_actual = max_t * 10;
    int16_t motor_speed = pm100->getmcMotorRPM();
    pedals->calculate_torque(motor_speed, max_t_actual);
    // end of test block
    pm100->inverter_kick(0);
    if (!accumulator->GetIfPrechargeAttempted())
    {
      accumulator->sendPrechargeStartMsg(); // we dont actually need to send this-precharge is automatic
#ifdef ACCDEBUG
      Serial.println("Sent precharge start msg");
#endif
    }

    bool accumulator_ready = false;
    // TODO might wanna check this out and make sure that this shit works, idk if it does
    if (accumulator->check_precharge_success() && (!accumulator->check_precharge_timeout()))
    {
      accumulator_ready = true;
#ifdef ACCDEBUG
// Serial.println("Precharge is ready and not timed out");
#endif
    }
    else if ((!accumulator->check_precharge_timeout()) && (!accumulator->check_precharge_success()))
    {
// if the accumulator hasnt finished precharge and it hasnt timed out yet, break
#ifdef ACCDEBUG
      Serial.println("Precharge not ready yet");
      Serial.println(accumulator_ready);
#endif
      break;
    }
    else if (accumulator->check_precharge_timeout() && (!accumulator->check_precharge_success()))
    {
// attempt pre-charge again if the pre-charge has timed out
// TODO add in re-try limitation number
// accumulator->sendPrechargeStartMsg();
#ifdef ACCDEBUG
      Serial.println("Sent precharge start msg AGAIN");
      Serial.println(accumulator_ready);
#endif
      break;
    }
    // else
    // {
    //   // accumulator has timed out but it also pre-charged, continue
    //   accumulator_ready = true;
    // }
    // if TS is above HV threshold, move to Tractive System Active
    if (pm100->check_TS_active() && accumulator_ready)
    {
#if DEBUGF
      Serial.println("Setting state to TS Active from TS Not Active");
#endif
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
    }
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE:
  {
#if DEBUG

    // Serial.println("TRACTIVE_SYSTEM_ACTIVE");
    // Serial.printf("button state: %i, pedal active %i\n", digitalRead(RTDbutton),
    // mcu_status.get_brake_pedal_active());
#endif
    if (!pm100->check_TS_active())
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
    }
    else if (accumulator->check_precharge_timeout())
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
    }
    pm100->inverter_kick(0);

    // if start button has been pressed and brake pedal is held down, transition to the next state
    if (digitalRead(RTDbutton) == 0 && mcu_status.get_brake_pedal_active())
    {
#if DEBUG
      Serial.println("Setting state to Enabling Inverter");
#endif
      set_state(mcu_status, MCU_STATE::ENABLING_INVERTER);
    }
    break;
  }
  case MCU_STATE::ENABLING_INVERTER:
  {
    // Serial.println("ENABLING INVERTER");
    pm100->inverter_kick(1);
    if (!pm100->check_TS_active())
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
      break;
    }
    else if (accumulator->check_precharge_timeout())
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
      break;
    }

    // inverter enabling timed out
    bool tuff = pm100->check_inverter_enable_timeout();

    if (tuff)
    {
      Serial.println("Setting state to TS Active from Enabling Inverter");
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
      break;
    }

    // motor controller indicates that inverter has enabled within timeout period
    if (pm100->check_inverter_ready())
    {
      Serial.println("Setting state to Waiting Ready to Drive Sound");
      set_state(mcu_status, MCU_STATE::WAITING_READY_TO_DRIVE_SOUND);
      break;
    }
    break;
  }
  case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND:
  {
    pm100->inverter_kick(1);
    if (!pm100->check_TS_active())
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
      break;
    }
    if (pm100->check_inverter_disabled())
    {
      Serial.println("daw, why you disabling");
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
      break;
    }

    // if the ready to drive sound has been playing for long enough, move to ready to drive mode
    if (timer_ready_sound->check())
    {
      Serial.println("Setting state to Ready to Drive");
      set_state(mcu_status, MCU_STATE::READY_TO_DRIVE);
    }
    break;
  }
  case MCU_STATE::READY_TO_DRIVE:
  {
    // pm100->inverter_kick(1);
    if (!pm100->check_TS_active())
    {
      Serial.println("Found TS No Longer Active");
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
      break;
    }

    if (pm100->check_inverter_disabled())
    {
      Serial.println("Found Inverter No Longer Active");
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
      break; // TODO idk if we should break here or not but it sure seems like it
    }
    if (accumulator->check_precharge_timeout())
    { // if the precharge hearbeat has timed out, we know it is no longer enabled-> the SDC is open
      Serial.println("Found Precharge No Longer Active");
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
      break;
    }

    int calculated_torque = 0;
    bool accel_is_plausible = false;
    bool brake_is_plausible = false;
    bool accel_and_brake_plausible = false;
    bool impl_occ = true;

    // FSAE EV.5.5
    // FSAE T.4.2.10
    pedals->verify_pedals(accel_is_plausible, brake_is_plausible, accel_and_brake_plausible, impl_occ);
    mcu_status.set_no_accel_implausability(!accel_is_plausible);
    mcu_status.set_no_brake_implausability(!brake_is_plausible);
    mcu_status.set_no_accel_brake_implausability(!accel_and_brake_plausible);

    if (accel_is_plausible && brake_is_plausible && accel_and_brake_plausible && (!impl_occ))
    {
      uint8_t max_t = mcu_status.get_max_torque();
      int max_t_actual = max_t * 10;

      int16_t motor_speed = pm100->getmcMotorRPM();
      calculated_torque = pedals->calculate_torque(motor_speed, max_t_actual);
    }
    // if (true)
    // {
    //   uint8_t max_t = mcu_status.get_max_torque();
    //   int max_t_actual = max_t * 10;

    //   int16_t motor_speed = pm100->getmcMotorRPM();
    //   calculated_torque = pedals->calculate_torque(motor_speed, max_t_actual);
    // }
    else
    {
      // Serial.println("not calculating torque");
      // Serial.print("implausibility occured: ");
      // Serial.println(impl_occ);
      // Serial.printf("no brake implausibility: %d\n", mcu_status.get_no_brake_implausability());
      // Serial.printf("no accel implausibility: %d\n", mcu_status.get_no_accel_implausability());
      // Serial.printf("no accel brake implausibility: %d\n", mcu_status.get_no_accel_brake_implausability());
    }
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
    
      pm100->command_torque(calculated_torque);

    break;
  }
  }

  if (debug_->check())
  {
    // Serial.printf("button state: %i, pedal active %i\n", digitalRead(RTDbutton),
    // mcu_status.get_brake_pedal_active());
    if(tempdisplay_>=1){
      dash_->refresh_dash(mcu_status.get_max_torque());
      tempdisplay_--; //this is so dumb
    }else{
      dash_->refresh_dash(pm100->getmcBusVoltage());
    }

    //pm100->debug_print();
    switch(digitalRead(TORQUEMODE)){
      case 0:{
        if(TORQUE_1!=mcu_status.get_max_torque()){
          tempdisplay_=10;
        }
        #if DEBUG
        #endif
        mcu_status.set_max_torque(TORQUE_1);
        break;
      }
      case 1:{
        if(TORQUE_2!=mcu_status.get_max_torque()){
          tempdisplay_=10;
        }
        #if DEBUG
        #endif
        mcu_status.set_max_torque(TORQUE_2);
        break;
      }
    }
    dash_->DashLedsBrightness();
  }
  // TODO update the dash here properly
}