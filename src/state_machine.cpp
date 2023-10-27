#include "state_machine.hpp"
#define ACCDEBUG
//  initializes the mcu status and pedal handler
void StateMachine::init_state_machine(MCU_status &mcu_status)
{
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

  // exit logic ----------------------------------------------------------------------------------------------------------------------------------------------------
  switch (mcu_status.get_state())
  {
  case MCU_STATE::STARTUP: // ----------
  {
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE: // ----------
  {
#if USE_INVERTER
    pm100->tryToClearMcFault();
#endif
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE: // ----------
  {
    // accumulator->resetPchgState(); // dealing with sus behavior, precharge timed out but would stay "ready"
    break;
  }
  case MCU_STATE::ENABLING_INVERTER: // ----------
  {
    break;
  }
  case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND: // ----------
  {
    // make dashboard stop buzzer
    mcu_status.set_activate_buzzer(false);
    digitalWrite(BUZZER, LOW);
    break;
  }
  case MCU_STATE::READY_TO_DRIVE: // ----------
  {
    // reset "state" of precharge in memory
    // accumulator->resetPchgState();
    // disable lowside outputs (pump, etc.)
    digitalWrite(LOWSIDE1, LOW);
    digitalWrite(LOWSIDE2, LOW);

    break;
  }
  }

  mcu_status.set_state(new_state);

  // entry logic ----------------------------------------------------------------------------------------------------------------------------------------------------
  // TODO unfuck the other pixel setting
  switch (new_state)
  {
  case MCU_STATE::STARTUP: // ----------
  {
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE: // ----------
  {
#if USE_INVERTER
    pm100->forceMCdischarge();
#endif
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE: // ----------
  {

    break;
  }
  case MCU_STATE::ENABLING_INVERTER: // ----------
  {

#if USE_INVERTER
    pm100->tryToClearMcFault();

    pm100->doStartup();
#endif
    break;
  }
  case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND: // ----------
  {
    // make dashboard sound buzzer
    mcu_status.set_activate_buzzer(true);
    digitalWrite(BUZZER, HIGH);
    timer_ready_sound->reset();
    break;
  }
  case MCU_STATE::READY_TO_DRIVE: // ----------
  {
    // enable low-side outputs
    digitalWrite(LOWSIDE1, HIGH);
    digitalWrite(LOWSIDE2, HIGH);
    break;
  }
  }
}


// Constant logic ----------------------------------------------------------------------------------------------------------------------------------------------------
void StateMachine::handle_state_machine(MCU_status &mcu_status)
{
  // things that are done every loop go here:
  // TODO make getting analog readings neater--this is the only necessary one for now
  mcu_status.set_imd_ok_high(accumulator->get_imd_state());
  mcu_status.set_bms_ok_high(accumulator->get_bms_state());
  mcu_status.set_bspd_ok_high(true); // not reading this value so it defaults to off for now

#ifdef DEBUG
  // Serial.print("1: ");
  // Serial.print(dash_->get_button1());
  // Serial.print("2: ");
  // Serial.print(dash_->get_button2());
  // Serial.print("3: ");
  // Serial.print(dash_->get_button3());
  // Serial.print("4: ");
  // Serial.println(dash_->get_button4());
#endif


#if USE_INVERTER
  pm100->updateInverterCAN();
#endif
  accumulator->updateAccumulatorCAN();
  mcu_status.set_brake_pedal_active(pedals->read_pedal_values());
  dash_->updateDashCAN();
  pedals->get_ws();

  switch (mcu_status.get_state())
  {
  case MCU_STATE::STARTUP: // --------------------
  {
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE: // --------------------
  {
    // end of test block
#if USE_INVERTER
    pm100->inverter_kick(0);
#endif
    if (!accumulator->GetIfPrechargeAttempted())
    {
      accumulator->sendPrechargeStartMsg(); // we dont actually need to send this-precharge is automatic
    }

    bool accumulator_ready = false;

    // TODO might wanna check this out and make sure that this shit works, idk if it does
    if (accumulator->check_precharge_success() && (!accumulator->check_precharge_timeout()))
    {
      accumulator_ready = true;
    }
    else if ((!accumulator->check_precharge_timeout()) && (!accumulator->check_precharge_success()))
    {
      // if the accumulator hasnt finished precharge and it hasnt timed out yet, break

      break;
    }
    else if (accumulator->check_precharge_timeout() && (!accumulator->check_precharge_success()))
    {
      // attempt pre-charge again if the pre-charge has timed out
      // TODO add in re-try limitation number
      accumulator->sendPrechargeStartMsg();
      break;
    }
    // if TS is above HV threshold, move to Tractive System Active
#if USE_INVERTER

    if (pm100->check_TS_active() && accumulator_ready) // TODO somewhere here, dont allow TS active if a fault is known
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
    }
#else
    if (accumulator_ready) // TODO somewhere here, dont allow TS active if a fault is known
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
    }
#endif

    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE: // --------------------
  {

    // TODO (Disabled to test error 3/27/23)
#if USE_INVERTER
    if (!pm100->check_TS_active())
    {

      // set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE); //uncomet stage 1
    }
#else
    if (false) {} // dummy
#endif
    else if (accumulator->check_precharge_timeout()) // uncomet stage 1
    {

      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
    }
#if USE_INVERTER
    pm100->inverter_kick(0);
#endif
    // Serial.println(dash_->get_button3());
    // if start button has been pressed and brake pedal is held down, transition to the next state
    if (!dash_->get_button3() && mcu_status.get_brake_pedal_active())
    {
      set_state(mcu_status, MCU_STATE::ENABLING_INVERTER);
    }

    break;
  }
  case MCU_STATE::ENABLING_INVERTER: // --------------------
  {
#if USE_INVERTER
    pm100->inverter_kick(1);
    if (!pm100->check_TS_active())
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
      break;
    }
#else
    if(false){}
#endif
    else if (accumulator->check_precharge_timeout())
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
      break;
    }

    // inverter enabling timed out
#if USE_INVERTER
    bool tuff = pm100->check_inverter_enable_timeout();

    if (tuff) // this does something is inverter times out
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
      break;
    }

    // motor controller indicates that inverter has enabled within timeout period
    if (pm100->check_inverter_ready())

    {
      set_state(mcu_status, MCU_STATE::WAITING_READY_TO_DRIVE_SOUND);
      break;
    }
#else
  set_state(mcu_status, MCU_STATE::WAITING_READY_TO_DRIVE_SOUND);
#endif

    break;
  }
  case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND: // --------------------
  {
  #if USE_INVERTER
    pm100->inverter_kick(1);
    if (!pm100->check_TS_active())
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
      break;
    }
    if (pm100->check_inverter_disabled())
    {

      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
      break;
    }
#endif
    // if the ready to drive sound has been playing for long enough, move to ready to drive mode
    if (timer_ready_sound->check())
    {

      set_state(mcu_status, MCU_STATE::READY_TO_DRIVE);
    }
    break;
  }
  case MCU_STATE::READY_TO_DRIVE: // --------------------
  {
#if USE_INVERTER
    
    if (!pm100->check_TS_active())
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
      break;
    }


    if (pm100->check_inverter_disabled())
    {

      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
      break; // TODO idk if we should break here or not but it sure seems like it
    }
#endif
    if (accumulator->check_precharge_timeout())
    { // if the precharge hearbeat has timed out, we know it is no longer enabled-> the SDC is open
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

      int16_t motor_speed = 0;
#if USE_INVERTER
      motor_speed = pm100->getmcMotorRPM();
#endif
      calculated_torque = pedals->calculate_torque(motor_speed, max_t_actual, dash_->get_button2());
    }
    
#if USE_INVERTER
    pm100->command_torque(calculated_torque);
#endif
    break;
  }
  }
  // TODO update the dash here properly
}
