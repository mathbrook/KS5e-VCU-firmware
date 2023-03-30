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
  
  Serial.print("new state: ");
  Serial.println(static_cast<int>(new_state));
  
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
    // accumulator->resetPchgState(); // dealing with sus behavior, precharge timed out but would stay "ready"
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
    // reset "state" of precharge in memory
    // accumulator->resetPchgState();
    // disable lowside outputs (pump, etc.)
    digitalWrite(LOWSIDE1, LOW);
    digitalWrite(LOWSIDE2, LOW);

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

    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE:
  {

    break;
  }
  case MCU_STATE::ENABLING_INVERTER:
  {

    pm100->tryToClearMcFault();
    pm100->doStartup();
    break;
  }
  case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND:
  {
    // make dashboard sound buzzer
    mcu_status.set_activate_buzzer(true);
    digitalWrite(BUZZER, HIGH);
    timer_ready_sound->reset();
    break;
  }
  case MCU_STATE::READY_TO_DRIVE:
  {
    // enable low-side outputs
    digitalWrite(LOWSIDE1, HIGH);
    digitalWrite(LOWSIDE2, HIGH);
    break;
  }
  }
}

void StateMachine::handle_state_machine(MCU_status &mcu_status)
{
  Serial.print("current state: ");
  MCU_STATE yeet = mcu_status.get_state();
  Serial.println(static_cast<int>(yeet));
  // things that are done every loop go here:
  
  //TODO make getting analog readings neater--this is the only necessary one for now
  mcu_status.set_bms_ok_high(true); // TODO BODGE TESTING, confirmed working 3/28/23, false = light ON, true = light OFF
  mcu_status.set_bspd_ok_high(true);
  mcu_status.set_imd_ok_high(true);

  pm100->updateInverterCAN();
  accumulator->updateAccumulatorCAN();
  mcu_status.set_brake_pedal_active(pedals->read_pedal_values());
  dash_->updateDashCAN();
  pedals->get_ws();
  switch (mcu_status.get_state())
  {
  case MCU_STATE::STARTUP:
  {
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
  {
    // delete this later, testing when not in TS mode
    uint8_t max_t = mcu_status.get_max_torque();
    int max_t_actual = max_t * 10;
    int16_t motor_speed = pm100->getmcMotorRPM();
    pedals->calculate_torque(motor_speed, max_t_actual);

    // end of test block
    pm100->inverter_kick(0);
    if (!accumulator->GetIfPrechargeAttempted())
    {
      accumulator->sendPrechargeStartMsg(); // we dont actually need to send this-precharge is automatic
    }

    bool accumulator_ready = false;

    // *we are ok up to here*

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
    Serial.print("check_TS_active?: ");
    Serial.println(pm100->check_TS_active());
    if (pm100->check_TS_active() && accumulator_ready) // TODO somewhere here, dont allow TS active if a fault is known
    {

      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
    }

    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE:
  {

    // TODO (Disabled to test error 3/27/23)

    if (!pm100->check_TS_active())
    {

      // set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE); //uncomet stage 1
    }
    else if (accumulator->check_precharge_timeout()) // uncomet stage 1
    {

      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
    }
    pm100->inverter_kick(0);

    // if start button has been pressed and brake pedal is held down, transition to the next state
    if (dash_->get_button1() && mcu_status.get_brake_pedal_active())
    {
      set_state(mcu_status, MCU_STATE::ENABLING_INVERTER);
    }

    break;
  }
  case MCU_STATE::ENABLING_INVERTER:
  {

    
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

      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
      break;
    }

    // if the ready to drive sound has been playing for long enough, move to ready to drive mode
    if (timer_ready_sound->check())
    {

      set_state(mcu_status, MCU_STATE::READY_TO_DRIVE);
    }
    break;
  }
  case MCU_STATE::READY_TO_DRIVE:
  {
    // pm100->inverter_kick(1);
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
    }

    pm100->command_torque(calculated_torque);

    break;
  }
  }

  if (debug_->check())
  {
    // if (dash_->get_button2())
    // {
    //   mcu_status.toggle_max_torque(mcu_status.get_torque_mode());
    //   mcu_status.set_max_torque(60 * mcu_status.get_torque_mode());
    // }
    // uint16_t rpm_wsfl = (int)(pedals->get_wsfl()*100);
    // uint16_t rpm_wsfr = (int)(pedals->get_wsfr()*100);

    //  mcu_status.get_brake_pedal_active());

    // pm100->debug_print();
  }
  // TODO update the dash here properly
}