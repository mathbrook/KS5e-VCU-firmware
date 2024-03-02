#include "state_machine.hpp"
#define ACCDEBUG
//  initializes the mcu status and pedal handler
void StateMachine::init_state_machine(MCU_status &mcu_status)
{
  set_state(mcu_status, MCU_STATE::STARTUP);
  pedals->init_pedal_handler();
}

// Send a state message on every state transition so we don't miss any
void StateMachine::send_state_msg(MCU_status &mcu_status)
{
  CAN_message_t tx_msg;
  mcu_status.write(tx_msg.buf);
  tx_msg.id = ID_VCU_STATUS;
  tx_msg.len = sizeof(mcu_status);
  WriteCANToInverter(tx_msg);
}
/* Handle changes in state */
void StateMachine::set_state(MCU_status &mcu_status, MCU_STATE new_state)
{
  // Send current mcu_status before state transition
  send_state_msg(mcu_status);
  // If current state is the same as new state, exit
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
    // Reset precharge status when exit of TS ACTIVE
    accumulator->resetPchgState(); // precharge will time out but stay "ready" if we don't reset it here
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
    accumulator->resetPchgState();
    // disable lowside outputs (pump, etc.)
    digitalWrite(LOWSIDE1, LOW);
    digitalWrite(LOWSIDE2, LOW);

    break;
  }
  }

  mcu_status.set_state(new_state);
  // Send new mcu_status after transition
  send_state_msg(mcu_status);
  // entry logic ----------------------------------------------------------------------------------------------------------------------------------------------------
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

#if USE_INVERTER
  pm100->updateInverterCAN();
#endif
  accumulator->updateAccumulatorCAN();
  // dash_->updateDashCAN(); Reading inverter CAN and "dash can" at the same time
  // causes a lot of delay. DOn't do it

  mcu_status.set_brake_pedal_active(pedals->read_pedal_values());
  pedals->run_pedals(); // Maybe wrap all of this in a "pedals superloop" function
  pedals->ws_run();
  mcu_status.set_imd_ok_high(accumulator->get_imd_state());
  mcu_status.set_bms_ok_high(accumulator->get_bms_state());
  mcu_status.set_bspd_ok_high(pedals->get_board_sensor_readings());
  mcu_status.set_bspd_current_high((accumulator->get_acc_current() > (bspd_current_high_threshold * 10)));

  pedals->send_readings();

  // If dash button is on and has been on for 
  if (dash_->get_button(6) && (dash_->get_button_last_pressed_time(6)) > 750)
  {
    dash_->set_button_last_pressed_time(0,6);
    mcu_status.toggle_max_torque(mcu_status.get_torque_mode());
    mcu_status.set_max_torque(torque_mode_list[mcu_status.get_torque_mode() - 1]);
    send_state_msg(mcu_status);
  }
  // Do Torque Calcs here
  int calculated_torque = 0;
  bool accel_is_plausible = false;
  bool brake_is_plausible = false;
  bool accel_and_brake_plausible = false;
  bool impl_occ = true;

  // FSAE EV.5.5
  // FSAE T.4.2.10
  pedals->verify_pedals(accel_is_plausible, brake_is_plausible, accel_and_brake_plausible, impl_occ);
  mcu_status.set_accel_implausible(!accel_is_plausible);
  mcu_status.set_brake_implausible(!brake_is_plausible);
  mcu_status.set_accel_brake_implausible(!accel_and_brake_plausible);

  if (accel_is_plausible && brake_is_plausible && accel_and_brake_plausible && (!impl_occ))
  {

    int max_t_actual = mcu_status.get_max_torque() * 10;

    int16_t motor_speed = 0;
#if USE_INVERTER
    motor_speed = pm100->getmcMotorRPM();
#endif
    calculated_torque = pedals->calculate_torque(motor_speed, max_t_actual);

    if (mcu_status.get_brake_pedal_active() && dash_->get_button2() && calculated_torque < 5)
    {
      calculated_torque = pedals->calculate_regen(motor_speed, REGEN_NM);
    }
  }

  // end of functions that run every loop unconditionally

  // start of state machine conditional functionality
  switch (mcu_status.get_state())
  {
  case MCU_STATE::STARTUP: // --------------------
  {
    set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE: // --------------------
  {
#if USE_INVERTER
    pm100->inverter_kick(0);
#endif
    if (!accumulator->GetIfPrechargeAttempted())
    {
      accumulator->sendPrechargeStartMsg(); // we dont actually need to send this-precharge is automatic
    }

    bool accumulator_ready = false;

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

    if (pm100->check_TS_active() && accumulator_ready)
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
    }
#else
    if (accumulator_ready)
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
    }
#endif

    break;
  }
  // TRACTIVE SYSTEM ACTIVE, HV ON
  case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE: // --------------------
  {

    // TODO (Disabled to test error 3/27/23)
#if USE_INVERTER
    if (!pm100->check_TS_active())
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE); // uncomet stage 1
    }
#else
    if (false)
    {
    } // dummy
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
  // ATTEMPTING TO ENABLE INVERTER
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
    if (false)
    {
    }
#endif
    else if (accumulator->check_precharge_timeout())
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
      break;
    }

    // inverter enabling timed out
#if USE_INVERTER
    bool tuff = pm100->check_inverter_enable_timeout();

    if (tuff) // if inverter times out , go from ENABLING_INVERTER back to TRACTIVE_SYSTEM_ACTIVE
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
  // WAITING TO ENTER READY TO DRIVE (BUZZER ACTIVE)
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
  // READY TO DRIVE
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

    // Torque calc always runs in the superloop

    if (mcu_status.get_launch_ctrl_active())
    {
      // Do launch control things
    }
#if USE_INVERTER
    pm100->calc_and_send_current_limit(pm100->getmcBusVoltage(), DISCHARGE_POWER_LIM, CHARGE_POWER_LIM);
    pm100->command_torque(calculated_torque);
#endif
    break;
  }
  }
#if DEBUG
  if (debug_->check())
  {

    // int16_t motor_speed = 1000;
    // int max_torque = 1000;
    // pedals->calculate_torque(motor_speed, max_torque); // Just to print the calced value and confirm it's right
    // Put debug prints here if/when needed
    pm100->debug_print();
    accumulator->acc_debug_print();
    Serial.printf("\tDASH BUTTONS \nONE: %d TWO: %d THREE: %d FOUR: %d FIVE: %d SIX: %d\n", dash_->get_button1(), dash_->get_button2(), dash_->get_button3(), dash_->get_button4(), dash_->get_button5(), dash_->get_button6());
  }
#endif
}
