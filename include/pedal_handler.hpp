#ifndef PEDAL_HANDLER_HPP
#define PEDAL_HANDLER_HPP
#include <Arduino.h>
#include <ADC_SPI.h>
#include <SPI.h>
#include <Metro.h>
#include <stdint.h>
#include <AutoPID.h>
#include <FreqMeasureMulti.h>
#include "pedal_sensor.hpp"

#include "pedal_readings.hpp"
#include "parameters.hpp"
#include "KS2eVCUgpios.hpp"
#include "FlexCAN_util.hpp"

// check that the pedals are reading within 10% of each other
// sum of the two readings should be within 10% of the average travel
// T.4.2.4

// BSE check
// EV.5.6
// FSAE T.4.3.4

// FSAE EV.5.7
// APPS/Brake Pedal Plausability Check

typedef struct wheelspeeds_t
{
    float current_rpm = 0;
    float prev_rpm;
    unsigned long current_rpm_change_time;
    int sum = 0;
    int count = 0;
    unsigned long time1;
} wheelspeeds_t;
class PedalHandler
{
private:
    Metro *timer_debug_raw_torque;
    Metro *pedal_out_20hz;
    Metro *pedal_out_1hz;
    bool brake_is_active_;

    ADC_SPI pedal_ADC;
    uint16_t accel1_{}, accel2_{}, brake1_{}, steering_angle_;
    bool implausibility_occured_;

    uint16_t sdc_voltage_{}, sdc_current_{}, bspd_voltage_{}, glv_voltage_{}, glv_current_{}, vcc_voltage_{}, analog_input_nine_voltage_{}, analog_input_ten_voltage_{};

    AutoPID *pid_;
    double *current_;
    double *set_;
    double *throttle_;


    // Wheel speed measuring class objects
    FreqMeasureMulti *wsfl_;
    FreqMeasureMulti *wsfr_;
    wheelspeeds_t wsfl_t;
    wheelspeeds_t wsfr_t;
    
    pedalSensor apps1;
    pedalSensor apps2;
    pedalSensor bse1;

    int16_t smoothed_regen_torque = 0;


public:
    PedalHandler(Metro *pedal_debug_tim, Metro *_pedal_out_20hz, Metro *_pedal_out_1hz,AutoPID *pid, double *current, double *set, double *throttle, FreqMeasureMulti *wsfl, FreqMeasureMulti *wsfr) 
    : timer_debug_raw_torque(pedal_debug_tim), pedal_out_20hz(_pedal_out_20hz), pedal_out_1hz(_pedal_out_1hz) ,pid_(pid), current_(current), set_(set), throttle_(throttle), wsfl_(wsfl), wsfr_(wsfr), 
    apps1(accel1_, MIN_ACCELERATOR_PEDAL_1, MAX_ACCELERATOR_PEDAL_1, START_ACCELERATOR_PEDAL_1, END_ACCELERATOR_PEDAL_1, 0.0f, 0.001220703125f, 0.0f), 
    apps2(accel2_, MIN_ACCELERATOR_PEDAL_2, MAX_ACCELERATOR_PEDAL_2, START_ACCELERATOR_PEDAL_2, END_ACCELERATOR_PEDAL_2, 0.0f, 0.00080586080586081f, 0.0f), 
    bse1(brake1_, MIN_BRAKE_PEDAL, MAX_BRAKE_PEDAL, START_BRAKE_PEDAL, END_BRAKE_PEDAL, 0.0f, 0.001220703125f, 0.0f){};
    void init_pedal_handler();
    MCU_pedal_readings VCUPedalReadings;

    int16_t calculate_regen(int16_t &motor_speed, int16_t max_regen_torque);
    int16_t calculate_torque(int16_t &motor_speed, int &max_torque);
    void verify_pedals(bool &accel_is_plausible, bool &brake_is_plausible, bool &accel_and_brake_plausible, bool &impl_occ);
    bool read_pedal_values();
    void run_pedals();
    void ws_run();
    void update_wheelspeed(unsigned long current_time_millis, wheelspeeds_t *ws, FreqMeasureMulti *freq);
    double get_wsfl();
    double get_wsfr();
    void send_readings();
    bool get_board_sensor_readings();
    void read_pedal_values_debug(uint16_t value);
};

typedef struct pedal_thresholds_0_t
{
    // const uint8_t mux = 0;
    const uint16_t brake_active_threshold = BRAKE_ACTIVE;
    const uint16_t apps1_uv_threshold = MIN_ACCELERATOR_PEDAL_1;
    const uint16_t apps1_start_threshold = START_ACCELERATOR_PEDAL_1;
} pedal_thresholds_0_t;

typedef struct pedal_thresholds_1_t
{
    // const uint8_t mux = 1;
    const uint16_t apps1_end_threshold = END_ACCELERATOR_PEDAL_1;
    const uint16_t apps1_ov_threshold = MAX_ACCELERATOR_PEDAL_1;
    const uint16_t apps2_uv_threshold = MIN_ACCELERATOR_PEDAL_2;
} pedal_thresholds_1_t;

typedef struct pedal_thresholds_2_t
{
    // const uint8_t mux = 2;
    const uint16_t apps2_start_threshold = START_ACCELERATOR_PEDAL_2;
    const uint16_t apps2_end_threshold = END_ACCELERATOR_PEDAL_2;
    const uint16_t apps2_ov_threshold = MAX_ACCELERATOR_PEDAL_2;
} pedal_thresholds_2_t;

typedef struct pedal_thresholds_t
{
    const pedal_thresholds_0_t pedal_thresholds_0;
    const pedal_thresholds_1_t pedal_thresholds_1;
    const pedal_thresholds_2_t pedal_thresholds_2;
} pedal_thresholds_t;
#endif