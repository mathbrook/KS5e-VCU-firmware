#ifndef __KS2e_H__
#define __KS2e_H__
#include <Arduino.h>
#include <ADC_SPI.h>
#include <FlexCAN_T4.h>
#include <Metro.h>
#include <MCU_status.h>
#include <SPI.h>
#include <string.h>
#include <stdint.h>
#include <KS2eCAN.h>
#include <KS2eVCUgpios.h>
#include <VCUNeoPixelBullshitLMFAO.h>
#include <Adafruit_MCP4725.h>
#include "Adafruit_LEDBackpack.h"
// #include <drivers.h>
//Pedalbox stuff
#define BRAKE_ACTIVE 2100               // Threshold for brake pedal active  
#define MIN_ACCELERATOR_PEDAL_1 850    // Low accelerator implausibility threshold
#define START_ACCELERATOR_PEDAL_1 914  // Position to start acceleration
#define END_ACCELERATOR_PEDAL_1 1660    // Position to max out acceleration
#define MAX_ACCELERATOR_PEDAL_1 1800    // High accelerator implausibility threshold
#define MIN_ACCELERATOR_PEDAL_2 600    // Low accelerator implausibility threshold
#define START_ACCELERATOR_PEDAL_2 630  // Position to start acceleration
#define END_ACCELERATOR_PEDAL_2 1100    // Position to max out acceleration
#define MAX_ACCELERATOR_PEDAL_2 1300    // High accelerator implausibility threshold
#define HALF_ACCELERATOR_PEDAL_1 ((START_ACCELERATOR_PEDAL_1 + END_ACCELERATOR_PEDAL_1)/2)
#define HALF_ACCELERATOR_PEDAL_2 ((START_ACCELERATOR_PEDAL_2 + END_ACCELERATOR_PEDAL_2)/2)
#define DRIVER Matthew
#define MIN_HV_VOLTAGE 600
#define HT_DEBUG_EN
//Torque Calculation Defines
#define ALPHA 0.9772
#define TORQUE_1 120
#define TORQUE_2 240
//Pump speed
#define PUMP_SPEED 2048
float accel1{},accel2{},brake1{},brake2{};
Adafruit_7segment DashDisplay = Adafruit_7segment();
ADC_SPI ADC(DEFAULT_SPI_CS, DEFAULT_SPI_SPEED);
Metro mcControlTimer=Metro(50);
Adafruit_MCP4725 dac;
class PM100Info{
    public:
    class MC_internal_states {
    public:
        MC_internal_states() = default;
        MC_internal_states(uint8_t buf[8]) { load(buf); }

        inline void load(uint8_t buf[])         { memcpy(this, buf, sizeof(*this)); }
        inline void write(uint8_t buf[])  const { memcpy(buf, this, sizeof(*this)); }

        inline uint8_t get_vsm_state()                          const { return vsm_state; }
        inline uint8_t get_inverter_state()                     const { return inverter_state; }
        inline bool get_relay_active_1()                        const { return relay_state & 0x01; } // @Parseflag(relay_state)
        inline bool get_relay_active_2()                        const { return relay_state & 0x02; } // @Parseflag(relay_state)
        inline bool get_relay_active_3()                        const { return relay_state & 0x04; } // @Parseflag(relay_state)
        inline bool get_relay_active_4()                        const { return relay_state & 0x08; } // @Parseflag(relay_state)
        inline bool get_relay_active_5()                        const { return relay_state & 0x10; } // @Parseflag(relay_state)
        inline bool get_relay_active_6()                        const { return relay_state & 0x20; } // @Parseflag(relay_state)
        inline bool get_inverter_run_mode()                     const { return inverter_run_mode_discharge_state & 1; }
        inline uint8_t get_inverter_active_discharge_state()    const { return inverter_run_mode_discharge_state >> 5; }
        inline bool get_inverter_command_mode()                 const { return inverter_command_mode; }
        inline bool get_inverter_enable_state()                 const { return inverter_enable & 1; }
        inline bool get_inverter_enable_lockout()               const { return inverter_enable & 0x80; }
        inline bool get_direction_command()                     const { return direction_command; }

    #ifdef HT_DEBUG_EN
        void print() {
        Serial.println("\n\nMC INTERNAL STATES");
        Serial.println(    "------------------");
        Serial.print("VSM STATE:                       ");  Serial.println(vsm_state, HEX);
        Serial.print("INVERTER STATE:                  ");  Serial.println(inverter_state, HEX);
        Serial.print("INVERTER RUN MODE:               ");  Serial.println(get_inverter_run_mode());
        Serial.print("INVERTER ACTIVE DISCHARGE STATE: ");  Serial.println(get_inverter_active_discharge_state());
        Serial.print("INVERTER COMMAND MODE:           ");  Serial.println(inverter_command_mode, HEX);
        Serial.print("INVERTER ENABLE:                 ");  Serial.println((uint32_t) get_inverter_enable_state());
        Serial.print("INVERTER LOCKOUT:                ");  Serial.println((uint32_t) get_inverter_enable_lockout());
        Serial.print("DIRECTION COMMAND:               ");  Serial.println(direction_command);
        }
    #endif

    private:
        uint16_t vsm_state;                         // @Parse @Hex
        uint8_t inverter_state;                     // @Parse @Hex
        uint8_t relay_state;                        // @Parse @Flagset
        uint8_t inverter_run_mode_discharge_state;  // @Parse @Flaglist(inverter_run_mode, inverter_active_discharge_state)
        uint8_t inverter_command_mode;              // @Parse @Hex
        uint8_t inverter_enable;                    // @Parse @Flaglist(inverter_enable_state, inverter_enable_lockout)
        uint8_t direction_command;                  // @Parse @Hex
    };
    class MC_voltage_information {
    public:
        MC_voltage_information() = default;
        MC_voltage_information(uint8_t buf[8]) { load(buf); }

        inline void load(uint8_t buf[]){ 
            memcpy(this, buf, sizeof(*this));
        }
        inline void write(uint8_t buf[])  const { memcpy(buf, this, sizeof(*this)); }

        inline int16_t get_dc_bus_voltage()   const { return dc_bus_voltage; }
        inline int16_t get_output_voltage()   const { return output_voltage; }
        inline int16_t get_phase_ab_voltage() const { return phase_ab_voltage; }
        inline int16_t get_phase_bc_voltage() const { return phase_bc_voltage; }

    #ifdef HT_DEBUG_EN
        void print() {
            Serial.println("\n\nMC VOLTAGE INFORMATION");
            Serial.println(    "----------------------");
            Serial.print("DC BUS VOLTAGE: ");   Serial.println(dc_bus_voltage / 10.0, 1);
            Serial.print("OUTPUT VOLTAGE: ");   Serial.println(output_voltage / 10.0, 1);
            Serial.print("PHASE AB VOLTAGE: "); Serial.println(phase_ab_voltage / 10.0, 1);
            Serial.print("PHASE BC VOLTAGE: "); Serial.println(phase_bc_voltage / 10.0, 1);
        }
    #endif

    private:
        int16_t dc_bus_voltage;     // @Parse @Scale(10) @Unit(V)
        int16_t output_voltage;     // @Parse @Scale(10) @Unit(V)
        int16_t phase_ab_voltage;   // @Parse @Scale(10) @Unit(V)
        int16_t phase_bc_voltage;   // @Parse @Scale(10) @Unit(V)
    };
            class MC_motor_position_information {
    public:
        MC_motor_position_information() = default;
        MC_motor_position_information(uint8_t buf[8]) { load(buf); }

        inline void load(uint8_t buf[])         { memcpy(this, buf, sizeof(*this)); }
        inline void write(uint8_t buf[])  const { memcpy(buf, this, sizeof(*this)); }

        inline int16_t get_motor_angle()                    const { return motor_angle; }
        inline int16_t get_motor_speed()                    const { return motor_speed; }
        inline int16_t get_electrical_output_frequency()    const { return electrical_output_frequency; }
        inline int16_t get_delta_resolver_filtered()        const { return delta_resolver_filtered; }

    #ifdef HT_DEBUG_EN
        void print() {
            Serial.println("\n\nMC Motor Position Information");
            Serial.println(    "-----------------------------");
            Serial.print("MOTOR ANGLE:         ");  Serial.println(motor_angle / 10.0, 1);
            Serial.print("MOTOR SPEED:         ");  Serial.println(motor_speed);
            Serial.print("ELEC OUTPUT FREQ:    ");  Serial.println(electrical_output_frequency);
            Serial.print("DELTA RESOLVER FILT: ");  Serial.println(delta_resolver_filtered);
        }
    #endif

    private:
        int16_t motor_angle;                    // @Parse @Scale(10)
        int16_t motor_speed;                    // @Parse @Unit(RPM)
        int16_t electrical_output_frequency;    // @Parse @Scale(10) @Name(elec_output_freq)
        int16_t delta_resolver_filtered;        // @Parse
    };
    class MC_temperatures_1 {
    public:
        MC_temperatures_1() = default;
        MC_temperatures_1(uint8_t buf[8]) { load(buf); }

        inline void load(uint8_t buf[])         { memcpy(this, buf, sizeof(*this)); }
        inline void write(uint8_t buf[])  const { memcpy(buf, this, sizeof(*this)); }

        inline int16_t get_module_a_temperature()           const { return module_a_temperature; }
        inline int16_t get_module_b_temperature()           const { return module_b_temperature; }
        inline int16_t get_module_c_temperature()           const { return module_c_temperature; }
        inline int16_t get_gate_driver_board_temperature()  const { return gate_driver_board_temperature; }

    #ifdef HT_DEBUG_EN
        void print() {
            Serial.println("\n\nMC TEMPERATURES 1");
            Serial.println(    "-----------------");
            Serial.print("MODULE A TEMP:          ");   Serial.println(module_a_temperature / 10.0, 1);
            Serial.print("MODULE B TEMP:          ");   Serial.println(module_b_temperature / 10.0, 1);
            Serial.print("MODULE C TEMP:          ");   Serial.println(module_c_temperature / 10.0, 1);
            Serial.print("GATE DRIVER BOARD TEMP: ");   Serial.println(gate_driver_board_temperature / 10.0, 1);
        }
    #endif

    private:
        int16_t module_a_temperature;           // @Parse @Scale(10) @Unit(C)
        int16_t module_b_temperature;           // @Parse @Scale(10) @Unit(C)
        int16_t module_c_temperature;           // @Parse @Scale(10) @Unit(C)
        int16_t gate_driver_board_temperature;  // @Parse @Scale(10) @Unit(C)
    };


};
#endif