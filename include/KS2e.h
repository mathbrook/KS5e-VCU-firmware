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
#define BRAKE_ACTIVE 2300               // Threshold for brake pedal active  
#define MIN_ACCELERATOR_PEDAL_1 200    // Low accelerator implausibility threshold
#define START_ACCELERATOR_PEDAL_1 904  // Position to start acceleration
#define END_ACCELERATOR_PEDAL_1 1820    // Position to max out acceleration
#define MAX_ACCELERATOR_PEDAL_1 2500    // High accelerator implausibility threshold
#define MIN_ACCELERATOR_PEDAL_2 200    // Low accelerator implausibility threshold
#define START_ACCELERATOR_PEDAL_2 620  // Position to start acceleration
#define END_ACCELERATOR_PEDAL_2 1250    // Position to max out acceleration
#define MAX_ACCELERATOR_PEDAL_2 2000    // High accelerator implausibility threshold
#define HALF_ACCELERATOR_PEDAL_1 ((START_ACCELERATOR_PEDAL_1 + END_ACCELERATOR_PEDAL_1)/2)
#define HALF_ACCELERATOR_PEDAL_2 ((START_ACCELERATOR_PEDAL_2 + END_ACCELERATOR_PEDAL_2)/2)
#define DRIVER Matthew
#define MIN_HV_VOLTAGE 600
#define HT_DEBUG_EN
//Torque Calculation Defines
#define ALPHA 0.9772
#define TORQUE_1 100    
#define TORQUE_2 160
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
    class MC_temperatures_2 {
public:
    MC_temperatures_2() = default;
    MC_temperatures_2(uint8_t buf[8]) { load(buf); }

    inline void load(uint8_t buf[])         { memcpy(this, buf, sizeof(*this)); }
    inline void write(uint8_t buf[])  const { memcpy(buf, this, sizeof(*this)); }

    inline int16_t get_control_board_temperature()  const { return control_board_temperature; }
    inline int16_t get_rtd_1_temperature()          const { return rtd_1_temperature; }
    inline int16_t get_rtd_2_temperature()          const { return rtd_2_temperature; }
    inline int16_t get_rtd_3_temperature()          const { return rtd_3_temperature; }

#ifdef HT_DEBUG_EN
    void print() {
        Serial.println("\n\nMC TEMPERATURES 2");
        Serial.println(    "-----------------");
        Serial.print("CONTROL BOARD TEMP: ");   Serial.println(control_board_temperature / 10.0, 1);
        Serial.print("RTD 1 TEMP:         ");   Serial.println(rtd_1_temperature / 10.0, 1);
        Serial.print("RTD 2 TEMP:         ");   Serial.println(rtd_2_temperature / 10.0, 1);
        Serial.print("RTD 3 TEMP:         ");   Serial.println(rtd_3_temperature / 10.0, 1);
    }
#endif

private:
    int16_t control_board_temperature;  // @Parse @Scale(10) @Unit(C)
    int16_t rtd_1_temperature;          // @Parse @Scale(10) @Unit(C)
    int16_t rtd_2_temperature;          // @Parse @Scale(10) @Unit(C)
    int16_t rtd_3_temperature;          // @Parse @Scale(10) @Unit(C)
};
class MC_temperatures_3 {
public:
    MC_temperatures_3() = default;
    MC_temperatures_3(uint8_t buf[8]) { load(buf); }

    inline void load(uint8_t buf[])         { memcpy(this, buf, sizeof(*this)); }
    inline void write(uint8_t buf[])  const { memcpy(buf, this, sizeof(*this)); }

    inline int16_t get_rtd_4_temperature()  const { return rtd_4_temperature; }
    inline int16_t get_rtd_5_temperature()  const { return rtd_5_temperature; }
    inline int16_t get_motor_temperature()  const { return motor_temperature; }
    inline int16_t get_torque_shudder()     const { return torque_shudder; }

#ifdef HT_DEBUG_EN
    void print() {
        Serial.println("\n\nMC TEMPERATURES 3");
        Serial.println(    "-----------------");
        Serial.print("RTD 4 TEMP:     ");   Serial.println(rtd_4_temperature);
        Serial.print("RTD 5 TEMP:     ");   Serial.println(rtd_5_temperature);
        Serial.print("MOTOR TEMP:     ");   Serial.println(motor_temperature / 10.0, 1);
        Serial.print("TORQUE SHUDDER: ");   Serial.println(torque_shudder / 10.0, 1);
    }
#endif

private:
    int16_t rtd_4_temperature;  // @Parse @Scale(10) @Unit(C)
    int16_t rtd_5_temperature;  // @Parse @Scale(10) @Unit(C)
    int16_t motor_temperature;  // @Parse @Scale(10) @Unit(C)
    int16_t torque_shudder;     // @Parse @Scale(10) @Unit(N-m)
};

    class MC_fault_codes {
public:
    MC_fault_codes() = default;
    MC_fault_codes(uint8_t buf[8]) { load(buf); }

    inline void load(uint8_t buf[])         { memcpy(this, buf, sizeof(*this)); }
    inline void write(uint8_t buf[])  const { memcpy(buf, this, sizeof(*this)); }

    uint16_t get_post_fault_lo() { return post_fault_lo; }
    uint16_t get_post_fault_hi() { return post_fault_hi; }
    uint16_t get_run_fault_lo() { return run_fault_lo; }
    uint16_t get_run_fault_hi() { return run_fault_hi; }

    inline bool get_post_lo_hw_gate_desaturation_fault()                const { return post_fault_lo & 0x0001; } // @Parseflag(post_fault_lo)
    inline bool get_post_lo_hw_overcurrent_fault()                      const { return post_fault_lo & 0x0002; } // @Parseflag(post_fault_lo)
    inline bool get_post_lo_accelerator_shorted()                       const { return post_fault_lo & 0x0004; } // @Parseflag(post_fault_lo)
    inline bool get_post_lo_accelerator_open()                          const { return post_fault_lo & 0x0008; } // @Parseflag(post_fault_lo)
    inline bool get_post_lo_current_sensor_low()                        const { return post_fault_lo & 0x0010; } // @Parseflag(post_fault_lo)
    inline bool get_post_lo_current_sensor_high()                       const { return post_fault_lo & 0x0020; } // @Parseflag(post_fault_lo)
    inline bool get_post_lo_module_temperature_low()                    const { return post_fault_lo & 0x0040; } // @Parseflag(post_fault_lo)
    inline bool get_post_lo_module_temperature_high()                   const { return post_fault_lo & 0x0080; } // @Parseflag(post_fault_lo)
    inline bool get_post_lo_ctrl_pcb_temperature_low()                  const { return post_fault_lo & 0x0100; } // @Parseflag(post_fault_lo)
    inline bool get_post_lo_ctrl_pcb_temperature_high()                 const { return post_fault_lo & 0x0200; } // @Parseflag(post_fault_lo)
    inline bool get_post_lo_gate_drive_pcb_temperature_low()            const { return post_fault_lo & 0x0400; } // @Parseflag(post_fault_lo)
    inline bool get_post_lo_gate_drive_pcb_temperature_high()           const { return post_fault_lo & 0x0800; } // @Parseflag(post_fault_lo)
    inline bool get_post_lo_5v_sense_voltage_low()                      const { return post_fault_lo & 0x1000; } // @Parseflag(post_fault_lo)
    inline bool get_post_lo_5v_sense_voltage_high()                     const { return post_fault_lo & 0x2000; } // @Parseflag(post_fault_lo)
    inline bool get_post_lo_12v_sense_voltage_low()                     const { return post_fault_lo & 0x4000; } // @Parseflag(post_fault_lo)
    inline bool get_post_lo_12v_sense_voltage_high()                    const { return post_fault_lo & 0x8000; } // @Parseflag(post_fault_lo)

    inline bool get_post_hi_25v_sense_voltage_low()                     const { return post_fault_hi & 0x0001; } // @Parseflag(post_fault_hi)
    inline bool get_post_hi_25v_sense_voltage_high()                    const { return post_fault_hi & 0x0002; } // @Parseflag(post_fault_hi)
    inline bool get_post_hi_15v_sense_voltage_low()                     const { return post_fault_hi & 0x0004; } // @Parseflag(post_fault_hi)
    inline bool get_post_hi_15v_sense_voltage_high()                    const { return post_fault_hi & 0x0008; } // @Parseflag(post_fault_hi)
    inline bool get_post_hi_dc_bus_voltage_high()                       const { return post_fault_hi & 0x0010; } // @Parseflag(post_fault_hi)
    inline bool get_post_hi_dc_bus_voltage_low()                        const { return post_fault_hi & 0x0020; } // @Parseflag(post_fault_hi)
    inline bool get_post_hi_precharge_timeout()                         const { return post_fault_hi & 0x0040; } // @Parseflag(post_fault_hi)
    inline bool get_post_hi_precharge_voltage_failure()                 const { return post_fault_hi & 0x0080; } // @Parseflag(post_fault_hi)
    inline bool get_post_hi_eeprom_checksum_invalid()                   const { return post_fault_hi & 0x0100; } // @Parseflag(post_fault_hi)
    inline bool get_post_hi_eeprom_data_out_of_range()                  const { return post_fault_hi & 0x0200; } // @Parseflag(post_fault_hi)
    inline bool get_post_hi_eeprom_update_required()                    const { return post_fault_hi & 0x0400; } // @Parseflag(post_fault_hi)
    inline bool get_post_hi_reserved1()                                 const { return post_fault_hi & 0x0800; } // TODO delete these?
    inline bool get_post_hi_reserved2()                                 const { return post_fault_hi & 0x1000; }
    inline bool get_post_hi_reserved3()                                 const { return post_fault_hi & 0x2000; }
    inline bool get_post_hi_brake_shorted()                             const { return post_fault_hi & 0x4000; } // @Parseflag(post_fault_hi)
    inline bool get_post_hi_brake_open()                                const { return post_fault_hi & 0x8000; } // @Parseflag(post_fault_hi)

    inline bool get_run_lo_motor_overspeed_fault()                      const { return run_fault_lo & 0x0001; } // @Parseflag(run_fault_lo)
    inline bool get_run_lo_overcurrent_fault()                          const { return run_fault_lo & 0x0002; } // @Parseflag(run_fault_lo)
    inline bool get_run_lo_overvoltage_fault()                          const { return run_fault_lo & 0x0004; } // @Parseflag(run_fault_lo)
    inline bool get_run_lo_inverter_overtemperature_fault()             const { return run_fault_lo & 0x0008; } // @Parseflag(run_fault_lo)
    inline bool get_run_lo_accelerator_input_shorted_fault()            const { return run_fault_lo & 0x0010; } // @Parseflag(run_fault_lo)
    inline bool get_run_lo_accelerator_input_open_fault()               const { return run_fault_lo & 0x0020; } // @Parseflag(run_fault_lo)
    inline bool get_run_lo_direction_command_fault()                    const { return run_fault_lo & 0x0040; } // @Parseflag(run_fault_lo)
    inline bool get_run_lo_inverter_response_timeout_fault()            const { return run_fault_lo & 0x0080; } // @Parseflag(run_fault_lo)
    inline bool get_run_lo_hardware_gatedesaturation_fault()            const { return run_fault_lo & 0x0100; } // @Parseflag(run_fault_lo)
    inline bool get_run_lo_hardware_overcurrent_fault()                 const { return run_fault_lo & 0x0200; } // @Parseflag(run_fault_lo)
    inline bool get_run_lo_undervoltage_fault()                         const { return run_fault_lo & 0x0400; } // @Parseflag(run_fault_lo)
    inline bool get_run_lo_can_command_message_lost_fault()             const { return run_fault_lo & 0x0800; } // @Parseflag(run_fault_lo)
    inline bool get_run_lo_motor_overtemperature_fault()                const { return run_fault_lo & 0x1000; } // @Parseflag(run_fault_lo)
    inline bool get_run_lo_reserved1()                                  const { return run_fault_lo & 0x2000; } // TODO delete these?
    inline bool get_run_lo_reserved2()                                  const { return run_fault_lo & 0x4000; }
    inline bool get_run_lo_reserved3()                                  const { return run_fault_lo & 0x8000; }

    inline bool get_run_hi_brake_input_shorted_fault()                  const { return run_fault_hi & 0x0001; } // @Parseflag(run_fault_hi)
    inline bool get_run_hi_brake_input_open_fault()                     const { return run_fault_hi & 0x0002; } // @Parseflag(run_fault_hi)
    inline bool get_run_hi_module_a_overtemperature_fault()             const { return run_fault_hi & 0x0004; } // @Parseflag(run_fault_hi)
    inline bool get_run_hi_module_b_overtemperature_fault()             const { return run_fault_hi & 0x0008; } // @Parseflag(run_fault_hi)
    inline bool get_run_hi_module_c_overtemperature_fault()             const { return run_fault_hi & 0x0010; } // @Parseflag(run_fault_hi)
    inline bool get_run_hi_pcb_overtemperature_fault()                  const { return run_fault_hi & 0x0020; } // @Parseflag(run_fault_hi)
    inline bool get_run_hi_gate_drive_board_1_overtemperature_fault()   const { return run_fault_hi & 0x0040; } // @Parseflag(run_fault_hi)
    inline bool get_run_hi_gate_drive_board_2_overtemperature_fault()   const { return run_fault_hi & 0x0080; } // @Parseflag(run_fault_hi)
    inline bool get_run_hi_gate_drive_board_3_overtemperature_fault()   const { return run_fault_hi & 0x0100; } // @Parseflag(run_fault_hi)
    inline bool get_run_hi_current_sensor_fault()                       const { return run_fault_hi & 0x0200; } // @Parseflag(run_fault_hi)
    inline bool get_run_hi_reserved1()                                  const { return run_fault_hi & 0x0400; } // TODO delete these?
    inline bool get_run_hi_reserved2()                                  const { return run_fault_hi & 0x0800; }
    inline bool get_run_hi_reserved3()                                  const { return run_fault_hi & 0x1000; }
    inline bool get_run_hi_reserved4()                                  const { return run_fault_hi & 0x2000; }
    inline bool get_run_hi_resolver_not_connected()                     const { return run_fault_hi & 0x4000; } // @Parseflag(run_fault_hi)
    inline bool get_run_hi_inverter_discharge_active()                  const { return run_fault_hi & 0x8000; } // @Parseflag(run_fault_hi)

#ifdef HT_DEBUG_EN
    void print() {
        Serial.println("\n\nMC FAULT CODES");
        Serial.println    ("--------------");
        Serial.print("POST FAULT LO: 0x");  Serial.println(post_fault_lo, HEX);
        Serial.print("POST FAULT HI: 0x");  Serial.println(post_fault_hi, HEX);
        Serial.print("RUN FAULT LO:  0x");  Serial.println(run_fault_lo, HEX);
        Serial.print("RUN FAULT HI:  0x");  Serial.println(run_fault_hi, HEX);
    }
#endif

private:
    uint16_t post_fault_lo; // @Parse @Flagset @Hex @Sparse
    uint16_t post_fault_hi; // @Parse @Flagset @Hex @Sparse
    uint16_t run_fault_lo;  // @Parse @Flagset @Hex @Sparse
    uint16_t run_fault_hi;  // @Parse @Flagset @Hex @Sparse
};
class MCU_pedal_readings {
public:
    MCU_pedal_readings() = default;

    MCU_pedal_readings(const uint8_t buf[8]) { load(buf); }

    inline void load(const uint8_t buf[8]) { memcpy(this, buf, sizeof(*this)); }
    inline void write(uint8_t buf[8]) const { memcpy(buf, this, sizeof(*this)); }

    // Getters
    inline uint16_t get_accelerator_pedal_1() const { return accelerator_pedal_1; }
    inline uint16_t get_accelerator_pedal_2() const { return accelerator_pedal_2; }
    inline uint16_t get_brake_transducer_1()  const { return brake_transducer_1; }
    inline uint16_t get_brake_transducer_2()  const { return brake_transducer_2; }

    // Setters
    inline void set_accelerator_pedal_1(uint16_t reading) { accelerator_pedal_1 = reading; }
    inline void set_accelerator_pedal_2(uint16_t reading) { accelerator_pedal_2 = reading; }
    inline void set_brake_transducer_1(uint16_t reading)  { brake_transducer_1  = reading; }
    inline void set_brake_transducer_2(uint16_t reading)  { brake_transducer_2  = reading; }

private:
    // @Parse
    uint16_t accelerator_pedal_1;
    // @Parse
    uint16_t accelerator_pedal_2;
    // @Parse
    uint16_t brake_transducer_1;
    // @Parse
    uint16_t brake_transducer_2;
};

};
#endif