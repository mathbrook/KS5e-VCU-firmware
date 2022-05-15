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
// #include <drivers.h>
//Pedalbox stuff
#define BRAKE_ACTIVE 25000               // Threshold for brake pedal active  
#define MIN_ACCELERATOR_PEDAL_1 21100    // Low accelerator implausibility threshold
#define START_ACCELERATOR_PEDAL_1 21700  // Position to start acceleration
#define END_ACCELERATOR_PEDAL_1 26000    // Position to max out acceleration
#define MAX_ACCELERATOR_PEDAL_1 26949    // High accelerator implausibility threshold
#define MIN_ACCELERATOR_PEDAL_2 13700    // Low accelerator implausibility threshold
#define START_ACCELERATOR_PEDAL_2 14200  // Position to start acceleration
#define END_ACCELERATOR_PEDAL_2 17250    // Position to max out acceleration
#define MAX_ACCELERATOR_PEDAL_2 17565    // High accelerator implausibility threshold
#define HALF_ACCELERATOR_PEDAL_1 ((START_ACCELERATOR_PEDAL_1 + END_ACCELERATOR_PEDAL_1)/2)
#define HALF_ACCELERATOR_PEDAL_2 ((START_ACCELERATOR_PEDAL_2 + END_ACCELERATOR_PEDAL_2)/2)
#define DRIVER Matthew
//Torque Calculation Defines
#define ALPHA 0.9772
#define TORQUE_1 120
#define TORQUE_2 240
float accel1{},accel2{},brake1{},brake2{};
ADC_SPI ADC(DEFAULT_SPI_CS, DEFAULT_SPI_SPEED);
Metro mcControlTimer=Metro(50);
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
};
#endif