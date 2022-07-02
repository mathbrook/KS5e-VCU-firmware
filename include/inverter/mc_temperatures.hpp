#ifndef MC_TEMPERATURES_H
#define MC_TEMPERATURES_H
#include <Arduino.h>

// TODO un-fuck this for the love of god

class MC_temperatures_1
{
public:
    MC_temperatures_1() = default;
    MC_temperatures_1(uint8_t buf[8]) { load(buf); }

    inline void load(uint8_t buf[]) { memcpy(this, buf, sizeof(*this)); }
    inline void write(uint8_t buf[]) const { memcpy(buf, this, sizeof(*this)); }

    inline int16_t get_module_a_temperature() const { return module_a_temperature; }
    inline int16_t get_module_b_temperature() const { return module_b_temperature; }
    inline int16_t get_module_c_temperature() const { return module_c_temperature; }
    inline int16_t get_gate_driver_board_temperature() const { return gate_driver_board_temperature; }

#ifdef HT_DEBUG_EN
    void print()
    {
        Serial.println("\n\nMC TEMPERATURES 1");
        Serial.println("-----------------");
        Serial.print("MODULE A TEMP:          ");
        Serial.println(module_a_temperature / 10.0, 1);
        Serial.print("MODULE B TEMP:          ");
        Serial.println(module_b_temperature / 10.0, 1);
        Serial.print("MODULE C TEMP:          ");
        Serial.println(module_c_temperature / 10.0, 1);
        Serial.print("GATE DRIVER BOARD TEMP: ");
        Serial.println(gate_driver_board_temperature / 10.0, 1);
    }
#endif

private:
    int16_t module_a_temperature;          // @Parse @Scale(10) @Unit(C)
    int16_t module_b_temperature;          // @Parse @Scale(10) @Unit(C)
    int16_t module_c_temperature;          // @Parse @Scale(10) @Unit(C)
    int16_t gate_driver_board_temperature; // @Parse @Scale(10) @Unit(C)
    

};
class MC_temperatures_2
{
public:
    MC_temperatures_2() = default;
    MC_temperatures_2(uint8_t buf[8]) { load(buf); }

    inline void load(uint8_t buf[]) { memcpy(this, buf, sizeof(*this)); }
    inline void write(uint8_t buf[]) const { memcpy(buf, this, sizeof(*this)); }

    inline int16_t get_control_board_temperature() const { return control_board_temperature; }
    inline int16_t get_rtd_1_temperature() const { return rtd_1_temperature; }
    inline int16_t get_rtd_2_temperature() const { return rtd_2_temperature; }
    inline int16_t get_rtd_3_temperature() const { return rtd_3_temperature; }

#ifdef HT_DEBUG_EN
    void print()
    {
        Serial.println("\n\nMC TEMPERATURES 2");
        Serial.println("-----------------");
        Serial.print("CONTROL BOARD TEMP: ");
        Serial.println(control_board_temperature / 10.0, 1);
        Serial.print("RTD 1 TEMP:         ");
        Serial.println(rtd_1_temperature / 10.0, 1);
        Serial.print("RTD 2 TEMP:         ");
        Serial.println(rtd_2_temperature / 10.0, 1);
        Serial.print("RTD 3 TEMP:         ");
        Serial.println(rtd_3_temperature / 10.0, 1);
    }
#endif

private:
    int16_t control_board_temperature; // @Parse @Scale(10) @Unit(C)
    int16_t rtd_1_temperature;         // @Parse @Scale(10) @Unit(C)
    int16_t rtd_2_temperature;         // @Parse @Scale(10) @Unit(C)
    int16_t rtd_3_temperature;         // @Parse @Scale(10) @Unit(C)
};
class MC_temperatures_3
{
public:
    MC_temperatures_3() = default;
    MC_temperatures_3(uint8_t buf[8]) { load(buf); }

    inline void load(uint8_t buf[]) { memcpy(this, buf, sizeof(*this)); }
    inline void write(uint8_t buf[]) const { memcpy(buf, this, sizeof(*this)); }

    inline int16_t get_rtd_4_temperature() const { return rtd_4_temperature; }
    inline int16_t get_rtd_5_temperature() const { return rtd_5_temperature; }
    inline int16_t get_motor_temperature() const { return motor_temperature; }
    inline int16_t get_torque_shudder() const { return torque_shudder; }

#ifdef HT_DEBUG_EN
    void print()
    {
        Serial.println("\n\nMC TEMPERATURES 3");
        Serial.println("-----------------");
        Serial.print("RTD 4 TEMP:     ");
        Serial.println(rtd_4_temperature);
        Serial.print("RTD 5 TEMP:     ");
        Serial.println(rtd_5_temperature);
        Serial.print("MOTOR TEMP:     ");
        Serial.println(motor_temperature / 10.0, 1);
        Serial.print("TORQUE SHUDDER: ");
        Serial.println(torque_shudder / 10.0, 1);
    }
#endif

private:
    int16_t rtd_4_temperature; // @Parse @Scale(10) @Unit(C)
    int16_t rtd_5_temperature; // @Parse @Scale(10) @Unit(C)
    int16_t motor_temperature; // @Parse @Scale(10) @Unit(C)
    int16_t torque_shudder;    // @Parse @Scale(10) @Unit(N-m)
};

#endif