#ifndef KS2ECAN_HPP
#define KS2ECAN_HPP
/*
 * CAN ID definitions
 */

#define ID_MC_TEMPERATURES_1                                        0xA0
#define ID_MC_TEMPERATURES_2                                        0xA1
#define ID_MC_TEMPERATURES_3                                        0xA2
#define ID_MC_ANALOG_INPUTS_VOLTAGES                                0xA3
#define ID_MC_DIGITAL_INPUT_STATUS                                  0xA4
#define ID_MC_MOTOR_POSITION_INFORMATION                            0xA5
#define ID_MC_CURRENT_INFORMATION                                   0xA6
#define ID_MC_VOLTAGE_INFORMATION                                   0xA7
#define ID_MC_FLUX_INFORMATION                                      0xA8
#define ID_MC_INTERNAL_VOLTAGES                                     0xA9
#define ID_MC_INTERNAL_STATES                                       0xAA
#define ID_MC_FAULT_CODES                                           0xAB
#define ID_MC_TORQUE_TIMER_INFORMATION                              0xAC
#define ID_MC_MODULATION_INDEX_FLUX_WEAKENING_OUTPUT_INFORMATION    0xAD
#define ID_MC_FIRMWARE_INFORMATION                                  0xAE
#define ID_MC_DIAGNOSTIC_DATA                                       0xAF
#define ID_MC_COMMAND_MESSAGE                                       0xC0
#define ID_MC_READ_WRITE_PARAMETER_COMMAND                          0xC1
#define ID_MC_READ_WRITE_PARAMETER_RESPONSE                         0xC2
#define ID_VCU_STATUS						                        0xC3
#define ID_VCU_PEDAL_READINGS				                        0xC4
#define ID_VCU_WS_READINGS                                          0xC6
#define ID_DASH_BUTTONS                                             0xEB
#define ID_DASH_BUSVOLT                                             0xD4
#define ID_BMS_SOC                                                  0x6B3 //made this real!


#define ID_BMS_INFO     0x6B1
#define ID_ACU_RELAY    0x68

// number of rx and tx mailboxes
#define NUM_TX_MAILBOXES 2
#define NUM_RX_MAILBOXES 6

#endif