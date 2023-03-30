# design 

the vcu CAN out is still TODO mostly, currently it spits out all RX / TX traffic

below is a loose flow of the data and what handles what



```mermaid
    graph LR;
        state_machine --> main_loop;
        main_loop --> mcu_state;

        pedal_handler --> state_machine;
        accumulator_handler --> state_machine;
        inverter --> state_machine;

        state_machine --> pedal_handler;
        state_machine --> accumulator_handler;
        state_machine --> inverter;
        state_machine --> dash;

        inverter --> VCU_CAN_OUT;
        accumulator_handler --> VCU_CAN_OUT;
        pedal_handler --> VCU_CAN_OUT;
        state_machine --> VCU_CAN_OUT;

       

```
This is tha basic of the state machien. 
    the car is set to state


```mermaid
flowchart TD
        A[Initalize State Machine] -->|Set State to startup| B(Current state)
        B --> C(Check If Current state == New state) -->|NEW = OLD| B(Current state)
        C --> |New *does not equal* OLD| D(run exit logic)
        D --> E(run exit logic)
        E --> F(run exit logic)
        F --> B(Current State) 
```        

```mermaid
flowchart TD
    A[State set to TRACTIVE_SYSTEM_NOT_ACTIVE] -->|Enter Entry logic| B(TRACTIVE_SYSTEM_NOT_ACTIVE)
    
        B --> C(Force MC Discharge)
        C --> D(Set Dash LED Yellow)
        D --> E(Dash LED Wipe)
        E --> 

    B --> |Break to Handle| F(Handle State TRACTIVE_SYSTEM_NOT_ACTIVE)
        F --> G(Get max torque)
        G --> H(Get max RPM)
        H --> I(inverter kick 0)
        I --> J(check if precharge is attempted)
        J --> |Failure| K(set ACC_Ready to false)
        J --> |sucuess| L(set ACC_Ready to True)

        L(set ACC_Ready to True) --> M(check  if TS_active & Acc_ready)
    M --> |sucuess| N(set state to TRACTIVE_SYSTEM_ACTIVE)

```  

```mermaid

flowchart TD
    A[Initalize] -->|start| B(check if PID_MODE is active)
    B -->|Yes| C{IDGAF}
    B -->|No| D(Check If PID_TC_MODE)
    D -->|Yes| E{ALSO IDGAF}
    D --> F(SET TORQUE TO ZERO)
    F --> G(SET TORQUES)

    G --> H(SET TORQUE 1)
    G --> I(SET TORQUE 2)

    G --> J(SET ACCLERATION PERCENTS)

    J --> K(SET TORQUE 1)
    J --> L(SET TORQUE 2)

    J --> M(SET TO PERCENTAGE*)
    M --> N(IS EXPO CURVE ON?)


    N -->|Yes| O{IDGAF}
    N -->|No| P(ENTER TORQUE CHECK)

    P --> Q(RETURN CALCULATED TORQUE)

```  