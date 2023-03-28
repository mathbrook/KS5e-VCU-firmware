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

```mermaid
flowchart TD
        A[Initalize State Machine] -->|Set State to startup| B(Current state)
        B --> C(Check If Current state == New state) -->|NEW = OLD| B(Current state)
        C --> |New *does not equal* OLD| D(run exit logic)
        D --> E(run exit logic)
        E --> F(run exit logic)
        F --> B(Current State) 
```        