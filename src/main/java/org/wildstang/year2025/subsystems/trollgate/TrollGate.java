package org.wildstang.year2025.subsystems.trollgate;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsServo;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2025.robot.WsOutputs;
import org.wildstang.year2025.robot.WsInputs;

import edu.wpi.first.wpilibj.PWM;


public class TrollGate implements Subsystem {

    
    // Output
    private PWM trollGate1;
    private PWM trollGate2;
    private double trollGateSpeed;

    // Types of States
    public enum TollgateStates {CONTRACT, DETRACT}
    public TollgateStates currentState;
    private AnalogInput rightTrigger;

    
    @Override
    public void inputUpdate(Input source) {
        trollGateSpeed = rightTrigger.getValue();
    }

    @Override
    public void init() {
        trollGate1 = new PWM(TrollGateConstants.TROLLGATE1CHANNEL);
        trollGate2 = new PWM(TrollGateConstants.TROLLGATE2CHANNEL);
        currentState = TollgateStates.DETRACT;
        rightTrigger = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_TRIGGER);
        rightTrigger.addInputListener(this);
        trollGateSpeed = 0;

    }

    @Override
    public void selfTest() {
       
    }

    @Override
    public void update() { // every 20 ms
                trollGate1.setPosition(trollGateSpeed);
                trollGate2.setPosition(trollGateSpeed);
    }

    public void setTrollGateState(TollgateStates newState){
       if (!newState.equals(currentState)){
            currentState = newState;
        }
    }

    @Override
    public void resetState() {
        currentState = TollgateStates.DETRACT;
    }

    @Override
    public void initSubsystems() {
        // what goes here?
    }

    @Override
    public String getName() {
        return "Tollgate Subsystem"; // Return the name of the subsystem
    }
}