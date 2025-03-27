package org.wildstang.year2025.subsystems.trollgate;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsServo;
import org.wildstang.year2025.robot.WsOutputs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.wildstang.year2025.robot.WsInputs;


public class TrollGate implements Subsystem {

    
    // Output
    private WsServo frontGate, backGate;
    private double trollGateSpeed;

    // Types of States
    public enum TollgateStates {EXTEND, RETRACT}
    public TollgateStates currentState;
    private AnalogInput rightTrigger;

    
    @Override
    public void inputUpdate(Input source) {
        trollGateSpeed = rightTrigger.getValue() * -180.0;
    }

    @Override
    public void init() {
        frontGate = (WsServo) Core.getOutputManager().getOutput(WsOutputs.TROLLGATEFRONT);
        backGate = (WsServo) Core.getOutputManager().getOutput(WsOutputs.TROLLGATEBACK);
        currentState = TollgateStates.RETRACT;
        rightTrigger = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_TRIGGER);
        rightTrigger.addInputListener(this);
        trollGateSpeed = 0;
    }

    @Override
    public void update() { // every 20 ms
        frontGate.setValue(trollGateSpeed);
        backGate.setValue(trollGateSpeed);
        SmartDashboard.putNumber("troll gate value", trollGateSpeed);
    }

    public void setTrollGateState(TollgateStates newState){
       if (!newState.equals(currentState)){
            currentState = newState;
        }
    }

    @Override
    public void resetState() {
        currentState = TollgateStates.RETRACT;
    }

    @Override
    public void initSubsystems() {
    }

    @Override
    public void selfTest() {
    }

    @Override
    public String getName() {
        return "TrollGate"; // Return the name of the subsystem, must match WsSubsystems
    }
}