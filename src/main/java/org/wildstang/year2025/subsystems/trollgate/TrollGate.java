package org.wildstang.year2025.subsystems.trollgate;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsServo;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2025.robot.WsOutputs;
import org.wildstang.year2025.robot.WsInputs;

import edu.wpi.first.wpilibj.Timer;


public class TrollGate implements Subsystem {

    
    // Output
    private WsServo trollGate1;
    private WsServo trollGate2;

    // Types of States
    private enum TollgateStates {CONTRACT, DETRACT}
    private TollgateStates currentState;

    private Timer timer;
    

    
    @Override
    public void inputUpdate(Input source) {
    }

    @Override
    public void init() {
        // // Initializing inputs
        // rightBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        // rightBumper.addInputListener(this);

        // leftBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_TRIGGER);
        // leftBumper.addInputListener(this);
    
        // // Initializing the motor
        // motorSpark = (WsSpark) Core.getInputManager().getOutput(WsOutputs.motorSpark); // Ensure the correct constant is used
    }

    @Override
    public void selfTest() {
       
    }

    @Override
    public void update() { // every 20 ms

        switch (currentState) {
            case DETRACT:
                motorSpark.setSpeed(1); // Move motor to make the tollgate go down?
                break;
            case CONTRACT:
                motorSpark.setSpeed(-1); // Move motor to make tollgate go back up?
                break;
            case IDLE:
                motorSpark.setSpeed(0); // nothing is happening
                break;
        }
    }

    @Override
    public void resetState() {
        currentState = TollgateStates.IDLE;
        motorSpark.setSpeed(0); // Reset motor
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