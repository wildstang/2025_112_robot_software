package org.wildstang.year2025.subsystems.climb;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2025.robot.WsInputs;
import org.wildstang.year2025.robot.WsOutputs;
// import org.wildstang.year2025.robot.WsSubsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Climb implements Subsystem {

    private AnalogInput rightTrigger;
    private WsSpark climbMotor;
    private double climbMotorSpeed;

    @Override
    public void inputUpdate(Input source) {
        climbMotorSpeed = rightTrigger.getValue();
    }

    @Override
    public void init() {
        rightTrigger = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_TRIGGER);
        rightTrigger.addInputListener(this);
        climbMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.CLIMBMOTOR); 
        climbMotorSpeed = 0.0;
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
       climbMotor.setSpeed(climbMotorSpeed);
       SmartDashboard.putNumber("climb speed", climbMotorSpeed);
    }

    @Override
    public void resetState() {
    }

    @Override
    public void initSubsystems() {
    }

    @Override
    public String getName() {
        return "Climb";
    }
    
}
