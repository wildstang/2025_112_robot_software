package org.wildstang.year2025.subsystems.climb;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsAbsoluteEncoder;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2025.robot.WsInputs;
import org.wildstang.year2025.robot.WsOutputs;
// import org.wildstang.year2025.robot.WsSubsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AutomateClimb implements Subsystem {

    private AnalogInput rightTrigger;
    private WsSpark climbMotor;
    private double climbMotorSpeed;
    private enum ClimbState{EXTEND, RETRACT}
    private ClimbState climbState;
    private WsAbsoluteEncoder absoluteEncoder;

    private double retractedPosition = Math.PI / 2;
    private double extendedPosition = 0;

    @Override
    public void inputUpdate(Input source) {
        if(source == rightTrigger && rightTrigger.getValue() >0.5){
            climbState = ClimbState.EXTEND;
        }else if (source == rightTrigger && rightTrigger.getValue() > 0.1){
            climbState = ClimbState.RETRACT;
        }
    }

    @Override
    public void init() {
        rightTrigger = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_TRIGGER);
        rightTrigger.addInputListener(this);
        climbMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.CLIMBMOTOR); 
        climbMotorSpeed = 0.0;
        
        absoluteEncoder = new WsAbsoluteEncoder(getName(), 0, 0);
        
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        
        switch(climbState){
            case EXTEND:
                if((absoluteEncoder.readRawValue()*(Math.PI/180)) > extendedPosition){
                    climbMotor.setSpeed(climbMotorSpeed);
                }
                break;
           case RETRACT:
                if(absoluteEncoder.readRawValue()*(Math.PI/180) < retractedPosition){
                    climbMotor.setSpeed(climbMotorSpeed);
                }
        }
       
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
