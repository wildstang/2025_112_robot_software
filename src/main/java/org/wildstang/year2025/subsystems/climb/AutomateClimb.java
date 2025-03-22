package org.wildstang.year2025.subsystems.climb;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsAbsoluteEncoder;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2025.robot.WsInputs;
import org.wildstang.year2025.robot.WsOutputs;
// import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift.GameStates;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AutomateClimb implements Subsystem {

    private DigitalInput driverStart;
    private WsSpark climbMotor;
    private double climbMotorSpeed;
    private enum ClimbState{EXTEND, RETRACT, INIT}
    private ClimbState climbState;
    private WsAbsoluteEncoder absoluteEncoder;
    private double climbGearRatio = 195.3125;
    private double motorPos;

    private final double retractedPosition = Math.PI;
    private final double extendedPosition = Math.PI / 2;
    private final double startPos = Math.PI / 6;

    @Override
    public void inputUpdate(Input source) {
        if(source == driverStart && driverStart.getValue()){
            setClimbState();
        }
    }

    @Override
    public void init() {
        driverStart = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_START);
        driverStart.addInputListener(this);
        climbMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.CLIMBMOTOR); 
        climbMotorSpeed = 1;
        climbState = ClimbState.INIT;
    }

    

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        motorPos = (climbMotor.getPosition()/climbGearRatio) * (Math.PI*2); // In Radians

        switch(climbState){
            case INIT:
                if(motorPos < startPos){
                    climbMotor.setSpeed(climbMotorSpeed);
                }
                break;
            case EXTEND:
                if(motorPos < extendedPosition){
                    climbMotor.setSpeed(climbMotorSpeed);
                }
                break;
           case RETRACT:
                if(motorPos < retractedPosition){
                    climbMotor.setSpeed(climbMotorSpeed);
                }
                break;
            default:
                climbMotor.setSpeed(0);
                break;

        }
       
       SmartDashboard.putNumber("climb speed", climbMotorSpeed);
       SmartDashboard.putNumber("Motor Position (Radians): ", motorPos);
    }

    public void setClimbState(){
        if(climbState == ClimbState.INIT){
            climbState = ClimbState.EXTEND;
        }else if(climbState == ClimbState.EXTEND){
            climbState = ClimbState.RETRACT;
        }else{
            ;
        }
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
