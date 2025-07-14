package org.wildstang.year2025.subsystems.climb;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;

import org.wildstang.hardware.roborio.outputs.WsSpark;

import org.wildstang.year2025.robot.WsInputs;
import org.wildstang.year2025.robot.WsOutputs;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift.GameStates;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AutomateClimb implements Subsystem {

    private DigitalInput driverStart;
    private WsSpark climbMotor;
    private double climbMotorSpeed;
    private enum ClimbState{EXTEND, RETRACT, INIT, MANUAL}
    private ClimbState climbState;
    private double climbGearRatio = 195.3125;
    private double motorPos;
    private AnalogInput rightTrigger;

    private final double retractedPosition = 19.25;
    private final double extendedPosition = 9.38;
    private final double startPos = 0.46;

    private ArmLift armLift;

    @Override
    public void inputUpdate(Input source) {
        if(source == driverStart && driverStart.getValue()){
            setClimbState();
        }
        if (climbState == ClimbState.MANUAL) climbMotorSpeed = -rightTrigger.getValue();

    }

    @Override
    public void init() {
        driverStart = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_START);
        driverStart.addInputListener(this);
        rightTrigger = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_TRIGGER);
        rightTrigger.addInputListener(this);
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
                } else {
                    climbMotor.setSpeed(0.0);
                }
                break;
            case EXTEND:
                armLift.setGameState(GameStates.CLIMB);
                if(motorPos < extendedPosition){
                    climbMotor.setSpeed(climbMotorSpeed);
                } else {
                    climbMotor.setSpeed(0.0);
                }
                break;
           case RETRACT:
                if(motorPos < retractedPosition){
                    climbMotor.setSpeed(climbMotorSpeed);
                } else {
                    climbMotor.setSpeed(0.0);
                    climbMotorSpeed = 0.0;
                    setClimbState();
                }
                break;
            case MANUAL:
                climbMotor.setSpeed(climbMotorSpeed);
                break;
        }
       
       SmartDashboard.putNumber("climb speed", climbMotorSpeed);
       SmartDashboard.putString("climb state", climbState.toString());
       SmartDashboard.putNumber("Climb Position", motorPos);
    }

    public void setClimbState(){
        if(climbState == ClimbState.INIT){
            climbState = ClimbState.EXTEND;
        }else if(climbState == ClimbState.EXTEND){
            climbState = ClimbState.RETRACT;
        } else if(climbState == ClimbState.RETRACT){
            climbState = ClimbState.MANUAL;
        }
    }

    @Override
    public void resetState() {
    }

    @Override
    public void initSubsystems() {
        armLift = (ArmLift) Core.getSubsystemManager().getSubsystem(WsSubsystems.ARMLIFT);
    }

    @Override
    public String getName() {
        return "Automate Climb";
    }
    
}
