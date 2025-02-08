
package org.wildstang.year2025.subsystems.Claw;

import java.nio.file.ClosedWatchServiceException;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2025.robot.WsInputs;
import org.wildstang.year2025.robot.WsOutputs;

import edu.wpi.first.wpilibj.Timer;

public class Claw implements Subsystem{

    //inputs or button on the controller
    private DigitalInput leftBumper;
    private DigitalInput rightBumper;
    private AnalogInput leftTrigger;

    //outputs
    private WsSpark clawMotor;

    private enum clawStates {INTAKE, OUTTAKE, IDLE}; 
    private clawStates currentState;
    private Timer timer; 
    public static boolean algaeInClaw;




    @Override
    //Called everytime an input/buttons is pressed
    public void inputUpdate(Input source) {
       if(leftBumper.getValue()){
        currentState = clawStates.INTAKE;
       }
       else if (rightBumper.getValue()){
        currentState = clawStates.OUTTAKE;
        timer.reset();
        timer.start();
       }
       else{
        currentState = clawStates.IDLE;
       }
    }

    @Override
    //initializes + ONE AND DONE + START
    public void init() {
        leftBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_SHOULDER);
        leftBumper.addInputListener(this);
        rightBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        rightBumper.addInputListener(this);
        leftTrigger = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_TRIGGER);
        leftTrigger.addInputListener(this);

        clawMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.CLAWMOTOR);
        timer = new Timer();

        algaeInClaw = false;
    }

    @Override
    //  runs every 20 MS
    public void update() {
       switch(currentState){
        case INTAKE:
            if(Math.abs(clawMotor.getVelocity()) < ClawConstants.CLAW_HOLD_SPEED || clawMotor.getController().getOutputCurrent() > ClawConstants.CLAW_CURRENT_HOLD){
                algaeInClaw = true; 
            }
            clawMotor.setCurrentLimit(5, 10, 3);
            clawMotor.setSpeed(ClawConstants.CLAW_INTAKE_SPEED);

        break;
        case OUTTAKE:
            clawMotor.setCurrentLimit(10, 15, 3);
            if(timer.get() > ClawConstants.OUTTAKE_TIME){
                currentState = clawStates.IDLE;
                timer.reset();
            }
            clawMotor.setSpeed(ClawConstants.CLAW_OUTTAKE_SPEED);
        break;
        case IDLE:
            clawMotor.setSpeed(0.0);
        break;
        default:
            clawMotor.setSpeed(0.0);
        break;

       }
    }

    @Override
    //reseting everything
    public void resetState() {
        currentState = clawStates.IDLE; 
    }

    @Override
    public void selfTest() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'selfTest'");
    }

    @Override
    public void initSubsystems() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'initSubsystems'");
    }

    @Override
    public String getName() {
       return "Claw";
    }
    
}

