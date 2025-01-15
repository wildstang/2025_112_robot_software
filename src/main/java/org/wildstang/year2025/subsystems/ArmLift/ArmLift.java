package org.wildstang.year2025.subsystems.ArmLift;

import org.wildstang.hardware.roborio.inputs.WsAnalogInput;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2025.robot.WsInputs;
import org.wildstang.year2025.robot.WsOutputs;

import edu.wpi.first.wpilibj.AnalogInput;


/**
 * Interface describing a subsystem class.
 */
public class ArmLift implements Subsystem {

    private WsSpark liftMotor1;
    private WsSpark liftMotor2;
    public DigitalInput leftBumper;
    public DigitalInput rightBumper;
    public double liftSpeed;
    public int liftDirection;
    public double maxLiftPos;
    public double minLiftPos;
    public AnalogInput leftTrigger;
    public AnalogInput rightTrigger;
    public double leftTriggerValue;
    public double rightTriggerValue;
    


     @Override
    public void init(){
        liftMotor1 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.LIFTONE);
        liftMotor2 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.LIFTTWO);
        leftBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_SHOULDER);
        rightBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        liftSpeed = 0.5;
        liftMotor1.setBrake();
        liftMotor2.setBrake();
        rightTrigger = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_TRIGGER);
        rightTrigger.addInputListener(this);
        leftTrigger = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_TRIGGER);
        leftTrigger.addInputListener(this);
        
    }
    public void inputUpdate(){
        if(leftBumper.getValue()){
            liftDirection = 1;
        }else if(rightBumper.getValue()){
            liftDirection = -1;
        }else{
            liftDirection =0;
        }
         leftTriggerValue = leftTrigger.getValue();
         rightTriggerValue = rightTrigger.getValue();

    }

    public void update(){
        testSubsystem();
    }

    // Press sensetive lift (not done)
    public void testAnalogSybsystem(){
        switch (){
            case -1:
                liftMotor1.setSpeed(leftTriggerValue);
                liftMotor2.setSpeed(leftTriggerValue);
            case 1:
                liftMotor1.setSpeed(rightTriggerValue);
                liftMotor2.setSpeed(rightTriggerValue);
            default:
                liftMotor1.stop();
                liftMotor2.stop();
        }
    }
    public void testSubsystem(){
        switch (liftDirection){
            case -1:
                liftMotor1.setSpeed(-liftSpeed);
                liftMotor2.setSpeed(-liftSpeed);
            case 1:
                liftMotor1.setSpeed(liftSpeed);
                liftMotor2.setSpeed(liftSpeed);
            default:
                liftMotor1.stop();
                liftMotor2.stop();
        }
    }
    public void selfTest(){
        
    }

    @Override
    public void inputUpdate(Input source) {
        
    }

    @Override
    public void resetState() {
        
    }

    @Override
    public void initSubsystems() {
        
    }

    @Override
    public String getName() {
        return "ArmLift";
    }
   
}
