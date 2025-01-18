package org.wildstang.year2025.subsystems.ArmLift;


import com.revrobotics.jni.CANSparkJNI;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2025.robot.WsInputs;
import org.wildstang.year2025.robot.WsOutputs;
import org.wildstang.year2025.subsystems.ArmLift.ArmLiftConstants;







/**
 * Interface describing a subsystem class.
 */
public class ArmLift implements Subsystem {

    /* Lift Variables */
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
    private double potentioVal;

    /* Arm Variables */

    public double maxArmRotation;
    public double minArmRotation;
    public double restArmRotation;
    private WsSpark armMotor;
    private DigitalInput dpadLeft;
    private DigitalInput dpadRight;
    private DigitalInput dpadUp;
    private boolean activateArm = false;
    private AbsoluteEncoder armEncoder;



    

    


     @Override
    public void init(){
        liftMotor1 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.LIFTONE);
        liftMotor2 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.LIFTTWO);
        leftBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_SHOULDER);
        rightBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        liftSpeed = 0.5;
        liftMotor1.setBrake();
        liftMotor2.setBrake();
        rightTrigger = (AnalogInput) WsInputs.DRIVER_RIGHT_TRIGGER.get();
        rightTrigger.addInputListener(this);
        leftTrigger = (AnalogInput) WsInputs.DRIVER_LEFT_TRIGGER.get();
        leftTrigger.addInputListener(this);
        dpadUp = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_UP);
        dpadUp.addInputListener(this);
        dpadLeft = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_LEFT);
        dpadLeft.addInputListener(this);
        dpadRight = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_RIGHT);
        dpadRight.addInputListener(this);
        armEncoder = armMotor.getController().getAbsoluteEncoder(Type.kDutyCycle);
        armEncoder.setPositionConversionFactor(2.0 * Math.PI);



        armMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ARMMOTOR);
        
    }
    public void inputUpdate(){
        if(leftBumper.getValue()){
            liftDirection = 1;
        }else if(rightBumper.getValue()){
            liftDirection = -1;
        }else{
            liftDirection =0;
        }

        if(dpadUp.getValue()){
            activateArm = true;
        }
         leftTriggerValue = leftTrigger.getValue();
         rightTriggerValue = rightTrigger.getValue();

    }

    public void update(){
        testLift();
    }

    // Press sensetive lift (not done)
    public void testAnalogLift(){
        
    }
    public void testLift(){
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

    public void testArm(){
        if(activateArm){
            /* Increase Lift Position 
             * Keep lift in that position
            */

            /* Pivot arm out 
             * move lift back down
             * Fine tune arm position if needed
             */
            
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
