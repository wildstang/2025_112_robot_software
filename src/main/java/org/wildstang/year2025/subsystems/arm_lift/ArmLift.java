package org.wildstang.year2025.subsystems.arm_lift;


import com.revrobotics.spark.SparkAbsoluteEncoder;


import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;

import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.framework.core.Core;

import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.year2025.robot.WsInputs;
import org.wildstang.year2025.robot.WsOutputs;
import org.wildstang.year2025.subsystems.arm_lift.ArmLiftConstants;







/**
 * Interface describing a subsystem class.
 */
public class ArmLift implements Subsystem {



    private double armPosition;
    private double targetPosition;


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
    public int armDirection; /* + is clockwise and - is counterclockwise */
    private WsSpark armMotor;
    private DigitalInput dpadLeft;
    private DigitalInput dpadRight;
    private DigitalInput dpadUp;
    private boolean activateArm = false;


    private int inputType = 0;


    

    


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
       



        armMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ARMMOTOR);
        
    }
    public void inputUpdate(Input source){
        if(source == leftBumper || source == rightBumper){
            if(leftBumper.getValue()){
                liftDirection = 1;
            }else if(rightBumper.getValue()){
                liftDirection = -1;
            }
        }else if(source == dpadRight || source == dpadLeft){
            if(dpadRight.getValue()){
                armDirection = 1;
            }else if(dpadLeft.getValue()){
                armDirection = -1;
            }
        }else{
            liftDirection = 0;
            armDirection = 0;
        }
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
                liftMotor2.setSpeed(liftSpeed);
            case 1:
                liftMotor1.setSpeed(liftSpeed);
                liftMotor2.setSpeed(-liftSpeed);
            default:
                liftMotor1.stop();
                liftMotor2.stop();
        }
    }

    public void testArm(){
        if(activateArm){

            
        }
    }
    public void selfTest(){
        
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
