package org.wildstang.year2025.subsystems.arm_lift;


import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;

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

    PIDController controller;
    MotionProfile profileController;

    private double armPosition;
    private double targetPosition;

    /*Inputs */
    private DigitalInput dpadDown; 
    private DigitalInput dpadLeft;
    private DigitalInput dpadRight;
    private DigitalInput dpadUp;
    private DigitalInput driverFaceLeft;
    private AnalogInput leftJoyStickY;
    private AnalogInput rightJoyStickX;


    /* Lift Variables */
    private WsSpark liftMotor1;
    private WsSpark liftMotor2;
    private double liftSpeed;
    private enum liftDirection {GROUND_INTAKE, LOW_ALGAE_REEF, HIGH_ALGAE_REEF, STORAGE, SCORE_PRELOAD, SHOOT_NET};
    private liftDirection liftState;
    private double maxLiftPos;
    private double minLiftPos;
    private double potentioVal;

    /* Arm Variables */
    public double maxArmRotation;
    public double minArmRotation;
    public double restArmRotation;
    public int armDirection; /* + is clockwise and - is counterclockwise */
    private WsSpark armMotor;
    private boolean activateArm = false;


    private int inputType = 0;


     @Override
    public void init(){
        initOutput();
        initInputs();
    }

    public void initOutput(){
        liftMotor1 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.LIFTONE);
        liftMotor2 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.LIFTTWO);
     
        liftSpeed = 0.5;
        liftMotor1.setBrake();
        liftMotor2.setBrake();
       
        armMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ARMMOTOR);
    }


    public void initInputs(){
        dpadUp = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_UP);
        dpadUp.addInputListener(this);
        dpadLeft = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_LEFT);
        dpadLeft.addInputListener(this);
        dpadRight = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_RIGHT);
        dpadRight.addInputListener(this);
        dpadDown = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_DOWN);
        dpadDown.addInputListener(this);
        driverFaceLeft = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_FACE_LEFT);
        driverFaceLeft.addInputListener(this);
    
    }

    public void inputUpdate(Input source){
        if(source == dpadRight || source == dpadLeft || source == dpadDown || source == dpadUp || source == driverFaceLeft){
            if(dpadDown.getValue()){
                liftState = liftDirection.GROUND_INTAKE;
            }else if(dpadLeft.getValue()){
                liftState = liftDirection.LOW_ALGAE_REEF;
            }
            else if(dpadRight.getValue()){
                liftState = liftDirection.HIGH_ALGAE_REEF;
            }
            else if (dpadUp.getValue()){
                liftState = liftDirection.SHOOT_NET;
            }
            else if(driverFaceLeft.getValue()){
                liftState = liftDirection.STORAGE;
            }
        }else{
            liftState = liftDirection.STORAGE;
            armDirection = 0;
        }
    
    }

    public void update(){
        testLift();
    }

    // Press sensetive lift (not done)
    public void testAnalogLift(){
        
    }

    private void competitionLift(){
         switch (liftState){
            case GROUND_INTAKE:
                liftMotor1.setSpeed(-liftSpeed);
                liftMotor2.setSpeed(liftSpeed);
                break;
            case LOW_ALGAE_REEF:
            break;
            case HIGH_ALGAE_REEF:
            break;
            case SHOOT_NET:
            break;

            default:
                liftMotor1.stop();
                liftMotor2.stop();
        }
    }
    public void testLift(){

        liftMotor1.setSpeed(liftSpeed);
        liftMotor2.setSpeed(-liftSpeed);
       
    }

    public void testArm(){
        if(activateArm){

            
        }
    }
    public void armSystem(double start_pos, double end_pos){
        //profileController.calculate(0, 0, Math.PI, 0);
        double goalPos = profileController.getSamples()[0];
        double goalVel = profileController.getSamples()[1];
        double goalAcc = profileController.getSamples()[2];

        
    }
    public void selfTest(){
        
    }

    @Override
    public void resetState() {
        liftSpeed = 0;
        liftState = liftDirection.STORAGE;
        armDirection = 0;
    }

    @Override
    public void initSubsystems() {
        
    }

    @Override
    public String getName() {
        return "ArmLift";
    }
   
}
