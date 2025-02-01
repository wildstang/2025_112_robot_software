package org.wildstang.year2025.subsystems.arm_lift;


import com.revrobotics.AbsoluteEncoder;
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


    /* Control System Variables */
    PIDController armPIDC;
    PIDController liftPIDC;
    MotionProfile armProfile;
    MotionProfile liftProfile;

    /*Inputs Variables*/
    private DigitalInput dpadDown; 
    private DigitalInput dpadLeft;
    private DigitalInput dpadRight;
    private DigitalInput dpadUp;
    private DigitalInput driverFaceLeft;
    private AnalogInput leftJoyStickY;
    private AnalogInput rightJoyStickX;
    private AnalogInput liftPotentiometer; // Input for the potentiometer on lift pulley


    /* Lift Variables */
    private WsSpark liftMotor1;
    private WsSpark liftMotor2;
    private double liftSpeed;
    private double currentLiftPos;
    private enum gameStates {GROUND_INTAKE, L2_ALGAE_REEF, L3_ALGAE_REEF, STORAGE, SCORE_PRELOAD, SHOOT_NET, START}; // Our Arm/Lift States
    private gameStates gameState;
    
   

    /* Arm Variables */
    //public double maxArmRotation;
    //public double minArmRotation;
    //public double restArmRotation;
    private double currentArmPos;
    public int armDirection; /* + is clockwise and - is counterclockwise */
    private WsSpark armMotor;
    //private boolean activateArm = false;


    //private int inputType = 0;


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
        liftMotor1.enableVoltageCompensation();
        liftMotor2.enableVoltageCompensation();
        armMotor.enableVoltageCompensation();

        armProfile = new MotionProfile(ArmLiftConstants.MAX_ARM_ACCELERATION, ArmLiftConstants.MAX_ARM_VELOCITY);
        liftProfile = new MotionProfile(ArmLiftConstants.MAX_LIFT_ACCELERATION, ArmLiftConstants.MAX_LIFT_VELOCITY);
        armPIDC = new PIDController(ArmLiftConstants.ARM_POS_P_GAIN, ArmLiftConstants.ARM_POS_I_GAIN
        , ArmLiftConstants.ARM_VEL_P_GAIN, ArmLiftConstants.MAX_INTEGRAL);
        liftPIDC = new PIDController(ArmLiftConstants.LIFT_POS_P_GAIN, ArmLiftConstants.LIFT_POS_I_GAIN
        , ArmLiftConstants.LIFT_VEL_P_GAIN, ArmLiftConstants.MAX_INTEGRAL);
    
    
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
                gameState = gameStates.GROUND_INTAKE;

                currentArmPos = armMotor.getController().getAbsoluteEncoder().getPosition() * (2*Math.PI); // multiplies encode value of 0-1 by 2pi for radians
                currentLiftPos = (liftPotentiometer.getValue() / 5) * 20; // Inch height of lift

                
                armProfile.calculate(currentArmPos, ArmLiftConstants.GROUND_INTAKE_RIGHT_ANGLE);
                liftProfile.calculate(currentLiftPos, ArmLiftConstants.MIN_LIFT_HEIGHT);

            }else if(dpadLeft.getValue()){
                liftCase = liftStates.L2_ALGAE_REEF;
            }
            else if(dpadRight.getValue()){
                liftCase = liftStates.L3_ALGAE_REEF;
            }
            else if (dpadUp.getValue()){
                liftCase = liftStates.SHOOT_NET;
            }
            else if(driverFaceLeft.getValue()){
                liftCase = liftStates.STORAGE;
            }
        }else{
            liftCase = liftStates.STORAGE;
            armDirection = 0;
        }
    
    }

    public void update(){
        testLift();
        
    }

    // Press sensetive lift (not done)
    public void testAnalogLift(){
        
    }

    private void competitionControlSystem(){
         switch (gameState){
            case GROUND_INTAKE:
                if(no algae){
                    armSystem(currentArmPos);
                    liftSystem(currentLiftPos);
                }
                
                
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

    

    
    public double armSystem(double currentPosition){
        
        double goalPos = armProfile.getSamples()[0];
        double goalVel = armProfile.getSamples()[1];
        double goalAcc = armProfile.getSamples()[2];

        double accelFF = goalAcc * ArmLiftConstants.ARM_ACCEL_GAIN;
        double posFF = Math.cos(currentPosition) * ArmLiftConstants.ARM_ANGLE_GAIN;
        double positionPI = armPIDC.positionPIController(currentPosition, goalPos);

        double velocityP = armPIDC.velocityPController(positionPI + goalVel, currentPosition);
        
        return accelFF + posFF + velocityP;
        
      
        
    }

    
    public double liftSystem(double currentPosition){

        double goalPos = liftProfile.getSamples()[0];
        double goalVel = liftProfile.getSamples()[1];
        double goalAcc = liftProfile.getSamples()[2];

        double accelFF = goalAcc * ArmLiftConstants.LIFT_ACCEL_GAIN;
        double posFF = Math.cos(currentPosition) * ArmLiftConstants.LIFT_POS_GAIN;
        double positionPI = liftPIDC.positionPIController(currentPosition, goalPos);

        double velocityP = liftPIDC.velocityPController(positionPI + goalVel, currentPosition);
        
        return accelFF + posFF + velocityP;
       
        
    }

    
    public void selfTest(){
        
    }

    @Override
    public void resetState() {
        liftSpeed = 0;
        liftCase = liftDirection.STORAGE;
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
