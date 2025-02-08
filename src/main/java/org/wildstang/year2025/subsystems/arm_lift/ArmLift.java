package org.wildstang.year2025.subsystems.arm_lift;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;

import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.framework.core.Core;

import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.year2025.robot.WsInputs;
import org.wildstang.year2025.robot.WsOutputs;
import org.wildstang.year2025.subsystems.Claw.Claw;
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

    double armTorque;
    double liftTorque;

    /*Inputs Variables*/
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
    private double currentLiftPos;
    private enum gameStates {GROUND_INTAKE, L2_ALGAE_REEF, L3_ALGAE_REEF, STORAGE, SCORE_PRELOAD, SHOOT_NET, START}; // Our Arm/Lift States
    private gameStates gameState;
    
   

    /* Arm Variables */
    private double currentArmAngle;
    public int armDirection; /* + is clockwise and - is counterclockwise */
    private WsSpark armMotor;
    private double armSetpoint;
    private double liftSetpoint;
    //private boolean activateArm = false;
    //private int inputType = 0;

    /*Other */
    private boolean recalculateFlag;
    private double validArmAngle;
    private double validLiftHeight;
    private double[] validSetpoints;
    


     @Override
    public void init(){
        initOutput();
        initInputs();
        recalculateFlag = false;
        armProfile = new MotionProfile(ArmLiftConstants.MAX_ARM_ACCELERATION, ArmLiftConstants.MAX_ARM_VELOCITY);
        liftProfile = new MotionProfile(ArmLiftConstants.MAX_LIFT_ACCELERATION, ArmLiftConstants.MAX_LIFT_VELOCITY);
        armPIDC = new PIDController(ArmLiftConstants.ARM_POS_P_GAIN, ArmLiftConstants.ARM_POS_I_GAIN
        , ArmLiftConstants.ARM_VEL_P_GAIN, ArmLiftConstants.MAX_INTEGRAL);
        liftPIDC = new PIDController(ArmLiftConstants.LIFT_POS_P_GAIN, ArmLiftConstants.LIFT_POS_I_GAIN
        , ArmLiftConstants.LIFT_VEL_P_GAIN, ArmLiftConstants.MAX_INTEGRAL);

    }

    public void initOutput(){
        liftMotor1 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.LIFTONE);
        liftMotor2 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.LIFTTWO);
        liftMotor1.setBrake();
        liftMotor2.setBrake();
       // liftMotor1.setCurrentLimit(armDirection, armDirection, armDirection);
       
        armMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ARMMOTOR);
        liftMotor1.enableVoltageCompensation();
        liftMotor2.enableVoltageCompensation();
        armMotor.enableVoltageCompensation();
    
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

                //preset setpoints
                armSetpoint = ArmLiftConstants.GROUND_INTAKE_RIGHT_ANGLE;
                liftSetpoint = ArmLiftConstants.MIN_LIFT_HEIGHT;
               
               calculateValidProfile();

            }else if(dpadLeft.getValue()){
                gameState = gameStates.L2_ALGAE_REEF;

                //preset setpoints
                armSetpoint = ArmLiftConstants.L2_ANGLE;
                liftSetpoint = ArmLiftConstants.L2_LIFT_HEIGHT;

                calculateValidProfile();
                
            }
            else if(dpadRight.getValue()){
                gameState = gameStates.L3_ALGAE_REEF;

                //preset setpoints
                armSetpoint = ArmLiftConstants.L3_ANGLE;
                liftSetpoint = ArmLiftConstants.L3_LIFT_HEIGHT;
                
                calculateValidProfile();
            }
            else if (dpadUp.getValue()){
                gameState = gameStates.SHOOT_NET;

                //preset setpoints
                armSetpoint = ArmLiftConstants.SHOOT_NET_ANGLE;
                liftSetpoint = ArmLiftConstants.SHOOT_NET_LIFT_HEIGHT;

                calculateValidProfile();
            }
            else if(driverFaceLeft.getValue()){
                gameState = gameStates.STORAGE;
                
                //preset setpoints
                armSetpoint = ArmLiftConstants.STORAGE_ANGLE;
                liftSetpoint = ArmLiftConstants.STORAGE_LIFT_HEIGHT;

                calculateValidProfile();
            } 
            ;}
    }
    

    public void update(){
        testAnalogSubsystem();
        //competitionControlSystem();
        
    }

    private void calculateValidProfile(){
        //getting setpoints within proper bounds
        validSetpoints = getValidSeptpoints(liftSetpoint, currentLiftPos, armSetpoint, currentArmAngle);
        validArmAngle = validSetpoints[0];
        validLiftHeight = validSetpoints[1];


        //multiple stage profile 
        if (liftSetpoint != validLiftHeight || armSetpoint != validArmAngle){
            recalculateFlag = true;
        }

        //generate a motion profile for the arm and the lift
        armProfile.calculate(currentArmAngle,validArmAngle);
        liftProfile.calculate(currentLiftPos, validLiftHeight);
    }

    // Press sensetive lift (not done)
    public void testAnalogSubsystem(){
        
        liftMotor1.setSpeed(leftJoyStickY.getValue());
        liftMotor2.setSpeed(leftJoyStickY.getValue());

        armMotor.setSpeed(rightJoyStickX.getValue());
    }

    private void competitionControlSystem(){

         //get current positions of arm and lift
         currentArmAngle = armMotor.getController().getAbsoluteEncoder().getPosition() * (2*Math.PI); // multiplies encode value of 0-1 by 2pi for radians
         currentLiftPos = getLiftHeight(liftMotor1.getController().getAnalog().getVoltage());

         if(recalculateFlag){
            double[] validSetpoints = getValidSeptpoints(currentLiftPos, liftSetpoint, currentArmAngle, armSetpoint);
            if(armProfile.profileDone && liftProfile.profileDone && validSetpoints[0] == armSetpoint && validSetpoints[1] == liftSetpoint){
                armProfile.calculate(currentArmAngle,armSetpoint);
                liftProfile.calculate(currentLiftPos, liftSetpoint);
            }
         }

         switch (gameState){
            case GROUND_INTAKE:
                armMotor.setSpeed(armControlOutput(currentArmAngle));
                liftMotor1.setSpeed(liftControlOutput(currentLiftPos));
                liftMotor2.setSpeed(liftControlOutput(-currentLiftPos));
                break;
            case L2_ALGAE_REEF:
                armMotor.setSpeed(armControlOutput(currentArmAngle));
                liftMotor1.setSpeed(liftControlOutput(currentLiftPos));
                liftMotor2.setSpeed(liftControlOutput(-currentLiftPos));
                break;
            case L3_ALGAE_REEF:
                armMotor.setSpeed(armControlOutput(currentArmAngle));
                liftMotor1.setSpeed(liftControlOutput(currentLiftPos));
                liftMotor2.setSpeed(liftControlOutput(-currentLiftPos));
                break;
            case SHOOT_NET:
                armMotor.setSpeed(armControlOutput(currentArmAngle));
                liftMotor1.setSpeed(liftControlOutput(currentLiftPos));
                liftMotor2.setSpeed(liftControlOutput(-currentLiftPos));
                break;

            default:
                liftMotor1.stop();
                liftMotor2.stop();
        }
        putDashboard();
    }

    private void putDashboard(){
        SmartDashboard.putBoolean("Algae in claw", Claw.algaeInClaw);
        SmartDashboard.putNumber("Current Arm Angle", currentArmAngle);
        SmartDashboard.putNumber("Current Lift Height", currentLiftPos);
        SmartDashboard.putNumber("Arm control output", armControlOutput(currentArmAngle));
        SmartDashboard.putNumber("Lift Control Output", liftControlOutput(currentLiftPos));
        SmartDashboard.putString("Current State", gameState.name());
        SmartDashboard.putNumber("Arm set point", armSetpoint);
        SmartDashboard.putNumber("Lift setpoint", liftSetpoint);

    }


    //calculating lift height from a function of voltage
    private double getLiftHeight(double potentiometerVoltage){
        double slopeOfHeight = ArmLiftConstants.MAX_LIFT_HEIGHT;
        return slopeOfHeight * (potentiometerVoltage - ArmLiftConstants.MAX_POTENTIOMETER_VOLTAGE) + ArmLiftConstants.MAX_LIFT_HEIGHT;
    }

    
    //generates an output for the motor with an acceleration feedforward, position feedforward, and PID
    public double armControlOutput(double currentAngle){
        
        double goalPos = armProfile.getSamples()[0];
        double goalVel = armProfile.getSamples()[1];
        double goalAcc = armProfile.getSamples()[2];

        double FF = 12 * (getCurrentArmTorque(goalAcc, currentAngle) / getMaxArmTorque(goalVel));

        double positionPI = armPIDC.positionPIController(goalPos, currentAngle);
        double velocityP = armPIDC.velocityPController(positionPI + goalVel, currentAngle);
        SmartDashboard.putNumber("Arm Position PI", positionPI);
        SmartDashboard.putNumber("Arm Velocity P", velocityP);
        return FF + velocityP;
        
    }


    //generates an output for the motor with an acceleration feedforward, position feedforward, and PID
    public double liftControlOutput(double currentPosition){

        double goalPos = liftProfile.getSamples()[0];
        double goalVel = liftProfile.getSamples()[1];
        double goalAcc = liftProfile.getSamples()[2];


        double positionPI = liftPIDC.positionPIController(currentPosition, goalPos);
        double velocityP = liftPIDC.velocityPController(positionPI + goalVel, currentPosition);

        SmartDashboard.putNumber("Lift Position PI", positionPI);
        SmartDashboard.putNumber("Lift Velocity P", velocityP);
        
        double FF = 12 * (getCurrentLiftTorque(goalAcc) / getMaxLiftTorque(goalVel));
        
        return velocityP + FF;
        
    }


    //returns valid setpoints for the arm and lift based on current positions 
    public double[] getValidSeptpoints(double goalLiftPos, double curLiftPos, double goalArmAngle, double curArmAngle){
        double validArmAngle = goalArmAngle;
        double validLiftHeight = goalLiftPos;
        //If Lift is at a low position, make sure arm angle is within a threshold so claw doesn't hit bumpers or lift
        if (curLiftPos < ArmLiftConstants.LOW_LIFT_HEIGHT){
            validArmAngle = Math.max(Math.min(goalArmAngle, ArmLiftConstants.MAX_LOW_ARM_ANGLE), ArmLiftConstants.MIN_LOW_ARM_ANGLE);
        }

        //if lift is at a high position, ensure arm angle doesn't go too low so that claw hits the lift
        else if (curLiftPos < ArmLiftConstants.HIGH_LIFT_HEIGHT && Claw.algaeInClaw){
            validArmAngle = Math.max(goalArmAngle,ArmLiftConstants.MIN_HIGH_ARM_ANGLE);
        }

        //if claw is angled up above the lift, make sure not to bring the lift down too low or it will hit the algae
        else if(curArmAngle  < ArmLiftConstants.MAX_LIFT_DOWN_ANGLE && curArmAngle > ArmLiftConstants.MIN_LIFT_DOWN_ANGLE){
            validLiftHeight = Math.max(goalLiftPos, ArmLiftConstants.LOW_LIFT_HEIGHT);
        }
    
        return new double[]{validArmAngle, validLiftHeight};

    }

    // Get the Torque of the 12 volt motor curve
    private double getMaxLiftTorque(double goalVel){
        double rotationalSpeed = goalVel/ArmLiftConstants.PULLEY_RADIUS;
        liftTorque = (-(ArmLiftConstants.STALL/ArmLiftConstants.FREE_SPEED)*rotationalSpeed) + ArmLiftConstants.STALL;
        return liftTorque;
    }

    private double getCurrentLiftTorque(double goalAccel){

        double Force = ArmLiftConstants.LIFT_ARM_MASS * (ArmLiftConstants.GRAVITY + goalAccel);
        double currentTorque = Force * ArmLiftConstants.PULLEY_RADIUS;
        return currentTorque;
    }

    private double getMaxArmTorque(double goalVel){
        double rotationalSpeed = goalVel/ArmLiftConstants.PULLEY_RADIUS;
        armTorque = (-(ArmLiftConstants.STALL/ArmLiftConstants.FREE_SPEED)*rotationalSpeed) + ArmLiftConstants.STALL;
        return armTorque;
    }

    private double getCurrentArmTorque(double goalAccel, double currentAngle){

        
        double currentTorque = 
            (ArmLiftConstants.ROTATIONAL_MASS * Math.cos(currentAngle) * ArmLiftConstants.GRAVITY) 
            + 
            (ArmLiftConstants.MOMEMENT_OF_INERTIA*goalAccel);
        return currentTorque;
    }

    public void selfTest(){
        
    }

    @Override
    public void resetState() {
        gameState = gameStates.STORAGE;
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
