package org.wildstang.year2025.subsystems.arm_lift;

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

/**
 * Interface describing a subsystem class.
 */
public class ArmLift implements Subsystem {
    /* Control System Variables */
    PIDController armPIDC;
    PIDController liftPIDC;
    MotionProfile armProfile;
    MotionProfile liftProfile;

    /*Input Variables*/
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
    private gameStates gameState = gameStates.STORAGE;
    
    /* Arm Variables */
    private double currentArmAngle;
    private WsSpark armMotor;
    private double armSetpoint;
    private double liftSetpoint;

    /*Other */
    private boolean recalculateFlag;
    private double validArmAngle;
    private double validLiftHeight;

     @Override
    public void init(){
        initOutput();
        initInputs();
        recalculateFlag = false;
        armProfile = new MotionProfile(ArmLiftConstants.MAX_ARM_ACCELERATION
        , ArmLiftConstants.MAX_ARM_VELOCITY, ArmLiftConstants.ARM_SMALL_DELTA_P);
        liftProfile = new MotionProfile(ArmLiftConstants.MAX_LIFT_ACCELERATION
        , ArmLiftConstants.MAX_LIFT_VELOCITY, ArmLiftConstants.LIFT_SMALL_DELTA_P);
        armPIDC = new PIDController(ArmLiftConstants.ARM_POS_P_GAIN, ArmLiftConstants.ARM_POS_I_GAIN
        , ArmLiftConstants.ARM_VEL_P_GAIN, ArmLiftConstants.MAX_ARM_VELOCITY);
        liftPIDC = new PIDController(ArmLiftConstants.LIFT_POS_P_GAIN, ArmLiftConstants.LIFT_POS_I_GAIN
        , ArmLiftConstants.LIFT_VEL_P_GAIN, ArmLiftConstants.MAX_LIFT_VELOCITY);
    }

    public void initOutput(){
        liftMotor1 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.LIFTONE);
        liftMotor2 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.LIFTTWO);
             liftMotor1.setBrake();
        liftMotor2.setBrake();
// liftMotor1.setCurrentLimit(armDirection, armDirection, armDirection);
       
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
        leftJoyStickY = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_JOYSTICK_Y);
        // leftJoyStickY.addInputListener(this);
        rightJoyStickX = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_JOYSTICK_X);
        // rightJoyStickX.addInputListener(this);
    }

    public void inputUpdate(Input source){
        if(source == dpadRight || source == dpadLeft || source == dpadDown || source == dpadUp || source == driverFaceLeft){
            if (dpadDown.getValue()) {
                gameState = gameStates.GROUND_INTAKE;

                //preset setpoints
                armSetpoint = ArmLiftConstants.GROUND_INTAKE_RIGHT_ANGLE;
                liftSetpoint = ArmLiftConstants.MIN_LIFT_HEIGHT;
            } else if (dpadLeft.getValue()) {
                gameState = gameStates.L2_ALGAE_REEF;

                //preset setpoints
                armSetpoint = ArmLiftConstants.L2_ANGLE;
                liftSetpoint = ArmLiftConstants.L2_LIFT_HEIGHT;
            } else if (dpadRight.getValue()) {
                gameState = gameStates.L3_ALGAE_REEF;

                //preset setpoints
                armSetpoint = ArmLiftConstants.L3_ANGLE;
                liftSetpoint = ArmLiftConstants.L3_LIFT_HEIGHT;
            } else if (dpadUp.getValue()) {
                gameState = gameStates.SHOOT_NET;

                //preset setpoints
                armSetpoint = ArmLiftConstants.SHOOT_NET_ANGLE;
                liftSetpoint = ArmLiftConstants.SHOOT_NET_LIFT_HEIGHT;
            } else if (driverFaceLeft.getValue()) {
                gameState = gameStates.STORAGE;
                
                //preset setpoints
                armSetpoint = ArmLiftConstants.STORAGE_ANGLE;
                liftSetpoint = ArmLiftConstants.STORAGE_LIFT_HEIGHT;
            }
                calculateValidProfile();
            } 
    }

    public void update(){
        testAnalogSubsystem();
        putDashboard();
    }

    private void calculateValidProfile(){
        //getting setpoints within proper bounds
        double[] validSetpoints = getValidSeptpoints(liftSetpoint, currentLiftPos, armSetpoint, currentArmAngle);
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
         currentLiftPos = getLiftHeight();

         if(recalculateFlag){
            if(armProfile.profileDone && liftProfile.profileDone) {
            double[] validSetpoints = getValidSeptpoints(currentLiftPos, liftSetpoint, currentArmAngle, armSetpoint);
                if (validSetpoints[0] == armSetpoint && validSetpoints[1] == liftSetpoint){
                armProfile.calculate(currentArmAngle,armSetpoint);
                liftProfile.calculate(currentLiftPos, liftSetpoint);
            }
         }
         }

                armMotor.getController().setVoltage(armControlOutput(currentArmAngle));
                liftMotor1.getController().setVoltage(liftControlOutput(currentLiftPos));
                liftMotor2.getController().setVoltage(liftControlOutput(-currentLiftPos));
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
    private double getLiftHeight(){
        double slope = (ArmLiftConstants.MAX_LIFT_HEIGHT-ArmLiftConstants.MIN_LIFT_HEIGHT) / (ArmLiftConstants.MAX_POTENTIOMETER_VOLTAGE - ArmLiftConstants.MIN_POTENTIOMETER_VOLTAGE);
        return (liftMotor1.getController().getAnalog().getVoltage() - ArmLiftConstants.MIN_POTENTIOMETER_VOLTAGE) * slope + ArmLiftConstants.MIN_LIFT_HEIGHT;
    }
    
    //generates an output for the motor with an acceleration feedforward, position feedforward, and PID
    public double armControlOutput(double currentAngle){
        double[] curTarget = armProfile.getSamples();
        double goalPos = curTarget[0];
        double goalVel = curTarget[1] + armPIDC.positionPIController(goalPos, currentAngle);
        double goalAcc = curTarget[2];

        double FF = 12 * (getCurrentArmTorque(goalAcc, currentAngle) / getMaxArmTorque(goalVel));
        double velocityP = armPIDC.velocityPController(goalVel, currentAngle);
        // SmartDashboard.putNumber("Arm Position PI", positionPI);
        SmartDashboard.putNumber("Arm Velocity P", velocityP);
        return FF + velocityP;
    }

    //generates an output for the motor with an acceleration feedforward, position feedforward, and PID
    public double liftControlOutput(double currentPosition){
        double[] curTarget = liftProfile.getSamples();
        double goalPos = curTarget[0];
        double goalVel = curTarget[1] + liftPIDC.positionPIController(goalPos, currentPosition);
        double goalAcc = curTarget[2];

        double FF = 12 * (getCurrentLiftForce(goalAcc) / getMaxLiftForce(goalVel));
        double velocityP = liftPIDC.velocityPController(goalVel, currentPosition);
        // SmartDashboard.putNumber("Lift Position PI", positionPI);
        SmartDashboard.putNumber("Lift Velocity P", velocityP);
        return velocityP + FF;
    }

    //returns valid setpoints for the arm and lift based on current positions 
    public double[] getValidSeptpoints(double goalLiftPos, double curLiftPos, double goalArmAngle, double curArmAngle){
        double validArmAngle = goalArmAngle;
        double validLiftHeight = goalLiftPos;
        if(Claw.algaeInClaw){
        //if lift is at a high position, ensure arm angle doesn't go too low so that claw w/ algae hits the lift
        if (curLiftPos < ArmLiftConstants.HIGH_LIFT_HEIGHT){
            validArmAngle = Math.max(goalArmAngle,ArmLiftConstants.MIN_HIGH_ARM_ANGLE);
        }
        //if claw is angled up above the lift, make sure not to bring the lift down too low or it will hit the algae
        if(curArmAngle  < ArmLiftConstants.MAX_LIFT_DOWN_ANGLE && curArmAngle > ArmLiftConstants.MIN_LIFT_DOWN_ANGLE){
            validLiftHeight = Math.max(goalLiftPos, ArmLiftConstants.LOW_LIFT_HEIGHT);
        }
     
        }
        else{
        
        //If Lift is at a low position, make sure arm angle is within a threshold so claw doesn't hit bumpers on both sides
        if (curLiftPos < ArmLiftConstants.LOW_LIFT_HEIGHT){
        validArmAngle = Math.max(Math.min(goalArmAngle, ArmLiftConstants.MAX_LOW_ARM_ANGLE), ArmLiftConstants.MIN_LOW_ARM_ANGLE);
        }

        //if lift is at a low position, ensure arm doesn't move out of a threshold where it might hit the power chain
        if(curLiftPos > ArmLiftConstants.LOW_LIFT_HEIGHT && (curArmAngle < ArmLiftConstants.UPPER_BOUND_POWER_CHAIN_ANGLE) 
        && curArmAngle > ArmLiftConstants.LOWER_BOUND_POWER_CHAIN_ANGLE){
            if(goalArmAngle > Math.PI){
                validArmAngle = Math.max(goalArmAngle, ArmLiftConstants.MAX_CLAW_POWER_CHAIN_ANGLE);
            }
            else{
                validArmAngle = Math.min(goalArmAngle, ArmLiftConstants.MIN_CLAW_POWER_CHAIN_ANGLE);
            }
        }
        }
        return new double[]{validArmAngle, validLiftHeight};
    }

    // Get the force of the 12 volt motor curve
    private double getMaxLiftForce(double goalVel){
        //mx + b where b is stall force and slope is change in force over angular velocity
        return ArmLiftConstants.LIFT_STALL_FORCE * (1 - goalVel/ArmLiftConstants.LIFT_FREE_SPEED);
    }

    private double getCurrentLiftForce(double goalAccel){
        //mg + ma
        return ArmLiftConstants.LIFT_ARM_MASS * (ArmLiftConstants.GRAVITY + goalAccel);
    }

    private double getMaxArmTorque(double goalVel){
         //mx + b where b is stall force and slope is change in force over angular velocity
        return ArmLiftConstants.ARM_STALL * (1 - goalVel/ArmLiftConstants.ARM_FREE_SPEED);
    }

    private double getCurrentArmTorque(double goalAccel, double currentAngle){
        //rotational inertia +  (moment of intertia * accleration)
        return (ArmLiftConstants.ARM_MASS * ArmLiftConstants.GRAVITY * ArmLiftConstants.ARM_COM_RADIUS * Math.sin(currentAngle)) 
        + (ArmLiftConstants.ARM_MOI * goalAccel);
    }

    public void selfTest(){
    }

    @Override
    public void resetState() {
        gameState = gameStates.STORAGE;
    }

    @Override
    public void initSubsystems() {
    }

    @Override
    public String getName() {
        return "Arm Lift";
    }
   
}
