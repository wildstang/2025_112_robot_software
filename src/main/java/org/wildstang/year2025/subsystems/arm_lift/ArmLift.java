package org.wildstang.year2025.subsystems.arm_lift;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.logger.Log;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;

import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.framework.core.Core;

import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.year2025.robot.WsInputs;
import org.wildstang.year2025.robot.WsOutputs;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.Claw.Claw;

import com.revrobotics.spark.SparkAnalogSensor;

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
    private DigitalInput faceDown; // Ground Intake Button
    private DigitalInput faceLeft; // L2 Algae Reef Intake
    private DigitalInput faceRight; // L3 Algae Reef Intake
    private DigitalInput faceUp; // Shoot Net
    private AnalogInput leftJoyStickY;
    private AnalogInput rightJoyStickX;
    private AnalogInput leftTrigger;
    
    /* Lift Variables */
    private WsSpark liftMotor1;
    private WsSpark liftMotor2;
    private SparkAnalogSensor liftPot;
    private double currentLiftPos, currentLiftVel;
    public enum gameStates {GROUND_INTAKE, L2_ALGAE_REEF, L3_ALGAE_REEF, STORAGE, SCORE_PRELOAD, SHOOT_NET, START, CORAL_INTAKE}; // Our Arm/Lift States
    private gameStates gameState = gameStates.START;
    
    /* Arm Variables */
    private double currentArmAngle, currentArmVel;
    private WsSpark armMotor;
    private double armSetpoint;
    private double liftSetpoint;

    /*Other */
    private boolean liftRecalculateFlag, armRecalculateFlag;
    private double validArmAngle;
    private double validLiftHeight;
    private Claw claw;
    private boolean isFront;

     @Override
    public void init(){
        initOutput();
        initInputs();
        liftPot = liftMotor1.getController().getAnalog();
        currentArmAngle = getArmAngle();
        currentLiftPos = getLiftHeight();
        liftRecalculateFlag = false;
        armRecalculateFlag = false;
        armProfile = new MotionProfile(ArmLiftConstants.MAX_ARM_ACCELERATION
        , ArmLiftConstants.MAX_ARM_VELOCITY, currentArmAngle);
        liftProfile = new MotionProfile(ArmLiftConstants.MAX_LIFT_ACCELERATION
        , ArmLiftConstants.MAX_LIFT_VELOCITY, currentLiftPos);
        armPIDC = new PIDController(ArmLiftConstants.ARM_POS_P_GAIN, ArmLiftConstants.ARM_POS_I_GAIN
        , ArmLiftConstants.ARM_VEL_P_GAIN, 1.0);
        liftPIDC = new PIDController(ArmLiftConstants.LIFT_POS_P_GAIN, ArmLiftConstants.LIFT_POS_I_GAIN
        , ArmLiftConstants.LIFT_VEL_P_GAIN, 0.5);
    }

    public void initOutput(){
        liftMotor1 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.LIFTONE);
        liftMotor2 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.LIFTTWO);
        liftMotor1.setBrake();
        liftMotor2.setBrake();
       
        armMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ARMMOTOR);
    }

    public void initInputs(){
        faceUp = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_FACE_UP);
        faceUp.addInputListener(this);
        faceLeft = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_FACE_LEFT);
        faceLeft.addInputListener(this);
        faceRight = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_FACE_RIGHT);
        faceRight.addInputListener(this);
        faceDown = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_FACE_DOWN);
        faceDown.addInputListener(this);
        leftTrigger = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_TRIGGER);
        leftTrigger.addInputListener(this);
        leftJoyStickY = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_JOYSTICK_Y);
        // leftJoyStickY.addInputListener(this);
        rightJoyStickX = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_JOYSTICK_X);
        // rightJoyStickX.addInputListener(this);
    }

    @Override
    public void initSubsystems() {
        claw = (Claw) Core.getSubsystemManager().getSubsystem(WsSubsystems.CLAW);
    }

    public void inputUpdate(Input source){
        Log.warn(source.getName());
        if (faceDown.getValue()) {
            setGameState(gameStates.STORAGE, isFront);
        } else if (faceLeft.getValue()) {
            setGameState(gameStates.L2_ALGAE_REEF);
        } else if (faceRight.getValue()) {
            setGameState(gameStates.L3_ALGAE_REEF);
        } else if (faceUp.getValue()) {
            setGameState(gameStates.SHOOT_NET);
        } else if (leftTrigger.getValue() != 0) {
            setGameState(gameStates.GROUND_INTAKE);
        }
    }

    public void update(){
        competitionControlSystem();
        // testAnalogSubsystem();
        putDashboard();
    }

    private void calculateValidProfile(){
        //getting setpoints within proper bounds
        double[] validSetpoints = getValidSeptpoints(liftSetpoint, currentLiftPos, armSetpoint, currentArmAngle);
        validArmAngle = validSetpoints[0];
        validLiftHeight = validSetpoints[1];

        //multiple stage profile 
        if (liftSetpoint != validLiftHeight) {
            liftRecalculateFlag = true;
        }
        if (armSetpoint != validArmAngle) {
            armRecalculateFlag = true;
        }
        armPIDC.resetIVal();
        liftPIDC.resetIVal();

        if ( armProfile.profileDone){
        //generate a motion profile for the arm and the lift
            armProfile.calculate(currentArmAngle,validArmAngle);
        } else {
            armRecalculateFlag = true;
        }
        if (liftProfile.profileDone) {
            liftProfile.calculate(currentLiftPos, validLiftHeight);
        } else {
            liftRecalculateFlag = true;
        }
    }

    private void testAnalogSubsystem(){
        liftMotor1.setSpeed(leftJoyStickY.getValue());
        liftMotor2.setSpeed(-leftJoyStickY.getValue());
        SmartDashboard.putNumber("Lift Speed", leftJoyStickY.getValue());

        armMotor.setSpeed(rightJoyStickX.getValue());
        SmartDashboard.putNumber("Arm Speed", rightJoyStickX.getValue());
    }

    private void competitionControlSystem(){
         //get current positions of arm and lift
         currentArmAngle = getArmAngle(); // multiplies encode value of 0-1 by 2pi for radians
         currentArmVel = getArmVel();
         currentLiftPos = getLiftHeight();
         currentLiftVel = getLiftVel();

         if(liftRecalculateFlag && liftProfile.profileDone){
            double[] validSetpoints = getValidSeptpoints(liftSetpoint, currentLiftPos, armSetpoint, currentArmAngle);
            liftRecalculateFlag = validSetpoints[1] != liftSetpoint;
            liftPIDC.resetIVal();
            liftProfile.calculate(currentLiftPos, validSetpoints[1]);
        }
        if (armRecalculateFlag && armProfile.profileDone){
            double[] validSetpoints = getValidSeptpoints(liftSetpoint, currentLiftPos, armSetpoint, currentArmAngle);
            armRecalculateFlag = validSetpoints[0] != armSetpoint;
            armPIDC.resetIVal();
            armProfile.calculate(currentArmAngle, validSetpoints[0]);
        }

        armMotor.setSpeed(armControlOutput(currentArmAngle, currentArmVel));
        liftMotor1.setSpeed(liftControlOutput(currentLiftPos, currentLiftVel));
        liftMotor2.setSpeed(-liftControlOutput(currentLiftPos, currentLiftVel));
    }

    private void putDashboard(){
        SmartDashboard.putNumber("Lift Pot Voltage", liftPot.getPosition());
        SmartDashboard.putNumber("Current Arm Angle", getArmAngle());
        SmartDashboard.putNumber("Current Lift Height", getLiftHeight());
        SmartDashboard.putNumber("Arm control output", armControlOutput(currentArmAngle, currentArmVel));
        SmartDashboard.putNumber("Lift Control Output", liftControlOutput(currentLiftPos, currentArmVel));
        SmartDashboard.putString("Current State", gameState.name());
        SmartDashboard.putNumber("Arm set point", armSetpoint);
        SmartDashboard.putNumber("Lift setpoint", liftSetpoint);
        SmartDashboard.putNumber("Lift Velocity", currentLiftVel);
        SmartDashboard.putNumber("Arm Velocity", currentArmVel);
        SmartDashboard.putNumber("Valid arm angle", validArmAngle);
        SmartDashboard.putNumber("Valid Lift Height", validLiftHeight);
        SmartDashboard.putBoolean("Score Front", isFront);
    }

    //calculating lift height from a function of voltage
    private double getLiftHeight(){
        // double slope = (ArmLiftConstants.MAX_LIFT_HEIGHT-ArmLiftConstants.MIN_LIFT_HEIGHT) / (ArmLiftConstants.MAX_POTENTIOMETER_VOLTAGE - ArmLiftConstants.MIN_POTENTIOMETER_VOLTAGE);
        // return (liftMotor1.getController().getAnalog().getVoltage() - ArmLiftConstants.MIN_POTENTIOMETER_VOLTAGE) * slope + ArmLiftConstants.MIN_LIFT_HEIGHT;
        return liftMotor1.getPosition() / ArmLiftConstants.LIFT_GEAR_RATIO * 2.0 * Math.PI * ArmLiftConstants.LIFT_PULLEY_RADIUS + ArmLiftConstants.START_LIFT_HEIGHT;
    }

    private double getLiftVel() {
        return liftMotor1.getVelocity() / ArmLiftConstants.LIFT_GEAR_RATIO * 2.0 * Math.PI * ArmLiftConstants.LIFT_PULLEY_RADIUS / 60.0;
    }

    private double getArmAngle() {
        return (armMotor.getPosition() / ArmLiftConstants.ARM_GEAR_RATO * 2.0 * Math.PI);
    }

    private double getArmVel() {
        return(armMotor.getVelocity() / ArmLiftConstants.ARM_GEAR_RATO * 2.0 * Math.PI / 60.0 );
    }
    
    //generates an output for the motor with an acceleration feedforward, position feedforward, and PID
    public double armControlOutput(double currentAngle, double curVel){
        double[] curTarget = armProfile.getSamples();
        double goalPos = curTarget[0];
        double goalVel = curTarget[1] + armPIDC.positionPIController(goalPos, currentAngle);
        double goalAcc = curTarget[2];
        SmartDashboard.putNumber("Arm GoalPos", goalPos);
        SmartDashboard.putNumber("Arm GoalVel", goalVel);
        double FF = getCurrentArmTorque(goalAcc, currentAngle) / getMaxArmTorque(goalVel);
        return armPIDC.velocityPController(goalVel, curVel) + FF;
    }

    //generates an output for the motor with an acceleration feedforward, position feedforward, and PID
    public double liftControlOutput(double currentPosition, double curVel){
        double[] curTarget = liftProfile.getSamples();
        double goalPos = curTarget[0];
        double goalVel = curTarget[1] + liftPIDC.positionPIController(goalPos, currentPosition);
        double goalAcc = curTarget[2];
        SmartDashboard.putNumber("Lift GoalPos", goalPos);
        SmartDashboard.putNumber("Lift GoalVel", goalVel);
        double FF = getCurrentLiftForce(goalAcc) / getMaxLiftForce(goalVel);
        return liftPIDC.velocityPController(goalVel, curVel) + FF;
    }

    /** returns valid setpoints for the arm and lift based on current positions 
     * @return {arm angle, lift height}
     */
    public double[] getValidSeptpoints(double goalLiftPos, double curLiftPos, double goalArmAngle, double curArmAngle){
        double validArmAngle = goalArmAngle;
        double validLiftHeight = goalLiftPos;
        if (curArmAngle < ArmLiftConstants.ARM_POWER_CHAIN_LOW_ANGLE || curArmAngle > ArmLiftConstants.ARM_POWER_CHAIN_HIGH_ANGLE) {
            if (curLiftPos < ArmLiftConstants.POWER_CHAIN_LIFT_HEIGHT_MIN || curLiftPos > ArmLiftConstants.POWER_CHAIN_LIFT_HEIGHT_MAX){
                validArmAngle = curArmAngle;
                validLiftHeight = ArmLiftConstants.POWER_CHAIN_LIFT_HEIGHT;
                return new double[]{validArmAngle, validLiftHeight};
            } else {
                validLiftHeight = ArmLiftConstants.POWER_CHAIN_LIFT_HEIGHT;
                return new double[]{validArmAngle, validLiftHeight};
            }
        }
        //If Lift is at a low position, make sure arm angle is within a threshold so claw doesn't hit bumpers on both sides
        if (curLiftPos < ArmLiftConstants.LOW_LIFT_HEIGHT){
            validArmAngle = Math.max(Math.min(validArmAngle, ArmLiftConstants.MAX_LOW_ARM_ANGLE), ArmLiftConstants.MIN_LOW_ARM_ANGLE);
        }
        if(claw.algaeInClaw){
            //if lift is at a high position, ensure arm angle doesn't go too low so that claw w/ algae hits the lift
            if (curLiftPos < ArmLiftConstants.LOW_LIFT_HEIGHT){
                validArmAngle = Math.max(Math.min(validArmAngle, ArmLiftConstants.ARM_POWER_CHAIN_HIGH_ANGLE),ArmLiftConstants.MIN_HIGH_ARM_ANGLE);
            }
            //if claw is angled up above the lift, make sure not to bring the lift down too low or it will hit the algae
            if(curArmAngle  < ArmLiftConstants.MAX_LIFT_DOWN_ANGLE && curArmAngle > ArmLiftConstants.MIN_LIFT_DOWN_ANGLE){
                validLiftHeight = Math.max(validLiftHeight, ArmLiftConstants.ALGAE_TOP_CLEARANCE);
            }
            if (curLiftPos < ArmLiftConstants.ALGAE_TOP_CLEARANCE){
                if (curArmAngle < ArmLiftConstants.MIN_LIFT_DOWN_ANGLE){
                    validArmAngle = Math.min(validArmAngle, ArmLiftConstants.MIN_LIFT_DOWN_ANGLE);
                } else if (curArmAngle > ArmLiftConstants.MAX_LIFT_DOWN_ANGLE){
                    validArmAngle = Math.max(validArmAngle, ArmLiftConstants.MAX_LIFT_DOWN_ANGLE);
                }
            }
        } else {
            //if lift is at a low position, ensure arm doesn't move out of a threshold where it might hit the power chain
            if(curLiftPos > ArmLiftConstants.POWER_CHAIN_LIFT_HEIGHT_MAX) {
                validArmAngle = Math.max(Math.min(validArmAngle, ArmLiftConstants.ARM_POWER_CHAIN_HIGH_ANGLE), ArmLiftConstants.ARM_POWER_CHAIN_LOW_ANGLE);
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
         //mx + b where b is stall torque and slope is change in torque over angular velocity
        return ArmLiftConstants.ARM_STALL * (1 - goalVel/ArmLiftConstants.ARM_FREE_SPEED);
    }

    private double getCurrentArmTorque(double goalAccel, double currentAngle){
        //gravitational torque +  (moment of intertia * accleration)
        return (ArmLiftConstants.ARM_MASS * ArmLiftConstants.GRAVITY * ArmLiftConstants.ARM_COM_RADIUS * Math.sin(currentAngle)) 
        + (ArmLiftConstants.ARM_MOI * goalAccel);
    }

    public void setGameState(gameStates newState) {
        setGameState(newState, !isFront);
    }

    public void setGameState(gameStates newState, boolean isFront){
        this.isFront = isFront;
        switch (newState) {
            case GROUND_INTAKE:
                this.isFront = true;
                gameState = gameStates.GROUND_INTAKE;
                armSetpoint = ArmLiftConstants.GROUND_INTAKE_RIGHT_ANGLE;
                liftSetpoint = ArmLiftConstants.GROUND_INTAKE_LIFT_HEIGHT;
                break;
            case STORAGE:
                gameState = gameStates.STORAGE;
                armSetpoint = ArmLiftConstants.STORAGE_ANGLE;
                liftSetpoint = ArmLiftConstants.STORAGE_LIFT_HEIGHT;
                break;
            case L2_ALGAE_REEF:
                gameState = gameStates.L2_ALGAE_REEF;
                armSetpoint = this.isFront ? ArmLiftConstants.L2_INTAKE_ANGLE : 2 * Math.PI - ArmLiftConstants.L2_INTAKE_ANGLE;
                liftSetpoint = ArmLiftConstants.L2_INTAKE_LIFT_HEIGHT;
                break;
            case L3_ALGAE_REEF:
                gameState = gameStates.L3_ALGAE_REEF;
                armSetpoint = this.isFront ? ArmLiftConstants.L3_INTAKE_ANGLE : 2 * Math.PI - ArmLiftConstants.L3_INTAKE_ANGLE;
                liftSetpoint = ArmLiftConstants.L3_INTAKE_LIFT_HEIGHT;
                break;
            case SHOOT_NET:
                gameState = gameStates.SHOOT_NET;
                armSetpoint = this.isFront ? ArmLiftConstants.SHOOT_NET_ANGLE : 2 * Math.PI - ArmLiftConstants.SHOOT_NET_ANGLE;
                liftSetpoint = ArmLiftConstants.SHOOT_NET_LIFT_HEIGHT;
                break;
            case CORAL_INTAKE:
                this.isFront = false;
                gameState = gameStates.CORAL_INTAKE;
                armSetpoint = ArmLiftConstants.CORAL_STATION_ANGLE;
                liftSetpoint = ArmLiftConstants.CORAL_STATION_HEIGHT;
                break;
            default:
                break;
        }
        calculateValidProfile();
    }

    public void selfTest(){
    }

    @Override
    public void resetState() {
    }

    @Override
    public String getName() {
        return "Arm Lift";
    }
   
}
