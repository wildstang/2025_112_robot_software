package org.wildstang.year2025.subsystems.arm_lift;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;

import org.wildstang.hardware.roborio.outputs.WsSpark;

import org.wildstang.year2025.robot.WsInputs;
import org.wildstang.year2025.robot.WsOutputs;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.Claw.Claw;
import org.wildstang.year2025.subsystems.Claw.Claw.clawStates;
import org.wildstang.year2025.subsystems.LED.LedSubsystem;
import org.wildstang.year2025.subsystems.LED.LedSubsystem.LEDstates;
import org.wildstang.year2025.subsystems.localization.Localization;
import org.wildstang.year2025.subsystems.swerve.SwerveDrive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

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
    private DigitalInput faceDown; // Storage
    private DigitalInput faceLeft; // L2 Algae Reef Intake
    private DigitalInput faceRight; // L3 Algae Reef Intake
    private DigitalInput faceUp; // Shoot Net
    private DigitalInput dpadDown, dpadUp;  // Manual arm angle adjustment
    private DigitalInput dpadLeft, dpadRight;  // Lollipop intake, processor score
    private DigitalInput leftJoyStickButton;
    private AnalogInput leftTrigger;
    private RelativeEncoder armEnc;
    private double manualArmAdjust;

    /* State Machine Variables */
    public enum GameStates {GROUND_INTAKE, L2_ALGAE_REEF, L3_ALGAE_REEF, STORAGE, SHOOT_NET, LOLIPOP, PROCESSOR, CLIMB}; // Our Arm/Lift States
    public GameStates gameState = GameStates.CLIMB;
    public GameStates newState = GameStates.CLIMB;
    private boolean isFront;
    
    /* Lift Variables */
    private WsSpark liftMotor1, liftMotor2;
    private double currentLiftHeight, currentLiftVel;
    private double liftSetpoint;
    private boolean liftRecalculateFlag;
    private double validLiftHeight;
    private double liftOut;
    
    /* Arm Variables */
    private WsSpark armMotor;
    private double currentArmAngle, currentArmVel;
    private double armSetpoint;
    private boolean armRecalculateFlag;
    private double validArmAngle;
    private double armOut;

    /*Other */
    private Claw claw;
    private Localization loc;
    private SwerveDrive swerve;
    private LedSubsystem led;

    @Override
    public void init(){
        initOutput();
        initInputs();
        manualArmAdjust = 0;
        isFront = true;

        currentArmAngle = getArmAngle();
        armSetpoint = currentArmAngle;
        validArmAngle = currentArmAngle;
        armOut = 0;
        armRecalculateFlag = false;

        currentLiftHeight = getLiftHeight();
        liftSetpoint = currentLiftHeight;
        validLiftHeight = currentLiftHeight;
        liftOut = 0;
        liftRecalculateFlag = false;

        armProfile = new MotionProfile(ArmLiftConstants.MAX_ARM_ACCELERATION, ArmLiftConstants.MAX_ARM_VELOCITY, currentArmAngle);
        liftProfile = new MotionProfile(ArmLiftConstants.MAX_LIFT_ACCELERATION, ArmLiftConstants.MAX_LIFT_VELOCITY, currentLiftHeight);
        armPIDC = new PIDController(ArmLiftConstants.ARM_POS_P_GAIN, ArmLiftConstants.ARM_POS_I_GAIN, ArmLiftConstants.ARM_VEL_P_GAIN, 1.0);
        liftPIDC = new PIDController(ArmLiftConstants.LIFT_POS_P_GAIN, ArmLiftConstants.LIFT_POS_I_GAIN, ArmLiftConstants.LIFT_VEL_P_GAIN, 0.5);
    }

    public void initOutput(){
        liftMotor1 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.LIFTONE);
        liftMotor2 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.LIFTTWO);
        liftMotor1.setBrake();
        liftMotor2.setBrake();
       
        armMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ARMMOTOR);
        armMotor.setBrake();
        
        armEnc = ((SparkMax) armMotor.getController()).getAlternateEncoder();
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
        dpadDown = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_DOWN);
        dpadDown.addInputListener(this);
        dpadLeft = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_LEFT);
        dpadLeft.addInputListener(this);
        dpadRight = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_RIGHT);
        dpadRight.addInputListener(this);
        dpadUp = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_UP);
        dpadUp.addInputListener(this);
        leftJoyStickButton = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_JOYSTICK_BUTTON);
        leftJoyStickButton.addInputListener(this);
    }

    @Override
    public void initSubsystems() {
        led = (LedSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.LED);
        claw = (Claw) Core.getSubsystemManager().getSubsystem(WsSubsystems.CLAW);
        loc = (Localization) Core.getSubsystemManager().getSubsystem(WsSubsystems.LOCALIZATION);
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
    }

    public void inputUpdate(Input source){
        if (faceDown.getValue()) {
            if (leftJoyStickButton.getValue()) {
                setGameState(GameStates.CLIMB);
                led.ledState = LEDstates.DEFENSE;
            } else if (source == faceDown) {
                setGameState(GameStates.STORAGE);
                led.ledState = LEDstates.NORMAL;  // if we manually go into storage mode, reset LEDs to normal
            }
        } else if (faceLeft.getValue()) {
            if (swerve.visionOverride){
                setGameState(GameStates.L2_ALGAE_REEF, true);
            } else {
                getArmReefHeight();
            }
        } else if (faceRight.getValue()) {
            if (swerve.visionOverride){
                setGameState(GameStates.L3_ALGAE_REEF, true);
            } else {
                getArmReefHeight();
            }
        } else if (faceUp.getValue()) {
            if (swerve.visionOverride){
                isFront = false;
            } else {
                isFront = loc.getNearestBargeDirection();
            }
            setGameState(GameStates.SHOOT_NET, isFront);
        } else if(dpadLeft.getValue()){
            setGameState(GameStates.LOLIPOP);
        } else if (leftTrigger.getValue() != 0) {
            setGameState(GameStates.GROUND_INTAKE);
        } else if (dpadDown.getValue()) {
            manualArmAdjust -= 0.05;
        } else if(dpadUp.getValue()){
            manualArmAdjust += 0.05;
        } else if (dpadRight.getValue()) {
            if (swerve.visionOverride){
                isFront = false;
            } else {
                isFront = loc.getNearestProcessorDirection();
            }
            setGameState(GameStates.PROCESSOR, isFront);
        }
    }

    public void update(){
        //get current state of arm and lift
        currentArmAngle = getArmAngle();
        currentArmVel = getArmVel();
        currentLiftHeight = getLiftHeight();
        currentLiftVel = getLiftVel();

        if(!swerve.visionOverride && (gameState.equals(GameStates.L2_ALGAE_REEF) || gameState.equals(GameStates.L3_ALGAE_REEF))){
            getArmReefHeight();
        }

        if (armRecalculateFlag && armProfile.profileDone) {
            double[] validSetpoints = getValidSeptpoints(liftSetpoint, currentLiftHeight, armSetpoint, currentArmAngle);
            if (validArmAngle != validSetpoints[0]) {
                validArmAngle = validSetpoints[0];
                armPIDC.resetIVal();
                armProfile.calculate(currentArmAngle, validArmAngle);
            }
        }
        armRecalculateFlag = validArmAngle != armSetpoint;

        if(liftRecalculateFlag && liftProfile.profileDone) {
            double[] validSetpoints = getValidSeptpoints(liftSetpoint, currentLiftHeight, armSetpoint, currentArmAngle);
            if (validLiftHeight != validSetpoints[1]) {
                validLiftHeight = validSetpoints[1];
                liftPIDC.resetIVal();
                liftProfile.calculate(currentLiftHeight, validLiftHeight);
            }
        }
        liftRecalculateFlag = validLiftHeight != liftSetpoint;

        armOut = armControlOutput(currentArmAngle, currentArmVel);
        liftOut = liftControlOutput(currentLiftHeight, currentLiftVel);

        armMotor.setSpeed(armOut);
        liftMotor1.setSpeed(liftOut);
        liftMotor2.setSpeed(liftOut);

        putDashboard();
    }

    private void putDashboard(){
        SmartDashboard.putString("ArmLift State", gameState.name());
        SmartDashboard.putBoolean("Score Front", isFront);
        SmartDashboard.putBoolean("ArmLift is at setpoint", isAtSetpoint());
        SmartDashboard.putNumber("Manual Arm Adjust", manualArmAdjust);

        SmartDashboard.putNumber("Arm Angle", currentArmAngle);
        SmartDashboard.putNumber("Arm control output", armOut);
        SmartDashboard.putNumber("Arm setpoint", armSetpoint);
        SmartDashboard.putNumber("Arm Velocity", currentArmVel);
        SmartDashboard.putNumber("Arm valid angle", validArmAngle);
        SmartDashboard.putBoolean("Arm Profile Done", armProfile.profileDone);
        SmartDashboard.putBoolean("Arm Recalculate Flag", armRecalculateFlag);
        SmartDashboard.putNumber("Arm alt enc", armEnc.getPosition());

        SmartDashboard.putNumber("Lift Height", currentLiftHeight);
        SmartDashboard.putNumber("Lift Control Output", liftOut);
        SmartDashboard.putNumber("Lift setpoint", liftSetpoint);
        SmartDashboard.putNumber("Lift Velocity", currentLiftVel);
        SmartDashboard.putNumber("Lift valid Height", validLiftHeight);
        SmartDashboard.putBoolean("Lift Profile Done", liftProfile.profileDone);
        SmartDashboard.putBoolean("Lift Recalculate Flag", liftRecalculateFlag);
    }

    //calculating lift height from a function of voltage
    private double getLiftHeight(){
        // double slope = (ArmLiftConstants.MAX_LIFT_HEIGHT-ArmLiftConstants.MIN_LIFT_HEIGHT) / (ArmLiftConstants.MAX_POTENTIOMETER_VOLTAGE - ArmLiftConstants.MIN_POTENTIOMETER_VOLTAGE);
        // return (liftMotor1.getController().getAnalog().getVoltage() - ArmLiftConstants.MIN_POTENTIOMETER_VOLTAGE) * slope + ArmLiftConstants.MIN_LIFT_HEIGHT;
        return liftMotor1.getPosition() / ArmLiftConstants.LIFT_GEAR_RATIO * 2.0 * Math.PI * ArmLiftConstants.LIFT_PULLEY_RADIUS + ArmLiftConstants.LIFT_START_HEIGHT;
    }

    private double getLiftVel() {
        return liftMotor1.getVelocity() / ArmLiftConstants.LIFT_GEAR_RATIO * 2.0 * Math.PI * ArmLiftConstants.LIFT_PULLEY_RADIUS / 60.0;
    }

    private double getArmAngle() {
        return (-armEnc.getPosition() * 2.0 * Math.PI / ArmLiftConstants.ARM_ENC_RATIO) - manualArmAdjust;
        // return (armMotor.getPosition() / ArmLiftConstants.ARM_GEAR_RATO * 2.0 * Math.PI) - manualArmAdjust;
    }

    private double getArmVel() {
        return(-armEnc.getVelocity() / ArmLiftConstants.ARM_ENC_RATIO * 2.0 * Math.PI / 60.0 );
        // return(armMotor.getVelocity() / ArmLiftConstants.ARM_GEAR_RATO * 2.0 * Math.PI / 60.0 );
    }

    public void getArmReefHeight(){
        Object[] objArray = loc.getNearestReefHeight();
        setGameState((GameStates) objArray[0], (Boolean) objArray[1]);
    }
    
    //generates an output for the motor with an acceleration feedforward, position feedforward, and PID
    public double armControlOutput(double currentAngle, double curVel){
        double[] curTarget = armProfile.getSamples();
        double goalPos = curTarget[0];
        double goalVel = curTarget[1] + armPIDC.positionPIController(goalPos, currentAngle);
        double goalAcc = curTarget[2];
        SmartDashboard.putNumber("Arm GoalPos", goalPos);
        SmartDashboard.putNumber("Arm GoalVel", goalVel);
        SmartDashboard.putNumber("Arm GoalAcc", goalAcc);
        // double FF = Math.max(Math.min(getCurrentArmTorque(goalAcc, currentAngle) / getMaxArmTorque(goalVel), 1), -1);
        // SmartDashboard.putNumber("Arm FF", FF);
        if (armProfile.profileDone && goalPos != armSetpoint) armRecalculateFlag = true;
        // return armPIDC.velocityPController(goalVel, curVel) + FF;
        return (goalPos - currentAngle) * ArmLiftConstants.ARM_POS_P_GAIN;

    }

    //generates an output for the motor with an acceleration feedforward, position feedforward, and PID
    public double liftControlOutput(double currentPosition, double curVel){
        double[] curTarget = liftProfile.getSamples();
        double goalPos = curTarget[0];
        double goalVel = curTarget[1] + liftPIDC.positionPIController(goalPos, currentPosition);
        double goalAcc = curTarget[2];
        SmartDashboard.putNumber("Lift GoalPos", goalPos);
        SmartDashboard.putNumber("Lift GoalVel", goalVel);
        SmartDashboard.putNumber("Lift GoalAcc", goalAcc);
        double FF = Math.max(Math.min(getCurrentLiftForce(goalAcc) / getMaxLiftForce(goalVel), 1), -1);
        SmartDashboard.putNumber("Lift FF", FF);
        if (liftProfile.profileDone && goalPos != liftSetpoint) liftRecalculateFlag = true;
        return liftPIDC.velocityPController(goalVel, curVel) + FF;
    }

    /** returns valid setpoints for the arm and lift based on current positions 
     * @return {arm angle, lift height}
     */
    public double[] getValidSeptpoints(double goalLiftPos, double curLiftPos, double goalArmAngle, double curArmAngle){
        double validArmAngle = Math.min(Math.max(goalArmAngle, ArmLiftConstants.MIN_ARM_ANGLE), ArmLiftConstants.MAX_ARM_ANGLE);
        double validLiftHeight = Math.min(Math.max(goalLiftPos, ArmLiftConstants.MIN_LIFT_HEIGHT), ArmLiftConstants.MAX_LIFT_HEIGHT);
        
        if (gameState == GameStates.CLIMB) {
            if (curArmAngle > ArmLiftConstants.ARM_POWER_CHAIN_LOW_ANGLE) {
                validLiftHeight = ArmLiftConstants.POWER_CHAIN_LIFT_HEIGHT;
                if (curLiftPos < ArmLiftConstants.POWER_CHAIN_LIFT_HEIGHT_MIN || curLiftPos > ArmLiftConstants.POWER_CHAIN_LIFT_HEIGHT_MAX){
                    validArmAngle = ArmLiftConstants.MIN_LOW_ARM_ANGLE;
                }
            }
            return new double[]{validArmAngle, validLiftHeight};
        }

        if (curArmAngle < ArmLiftConstants.ARM_POWER_CHAIN_LOW_ANGLE || curArmAngle > ArmLiftConstants.ARM_POWER_CHAIN_HIGH_ANGLE) {
            if (curLiftPos < ArmLiftConstants.POWER_CHAIN_LIFT_HEIGHT_MIN || curLiftPos > ArmLiftConstants.POWER_CHAIN_LIFT_HEIGHT_MAX){
                validArmAngle = 0.0;
            }
            validLiftHeight = ArmLiftConstants.POWER_CHAIN_LIFT_HEIGHT;
            return new double[]{validArmAngle, validLiftHeight};
        }

        //If Lift is at a low position, make sure arm angle is within a threshold so claw doesn't hit bumpers on both sides
        if (curLiftPos < ArmLiftConstants.LOW_LIFT_HEIGHT){
            validArmAngle = Math.max(Math.min(validArmAngle, ArmLiftConstants.MAX_LOW_ARM_ANGLE), ArmLiftConstants.MIN_LOW_ARM_ANGLE);
        }
        
        return new double[]{validArmAngle, validLiftHeight};
    }

    // Get the force of the 12 volt motor curve
    private double getMaxLiftForce(double goalVel){
        //mx + b where b is stall force and slope is change in force over angular velocity
        goalVel = Math.min(goalVel, ArmLiftConstants.MAX_LIFT_VELOCITY);
        return ArmLiftConstants.LIFT_STALL_FORCE * (1 - goalVel/ArmLiftConstants.LIFT_FREE_SPEED);
    }

    private double getCurrentLiftForce(double goalAccel){
        //mg + ma
        return ArmLiftConstants.LIFT_ARM_MASS * (ArmLiftConstants.GRAVITY + goalAccel);
    }

    private double getMaxArmTorque(double goalVel){
         //mx + b where b is stall torque and slope is change in torque over angular velocity
        goalVel = Math.min(goalVel, ArmLiftConstants.MAX_ARM_VELOCITY);
        return ArmLiftConstants.ARM_STALL * (1 - goalVel/ArmLiftConstants.ARM_FREE_SPEED);
    }

    private double getCurrentArmTorque(double goalAccel, double currentAngle){
        //gravitational torque +  (moment of intertia * accleration)
        return (ArmLiftConstants.GRAVITY * ArmLiftConstants.ARM_STATIC_TORQUE * Math.sin(currentAngle)) + (ArmLiftConstants.ARM_MOI * goalAccel);
    }

    public void setGameState(GameStates newState) {
        setGameState(newState, isFront);
    }

    public void setGameState(GameStates newState, boolean isFront){
        if(newState.equals(gameState) && isFront == this.isFront){
            return;
        }
        this.gameState = newState;
        this.isFront = isFront;
        switch (newState) {
            case GROUND_INTAKE:
                this.isFront = true;
                armSetpoint = ArmLiftConstants.GROUND_INTAKE_RIGHT_ANGLE;
                liftSetpoint = ArmLiftConstants.GROUND_INTAKE_LIFT_HEIGHT;
                led.ledState = LEDstates.ALGAE_DETECT;
                break;

            case STORAGE:
                armSetpoint = ArmLiftConstants.STORAGE_ANGLE;
                liftSetpoint = ArmLiftConstants.STORAGE_LIFT_HEIGHT;
                break;

            case L2_ALGAE_REEF:
                claw.setGameState(clawStates.INTAKE);
                armSetpoint = this.isFront ? ArmLiftConstants.L2_INTAKE_ANGLE : 2.0 * Math.PI - ArmLiftConstants.L2_INTAKE_ANGLE;
                liftSetpoint = ArmLiftConstants.L2_INTAKE_LIFT_HEIGHT;
                led.ledState = LEDstates.L2;
                break;

            case L3_ALGAE_REEF:
                claw.setGameState(clawStates.INTAKE);
                armSetpoint = this.isFront ? ArmLiftConstants.L3_INTAKE_ANGLE : 2.0 * Math.PI - ArmLiftConstants.L3_INTAKE_ANGLE;
                liftSetpoint = ArmLiftConstants.L3_INTAKE_LIFT_HEIGHT;
                led.ledState = LEDstates.L3;
                break;

            case SHOOT_NET:
                armSetpoint = this.isFront ? ArmLiftConstants.SHOOT_NET_ANGLE : 2.0 * Math.PI - ArmLiftConstants.SHOOT_NET_ANGLE;
                liftSetpoint = ArmLiftConstants.SHOOT_NET_LIFT_HEIGHT;
                break;

            case PROCESSOR:
                armSetpoint = this.isFront ? ArmLiftConstants.PROCESSOR_ANGLE : 2.0 * Math.PI - ArmLiftConstants.PROCESSOR_ANGLE;;
                liftSetpoint = ArmLiftConstants.PROCESSOR_HEIGHT;
                break;

            case LOLIPOP:
                claw.setGameState(clawStates.INTAKE);
                armSetpoint = ArmLiftConstants.LOLIPOP_ANGLE;
                liftSetpoint = ArmLiftConstants.LOLIPOP_HEIGHT; 
                break;

            case CLIMB:
                claw.setGameState(clawStates.IDLE);
                armSetpoint = ArmLiftConstants.CLIMB_ANGLE;
                liftSetpoint = ArmLiftConstants.CLIMB_HEIGHT;
                break;
        }
        liftRecalculateFlag = true;
        armRecalculateFlag = true;
    }

    public boolean isAtSetpoint() {
        return armProfile.profileDone && Math.abs(currentArmAngle - armSetpoint) < ArmLiftConstants.ARM_TOL && liftProfile.profileDone && Math.abs(currentLiftHeight - liftSetpoint) < ArmLiftConstants.LIFT_TOL;
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
