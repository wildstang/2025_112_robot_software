
package org.wildstang.year2025.subsystems.Claw;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;

import org.wildstang.hardware.roborio.outputs.WsSpark;

import org.wildstang.year2025.robot.WsInputs;
import org.wildstang.year2025.robot.WsOutputs;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.LED.LedSubsystem;
import org.wildstang.year2025.subsystems.LED.LedSubsystem.LEDstates;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift.GameStates;
import org.wildstang.year2025.subsystems.trollgate.TrollGate;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw implements Subsystem{

    //inputs or button on the controller
    private DigitalInput leftBumper;
    private DigitalInput rightBumper;
    private AnalogInput leftTrigger;

    //outputs
    private WsSpark clawMotor, clawMotor2;

    public static enum clawStates {INTAKE, OUTTAKE, IDLE, HOLD}; 
    public clawStates currentState;
    private Timer timer; 
    public boolean algaeInClaw;
    private int algaeHoldCount;
    XboxController controller = new XboxController(0);

    LedSubsystem led;
    ArmLift armLift;
    TrollGate trollgate;

    @Override
    //Called everytime an input/buttons is pressed
    public void inputUpdate(Input source) {
         if (leftBumper.getValue()) {
            setGameState(clawStates.INTAKE);
        } else if (rightBumper.getValue()) {
            setGameState(clawStates.OUTTAKE);
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
        clawMotor2 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.CLAWMOTOR2);
        timer = new Timer();

        algaeInClaw = false;
        currentState = clawStates.IDLE;
        algaeHoldCount = 0;
    }

    @Override
    //  runs every 20 MS
    public void update() {
        SmartDashboard.putNumber("claw velocity", clawMotor.getVelocity());
        SmartDashboard.putNumber("claw current", clawMotor.getOutputCurrent());
        switch(currentState){
            case INTAKE:
                if (clawMotor.getOutputCurrent() > ClawConstants.CLAW_CURRENT_HOLD) { //Math.abs(clawMotor.getVelocity()) < ClawConstants.CLAW_CURRENT_VEL &&
                    algaeHoldCount++;
                    if (algaeHoldCount >= ClawConstants.CLAW_HOLD_COUNT){
                        algaeInClaw = true;
                        led.ledState = LEDstates.INTAKE;
                        armLift.setGameState(GameStates.STORAGE);
                        setGameState(clawStates.HOLD);
                    }
                } else {
                    algaeHoldCount = 0;
                }
                clawMotor.setSpeed(ClawConstants.CLAW_INTAKE_SPEED);
                clawMotor2.setSpeed(-ClawConstants.CLAW_INTAKE_SPEED);
                break;

            case HOLD:
                clawMotor.setSpeed(ClawConstants.CLAW_HOLD_SPEED);
                clawMotor2.setSpeed(-ClawConstants.CLAW_HOLD_SPEED);
                break;

            case OUTTAKE:
                if(timer.get() > ClawConstants.OUTTAKE_TIME){
                    // if we are in scoring position and finished outtaking, go to storage
                    if (armLift.gameState == GameStates.SHOOT_NET || armLift.gameState == GameStates.PROCESSOR) {
                        armLift.setGameState(GameStates.STORAGE);
                    }
                    setGameState(clawStates.IDLE);
                    timer.reset();
                    algaeInClaw = false;
                    led.ledState = LEDstates.SHOOT;
                }
                if (armLift.gameState == GameStates.PROCESSOR){
                    clawMotor.setSpeed(ClawConstants.CLAW_PROCESSOR_SPEED);
                    clawMotor2.setSpeed(-ClawConstants.CLAW_PROCESSOR_SPEED);
                } else if (armLift.gameState == GameStates.GROUND_INTAKE) {
                    clawMotor.setSpeed(-0.2);
                    clawMotor2.setSpeed(0.2);
                } else {
                    clawMotor.setSpeed(ClawConstants.CLAW_OUTTAKE_SPEED);
                    clawMotor2.setSpeed(-ClawConstants.CLAW_OUTTAKE_SPEED);
                }
                break;

            case IDLE:
                clawMotor.setSpeed(0.0);
                clawMotor2.setSpeed(0.0);
                break;
       }
       putDashboard();
    }

    public void setGameState(clawStates desiredState){
        if(desiredState != currentState){
            switch(desiredState){
                case INTAKE:
                    currentState = clawStates.INTAKE;
                    break;
                case OUTTAKE:
                    currentState = clawStates.OUTTAKE;
                    timer.reset();
                    timer.start();
                    break;
                case IDLE:
                    currentState = clawStates.IDLE;
                    break;
                case HOLD:
                    currentState = clawStates.HOLD; 
                    break;
                default:
                    break;
            }
        }
    }

    private void putDashboard(){
        SmartDashboard.putString("Claw State", currentState.name());
        SmartDashboard.putBoolean("Algae in claw", algaeInClaw);
        
    }

    @Override
    //reseting everything
    public void resetState() {
        currentState = clawStates.IDLE; 
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void initSubsystems() {
        led = (LedSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.LED);
        armLift = (ArmLift) Core.getSubsystemManager().getSubsystem(WsSubsystems.ARMLIFT);
        trollgate = (TrollGate) Core.getSubsystemManager().getSubsystem(WsSubsystems.TROLLGATE);
    }


    @Override
    public String getName() {
       return "Claw";
    }
    
}

