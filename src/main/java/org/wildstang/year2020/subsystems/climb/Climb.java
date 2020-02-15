package org.wildstang.year2020.subsystems.climb;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.IInputManager;
import org.wildstang.framework.io.Input;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.year2020.robot.CANConstants;
import org.wildstang.year2020.robot.WSInputs;

public class Climb implements Subsystem {

    // Inputs
    private DigitalInput selectButton;
    private DigitalInput startButton;
    private DigitalInput downButton;

    // Outputs
    private CANSparkMax climbMotor1;
    private CANSparkMax climbMotor2;

    // Variables
    private double motorspeed;
    private final double LIFT_HEIGHT = 0;

    // Statuses
    private boolean climbInputStatus;
    private boolean climbActiveStatus; // For Shuffleboard
    private boolean climbCompleteStatus;
    private boolean downPressed;

    @Override
    public void inputUpdate(Input source) {
        if (selectButton.getValue() && startButton.getValue()) {
            climbInputStatus = true;
            motorspeed = 1.0; // Extends climb
        }
        if (downButton.getValue()) {
            downPressed = true;
        } else {
          downPressed = false;
        }
    }

    @Override
    public void init() {
        initInputs();
        initOutputs();
        resetState();
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
         // If button is pressed, set the motorspeed to the defined value in the inputUpdate method
        if (climbInputStatus) {
            climbActiveStatus = true; // For Shuffleboard
            climbMotor1.set(motorspeed);
            climbMotor2.set(motorspeed);
            if (climbMotor1.getEncoder().getPosition() >= LIFT_HEIGHT && climbMotor2.getEncoder().getPosition() >= LIFT_HEIGHT) {
                climbActiveStatus = false;
                climbCompleteStatus = true;
            }
        }

        if (climbCompleteStatus == true && downPressed == true) {
            climbActiveStatus = true;
            climbMotor1.set(motorspeed);
            climbMotor2.set(motorspeed);
        }
        // If anything else, set motorspeed to 0
        if (climbCompleteStatus == true && downPressed == false) {
            climbActiveStatus = false; // For Shuffleboard
            climbMotor1.set(0);
            climbMotor2.set(0);
        }
        
    }

    @Override
    public void resetState() {
        climbInputStatus = false;
        climbActiveStatus = false;
        climbCompleteStatus = false;
        //climbMotor1.restoreFactoryDefaults();
        //climbMotor2.restoreFactoryDefaults();
    }

    @Override
    public String getName() {
        return "Climb";
    }

    private void initOutputs() {
        climbMotor1 = new CANSparkMax(CANConstants.CLIMB_VICTOR_1,MotorType.kBrushless);
        climbMotor2 = new CANSparkMax(CANConstants.CLIMB_VICTOR_2,MotorType.kBrushless);
    }

    private void initInputs() {
        IInputManager inputManager = Core.getInputManager();
        selectButton = (DigitalInput) inputManager.getInput(WSInputs.DRIVER_SELECT.getName());
        selectButton.addInputListener(this);
        startButton = (DigitalInput) inputManager.getInput(WSInputs.DRIVER_START.getName());
        startButton.addInputListener(this);
        downButton = (DigitalInput) inputManager.getInput(WSInputs.MANIPULATOR_DPAD_DOWN.getName());
        downButton.addInputListener(this);
    }

}