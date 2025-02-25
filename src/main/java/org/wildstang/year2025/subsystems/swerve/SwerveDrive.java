package org.wildstang.year2025.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkLimitSwitch;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.swerve.SwerveDriveTemplate;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2025.robot.CANConstants;
import org.wildstang.year2025.robot.WsInputs;
import org.wildstang.year2025.robot.WsOutputs;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.Claw.Claw;
import org.wildstang.year2025.subsystems.LED.LedSubsystem;
import org.wildstang.year2025.subsystems.LED.LedSubsystem.LEDstates;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**Class: SwerveDrive
 * inputs: driver left joystick x/y, right joystick x, right trigger, right bumper, select, face buttons all, gyro
 * outputs: four swerveModule objects
 * description: controls a swerve drive for four swerveModules through autonomous and teleoperated control
 */
public class SwerveDrive extends SwerveDriveTemplate {

    private AnalogInput leftStickX;  // translation joystick x
    private AnalogInput leftStickY;  // translation joystick y
    private AnalogInput rightStickX;  // rot joystick
    private AnalogInput leftTrigger;  //speed derate 
    private DigitalInput select;  // gyro reset
    private DigitalInput leftStickButton;

    private SparkLimitSwitch pixyDigital;
    private SparkAnalogSensor pixyAnalog;

    private double xOutput;
    private double yOutput;
    private double rotOutput;
    private double xSpeed;
    private double ySpeed;
    private double wSpeed;
    private double derateValue;
    private boolean rotLocked;
    private double rotTarget;
    private double pathXTarget;
    private double pathYTarget;

    public final Pigeon2 gyro = new Pigeon2(CANConstants.GYRO);
    public SwerveModule[] modules;
    private SwerveSignal swerveSignal;
    private WsSwerveHelper swerveHelper = new WsSwerveHelper();
    private SwerveDrivePoseEstimator poseEstimator;

    SwerveDriveKinematics swerveKinematics;
    private Claw claw;
    private ArmLift armLift;
    private LedSubsystem led;

    public enum driveType {TELEOP, AUTO, GROUND_INTAKE};
    public driveType driveState;
    private Pose2d curPose;

    private static final double DEG_TO_RAD = Math.PI / 180.0;
    private static final double RAD_TO_DEG = 180.0 / Math.PI;
    private Boolean sensorOverride = false;

    @Override
    public void init() {
        initInputs();
        initOutputs();
        resetState();
        gyro.setYaw(0.0);

    }

    public void initInputs() {
        leftStickX = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_JOYSTICK_X);
        leftStickX.addInputListener(this);
        leftStickY = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_JOYSTICK_Y);
        leftStickY.addInputListener(this);
        rightStickX = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_JOYSTICK_X);
        rightStickX.addInputListener(this);
        leftTrigger = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_TRIGGER);
        leftTrigger.addInputListener(this);
        select = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_SELECT);
        select.addInputListener(this);
        leftStickButton = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_JOYSTICK_BUTTON);
        leftStickButton.addInputListener(this);

        WsSpark clawMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.CLAWMOTOR);
        pixyDigital = clawMotor.getController().getForwardLimitSwitch();
        pixyAnalog = clawMotor.getController().getAnalog();
    }

    public void initOutputs() {
        //create four swerve modules
        modules = new SwerveModule[]{
            new SwerveModule((WsSpark) Core.getOutputManager().getOutput(WsOutputs.DRIVE1), 
                (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ANGLE1), DriveConstants.FRONT_LEFT_OFFSET),
            new SwerveModule((WsSpark) Core.getOutputManager().getOutput(WsOutputs.DRIVE2), 
                (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ANGLE2), DriveConstants.FRONT_RIGHT_OFFSET),
            new SwerveModule((WsSpark) Core.getOutputManager().getOutput(WsOutputs.DRIVE3), 
                (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ANGLE3), DriveConstants.REAR_LEFT_OFFSET),
            new SwerveModule((WsSpark) Core.getOutputManager().getOutput(WsOutputs.DRIVE4), 
                (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ANGLE4), DriveConstants.REAR_RIGHT_OFFSET)
        };

        swerveKinematics = new SwerveDriveKinematics(new Translation2d(DriveConstants.ROBOT_WIDTH/2, DriveConstants.ROBOT_LENGTH/2), new Translation2d(DriveConstants.ROBOT_WIDTH/2, -DriveConstants.ROBOT_LENGTH/2), new Translation2d(-DriveConstants.ROBOT_WIDTH/2, DriveConstants.ROBOT_LENGTH/2), new Translation2d(-DriveConstants.ROBOT_WIDTH/2, -DriveConstants.ROBOT_LENGTH/2));
        //create default swerveSignal
        swerveSignal = new SwerveSignal(new double[]{0.0, 0.0, 0.0, 0.0}, new double[]{0.0, 0.0, 0.0, 0.0});
        poseEstimator = new SwerveDrivePoseEstimator(swerveKinematics, odoAngle(), odoPosition(), new Pose2d());
    }

    @Override
    public void initSubsystems(){
        led = (LedSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.LED);
        claw = (Claw) Core.getSubsystemManager().getSubsystem(WsSubsystems.CLAW);
        armLift = (ArmLift) Core.getSubsystemManager().getSubsystem(WsSubsystems.ARMLIFT);
    }

    @Override
    public void inputUpdate(Input source) {
        if (source == leftStickButton && leftStickButton.getValue()) sensorOverride = !sensorOverride;
        // reset gyro when facing away from alliance station
        if (source == select && select.getValue()) {
            if (Core.isBlueAlliance()) {
                setGyro(0);
            } else {
                setGyro(Math.PI);
            }
        }

        // get x and y speeds
        // joystick axes: +X is to the driver's right, +Y is away from the driver
        // field axes: +X is away from the blue alliance driver station, +Y is to the left from the blue alliance driver station perspective
        xOutput = swerveHelper.scaleDeadband(leftStickY.getValue(), DriveConstants.DEADBAND);  // joystick y value corresponds to driving forward, i.e. pushing the stick away from the driver (stick +Y) should make the robot drive away from the driver station (field +X)
        yOutput = swerveHelper.scaleDeadband(-leftStickX.getValue(), DriveConstants.DEADBAND);  // joystick x value corresonds to driving sideways in the opposite direction, i.e. pushing the stick to the drivers left (stick -X) should make the robot drive towards the 

        // reverse x/y directions if on red alliance to match field coordinate system
        if (!Core.isBlueAlliance()) {
            xOutput *= -1;
            yOutput *= -1;
        }

        //get rotational joystick
        // joystick positive value corresponds to cw rotation
        // drivetrain positive value corresponds to ccw rotation
        rotOutput = swerveHelper.scaleDeadband(-rightStickX.getValue(), DriveConstants.DEADBAND);  // negate joystick value so positive on the joystick (right) commands a negative rot speed (turn cw) and vice versa
        
        if(leftTrigger.getValue() != 0 && !sensorOverride){
            driveState = driveType.GROUND_INTAKE;
         }else if(rotOutput != 0){
             driveState = driveType.TELEOP;
         } 

        // if the rotational joystick is being used, the robot should not be auto tracking heading
        // otherwise engage rotation lock at current heading
        if (rotOutput != 0) {
            rotOutput *= Math.abs(rotOutput);
            rotOutput *= DriveConstants.ROTATION_SPEED;
            rotLocked = false;
        } 
        else if (rotLocked == false) {
            rotTarget = getGyroAngle();
            rotLocked = true;
        }
        
        //assign thrust
        derateValue = (DriveConstants.DRIVE_DERATE * Math.abs(leftTrigger.getValue()) + 1);
        xOutput /= derateValue;
        yOutput /= derateValue;
        rotOutput /= derateValue;
    }
    
    @Override
    public void resetState() {
        setToTeleop();
    }

    @Override
    public void update() {
        poseEstimator.update(odoAngle(), odoPosition());
        curPose = poseEstimator.getEstimatedPosition();

        switch (driveState) {
            case TELEOP:
                if (rotLocked) rotOutput = swerveHelper.getRotControl(rotTarget, getGyroAngle());  // if rotation tracking, replace rotational joystick value with controller generated one
                break;
            case AUTO:
                rotOutput = wSpeed * DriveConstants.DRIVE_F_ROT + swerveHelper.getRotControl(rotTarget, getGyroAngle());
                xOutput = xSpeed * DriveConstants.DRIVE_F_K + (pathXTarget - curPose.getX()) * DriveConstants.POS_P;
                yOutput = ySpeed * DriveConstants.DRIVE_F_K + (pathYTarget - curPose.getY()) * DriveConstants.POS_P;
                break;
            case GROUND_INTAKE:
                led.ledState = LEDstates.ALGAE_DETECT;
                if(claw.algaeInClaw){
                    driveState = driveType.TELEOP;
                }
                if (algaeInView() && armLift.isAtSetpoint()) {
                    rotLocked = true;
                    rotTarget =  ((1.0 - pixyAnalog.getVoltage()) * 0.524 + getGyroAngle() + 2.0 * Math.PI) % (2.0 * Math.PI);
                    rotOutput = swerveHelper.getRotControl(rotTarget, getGyroAngle());
                }
                break; 
        }
        
        rotOutput = Math.min(Math.max(rotOutput, -1.0), 1.0);
        xOutput = Math.min(Math.max(xOutput, -1.0), 1.0);
        yOutput = Math.min(Math.max(yOutput, -1.0), 1.0);
        this.swerveSignal = swerveHelper.setDrive(xOutput , yOutput, rotOutput, getGyroAngle());
        drive();

        SmartDashboard.putNumber("Gyro Reading", getGyroAngle());
        SmartDashboard.putNumber("X output", xOutput);
        SmartDashboard.putNumber("Y output", yOutput);
        SmartDashboard.putNumber("rotOutput", rotOutput);
        SmartDashboard.putString("Drive mode", driveState.toString());
        SmartDashboard.putNumber("Auto x velocity", xSpeed);
        SmartDashboard.putNumber("Auto y velocity", ySpeed);
        SmartDashboard.putNumber("Auto ang velocity", wSpeed);
        SmartDashboard.putNumber("Auto x pos", pathXTarget);
        SmartDashboard.putNumber("Auto y pos", pathYTarget);
        SmartDashboard.putNumber("rot target", rotTarget);
        SmartDashboard.putBoolean("Blue Alliance", Core.isBlueAlliance());
        SmartDashboard.putString("cur pose", curPose.toString());
        SmartDashboard.putNumber("Pixy Voltage", pixyAnalog.getVoltage());
        SmartDashboard.putBoolean("Pixy Obj Det", pixyDigital.isPressed());
        SmartDashboard.putBoolean("Swerve Override", sensorOverride);
    }

    /** sets the drive to teleop/cross, and sets drive motors to coast */
    public void setToTeleop() {
        driveState = driveType.TELEOP;
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDriveBrake(false);
        }
        rotOutput = 0;
        xOutput = 0;
        yOutput = 0;
        rotTarget = getGyroAngle();
        rotLocked = true;
    }

    /**sets the drive to autonomous */
    public void setToAuto() {
        driveState = driveType.AUTO;
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDriveBrake(true);
        }
        xSpeed = 0;
        ySpeed = 0;
        wSpeed = 0;
        pathXTarget = curPose.getX();
        pathYTarget = curPose.getY();
        rotTarget = getGyroAngle();
    }

    /**drives the robot at the current swerveSignal, and displays information for each swerve module */
    private void drive() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].run(swerveSignal.getSpeed(i), swerveSignal.getAngle(i));
            modules[i].displayNumbers(DriveConstants.POD_NAMES[i]);
        }
    }

    @Override
    /**sets autonomous values from the path data file */
    public void setAutoValues(double velocityX, double velocityY, double angVel, double xTarget, double yTarget, double heading) {
        xSpeed = velocityX;
        ySpeed = velocityY;
        wSpeed = angVel;
        pathXTarget = xTarget;
        pathYTarget = yTarget;
        rotTarget = heading;
    }

    /**sets the autonomous heading controller to a new target */
    public void setAutoHeading(double headingTarget) {
        rotTarget = headingTarget;
    }

    public boolean algaeInView(){
        return pixyDigital.isPressed();
        
    }

    /**
     * Resets the gyro, and sets it the input number of radians
     * Used for starting the match at a non-0 angle
     * @param degrees the current value the gyro should read
     */
    public void setGyro(double radians) {
        gyro.setYaw(radians * RAD_TO_DEG);
        rotTarget = radians;
    }

    public double getGyroAngle() {
        return (((gyro.getYaw().getValueAsDouble() * DEG_TO_RAD) % (2.0 * Math.PI)) + 2.0 * Math.PI) % (2.0 * Math.PI);
    }

    public Rotation2d odoAngle(){
        return new Rotation2d(getGyroAngle());
    }

    public SwerveModulePosition[] odoPosition(){
        return new SwerveModulePosition[]{modules[0].odoPosition(), modules[1].odoPosition(), modules[2].odoPosition(), modules[3].odoPosition()};
    }

    public void setPose(Pose2d pos){
        this.poseEstimator.resetPosition(odoAngle(), odoPosition(), pos);
    }

    public Pose2d returnPose(){
        return curPose;
    }
    
    @Override
    public void selfTest() {
    }

    @Override
    public String getName() {
        return "Swerve Drive";
    }
}