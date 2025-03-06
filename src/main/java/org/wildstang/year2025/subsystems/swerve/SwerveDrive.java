package org.wildstang.year2025.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkLimitSwitch;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.logger.Log;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.swerve.SwerveDriveTemplate;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2025.robot.CANConstants;
import org.wildstang.year2025.robot.WsInputs;
import org.wildstang.year2025.robot.WsOutputs;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift;
import org.wildstang.year2025.subsystems.localization.Localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**Class: SwerveDrive
 * inputs: driver left joystick x/y, right joystick x, right trigger, right bumper, select, face buttons all, gyro
 * outputs: four swerveModule objects
 * description: controls a swerve drive for four swerveModules through autonomous and teleoperated control
 */
public class SwerveDrive extends SwerveDriveTemplate {

    private Localization loc;

    private AnalogInput leftStickX;  // translation joystick x
    private AnalogInput leftStickY;  // translation joystick y
    private AnalogInput rightStickX;  // rot joystick
    private AnalogInput leftTrigger;  //speed derate 
    private DigitalInput select;  // gyro reset
    private DigitalInput rightStickButton;

    private SparkLimitSwitch pixyDigital;
    private SparkAnalogSensor pixyAnalog;

    private double xOutput;
    private double yOutput;
    private double rotOutput;
    private double xSpeed;
    private double ySpeed;
    private double wSpeed;
    private boolean rotLocked;
    private double rotTarget;
    private double pathXTarget;
    private double pathYTarget;

    public final Pigeon2 gyro = new Pigeon2(CANConstants.GYRO);
    public SwerveModule[] modules;
    private SwerveSignal swerveSignal;
    private WsSwerveHelper swerveHelper = new WsSwerveHelper();

    public SwerveDriveKinematics swerveKinematics;
    private ArmLift armLift;

    public enum DriveState {TELEOP, AUTO, REEF, NET, PROCESSOR, GROUND_INTAKE};
    public DriveState driveState;
    private Pose2d curPose;

    private static final double DEG_TO_RAD = Math.PI / 180.0;
    private static final double RAD_TO_DEG = 180.0 / Math.PI;
    private Boolean rotHelperOverride = false;

    public final Field2d m_field = new Field2d();
    private SwerveModuleState[] moduleStates;
    StructArrayPublisher<SwerveModuleState> moduleStatePublisher;
    StructPublisher<ChassisSpeeds> chassisSpeedPublisher;

    @Override
    public void init() {
        initInputs();
        initOutputs();
        resetState();
        moduleStates = new SwerveModuleState[] {modules[0].getModuleState(),modules[1].getModuleState(),modules[2].getModuleState(),modules[3].getModuleState()};
        gyro.setYaw(0.0);
        SmartDashboard.putData("Field", m_field);
        moduleStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Swerve Module States", SwerveModuleState.struct).publish();
        chassisSpeedPublisher = NetworkTableInstance.getDefault().getStructTopic("Chassis Speeds", ChassisSpeeds.struct).publish();
        curPose = new Pose2d(0.0,0.0,new Rotation2d(0.0));

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
        rightStickButton = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_JOYSTICK_BUTTON);
        rightStickButton.addInputListener(this);

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
        swerveSignal = new SwerveSignal(new double[]{0.0, 0.0, 0.0, 0.0}, new double[]{0.0, 0.0, 0.0, 0.0});
    }

    @Override
    public void initSubsystems(){
        armLift = (ArmLift) Core.getSubsystemManager().getSubsystem(WsSubsystems.ARMLIFT);
        loc = (Localization) Core.getSubsystemManager().getSubsystem(WsSubsystems.LOCALIZATION);

    }

    @Override
    public void inputUpdate(Input source) {
        if (source == rightStickButton && rightStickButton.getValue()) rotHelperOverride = !rotHelperOverride;
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

        // if the rotational joystick is being used, the robot should not be auto tracking heading
        // otherwise engage rotation lock at current heading
        if (rotOutput != 0) {
            rotOutput *= Math.abs(rotOutput);
            rotLocked = false;
        } else {
            rotLocked = true;
        }
    }

    @Override
    public void update() {
        curPose = loc.getCurrentPose();

        switch (driveState) {
            case AUTO:
                rotOutput = wSpeed * DriveConstants.DRIVE_F_ROT + swerveHelper.getRotControl(rotTarget, getGyroAngle());
                xOutput = xSpeed * DriveConstants.DRIVE_F_K + (pathXTarget - curPose.getX()) * DriveConstants.POS_P;
                yOutput = ySpeed * DriveConstants.DRIVE_F_K + (pathYTarget - curPose.getY()) * DriveConstants.POS_P;
                break;

            case TELEOP:
                break;

            case GROUND_INTAKE:
                if (algaeInView() && armLift.isAtSetpoint() && rotLocked) {
                    rotOutput = (1.0 - pixyAnalog.getVoltage()) * 0.40;
                }
                break;

            case REEF:
                if (rotLocked) {
                    double curAngle = getGyroAngle();
                    if (curAngle > 11.0 * Math.PI / 6.0 || curAngle < Math.PI / 6.0) rotTarget = 0.0;
                    else if (curAngle < 3.0 * Math.PI / 6.0) rotTarget = 2.0 * Math.PI / 6.0;
                    else if (curAngle < 5.0 * Math.PI / 6.0) rotTarget = 4.0 * Math.PI / 6.0;
                    else if (curAngle < 7.0 * Math.PI / 6.0) rotTarget = 6.0 * Math.PI / 6.0;
                    else if (curAngle < 9.0 * Math.PI / 6.0) rotTarget = 8.0 * Math.PI / 6.0;
                    else rotTarget = 10.0 * Math.PI / 6.0;
                    rotOutput = swerveHelper.getRotControl(rotTarget, getGyroAngle());
                }
                break;

            case PROCESSOR:
                if (rotLocked) {
                    rotTarget = (getGyroAngle() <= Math.PI) ? Math.PI / 2.0 : 3.0 * Math.PI / 2.0;
                    rotOutput = swerveHelper.getRotControl(rotTarget, getGyroAngle());
                }
                break;

            case NET:
                if (rotLocked) {
                    rotTarget = (getGyroAngle() <= Math.PI / 2.0 || getGyroAngle() >= 3.0 * Math.PI / 2.0) ? 0 : Math.PI;
                    rotOutput = swerveHelper.getRotControl(rotTarget, getGyroAngle());
                }
                break;
        }
        
        rotOutput = Math.min(Math.max(rotOutput, -1.0), 1.0);
        xOutput = Math.min(Math.max(xOutput, -1.0), 1.0);// * derateValue;
        yOutput = Math.min(Math.max(yOutput, -1.0), 1.0);// * derateValue;
        this.swerveSignal = swerveHelper.setDrive(xOutput , yOutput, rotOutput, getGyroAngle());
        drive();
        putDashboard();
    }

    public void setDriveState(DriveState newState) {
        driveState = newState;
    }

    private void putDashboard() {
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
        SmartDashboard.putNumber("Pixy Voltage", pixyAnalog.getVoltage());
        SmartDashboard.putBoolean("Pixy Obj Det", pixyDigital.isPressed());
        SmartDashboard.putBoolean("Rot Control Override", rotHelperOverride);
        SmartDashboard.putBoolean("rot lock", rotLocked);
        moduleStates = new SwerveModuleState[] {modules[0].getModuleState(),modules[1].getModuleState(),modules[2].getModuleState(),modules[3].getModuleState()};
        moduleStatePublisher.set(moduleStates);
        chassisSpeedPublisher.set(swerveKinematics.toChassisSpeeds(moduleStates));
        m_field.setRobotPose(curPose);
    }

    /** sets the drive to teleop/cross, and sets drive motors to coast */
    public void setToTeleop() {
        driveState = DriveState.TELEOP;
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDriveBrake(false);
        }
        rotOutput = 0;
        xOutput = 0;
        yOutput = 0;
        rotTarget = getGyroAngle();
        rotLocked = false;
    }

    /**sets the drive to autonomous */
    public void setToAuto() {
        driveState = DriveState.AUTO;
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
     * @param radians the current value the gyro should read
     */
    public void setGyro(double radians) {
        StatusCode code = gyro.setYaw(radians * RAD_TO_DEG);
        Log.warn("Gyro Status Code: " + code.getName());
        rotTarget = radians;
    }

    public double getGyroAngle() {
        return (((gyro.getYaw().getValueAsDouble() * DEG_TO_RAD) % (2.0 * Math.PI)) + 2.0 * Math.PI) % (2.0 * Math.PI);
    }

    public Rotation2d getOdoAngle(){
        return new Rotation2d(getGyroAngle());
    }

    public SwerveModulePosition[] getOdoPosition(){
        return new SwerveModulePosition[]{modules[0].odoPosition(), modules[1].odoPosition(), modules[2].odoPosition(), modules[3].odoPosition()};
    }
    
    @Override
    public void resetState() {
        setToTeleop();
    }
    
    @Override
    public void selfTest() {
    }

    @Override
    public String getName() {
        return "Swerve Drive";
    }
}