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

import edu.wpi.first.math.MathUtil;
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
    private double rOutput;

    private double xInput;
    private double yInput;
    private double rInput;

    // private double xSpeed;
    // private double ySpeed;
    // private double wSpeed;
    // private boolean rotLocked;
    // private double rotTarget;
    // private double pathXTarget;
    // private double pathYTarget;
    private Pose2d targetPose;
    private ChassisSpeeds targetSpeed;

    public final Pigeon2 gyro = new Pigeon2(CANConstants.GYRO);
    public SwerveModule[] modules;
    private SwerveSignal swerveSignal;
    private WsSwerveHelper swerveHelper = new WsSwerveHelper();

    public SwerveDriveKinematics swerveKinematics;
    private ArmLift armLift;

    public enum DriveState {TELEOP, AUTO};
    public DriveState driveState;
    private Pose2d curPose;

    private static final double DEG_TO_RAD = Math.PI / 180.0;
    private static final double RAD_TO_DEG = 180.0 / Math.PI;
    private Boolean rotHelperOverride = false;

    public final Field2d m_field = new Field2d();
    private SwerveModuleState[] moduleStates;
    StructArrayPublisher<SwerveModuleState> moduleStatePublisher;
    StructPublisher<ChassisSpeeds> chassisSpeedPublisher, targetSpeedPublisher;
    StructPublisher<Pose2d> targetPosePublisher;

    private final ChassisSpeeds kZeroSpeed = new ChassisSpeeds();

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
        targetSpeedPublisher = NetworkTableInstance.getDefault().getStructTopic("Target Speeds", ChassisSpeeds.struct).publish();
        targetPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Target Pose", Pose2d.struct).publish();
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
        curPose = loc.getCurrentPose();
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
        xInput = swerveHelper.scaleDeadband(leftStickY.getValue(), DriveConstants.DEADBAND);  // joystick y value corresponds to driving forward, i.e. pushing the stick away from the driver (stick +Y) should make the robot drive away from the driver station (field +X)
        yInput = swerveHelper.scaleDeadband(-leftStickX.getValue(), DriveConstants.DEADBAND);  // joystick x value corresonds to driving sideways in the opposite direction, i.e. pushing the stick to the drivers left (stick -X) should make the robot drive towards the 

        // reverse x/y directions if on red alliance to match field coordinate system
        if (!Core.isBlueAlliance()) {
            xInput *= -1;
            yInput *= -1;
        }

        xInput *= Math.abs(xInput);
        yInput *= Math.abs(yInput);

        //get rotational joystick
        // joystick positive value corresponds to cw rotation
        // drivetrain positive value corresponds to ccw rotation
        rInput = swerveHelper.scaleDeadband(-rightStickX.getValue(), DriveConstants.DEADBAND);  // negate joystick value so positive on the joystick (right) commands a negative rot speed (turn cw) and vice versa

        rInput *= Math.abs(rInput);
    }

    @Override
    public void update() {
        curPose = loc.getCurrentPose();

        switch (driveState) {
            case AUTO:
                setAutoDriveValues(targetSpeed, targetPose);
                break;

            case TELEOP:
                if (!rotHelperOverride) {
                    switch (armLift.gameState) {
                        case GROUND_INTAKE:
                            xOutput = xInput;
                            yOutput = yInput;
                            if (algaeInView() && armLift.isAtSetpoint() && rInput == 0) {
                                rOutput = (1.0 - pixyAnalog.getVoltage()) * 0.40;  // TODO: tune these values
                            } else {
                                rOutput = rInput;
                            }
                            break;

                        case L2_ALGAE_REEF:
                        case L3_ALGAE_REEF:
                            targetPose = loc.getNearestReefPose();
                            targetPose = new Pose2d(targetPose.getX() + xInput, targetPose.getY() + yInput, targetPose.getRotation());
                            setAutoDriveValues(kZeroSpeed, targetPose);
                            break;

                        case PROCESSOR:
                            targetPose = loc.getNearestProcessorPose();
                            targetPose = new Pose2d(targetPose.getX() + xInput, targetPose.getY() + yInput, targetPose.getRotation());
                            setAutoDriveValues(kZeroSpeed, targetPose);
                            break;

                        case SHOOT_NET:
                            targetPose = loc.getNearestBargePose();
                            targetPose = new Pose2d(targetPose.getX() + xInput, targetPose.getY() + yInput, targetPose.getRotation());
                            setAutoDriveValues(kZeroSpeed, targetPose);
                            yOutput = yInput;  // override yOutput to allow driver to align to clear spot on the barge
                            break;
                        
                        default:
                            xOutput = xInput;
                            yOutput = yInput;
                            rOutput = rInput;
                            break;
                    }
                } else {
                    xOutput = xInput;
                    yOutput = yInput;
                    rOutput = rInput;
                }
                break;
        }
        
        rOutput = Math.min(Math.max(rOutput, -1.0), 1.0);
        xOutput = Math.min(Math.max(xOutput, -1.0), 1.0);
        yOutput = Math.min(Math.max(yOutput, -1.0), 1.0);
        this.swerveSignal = swerveHelper.setDrive(xOutput , yOutput, rOutput, getGyroAngle());
        drive();
        putDashboard();
    }

    public void setDriveState(DriveState newState) {
        driveState = newState;
    }

    private void putDashboard() {
        SmartDashboard.putNumber("Gyro Reading", getGyroAngle());
        SmartDashboard.putNumber("X input", xInput);
        SmartDashboard.putNumber("Y input", yInput);
        SmartDashboard.putNumber("X output", xOutput);
        SmartDashboard.putNumber("Y output", yOutput);
        SmartDashboard.putNumber("rOutput", rOutput);
        SmartDashboard.putString("Drive mode", driveState.toString());
        SmartDashboard.putBoolean("Blue Alliance", Core.isBlueAlliance());
        SmartDashboard.putNumber("Pixy Voltage", pixyAnalog.getVoltage());
        SmartDashboard.putBoolean("Pixy Obj Det", pixyDigital.isPressed());
        SmartDashboard.putBoolean("Rot Control Override", rotHelperOverride);
        moduleStates = new SwerveModuleState[] {modules[0].getModuleState(),modules[1].getModuleState(),modules[2].getModuleState(),modules[3].getModuleState()};
        moduleStatePublisher.set(moduleStates);
        chassisSpeedPublisher.set(swerveKinematics.toChassisSpeeds(moduleStates));
        targetSpeedPublisher.set(targetSpeed);
        targetPosePublisher.set(targetPose);
        m_field.setRobotPose(curPose);  // TODO: check if this is needed to post to elastic or if we can use the Localization publisher
    }

    /** sets the drive to teleop/cross, and sets drive motors to coast */
    public void setToTeleop() {
        driveState = DriveState.TELEOP;
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDriveBrake(false);
        }
        rOutput = 0;
        xOutput = 0;
        yOutput = 0;
    }

    /**sets the drive to autonomous */
    public void setToAuto() {
        driveState = DriveState.AUTO;
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDriveBrake(true);
        }
        targetSpeed = new ChassisSpeeds();
        targetPose = curPose;
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
    public void setAutoValues(ChassisSpeeds speed, Pose2d pose) {
        targetSpeed = speed;
        targetPose = pose;
    }

    /** Sets output values based on given ChassisSpeeds and target pose, but *does not* 
     * call drive(). This gives the user to option to override certain values if needed.
     * Used for both autonomous mode pathfinding and teleop autoalign.
     */
    private void setAutoDriveValues(ChassisSpeeds speed, Pose2d pose) {
        xOutput = speed.vxMetersPerSecond * DriveConstants.DRIVE_F_K + (pose.getX() - curPose.getX()) * DriveConstants.POS_P;
        yOutput = speed.vyMetersPerSecond * DriveConstants.DRIVE_F_K + (pose.getY() - curPose.getY()) * DriveConstants.POS_P;
        rOutput = speed.omegaRadiansPerSecond * DriveConstants.DRIVE_F_ROT + swerveHelper.getRotControl(MathUtil.angleModulus(pose.getRotation().getRadians()), MathUtil.angleModulus(curPose.getRotation().getRadians()));  // TODO: tune rot controller

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
    }

    public double getGyroAngle() {
        return MathUtil.angleModulus(gyro.getYaw().getValueAsDouble() * DEG_TO_RAD);
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