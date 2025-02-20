package org.wildstang.year2025.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkLimitSwitch;

// import org.photonvision.PhotonUtils;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
// import org.wildstang.framework.logger.Log;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.swerve.SwerveDriveTemplate;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2025.robot.CANConstants;
import org.wildstang.year2025.robot.WsInputs;
import org.wildstang.year2025.robot.WsOutputs;
// import org.wildstang.year2025.subsystems.localization.WsVision;

import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private AnalogInput rightTrigger;  //speed derate 
    // private DigitalInput leftBumper, rightBumper;  // intake, shoot
    private DigitalInput select;  // gyro reset
    private DigitalInput start;  // 
    private DigitalInput faceUp;  // rotation lock 0 degrees
    private DigitalInput faceRight;  // rotation lock 90 degrees
    private DigitalInput faceLeft;  // rotation lock 270 degrees
    private DigitalInput faceDown;  // rotation lock 180 degrees
    // private DigitalInput dpadUp;
    private DigitalInput leftStickButton;

    private SparkLimitSwitch pixyDio;
    private SparkAnalogSensor pixyAnalog;

    private double xOutput;
    private double yOutput;
    private double rotOutput;
    private double xSpeed;
    private double ySpeed;
    private double wSpeed;
    // private double thrustValue;
    private double derateValue;
    private boolean rotLocked;
    private double rotTarget;
    private double pathXErr;
    private double pathYErr;
    // private double curX, curY, curTheta;

    public final Pigeon2 gyro = new Pigeon2(CANConstants.GYRO);
    public SwerveModule[] modules;
    private SwerveSignal swerveSignal;
    private WsSwerveHelper swerveHelper = new WsSwerveHelper();
    // private SwerveDrivePoseEstimator poseEstimator;

    // ChassisSpeeds chassisSpeeds;
    // SwerveDriveKinematics swerveKinematics;
    // SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    // private WsVision pvCam;
    // private double targetYaw;

    public enum driveType {TELEOP, AUTO};
    public driveType driveState;
    private boolean isBlueAlliance;
    // private Pose2d curPose;
    public boolean sensorOverride;

    private static final double DEG_TO_RAD = Math.PI / 180.0;
    private static final double RAD_TO_DEG = 180.0 / Math.PI;

    @Override
    public void init() {
        initInputs();
        initOutputs();
        resetState();
        gyro.setYaw(0.0);
    }

    public void initInputs() {
        // pvCam = (WsVision) Core.getSubsystemManager().getSubsystem("Ws Vision");

        leftStickX = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_JOYSTICK_X);
        leftStickX.addInputListener(this);
        leftStickY = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_JOYSTICK_Y);
        leftStickY.addInputListener(this);
        rightStickX = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_JOYSTICK_X);
        rightStickX.addInputListener(this);
        rightTrigger = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_TRIGGER);
        rightTrigger.addInputListener(this);
        // leftTrigger = (AnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_TRIGGER);
        // leftTrigger.addInputListener(this);
        // rightBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        // rightBumper.addInputListener(this);
        // leftBumper = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_SHOULDER);
        // leftBumper.addInputListener(this);
        select = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_SELECT);
        select.addInputListener(this);
        start = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_START);
        start.addInputListener(this);
        // faceUp = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_FACE_UP);
        // faceUp.addInputListener(this);
        // faceLeft = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_FACE_LEFT);
        // faceLeft.addInputListener(this);
        // faceRight = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_FACE_RIGHT);
        // faceRight.addInputListener(this);
        // faceDown = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_FACE_DOWN);
        // faceDown.addInputListener(this);
        // dpadUp = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_UP);
        // dpadUp.addInputListener(this);
        // leftStickButton = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_JOYSTICK_BUTTON);
        // leftStickButton.addInputListener(this);

        WsSpark clawMotor = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.CLAWMOTOR);
        pixyDio = clawMotor.getController().getForwardLimitSwitch();
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

        // swerveKinematics = new SwerveDriveKinematics(new Translation2d(DriveConstants.ROBOT_WIDTH/2, DriveConstants.ROBOT_LENGTH/2), new Translation2d(DriveConstants.ROBOT_WIDTH/2, -DriveConstants.ROBOT_LENGTH/2), new Translation2d(-DriveConstants.ROBOT_WIDTH/2, DriveConstants.ROBOT_LENGTH/2), new Translation2d(-DriveConstants.ROBOT_WIDTH/2, -DriveConstants.ROBOT_LENGTH/2));
        //create default swerveSignal
        swerveSignal = new SwerveSignal(new double[]{0.0, 0.0, 0.0, 0.0}, new double[]{0.0, 0.0, 0.0, 0.0});
        // poseEstimator = new SwerveDrivePoseEstimator(swerveKinematics, odoAngle(), odoPosition(), new Pose2d());
    }

    @Override
    public void inputUpdate(Input source) {
        if (source == leftStickButton && leftStickButton.getValue()) sensorOverride = !sensorOverride;
        
        else driveState = driveType.TELEOP;
        
        // reset gyro when facing away from alliance station
        if (source == select && select.getValue()) {
            if (isBlueAlliance) {
                gyro.setYaw(0.0);  // away from alliance station is 0 rad on blue alliance
                rotTarget = 0.0;
            } else {
                gyro.setYaw(Math.PI * RAD_TO_DEG);  // away from alliance station is pi rad on red alliance
                rotTarget = Math.PI;
            }
        } else if (source == start && start.getValue()) {
            // setGyro(getPosTheta());
        }

        // get x and y speeds
        // joystick axes: +X is to the driver's right, +Y is away from the driver
        // field axes: +X is away from the blue alliance driver station, +Y is to the left from the blue alliance driver station perspective
        xOutput = swerveHelper.scaleDeadband(leftStickY.getValue(), DriveConstants.DEADBAND);  // joystick y value corresponds to driving forward, i.e. pushing the stick away from the driver (stick +Y) should make the robot drive away from the driver station (field +X)
        yOutput = swerveHelper.scaleDeadband(-leftStickX.getValue(), DriveConstants.DEADBAND);  // joystick x value corresonds to driving sideways in the opposite direction, i.e. pushing the stick to the drivers left (stick -X) should make the robot drive towards the 

        // reverse x/y directions if on red alliance to match field coordinate system
        if (!isBlueAlliance) {
            xOutput *= -1;
            yOutput *= -1;
        }

        if ((source == faceUp && faceUp.getValue()) || (source == faceRight && faceRight.getValue()) || (source == faceDown && faceDown.getValue()) || (source == faceLeft && faceLeft.getValue())){
            if (faceUp.getValue()){
                if (faceLeft.getValue()){
                    rotTarget = Math.PI / 4.0;
                } else if (faceRight.getValue()){ 
                    rotTarget = 7.0 * Math.PI / 4.0;
                } else  rotTarget = 0.0;
            } else if (faceDown.getValue()){
                if (faceLeft.getValue()){
                    rotTarget = 3.0 * Math.PI / 4.0;
                } else if (faceRight.getValue()){ 
                    rotTarget = 5.0 * Math.PI / 4.0;
                } else  rotTarget = Math.PI;
            } else if (faceLeft.getValue()){
                rotTarget = Math.PI / 2.0;
            } else if (faceRight.getValue()){
                rotTarget = 3.0 * Math.PI / 2.0;
            }
            rotLocked = true;
            if (!isBlueAlliance) {
                rotTarget = (rotTarget + Math.PI) % (2.0 * Math.PI);  // flip rotation target if on red alliance to match field coordinate system
            }
        }

        //get rotational joystick
        // joystick positive value corresponds to cw rotation
        // drivetrain positive value corresponds to ccw rotation
        rotOutput = swerveHelper.scaleDeadband(-rightStickX.getValue(), DriveConstants.DEADBAND);  // negate joystick value so positive on the joystick (right) commands a negative rot speed (turn cw) and vice versa
        
        // if the rotational joystick is being used, the robot should not be auto tracking heading
        // otherwise engage rotation lock at current heading
        if (rotOutput != 0) {
            rotOutput *= Math.abs(rotOutput);
            rotOutput *= DriveConstants.ROTATION_SPEED;
            rotLocked = false;
        } 
        // else if (rotLocked == false) {
        //     rotTarget = getGyroAngle();
        //     rotLocked = true;
        // }
        
        //assign thrust
        // thrustValue = 1 - DriveConstants.DRIVE_THRUST + DriveConstants.DRIVE_THRUST * Math.abs(rightTrigger.getValue());
        // derateValue = (DriveConstants.DRIVE_DERATE * Math.abs(leftTrigger.getValue()) + 1);
        // xOutput *= thrustValue/derateValue;
        // yOutput *= thrustValue/derateValue;
        // rotOutput *= thrustValue/derateValue;

    }
    
    @Override
    public void resetState() {
        setToTeleop();
        rotTarget = getGyroAngle();
        sensorOverride = false;
        // isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        // goalPose = FieldConstants.BLUE_AMP;
        // curX = 0.0;
        // curY = 0.0;
        // curTheta = 0.0;
    }

    @Override
    public void update() {
        // isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        
        // poseEstimator.update(odoAngle(), odoPosition());
        // targetYaw = pvCam.getYaw();
        // pvCam.odometryUpdate(poseEstimator);
        // curPose = poseEstimator.getEstimatedPosition();
        // for(int i = 0; i < modules.length; i++){
        //     moduleStates[i] = modules[i].getModuleState();
        // }

        switch (driveState) {
            case TELEOP:
                if (rotLocked) rotOutput = swerveHelper.getRotControl(rotTarget, getGyroAngle());  // if rotation tracking, replace rotational joystick value with controller generated one
                break;
            case AUTO:
                rotOutput = wSpeed * DriveConstants.DRIVE_F_ROT + swerveHelper.getRotControl(rotTarget, getGyroAngle());
                xOutput = xSpeed * DriveConstants.DRIVE_F_K + pathXErr * DriveConstants.POS_P;
                yOutput = ySpeed * DriveConstants.DRIVE_F_K + pathYErr * DriveConstants.POS_P;
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
        SmartDashboard.putNumber("Auto x pos", pathXErr);
        SmartDashboard.putNumber("Auto y pos", pathYErr);
        // SmartDashboard.putNumber("pose x", getPosX());
        // SmartDashboard.putNumber("pose y", getPosY());
        // SmartDashboard.putNumber("pose theta", getPosTheta());
        SmartDashboard.putNumber("rot target", rotTarget);
        SmartDashboard.putBoolean("Blue Alliance", isBlueAlliance);
        // SmartDashboard.putString("cur pose", curPose.toString());
    }
    
    @Override
    public void selfTest() {
    }

    @Override
    public String getName() {
        return "Swerve Drive";
    }

    /** resets the drive encoders on each module */
    public void resetDriveEncoders() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].resetDriveEncoders();
        }
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
        rotLocked = false;
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
        pathXErr = 0;
        pathYErr = 0;
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
    public void setAutoValues(double in1, double in2, double in3, double in4) {

    }

    /**sets autonomous values from the path data file */
    public void setAutoValues(double velocityX, double velocityY, double angVel, double xErr, double yErr, double heading) {
        xSpeed = velocityX;
        ySpeed = velocityY;
        wSpeed = angVel;
        pathXErr = xErr;
        pathYErr = yErr;
        rotTarget = heading;
    }

    /**sets the autonomous heading controller to a new target */
    public void setAutoHeading(double headingTarget) {
        rotTarget = headingTarget;
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

    // public Rotation2d odoAngle(){
    //     return new Rotation2d(getGyroAngle());
    // }

    // public SwerveModulePosition[] odoPosition(){
    //     return new SwerveModulePosition[]{modules[0].odoPosition(), modules[1].odoPosition(), modules[2].odoPosition(), modules[3].odoPosition()};
    // }

    // public void setPose(Pose2d pos){
    //     this.poseEstimator.resetPosition(odoAngle(), odoPosition(), pos);
    // }

    public Pose2d returnPose(){
        // return poseEstimator.getEstimatedPosition();
        return new Pose2d();
    }

    // public double getPosX(){
    //     double newX = poseEstimator.getEstimatedPosition().getX();
    //     if (!Double.isNaN(newX)) curX = newX;
    //     return curX;
    // }

    // public double getPosY(){
    //     double newY = poseEstimator.getEstimatedPosition().getY();
    //     if (!Double.isNaN(newY)) curY = newY;
    //     return curY;
    // }

    // public double getPosTheta(){
    //     double newTheta = (2.0 * Math.PI + (poseEstimator.getEstimatedPosition().getRotation().getRadians() % (2.0 * Math.PI))) % (2.0 * Math.PI);
    //     if (!Double.isNaN(newTheta)) curTheta = newTheta;
    //     return curTheta;
    // }

    // public Boolean isAtTarget(){
    //     if (sensorOverride) return true;
    //     return true;
    // }

    @Override
    public void initSubsystems(){

    }

    
}