package org.wildstang.year2025.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkLimitSwitch;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
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
// import org.wildstang.year2025.subsystems.Claw.Claw;
// import org.wildstang.year2025.subsystems.LED.LedSubsystem;
// import org.wildstang.year2025.subsystems.LED.LedSubsystem.LEDstates;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift.gameStates;
import org.wildstang.year2025.subsystems.localization.PoPVConstants;
import org.wildstang.year2025.subsystems.localization.WsPV;
import org.wildstang.year2025.subsystems.localization.Localization;
import org.wildstang.year2025.subsystems.localization.PoPV;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
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
    // private double derateValue = 1.0;
    private boolean rotLocked;
    private double rotTarget;
    private double pathXTarget;
    private double pathYTarget;

    public final Pigeon2 gyro = new Pigeon2(CANConstants.GYRO);
    public SwerveModule[] modules;
    private SwerveSignal swerveSignal;
    private WsSwerveHelper swerveHelper = new WsSwerveHelper();

    public SwerveDriveKinematics swerveKinematics;
    // private Claw claw;
    private ArmLift armLift;
    // private LedSubsystem led;

    public enum DriveState {TELEOP, AUTO, REEF, NET, PROCESSOR, GROUND_INTAKE};
    public DriveState driveState;
    private Pose2d curPose;

    private PoPV vision;
    private WsPV photonVision;

    private static final double DEG_TO_RAD = Math.PI / 180.0;
    private static final double RAD_TO_DEG = 180.0 / Math.PI;
    private Boolean rotHelperOverride = false;

    public final Field2d m_field = new Field2d();
    StructPublisher<Pose2d> publisher;

    @Override
    public void init() {
        initInputs();
        initOutputs();
        resetState();
        gyro.setYaw(0.0);
        SmartDashboard.putData("Field", m_field);
        publisher = NetworkTableInstance.getDefault().getStructTopic("Pose Estimator", Pose2d.struct).publish();
        vision = new PoPV();
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
        //create default swerveSignal
        swerveSignal = new SwerveSignal(new double[]{0.0, 0.0, 0.0, 0.0}, new double[]{0.0, 0.0, 0.0, 0.0});
    }

    @Override
    public void initSubsystems(){
        // led = (LedSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.LED);
        // claw = (Claw) Core.getSubsystemManager().getSubsystem(WsSubsystems.CLAW);
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
        
        // if(leftTrigger.getValue() != 0 && !rotHelperOverride){
        //     driveState = driveType.GROUND_INTAKE;
        //  }else if(rotOutput != 0){
        //      driveState = driveType.TELEOP;
        //  } 

        // if the rotational joystick is being used, the robot should not be auto tracking heading
        // otherwise engage rotation lock at current heading
        if (rotOutput != 0) {
            // if (armLift.gameState == gameStates.L2_ALGAE_REEF || armLift.gameState == gameStates.L3_ALGAE_REEF) {

            // } else {
                rotOutput *= Math.abs(rotOutput);
                rotOutput *= DriveConstants.ROTATION_SPEED;
                rotLocked = false;
            // }
        // }  else if (rotLocked == false) {
        //     rotTarget = getGyroAngle();
        //     rotLocked = true;
        } else {
            rotLocked = true;
        }
        
        //assign thrust
        // derateValue = (DriveConstants.DRIVE_DERATE * Math.abs(leftTrigger.getValue()) + 1);
        // xOutput /= derateValue;
        // yOutput /= derateValue;
        // rotOutput /= derateValue;
    }
    
    @Override
    public void resetState() {
        setToTeleop();
    }

    @Override
    public void update() {
        curPose = photonVision.curPose;
        switch (driveState) {
            case TELEOP:
                if (!rotHelperOverride && rotLocked) {
                    switch (armLift.gameState){
                        case GROUND_INTAKE:
                            if (algaeInView() && armLift.isAtSetpoint()) {
                                // rotLocked = true;
                                // derateValue = 0.75;
                                rotTarget =  ((1.0 - pixyAnalog.getVoltage()) * 0.70 + getGyroAngle() + 2.0 * Math.PI) % (2.0 * Math.PI);
                                rotOutput = swerveHelper.getRotControl(rotTarget, getGyroAngle());
                            }
                            break;
                        case L2_ALGAE_REEF:
                            
                            if(DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)){
                                if(!vision.leftCameraResults.isEmpty()){
                                    for(PhotonPipelineResult result : vision.leftCameraResults){
                                        if (result.hasTargets()){
                                            PhotonTrackedTarget closestTarget = result.getBestTarget();
                                            int closestTargetID = closestTarget.getFiducialId();
                                            for(PhotonTrackedTarget target: result.getTargets()){
                                                if(target.getFiducialId() == closestTargetID && (closestTargetID % 2 != 0)){
                                                    xOutput = PoPVConstants.get(target.getFiducialId()).getX();
                                                    yOutput = PoPVConstants.get(target.getFiducialId()).getY();
                                                    rotOutput = swerveHelper.getRotControl(target.getYaw(), getGyroAngle());
                                                    this.swerveSignal = swerveHelper.setDrive(xOutput, yOutput, rotOutput, getGyroAngle());
                                                    drive();
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                            if(DriverStation.getAlliance().equals(DriverStation.Alliance.Red)){
                                if(!vision.leftCameraResults.isEmpty()){
                                    for(PhotonPipelineResult result : vision.leftCameraResults){
                                        if (result.hasTargets()){
                                            PhotonTrackedTarget closestTarget = result.getBestTarget();
                                            int closestTargetID = closestTarget.getFiducialId();
                                            for(PhotonTrackedTarget target: result.getTargets()){
                                                if(target.getFiducialId() == closestTargetID && (closestTargetID % 2 == 0)){
                                                    xOutput = PoPVConstants.get(target.getFiducialId()).getX();
                                                    yOutput = PoPVConstants.get(target.getFiducialId()).getY();
                                                    rotOutput = swerveHelper.getRotControl(target.getYaw(), getGyroAngle());
                                                    this.swerveSignal = swerveHelper.setDrive(xOutput, yOutput, rotOutput, getGyroAngle());
                                                    drive();
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            break;
                            
                        case L3_ALGAE_REEF:
                            // derateValue = 0.75;
                            double curAngle = getGyroAngle();
                            if (curAngle > 11.0 * Math.PI / 6.0 || curAngle < Math.PI / 6.0) rotTarget = 0.0;
                            else if (curAngle < 3.0 * Math.PI / 6.0) rotTarget = 2.0 * Math.PI / 6.0;
                            else if (curAngle < 5.0 * Math.PI / 6.0) rotTarget = 4.0 * Math.PI / 6.0;
                            else if (curAngle < 7.0 * Math.PI / 6.0) rotTarget = 6.0 * Math.PI / 6.0;
                            else if (curAngle < 9.0 * Math.PI / 6.0) rotTarget = 8.0 * Math.PI / 6.0;
                            else rotTarget = 10.0 * Math.PI / 6.0;
                            // rotTarget = ((double) Math.round(getGyroAngle() / (Math.PI / 3.0)) % 6.0) * (Math.PI / 3.0);
                            rotOutput = swerveHelper.getRotControl(rotTarget, getGyroAngle());
                            break;
                        //     if (algaeInView() && armLift.isAtSetpoint()){
                        //         yOutput = (1.0 - pixyAnalog.getVoltage()) * 0.30;// * derateValue;
                        //         rotOutput = Math.min(Math.max(rotOutput, -1.0), 1.0);
                        //         //TODO: this is very janky and should be refactored to be more logical
                        //         //undo red alliance inversion so while in robot relative mode forward always moves forward
                        //         if (!Core.isBlueAlliance()) {
                        //             xOutput *= -1;
                        //         }
                        //         xOutput = Math.min(Math.max(xOutput, -1.0), 1.0);// * derateValue;
                        //         yOutput = Math.min(Math.max(yOutput, -1.0), 1.0);// * derateValue;
                        //         this.swerveSignal = swerveHelper.setDrive(xOutput , yOutput, rotOutput, 0);
                        //         drive();
                        //         putDashboard();
                        //         return;
                        //     }
                        case PROCESSOR:
                            // derateValue = 0.75;
                            rotTarget = (getGyroAngle() <= Math.PI) ? Math.PI / 2.0 : 3.0 * Math.PI / 2.0;
                            rotOutput = swerveHelper.getRotControl(rotTarget, getGyroAngle());
                            break;
                        case SHOOT_NET:
                            rotTarget = (getGyroAngle() <= Math.PI / 2.0 || getGyroAngle() >= 3.0 * Math.PI / 2.0) ? 0 : Math.PI;
                            rotOutput = swerveHelper.getRotControl(rotTarget, getGyroAngle());
                            alignBarge();
                            break;
                            // derateValue = 0.5;
                        default:
                            // derateValue = 1.0;
                            break;
                    }
                }
                // if (rotLocked) rotOutput = swerveHelper.getRotControl(rotTarget, getGyroAngle());  // if rotation tracking, replace rotational joystick value with controller generated one
                break;
            case AUTO:
                rotOutput = wSpeed * DriveConstants.DRIVE_F_ROT + swerveHelper.getRotControl(rotTarget, getGyroAngle());
                xOutput = xSpeed * DriveConstants.DRIVE_F_K + (pathXTarget - curPose.getX()) * DriveConstants.POS_P;
                yOutput = ySpeed * DriveConstants.DRIVE_F_K + (pathYTarget - curPose.getY()) * DriveConstants.POS_P;
                break;
            // case GROUND_INTAKE:
            //     led.ledState = LEDstates.ALGAE_DETECT;
            //     if(claw.algaeInClaw){
            //         driveState = driveType.TELEOP;
            //     }
            //     if (algaeInView() && armLift.isAtSetpoint()) {
            //         rotLocked = true;
            //         rotTarget =  ((1.0 - pixyAnalog.getVoltage()) * 0.70 + getGyroAngle() + 2.0 * Math.PI) % (2.0 * Math.PI);
            //         rotOutput = swerveHelper.getRotControl(rotTarget, getGyroAngle());
            //     }
            //     break; 
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

    private void alignBarge(){
        if(DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)){
            if(!vision.leftCameraResults.isEmpty()){
                for(PhotonPipelineResult result : vision.leftCameraResults){
                    if (result.hasTargets()){
                        for(PhotonTrackedTarget target : result.getTargets()){
                            if(target.getFiducialId() == 14){
                                
                                
                                if(robotPose.getReferencePose().getX() < 200 && robotPose.getReferencePose().getY() < 300){
        
                                    var cameraPose = curPose.transformBy(new Transform2d(new Translation2d(PoPVConstants.leftCameraToRobot.getX(), PoPVConstants.leftCameraToRobot.getY()), new Rotation2d(PoPVConstants.leftCameraToRobot.getRotation().getAngle())));
                                    var targetPose = cameraPose.transformBy(new Transform2d(new Translation2d(camToTarget.getX(), camToTarget.getY()), new Rotation2d(camToTarget.getRotation().getAngle())));
                                    
                                    xOutput = PoPVConstants.get(target.getFiducialId()).getX();
                                    yOutput = PoPVConstants.get(target.getFiducialId()).getY();
                                    rotOutput = swerveHelper.getRotControl(target.getYaw(), getGyroAngle());
                                }
                                
                                
                                this.swerveSignal = swerveHelper.setDrive(xOutput, yOutput, rotOutput, getGyroAngle());
                                drive();
                            }
                        }
                    }
                }
            }
        }
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
        // SmartDashboard.putString("cur pose", curPose.toString());
        SmartDashboard.putNumber("Pixy Voltage", pixyAnalog.getVoltage());
        SmartDashboard.putBoolean("Pixy Obj Det", pixyDigital.isPressed());
        SmartDashboard.putBoolean("Rot Control Override", rotHelperOverride);
        SmartDashboard.putBoolean("rot lock", rotLocked);
        publisher.set(curPose);
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
     * @param degrees the current value the gyro should read
     */
    public void setGyro(double radians) {
        StatusCode code = gyro.setYaw(radians * RAD_TO_DEG);
        SmartDashboard.putString("gyro status code", code.getName());
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