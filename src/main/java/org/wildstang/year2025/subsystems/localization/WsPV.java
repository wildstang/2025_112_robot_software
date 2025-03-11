package org.wildstang.year2025.subsystems.localization;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.year2025.robot.WsOutputs;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.swerve.DriveConstants;
import org.wildstang.year2025.subsystems.swerve.SwerveDrive;
import org.wildstang.year2025.subsystems.swerve.SwerveModule;
import org.wildstang.year2025.subsystems.swerve.SwerveSignal;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WsPV implements Subsystem {

    public PhotonCamera camera;
    public String cameraID;
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public PhotonTrackedTarget target;
    public List<PhotonPipelineResult> result;
    private Transform3d robotToCamera = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0,0,0));
    private SwerveDrive swerve;
    private SwerveDrivePoseEstimator poseEstimator; 
    private SwerveModule[] modules;
    private SwerveDriveKinematics swerveKinematics;
    

    // private VisionConsts VC = new VisionConsts();

    public int tid = 0;
    public double tx = 0;
    public double ty = 0;
    public boolean tv = false;
    public Transform3d aprilTag = new Transform3d();
    public Pose3d estimatedPose = new Pose3d();
    public Pose2d curPose;
    public PhotonCamera frontCamera, backCamera; 
    private EstimatedRobotPose frontPose, rearPose;

    //whether the cam detects ATs or Notes
    public boolean isAT;

    public WsPV(String cameraID, boolean isAprilTag){
        this.cameraID = cameraID;
        camera = new PhotonCamera(cameraID);
        isAT = isAprilTag;
    }
    public WsPV(String cameraID){
        this(cameraID, false);
    }

    @Override
    public void inputUpdate(Input source) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'inputUpdate'");
    }
    @Override
    public void init() {
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

        swerveKinematics = new SwerveDriveKinematics(new Translation2d(DriveConstants.ROBOT_WIDTH/2, DriveConstants.ROBOT_LENGTH/2), new Translation2d(DriveConstants.ROBOT_WIDTH/2, -DriveConstants.ROBOT_LENGTH/2),
         new Translation2d(-DriveConstants.ROBOT_WIDTH/2, DriveConstants.ROBOT_LENGTH/2), new Translation2d(-DriveConstants.ROBOT_WIDTH/2, -DriveConstants.ROBOT_LENGTH/2));
         curPose = new Pose2d(0.0,0.0, new Rotation2d(0.0));

         frontCamera = new PhotonCamera("Front Camera");
         backCamera = new PhotonCamera("Back Camera"); 
    }
    @Override
    public void initSubsystems() {
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        poseEstimator = new SwerveDrivePoseEstimator(swerveKinematics, swerve.getOdoAngle(), swerve.getOdoPosition(), new Pose2d());
    }

    public void update(){
        poseEstimator.update(swerve.getOdoAngle(), swerve.getOdoPosition());
        curPose = poseEstimator.getEstimatedPosition();
        result = camera.getAllUnreadResults();
        if (!result.isEmpty()) {
            for (int i = 0; i < result.size(); i++) {
                SmartDashboard.putBoolean("hasTargets", result.get(i).hasTargets());
                tv = result.get(i).hasTargets();
                if(tv) {
                    target = result.get(i).getBestTarget();
                    tx = target.getYaw();
                    ty = target.getPitch();

                    //only used if it is an AT cam
                    if(isAT){
                        tid = target.getFiducialId();
                        aprilTag = target.getBestCameraToTarget();
                        estimatedPose = PhotonUtils.estimateFieldToRobotAprilTag(aprilTag,
                            aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), robotToCamera);
                    }   

                }
            }
        }
        updateDashboard();
    }

    public void updateDashboard(){
        SmartDashboard.putBoolean(cameraID + " tv", tv);
        SmartDashboard.putNumber(cameraID + " tid", tid);
        SmartDashboard.putNumber(cameraID + " Y", ty);
        SmartDashboard.putNumber(cameraID + " X", tx);
        SmartDashboard.putBoolean(cameraID + " isAT", isAT);
        // SmartDashboard.putNumber(cameraID + "poseX", estimatedPose.getX()*VC.mToIn);
        // SmartDashboard.putNumber(cameraID + "posey", estimatedPose.getY()*VC.mToIn);
    }

    public boolean TargetInView(){
        return tv;
    }

    public double getPoseX(){
        return estimatedPose.getX();
    }

    public double getPoseY(){
        return estimatedPose.getY();
    }

    public int getAprilTagID() {
        return tid;
    }

    public void odometryUpdate(SwerveDrivePoseEstimator estimator) {
        if(frontPose != null){
            estimator.addVisionMeasurement(frontPose.estimatedPose.toPose2d(), frontPose.timestampSeconds);
        }
        if(rearPose != null){
            estimator.addVisionMeasurement(rearPose.estimatedPose.toPose2d(), rearPose.timestampSeconds);
        }
    }
   
    @Override
    public void selfTest() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'selfTest'");
    }
    @Override
    public void resetState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetState'");
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getName'");
    }

}
