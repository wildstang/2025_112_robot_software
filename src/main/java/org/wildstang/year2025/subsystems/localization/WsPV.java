package org.wildstang.year2025.subsystems.localization;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WsPV {

    public PhotonCamera camera;
    public String cameraID;
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public PhotonTrackedTarget target;
    public List<PhotonPipelineResult> result;
    private Transform3d robotToCamera = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0,0,0));

    private VisionConsts VC = new VisionConsts();

    public int tid = 0;
    public double tx = 0;
    public double ty = 0;
    public boolean tv = false;
    public Transform3d aprilTag = new Transform3d();
    public Pose3d estimatedPose = new Pose3d();

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

    public void update(){
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
        SmartDashboard.putNumber(cameraID + "poseX", estimatedPose.getX()*VC.mToIn);
        SmartDashboard.putNumber(cameraID + "posey", estimatedPose.getY()*VC.mToIn);
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

}
