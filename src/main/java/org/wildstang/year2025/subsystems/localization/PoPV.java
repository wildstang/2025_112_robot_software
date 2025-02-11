package org.wildstang.year2025.subsystems.localization;



import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class PoPV {

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    PhotonCamera liftTopCamera;
    PhotonPipelineResult cameraResults;
    boolean cameraHasResult;
    PhotonTrackedTarget cameraTarget;
    Transform3d currentAprilTag;
    Pose3d estimatedRobotPose;


    public PoPV(String cameraID){
        liftTopCamera = new PhotonCamera(cameraID);
    }

    public void update(){
        cameraResults = liftTopCamera.getLatestResult();
        cameraHasResult = cameraResults.hasTargets();
    
        if(cameraHasResult){
            cameraTarget = cameraResults.getBestTarget();
            currentAprilTag = cameraTarget.getBestCameraToTarget();
            
        }
    }



}
