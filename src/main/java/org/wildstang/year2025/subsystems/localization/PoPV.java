package org.wildstang.year2025.subsystems.localization;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class PoPV {

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    PhotonCamera liftTopCamera;
    List<PhotonPipelineResult> cameraResults;
    boolean cameraHasResult;
    PhotonTrackedTarget cameraTarget;
    Transform3d currentAprilTag;
    Pose3d estimatedRobotPose;


    public PoPV(String cameraID){
        liftTopCamera = new PhotonCamera(cameraID);
    }

    public void update(){
        cameraResults = liftTopCamera.getAllUnreadResults();
        if (!cameraResults.isEmpty()) {
            for (int i = 0; i < cameraResults.size(); i++) {
            cameraHasResult = cameraResults.get(i).hasTargets();
            
                if(cameraHasResult){
                    cameraTarget = cameraResults.get(i).getBestTarget();
                    currentAprilTag = cameraTarget.getBestCameraToTarget();
                    
                }
            }
        }
    }



}
