package org.wildstang.year2025.subsystems.localization; 

import java.util.ArrayList;
import java.util.List;
// import java.util.Optional;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
// import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class PoPV {

    public AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    /* Left Camera */
    PhotonCamera leftCamera; //Vision camera
    PhotonCamera rightCamera;

    public List<PhotonPipelineResult> leftCameraResults;
    boolean leftCameraHasTarget;
    PhotonTrackedTarget leftCameraBestTarget;
     // Defines position and orientation of how the camera is mounted
  


     /* ------------------ */

    PhotonCamera liftTopCamera;
    
    boolean cameraHasResult;
    PhotonTrackedTarget cameraTarget;
    Transform3d currentAprilTag;
    Pose3d estimatedRobotPose;
    PhotonPoseEstimator photonPoseEstimator; // Fuses the camera's AprilTag detections with the known tag layout to compute an estimated robot pose on the field
    Matrix<N3,N1> currentStandardDeviation; // 3x1 matrix (vector) holding the standard deviation values for the pose estimation error (X,Y, and heading)

    /* Right Camera */

    public PoPV(){
        // leftCamera = new PhotonCamera(VisionConsts.leftCameraID);
        // rightCamera = new PhotonCamera(VisionConsts.rightCameraID);
        
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PoPVConstants.leftCameraToRobot);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); // if pose estimator cannot determine pose with multipe tags, this will choose the solution with the lowest ambiguity
    }


    public PhotonPoseEstimator getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose){
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        for(var target : leftCameraResults){
           photonPoseEstimator.update(target);
        }
        return photonPoseEstimator;
    }
        

    public void update(){
        leftCameraResults = leftCamera.getAllUnreadResults();

        
        

        /*
         *   if (!cameraResults.isEmpty()) {
            for (int i = 0; i < cameraResults.size(); i++) {
                cameraHasResult = cameraResults.get(i).hasTargets();
            
                if(cameraHasResult){
                    cameraTarget = cameraResults.get(i).getBestTarget();
                    currentAprilTag = cameraTarget.getBestCameraToTarget();
                    
                }
            }

            if(aprilTagFieldLayout.getTagPose(cameraTarget.getFiducialId()).isPresent()){
                estimatedRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraTarget.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(cameraTarget.getFiducialId()).get(), leftCameraToRobot);
            }
         */
    }
    
        
    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(){//Process new vision data and return and estimated pose if can be calculated
    //     Optional<EstimatedRobotPose> visionEst = Optional.empty(); //Creates empty optional to hold estimation result
    //     for(var change : leftCamera.getAllUnreadResults()){ //Loops through each new result from camera
    //         visionEst = photonPoseEstimator.update(change); // For each new vision result, the photonestimater is updated and estimated pose is stored
    //         // updateEstimationStdDevs(visionEst, change.getTargets()); // Unimplemented method that will update the standard deviations values on the new estimated lsit of targets
    //     }
    // }

    public int getCurrentAprilTagID(){
        return cameraTarget.getFiducialId();
    }

    public Transform3d getCurrentAprilTagTransform3d(){
        return currentAprilTag;
    }
    public PhotonTrackedTarget getCurrentTarget(){
        return cameraTarget;
    }
   

    
}
