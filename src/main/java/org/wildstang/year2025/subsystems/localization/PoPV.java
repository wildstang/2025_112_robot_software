import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PoPV {

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    /* Left Camera */
    PhotonCamera leftCamera;
    PhotonCamera rightCamera;

    PhotonPipelineResult leftCameraResults;
    boolean leftCameraHasTarget;
    PhotonTrackedTarget leftCameraBestTarget;
    Transform3d leftCameraToRobot;

     /* ------------------ */

    PhotonCamera liftTopCamera;
    List<PhotonPipelineResult> cameraResults;
    boolean cameraHasResult;
    PhotonTrackedTarget cameraTarget;
    Transform3d currentAprilTag;
    Pose3d estimatedRobotPose;
    PhotonPoseEstimator photonPoseEstimator;

    /* Right Camera */

    public PoPV(String leftCameraID, String rightCameraID){
        leftCamera = new PhotonCamera(leftCameraID);
        rightCamera = new PhotonCamera(rightCameraID);
        leftCameraToRobot = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,90));
        photonPoseEstimator= new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, leftCameraToRobot);
    }

    public void update(){
        if (!cameraResults.isEmpty()) {
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
        
        }
    }
        
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedPose){
        photonPoseEstimator.setReferencePose(prevEstimatedPose);
        return photonPoseEstimator.update(cameraTarget);
    }



    

}
