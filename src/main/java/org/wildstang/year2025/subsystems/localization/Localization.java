package org.wildstang.year2025.subsystems.localization;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
// import org.wildstang.framework.logger.Log;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;

public class Localization implements Subsystem {
    
    private SwerveDrive drive;
    private SwerveDrivePoseEstimator estimator;
    private PhotonPoseEstimator frontEstimator;
    private PhotonCamera frontCam;
    private Matrix<N3, N1> curStdDevs;
    private Pose2d currentPose;
    StructPublisher<Pose2d> posePublisher, frontCamPublisher;
    StructArrayPublisher<Pose2d> visionTargetPublisher;
    Optional<EstimatedRobotPose> frontVisionEst;
    Pose2d[] frontVisTargets;

    @Override
    public void init() {
        frontCam = new PhotonCamera(LocalizationConstants.kFrontCam);
        frontEstimator = new PhotonPoseEstimator(LocalizationConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, LocalizationConstants.kBotToFrontCam);
        frontEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        currentPose = new Pose2d();
        posePublisher = NetworkTableInstance.getDefault().getStructTopic("Pose Estimator", Pose2d.struct).publish();
        frontCamPublisher = NetworkTableInstance.getDefault().getStructTopic("Front Cam Pose Estimator", Pose2d.struct).publish();
        visionTargetPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Vision Targets", Pose2d.struct).publish();
        frontVisionEst = Optional.empty();
    }

    @Override
    public void initSubsystems() {
        drive = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        estimator = new SwerveDrivePoseEstimator(drive.swerveKinematics, drive.getOdoAngle(), drive.getOdoPosition(), new Pose2d());
    }

    @Override
    public void inputUpdate(Input source) {
    }

    @Override
    public void update() {
        // Update pose estimator with drivetrain odometry values
        estimator.update(drive.getOdoAngle(), drive.getOdoPosition());

        // Update pose estimator with front camera
        frontVisionEst = Optional.empty();
        List<PhotonPipelineResult> temp = frontCam.getAllUnreadResults();
        for (var change : temp) {
            frontVisionEst = frontEstimator.update(change);
            List<PhotonTrackedTarget> targets = change.getTargets();
            frontVisTargets = new Pose2d[targets.size()];
            for (int i = 0; i < targets.size(); i++){
                frontVisTargets[i] = frontEstimator.getFieldTags().getTagPose(targets.get(i).getFiducialId()).get().toPose2d();
            }
            if (frontVisionEst.isPresent()) {  // this runs whenever we have had new camera data in the last ~20ms; if no new data since the last loop, don't bother updating
                visionTargetPublisher.set(frontVisTargets, (long) (1_000_000 * frontVisionEst.get().timestampSeconds));
                frontCamPublisher.set(frontVisionEst.get().estimatedPose.toPose2d(), (long) (1_000_000 * frontVisionEst.get().timestampSeconds));
                // Log.info("Processing FrontCam from timestamp " + frontVisionEst.get().timestampSeconds);
                updateEstimationStdDevs(frontVisionEst, targets);
                estimator.addVisionMeasurement(frontVisionEst.get().estimatedPose.toPose2d(), frontVisionEst.get().timestampSeconds, curStdDevs);
            }
        }

        // TODO: copy above code for back cam

        // Get current pose estimate after all updates
        currentPose = estimator.getEstimatedPosition();
        putDashboard();
    }

    private void putDashboard () {
        posePublisher.set(currentPose);
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = LocalizationConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = LocalizationConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = frontEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = LocalizationConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                estStdDevs = estStdDevs.times(1.0 / numTags);
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4) estStdDevs = LocalizationConstants.kMaxStdDevs;
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    public Pose2d getCurrentPose(){
        return currentPose;
    }

    public void setCurrentPose(Pose2d newPose){
        estimator.resetPosition(drive.getOdoAngle(), drive.getOdoPosition(), newPose);
    }

    // TODO: fill in
    // returns the nearest reef pose to align for intaking
    public Pose2d getNearestReefPose() {
        return new Pose2d();
    }

    // returns the processor target pose corresponding to the current side of the field the robot is on
    public Pose2d getProcessorTargetPose() {
        return (currentPose.getX() < LocalizationConstants.MID_FIELD_X) ? LocalizationConstants.BLUE_PROCESSOR : LocalizationConstants.RED_PROCESSOR;
    }

    public Pose2d getNetTargetPose() {
        double xTarget = (currentPose.getX() < LocalizationConstants.MID_FIELD_X) ? LocalizationConstants.BLUE_NET_X : LocalizationConstants.RED_NET_X;
        double rTarget = ((currentPose.getRotation().getRadians() + 2.0 * Math.PI) % (2.0 * Math.PI) > 3.0 * Math.PI / 2.0 || (currentPose.getRotation().getRadians() + 2.0 * Math.PI) % (2.0 * Math.PI) < Math.PI / 2.0) ? 0.0 : Math.PI;
        return new Pose2d(xTarget, currentPose.getY(), new Rotation2d(rTarget));
    }

    @Override
    public void resetState() {
    }

    @Override
    public void selfTest() {
    }

    @Override
    public String getName() {
        return "Localization";
    }
    
}
