package org.wildstang.year2025.subsystems.localization;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.logger.Log;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift.GameStates;
import org.wildstang.year2025.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;

public class Localization implements Subsystem {
    
    private SwerveDrive drive;
    private SwerveDrivePoseEstimator estimator;
    private PhotonPoseEstimator frontEstimator, rearEstimator;
    private PhotonCamera frontCam, rearCam;
    private Matrix<N3, N1> curStdDevs;
    private Pose2d currentPose, bestPose;
    StructPublisher<Pose2d> posePublisher, bestPosePublisher, frontCamPublisher, rearCamPublisher;
    StructArrayPublisher<Pose2d> frontVisTargetPublisher, rearVisTargetPublisher;
    StructArrayPublisher<SwerveModulePosition> modulePosPublisher;
    StructPublisher<Rotation2d> odoAngPublisher;
    Optional<EstimatedRobotPose> visionEst;
    List<PhotonTrackedTarget> targets;
    Pose2d[] visTargets;

    private static enum TargetType {REEF, PROCESSOR, BARGE};
    private TargetType target = null;

    @Override
    public void init() {
        modulePosPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("odo position", SwerveModulePosition.struct).publish();
        odoAngPublisher = NetworkTableInstance.getDefault().getStructTopic("odo angle", Rotation2d.struct).publish();
        currentPose = new Pose2d();
        posePublisher = NetworkTableInstance.getDefault().getStructTopic("Pose Estimator", Pose2d.struct).publish();
        bestPose = new Pose2d();
        bestPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Best Pose Target", Pose2d.struct).publish();

        frontCam = new PhotonCamera(LocalizationConstants.kFrontCam);
        frontEstimator = new PhotonPoseEstimator(LocalizationConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, LocalizationConstants.kBotToFrontCam);
        frontEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        frontCamPublisher = NetworkTableInstance.getDefault().getStructTopic("Front Cam Pose Estimator", Pose2d.struct).publish();
        frontVisTargetPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Front Vision Targets", Pose2d.struct).publish();

        rearCam = new PhotonCamera(LocalizationConstants.kRearCam);
        rearEstimator = new PhotonPoseEstimator(LocalizationConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, LocalizationConstants.kBotToRearCam);
        rearEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        rearCamPublisher = NetworkTableInstance.getDefault().getStructTopic("Rear Cam Pose Estimator", Pose2d.struct).publish();
        rearVisTargetPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Rear Vision Targets", Pose2d.struct).publish();
    }

    @Override
    public void initSubsystems() {
        drive = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        estimator = new SwerveDrivePoseEstimator(drive.swerveKinematics, drive.getOdoAngle(), drive.getOdoPosition(), currentPose);
    }

    @Override
    public void inputUpdate(Input source) {
    }

    @Override
    public void update() {
        // Update pose estimator with drivetrain odometry values
        estimator.update(drive.getOdoAngle(), drive.getOdoPosition());
        modulePosPublisher.set(drive.getOdoPosition());
        odoAngPublisher.set(drive.getOdoAngle());

        // Update pose estimator with vision estimates
        processPVResults(frontCam, frontEstimator, frontVisTargetPublisher, frontCamPublisher);
        processPVResults(rearCam, rearEstimator, rearVisTargetPublisher, rearCamPublisher);

        // Get current pose estimate after all updates
        currentPose = estimator.getEstimatedPosition();
        putDashboard();
    }

    private void processPVResults(PhotonCamera cam, PhotonPoseEstimator camEstimator, StructArrayPublisher<Pose2d> visTargetPublisher, StructPublisher<Pose2d> camPosePublisher){
        // Update pose estimator with camera
        for (var change : cam.getAllUnreadResults()) {
            // create List of targets to publish
            targets = change.getTargets();
            visTargets = new Pose2d[targets.size()];
            for (int i = 0; i < targets.size(); i++){
                visTargets[i] = LocalizationConstants.kTagLayout.getTagPose(targets.get(i).getFiducialId()).get().toPose2d();
            }
            visTargetPublisher.set(visTargets, (long) (1_000_000 * change.getTimestampSeconds()));

            // update pose estimator
            visionEst = camEstimator.update(change);
            if (visionEst.isPresent()) {
                camPosePublisher.set(visionEst.get().estimatedPose.toPose2d(), (long) (1_000_000 * visionEst.get().timestampSeconds));
                updateEstimationStdDevs(visionEst, targets);
                estimator.addVisionMeasurement(visionEst.get().estimatedPose.toPose2d(), visionEst.get().timestampSeconds, curStdDevs);
            } else {
                camPosePublisher.set(null, (long) (1_000_000 * change.getTimestampSeconds()));
            }
        }
    }

    private void putDashboard () {
        posePublisher.set(currentPose);
        // bestPosePublisher.set(getNearestReefPose());
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
                var tagPose = LocalizationConstants.kTagLayout.getTagPose(tgt.getFiducialId());
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
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));  // TODO: tune std devs
                // TODO: possibly add logic to reject or greatly increase std devs if the new estimate is far from the previous estimate
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

    // returns the nearest reef pose to align for intaking
    public Object[] getNearestReefHeight() {
        GameStates newState;
        Boolean isFront = true;
        bestPose = currentPose.nearest(LocalizationConstants.REEF_POSES);

        // determine correct reef height
        if (LocalizationConstants.L2_POSES.contains(bestPose)) newState = GameStates.L2_ALGAE_REEF;
        else newState = GameStates.L3_ALGAE_REEF;

        if (Math.abs(MathUtil.angleModulus(currentPose.getRotation().getRadians() - bestPose.getRotation().getRadians())) > Math.PI / 2.0) {
            bestPose = new Pose2d(bestPose.getTranslation(), bestPose.getRotation().plus(Rotation2d.kPi)).plus(LocalizationConstants.CLAW_OFFSET);
            isFront = false;
        }

        target = TargetType.REEF;
        bestPosePublisher.set(bestPose);  // publish updated bestPose
        return new Object[] {newState, isFront};
    }

    // returns the nearest reef pose to align for intaking
    public Pose2d getNearestReefPose() {
        if (target != TargetType.REEF) {
            Log.warn("Returning Reef pose, but Reef has not been called by ArmLift yet");
            getNearestReefHeight();
        }
        target = null;  // reset target to ensure proper function call order each loop
        return bestPose;
    }

    // return closest direction to face for scoring in processor
    public boolean getNearestProcessorDirection() {
        boolean isFront = true;
        bestPose = (currentPose.getX() < LocalizationConstants.MID_FIELD_X) ? LocalizationConstants.BLUE_PROCESSOR : LocalizationConstants.RED_PROCESSOR;
        if (Math.abs(MathUtil.angleModulus(currentPose.getRotation().getRadians() - bestPose.getRotation().getRadians())) > Math.PI / 2.0) {
            bestPose = new Pose2d(bestPose.getTranslation(), bestPose.getRotation().plus(Rotation2d.kPi));
            isFront = false;
        }

        target = TargetType.PROCESSOR;
        bestPosePublisher.set(bestPose);  // publish updated bestPose
        return isFront;
    }

    // returns the processor target pose corresponding to the current side of the field the robot is on
    public Pose2d getNearestProcessorPose() {
        if (target != TargetType.PROCESSOR) {
            Log.warn("Returning Processor pose, but Processor has not been called by ArmLift yet");
            getNearestProcessorDirection();
        }
        target = null;  // reset target to ensure proper function call order each loop
        return bestPose;
    }

    // return closest direction to face for scoring in the net
    public boolean getNearestBargeDirection() {
        double xTarget; 
        double rTarget = 0.0;
        boolean isFront;
        if (currentPose.getX() < LocalizationConstants.MID_FIELD_X) {
            xTarget = LocalizationConstants.BLUE_NET_X;
            isFront = true;
            if (Math.abs(MathUtil.angleModulus(currentPose.getRotation().getRadians())) > Math.PI / 2.0) {
                rTarget = Math.PI;
                isFront = false;
            }
        } else {
            xTarget = LocalizationConstants.RED_NET_X;
            isFront = false;
            if (Math.abs(MathUtil.angleModulus(currentPose.getRotation().getRadians())) > Math.PI / 2.0) {
                rTarget = Math.PI;
                isFront = true;
            }
        }
        bestPose = new Pose2d(xTarget, currentPose.getY(), new Rotation2d(rTarget));

        target = TargetType.BARGE;
        bestPosePublisher.set(bestPose);  // publish updated bestPose
        return isFront;
    }

    public Pose2d getNearestBargePose() {
        if (target != TargetType.BARGE) {
            Log.warn("Returning Barge pose, but Barge has not been called by ArmLift yet");
            getNearestBargeDirection();
        }
        target = null;  // reset target to ensure proper function call order each loop
        return bestPose;
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
