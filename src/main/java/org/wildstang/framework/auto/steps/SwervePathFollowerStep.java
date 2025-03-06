package org.wildstang.framework.auto.steps;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.Optional;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.logger.Log;
import org.wildstang.framework.subsystems.swerve.SwerveDriveTemplate;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.localization.Localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import choreo.*;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import com.google.gson.Gson;

public class SwervePathFollowerStep extends AutoStep {

    private final Optional<Trajectory<SwerveSample>> pathtraj;

    private Localization loc;

    private SwerveDriveTemplate m_drive;
    private SwerveSample sample;
    private ChassisSpeeds sampleVel;
    private Pose2d drivePose;

    private Timer timer;

    private Boolean isBlue;

    private final double endTime;
    StructArrayPublisher<Pose2d> trajPublisher;

    /** Sets the robot to track a new path
     * finishes after all values have been read to robot
     * @param pathData double[][] that contains path, should be from \frc\paths
     * @param drive the swerveDrive subsystem
     */
    public SwervePathFollowerStep(String pathData, SwerveDriveTemplate drive) {
        pathtraj = Choreo.loadTrajectory(pathData);
        m_drive = drive;
        loc = (Localization) Core.getSubsystemManager().getSubsystem(WsSubsystems.LOCALIZATION);
        timer = new Timer();
        endTime = pathtraj.get().getTotalTime();
        isBlue = Core.isBlueAlliance();
        trajPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Traj Pose", Pose2d.struct).publish();

    }

    /** Sets the robot to track a new path
     * finishes after all values have been read to robot
     * @param pathData double[][] that contains path, should be from \frc\paths
     * @param drive the swerveDrive subsystem
     * @param isFirstPath if true, resets swerveDrive gyro and pose estimator using the initial pose of the path
     */
    public SwervePathFollowerStep(String pathData, SwerveDriveTemplate drive, Boolean isFirstPath) {
        this(pathData, drive);
        if (isFirstPath) {
            m_drive.setGyro(pathtraj.get().getInitialPose(!isBlue).get().getRotation().getRadians());
            loc.setCurrentPose(pathtraj.get().getInitialPose(!isBlue).get());
            trajPublisher.set(pathtraj.get().getPoses());
        }
    }

    @Override
    public void initialize() {
        //start path
        m_drive.setToAuto();
        timer.start();
        trajPublisher.set(pathtraj.get().getPoses());
    }

    @Override
    public void update() {
        if (timer.get() >= endTime) {
            sample = pathtraj.get().getFinalSample(!isBlue).get();
            drivePose = loc.getCurrentPose();
            Log.warn(Double.toString(sample.x - drivePose.getX()) + Double.toString(sample.y - drivePose.getY()));
            m_drive.setAutoValues(0.0, 0.0, 0.0, sample.x, sample.y, (((2.0 * Math.PI)+sample.heading)%(2.0 * Math.PI)));
            setFinished();
        } else {
            sample = pathtraj.get().sampleAt(timer.get(), !isBlue).get();
            sampleVel = ChassisSpeeds.discretize(sample.getChassisSpeeds(), 0.02);

            m_drive.setAutoValues(sampleVel.vxMetersPerSecond, sampleVel.vyMetersPerSecond, sampleVel.omegaRadiansPerSecond, sample.x, sample.y, (((2.0 * Math.PI)+sample.heading)%(2.0 * Math.PI)));
        }
    }

    @Override
    public String toString() {
        return "Swerve Path Follower";
    }

    @SuppressWarnings("unchecked")
    public Trajectory<SwerveSample> getTraj(String fileName){
        Gson gson = new Gson();
        var tempfile = Filesystem.getDeployDirectory();
        var traj_dir = new File(tempfile, "choreo");

        var traj_file = new File(traj_dir, fileName + ".traj");
        try {
            var reader = new BufferedReader(new FileReader(traj_file));
            return  gson.fromJson(reader, Trajectory.class);
        } catch (Exception ex) {
            DriverStation.reportError("Choreo Trajectory get Error", ex.getStackTrace());
        }return new Trajectory<SwerveSample>(null, null, null, null);
    }
}
