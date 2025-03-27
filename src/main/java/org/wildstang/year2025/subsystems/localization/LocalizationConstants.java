package org.wildstang.year2025.subsystems.localization;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class LocalizationConstants {
    public static final String kFrontCam = "FrontCam";
    public static final String kRearCam = "RearCam";
    public static final Transform3d kBotToFrontCam = new Transform3d(new Translation3d(0.184, 0.161, 0.170), new Rotation3d(0, -0.367, 0));
    public static final Transform3d kBotToRearCam = new Transform3d(new Translation3d(-0.184, 0.161, 0.170), new Rotation3d(0, -0.367, Math.PI));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect correction rate
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, Double.MAX_VALUE);  // TODO: tune these values
    public static final Matrix<N3, N1> kMaxStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

    public static final Transform2d CLAW_OFFSET = new Transform2d(0, -0.07, Rotation2d.kZero);
    /* Goal Poses */
    public static final double MID_FIELD_X = 8.77;
    
    public static final Pose2d BLUE_PROCESSOR = new Pose2d(6.00, 0.81, Rotation2d.fromDegrees(-90.0));
    public static final Pose2d RED_PROCESSOR = new Pose2d(11.50, 7.27, Rotation2d.fromDegrees(90.0));
    
    public static final double BLUE_NET_X = 7.60;
    public static final double RED_NET_X = 10.00;

    public static final Pose2d BLUE_REEF_AB = new Pose2d(2.82,3.98, Rotation2d.kZero);
    public static final Pose2d BLUE_REEF_CD = new Pose2d(3.69,2.61, Rotation2d.fromDegrees(60.0));
    public static final Pose2d BLUE_REEF_EF = new Pose2d(5.35, 2.61, Rotation2d.fromDegrees(120.0));
    public static final Pose2d BLUE_REEF_GH = new Pose2d(6.15, 4.08, Rotation2d.kPi);
    public static final Pose2d BLUE_REEF_IJ = new Pose2d(5.27,5.48, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d BLUE_REEF_KL = new Pose2d(3.63,5.44, Rotation2d.fromDegrees(-60.0));

    public static final Pose2d RED_REEF_AB = new Pose2d(14.70, 4.06, Rotation2d.kPi);
    public static final Pose2d RED_REEF_CD = new Pose2d(13.46, 5.48, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d RED_REEF_EF = new Pose2d(12.22, 5.47, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d RED_REEF_GH = new Pose2d(11.42, 3.99, Rotation2d.kZero);
    public static final Pose2d RED_REEF_IJ = new Pose2d(12.26, 2.62, Rotation2d.fromDegrees(60.0));
    public static final Pose2d RED_REEF_KL = new Pose2d(13.83, 2.65, Rotation2d.fromDegrees(120.0));
    public static final List<Pose2d> REEF_POSES = List.of(BLUE_REEF_AB, BLUE_REEF_CD, BLUE_REEF_EF, BLUE_REEF_GH, BLUE_REEF_IJ, BLUE_REEF_KL, RED_REEF_AB, RED_REEF_CD, RED_REEF_EF, RED_REEF_GH, RED_REEF_IJ, RED_REEF_KL);
    public static final List<Pose2d> L2_POSES = List.of(BLUE_REEF_CD, BLUE_REEF_GH, BLUE_REEF_KL, RED_REEF_CD, RED_REEF_GH, RED_REEF_KL);

    /* ---------- */
}