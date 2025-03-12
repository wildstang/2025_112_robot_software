package org.wildstang.year2025.subsystems.localization;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class LocalizationConstants {
    public static final String kFrontCam = "FrontCam";
    public static final Transform3d kBotToFrontCam = new Transform3d(new Translation3d(0.18, 0.07, 0.17), new Rotation3d(0, 0.459, 0));
    public static final Transform3d kBotToRearCam = new Transform3d(new Translation3d(-0.18, 0.07, 0.17), new Rotation3d(0, 0.459, Math.PI));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMaxStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

    /* Goal Poses */
    public static final double MID_FIELD_X = 8.77;
    
    public static final Pose2d BLUE_PROCESSOR = new Pose2d(6.25, 0.55, new Rotation2d(Math.PI / 2.0));
    public static final Pose2d RED_PROCESSOR = new Pose2d(11.30, 7.50, new Rotation2d(3.0 * Math.PI / 2.0));
    
    public static final double BLUE_NET_X = 7.60;
    public static final double RED_NET_X = 9.95;

    public static final Pose2d BLUE_REEF_AB = new Pose2d();
    public static final Pose2d BLUE_REEF_CD = new Pose2d();
    public static final Pose2d BLUE_REEF_EF = new Pose2d();
    public static final Pose2d BLUE_REEF_GH = new Pose2d();
    public static final Pose2d BLUE_REEF_IJ = new Pose2d();
    public static final Pose2d BLUE_REEF_KL = new Pose2d();
    public static final List<Pose2d> BLUE_REEF = List.of(BLUE_REEF_AB, BLUE_REEF_CD, BLUE_REEF_EF, BLUE_REEF_GH, BLUE_REEF_IJ, BLUE_REEF_KL);

    public static final Pose2d RED_REEF_AB = new Pose2d();
    public static final Pose2d RED_REEF_CD = new Pose2d();
    public static final Pose2d RED_REEF_EF = new Pose2d();
    public static final Pose2d RED_REEF_GH = new Pose2d();
    public static final Pose2d RED_REEF_IJ = new Pose2d();
    public static final Pose2d RED_REEF_KL = new Pose2d();
    public static final List<Pose2d> RED_REEF = List.of(RED_REEF_AB, RED_REEF_CD, RED_REEF_EF, RED_REEF_GH, RED_REEF_IJ, RED_REEF_KL);

    /* ---------- */
}