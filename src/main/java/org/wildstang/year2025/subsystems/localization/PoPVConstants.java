package org.wildstang.year2025.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class PoPVConstants {

   public static Transform3d leftCameraToRobot = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,90));
   
   public static final Pose2d[] blueReefTagPose = {};
   public static final Pose2d[] redReefTagPose = {};

   private static final Transform2d BARGE_OFFSET = new Transform2d(0.0,0.0, new Rotation2d());

   public static final Pose2d BARGE_GOAL = PoPVConstants.get(14).transformBy(BARGE_OFFSET);

   public static Pose2d get(int tagID){

    Pose2d tagPose = new Pose2d();
     switch(tagID){
        case 14:
            tagPose = blueReefTagPose[18];
     }

     return tagPose;
   }

    
}