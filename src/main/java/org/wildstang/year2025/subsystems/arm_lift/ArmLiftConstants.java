package org.wildstang.year2025.subsystems.arm_lift;

public final class ArmLiftConstants {
    


    /* -------------------- Feed Forward ---------------------*/

    public static final double ARM_ACCEL_GAIN = 1;
    public static final double ARM_POS_GAIN = 1;
    public static final double LIFT_POS_GAIN = 1;
    public static final double LIFT_ACCEL_GAIN = 1;
    public static final double ARM_ANGLE_GAIN = 1;

    /* ------------------------------------------------------- */


    /* -------------------- Motion Profile -------------------*/

    public static final double MAX_ARM_ACCELERATION = 0;
    public static final double MAX_ARM_VELOCITY = 0;
    public static final double MAX_LIFT_ACCELERATION = 0;
    public static final double MAX_LIFT_VELOCITY = 0;

    /* ------------------------------------------------------- */


    /* -------------------- PID -------------------------------*/

    public static final double ARM_POS_P_GAIN = 1;
    public static final double ARM_POS_I_GAIN = 1;
    public static final double ARM_VEL_P_GAIN = 1;

    public static final double LIFT_POS_P_GAIN = 1;
    public static final double LIFT_POS_I_GAIN = 1;
    public static final double LIFT_VEL_P_GAIN = 1;

    public static final double MAX_INTEGRAL = 10;
    

    /* ------------------------------------------------------- */

    /* -------------------- Game Positions -------------------------------*/

    public static final double MAX_LIFT_HEIGHT = 20;
    public static final double MIN_LIFT_HEIGHT = 0;//Inches
    public static final double MAX_ARM_ANGLE = 0;
    public static final double MIN_MAX_ANGLE = 270;

    public static final double aHeight;
    public static final double bheight;
    public static final double cHeight;
    public static final double dHeight; //L2 algae intake

    public static final double GROUND_INTAKE_RIGHT_ANGLE = 270/2;
    public static final double L2_ALGAE_REEF_INTAKE_ANGLE = 1;
  
    

    /* ------------------------------------------------------- */

}
