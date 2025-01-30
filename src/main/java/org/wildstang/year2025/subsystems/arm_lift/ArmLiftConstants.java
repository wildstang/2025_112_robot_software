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
    

    /* ------------------------------------------------------- */

}
