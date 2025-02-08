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
    public static final double MIN_ARM_ANGLE = 0;
    public static final double MAX_ARM_ANGLE = 3*Math.PI/2;

    public static final double L2_ANGLE = Math.PI/2;
    public static final double L2_LIFT_HEIGHT = 6;

    public static final double L3_ANGLE = Math.PI/2;
    public static final double L3_LIFT_HEIGHT = 7;
    
    public static final double SHOOT_NET_ANGLE = Math.PI;
    public static final double SHOOT_NET_LIFT_HEIGHT = 8;

    public static final double STORAGE_ANGLE = Math.PI/3;
    public static final double STORAGE_LIFT_HEIGHT = 3;

    //Inaking
    public static final double GROUND_INTAKE_RIGHT_ANGLE = 270/2;
    public static final double INTAKE_LIFT_HEIGHT = 5;


    /* ------------------------------------------------------- */
    
    /* ---------------ARM AND LIFT BOUND CHECKING -------------*/

    public static final double LOW_LIFT_HEIGHT = 5;
    public static final double HIGH_LIFT_HEIGHT = 15;

    //threshold angles where the lift can move down without hitting algae
    public static final double MAX_LIFT_DOWN_ANGLE = 7*Math.PI/6;
    public static final double MIN_LIFT_DOWN_ANGLE = 3*Math.PI/4;

    //lowest angle possible without hitting the lift (when the lift is at a high position)
    public final static double MIN_HIGH_ARM_ANGLE = Math.PI/6;

    //bounds  when the lift is at a low position so the arm doesn't hit bumpers or lift
    public static final double MAX_LOW_ARM_ANGLE = 3*Math.PI/4;
    public static final double MIN_LOW_ARM_ANGLE = Math.PI/4; 
    /*------------------------------------------------------- */
    /*-------------------------Other--------------------- */

    public static final double MAX_POTENTIOMETER_VOLTAGE = 7;
    public static final double MIN_POTENTIOMETER_VOLTAGE = 2;


}
