package org.wildstang.year2025.subsystems.arm_lift;

public final class ArmLiftConstants {
    


    


    /* -------------------- Motion Profile -------------------*/

    public static final double MAX_ARM_ACCELERATION = 0;
    public static final double MAX_ARM_VELOCITY = 0;
    public static final double MAX_LIFT_ACCELERATION = 0;
    public static final double MAX_LIFT_VELOCITY = 0;
    public static final double ARM_SMALL_DELTA_P = 1; //small number that deltaP can be under that defaults motion profile to have 0 acceleration and velocity
    public static final double LIFT_SMALL_DELTA_P = 6; //small number that deltaP can be under that defaults motion profile to have 0 acceleration and velocity
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

    public static final double MAX_LIFT_HEIGHT = 29; //inches
    public static final double MIN_LIFT_HEIGHT = 0;//Inches
    public static final double MIN_ARM_ANGLE = 0;
    public static final double MAX_ARM_ANGLE = 2*Math.PI;

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


    public static final double MIN_CLAW_POWER_CHAIN_ANGLE = Math.PI/6; //minimum claw angle that the claw can be within without hitting energy chain
    public static final double MAX_CLAW_POWER_CHAIN_ANGLE = 5*Math.PI/6; //max claw angle that the claw can be within without hitting energy chain

    public static final double LOWER_BOUND_POWER_CHAIN_ANGLE = 7*Math.PI/8; //lower bound that the claw angle must be above to be in danger of hitting power chain
    public static final double UPPER_BOUND_POWER_CHAIN_ANGLE = Math.Pi/8; //upper bound that the claw angle must be below to be in danger of hitting power chain

    /*------------------------------------------------------- */
    /*-------------------------Other--------------------- */

    public static final double MAX_POTENTIOMETER_VOLTAGE = 7;
    public static final double MIN_POTENTIOMETER_VOLTAGE = 2;
    public static final double PULLEY_RADIUS = 24.4/1000; // m
    public static final double GEAR_RATIO_LIFT = 62/14*58/40; 
    public static final double FREE_SPEED = 5676 / GEAR_RATIO_LIFT * 2 * Math.PI / 60; // Radians per second
    public static final double GEAT_RATO_ARM = 5;
    public static final double STALL = 2.6 * GEAR_RATIO_LIFT * 2; // Newton Meters
    public static final double LIFT_ARM_MASS = 12; // in kilograms
    public static final double LIFT_MASS = 10; //in kilograms

    public static final double GRAVITY = 9.80665; // m/s^2 gravity
    public static final double ROTATIONAL_MASS = 0000;
    public static final double MOMEMENT_OF_INERTIA = 0000;
   


}
