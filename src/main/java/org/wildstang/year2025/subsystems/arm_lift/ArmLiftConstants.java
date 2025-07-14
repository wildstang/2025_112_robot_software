package org.wildstang.year2025.subsystems.arm_lift;

public final class ArmLiftConstants {
    /* -------------------- Motion Profile -------------------*/
    public static final double MAX_ARM_ACCELERATION = 20;//20.18;
    public static final double MAX_ARM_VELOCITY = 3.0; //1.94;
    public static final double MAX_LIFT_ACCELERATION = 5.0; // 1.51;
    public static final double MAX_LIFT_VELOCITY = 2.0; //1.42;
    /* ------------------------------------------------------- */

    /* -------------------- PID -------------------------------*/
    public static final double ARM_POS_P_GAIN = 1.0;
    public static final double ARM_POS_I_GAIN = 0.2;
    public static final double ARM_VEL_P_GAIN = 0.15;
    public static final double ARM_TOL = 0.12;

    public static final double LIFT_POS_P_GAIN = 16.0;
    public static final double LIFT_POS_I_GAIN = 8.0;
    public static final double LIFT_VEL_P_GAIN = 0.3;
    public static final double LIFT_TOL = 0.02;
    /* ------------------------------------------------------- */

    /* -------------------- Game Positions -------------------------------*/
    public static final double MAX_LIFT_HEIGHT = 0.60;  // meters
    public static final double MIN_LIFT_HEIGHT = 0;  // meters
    public static final double LIFT_START_HEIGHT = 0.26;
    public static final double MIN_ARM_ANGLE = 0;
    public static final double MAX_ARM_ANGLE = 5.5;

    public static final double L2_INTAKE_ANGLE = 1.55;
    public static final double L2_INTAKE_LIFT_HEIGHT = 0.42;

    public static final double L3_INTAKE_ANGLE = 1.65;
    public static final double L3_INTAKE_LIFT_HEIGHT = MAX_LIFT_HEIGHT;

    public static final double PROCESSOR_ANGLE = 1.2;
    public static final double PROCESSOR_HEIGHT = 0.06;
    
    public static final double SHOOT_NET_ANGLE = 2.80;
    public static final double SHOOT_NET_LIFT_HEIGHT = MAX_LIFT_HEIGHT;

    public static final double STORAGE_ANGLE = Math.PI;
    public static final double STORAGE_LIFT_HEIGHT = 0.0;

    public static final double LOLIPOP_ANGLE = 1.2;
    public static final double LOLIPOP_HEIGHT = 0.2;

    public static final double GROUND_INTAKE_RIGHT_ANGLE = 1.0;  // radians
    public static final double GROUND_INTAKE_LIFT_HEIGHT = 0.0;  // meters

    public static final double CLIMB_ANGLE = 0.0;
    public static final double CLIMB_HEIGHT = LIFT_START_HEIGHT;
/* ------------------------------------------------------------------- */

/* ---------------ARM AND LIFT BOUND CHECKING -------------*/
    public static final double LOW_LIFT_HEIGHT = 0.2;
    public static final double HIGH_LIFT_HEIGHT = 0.22;

    //threshold angles where the lift can move down without hitting algae
    public static final double MAX_LIFT_DOWN_ANGLE = 3.62;
    public static final double MIN_LIFT_DOWN_ANGLE = 2.62;

    //angle bounds without hitting the lift (when the lift is at a high position)
    public final static double MIN_HIGH_ARM_ANGLE = 0.72;
    public static final double ALGAE_TOP_CLEARANCE = 0.06;

    //bounds  when the lift is at a low position so the arm doesn't hit bumpers or lift
    public static final double MAX_LOW_ARM_ANGLE = 5.0;
    public static final double MIN_LOW_ARM_ANGLE = 1.0;

    public static final double POWER_CHAIN_LIFT_HEIGHT = 0.45;
    public static final double POWER_CHAIN_LIFT_HEIGHT_MIN = 0.42;
    public static final double POWER_CHAIN_LIFT_HEIGHT_MAX = 0.48;

    // public static final double ARM_POWER_CHAIN_LOW_LIMIT = 0;
    public static final double ARM_POWER_CHAIN_HIGH_ANGLE = 5.4;
    public static final double ARM_POWER_CHAIN_LOW_ANGLE = 0.15;

    public static final double MIN_CLAW_POWER_CHAIN_ANGLE = 0.37; //minimum claw angle that the claw can be within without hitting energy chain
    public static final double MAX_CLAW_POWER_CHAIN_ANGLE = 5.0 * Math.PI / 6.0; //max claw angle that the claw can be within without hitting energy chain

    public static final double LOWER_BOUND_POWER_CHAIN_ANGLE = 7.0 * Math.PI / 8.0; //lower bound that the claw angle must be above to be in danger of hitting power chain
    public static final double UPPER_BOUND_POWER_CHAIN_ANGLE = Math.PI / 8.0; //upper bound that the claw angle must be below to be in danger of hitting power chain
    /*------------------------------------------------------- */

    /*-------------------------Other--------------------- */
    public static final double MAX_POTENTIOMETER_VOLTAGE = 4.5;
    public static final double MIN_POTENTIOMETER_VOLTAGE = 0.3;

    public static final double LIFT_PULLEY_RADIUS = .0254; // m
    public static final double LIFT_GEAR_RATIO = 62.0 / 14.0 * 58.0 / 40.0;
    public static final double LIFT_FREE_SPEED = 5676.0 / LIFT_GEAR_RATIO * 2.0 * Math.PI * LIFT_PULLEY_RADIUS / 60.0; // meters per second
    public static final double LIFT_STALL_FORCE = 2.6 * LIFT_GEAR_RATIO * 2 / LIFT_PULLEY_RADIUS; // Newton
    public static final double LIFT_ARM_MASS = 4.0; // in kilograms

    public static final double ARM_GEAR_RATO = 68.0 / 12.0 * 68.0 / 22.0 * 68.0 / 22.0 * 68.0 / 16.0;
    public static final double ARM_FREE_SPEED = 5676.0 / ARM_GEAR_RATO * 2.0 * Math.PI / 60.0; // Radians per second
    public static final double ARM_STALL = 2.6 * ARM_GEAR_RATO; // Newton Meters
    public static final double ARM_STATIC_TORQUE = 0.0; //in kilograms * m
    public static final double ARM_MOI = 0.684;  // kg m^2
    public static final double ARM_ENC_RATIO = 68.0 / 8.0;
    
    public static final double GRAVITY = 9.80665; // m/s^2 gravity
}
