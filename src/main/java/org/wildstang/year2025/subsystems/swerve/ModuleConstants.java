package org.wildstang.year2025.subsystems.swerve;

public class ModuleConstants {
    
    /**drive motor gear ratio */
    public static final double DRIVE_RATIO = 5.50;
    /**diameter of drive wheel, in inches */
    public static final double WHEEL_RADIUS = 0.0381;//3.0; updated from 111 testing
    /**PID values for drive P */
    public static final double DRIVE_P = 0;//-0.02;//0.02
    /**PID values for drive I */
    public static final double DRIVE_I = 0.01;
    /**PID values for drive D */
    public static final double DRIVE_D = 0.1;
    /**PID values for angle P */
    public static final double ANGLE_P = 0.57;
    /**PID values for angle I */
    public static final double ANGLE_I = 0.0;
    /**PID values for angle D */
    public static final double ANGLE_D = 0.0;
    
    /**Drive motor current limit */
    public static final int DRIVE_CURRENT_LIMIT = 50;
    /**Angle motor current limit */
    public static final int ANGLE_CURRENT_LIMIT = 10;
}