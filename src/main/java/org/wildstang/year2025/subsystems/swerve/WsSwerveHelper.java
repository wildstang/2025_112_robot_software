package org.wildstang.year2025.subsystems.swerve;

import edu.wpi.first.math.MathUtil;

public class WsSwerveHelper {

    private SwerveSignal swerveSignal;
    private double baseV;
    private double[] xCoords = new double[]{0.0, 0.0, 0.0, 0.0};
    private double[] yCoords = new double[]{0.0, 0.0, 0.0, 0.0};
    private double rotErr;
    private static double prevRotErr = 0;
    private double rotPID;

    /** sets the robot in the immobile "cross" defensive position
     * 
     * @return driveSignal for cross, with 0 magnitude and crossed directions
     */
    public SwerveSignal setCross() {
        return new SwerveSignal(new double[]{0.0, 0.0, 0.0, 0.0}, new double[]{135.0, 45.0, 45.0, 135.0});
    }

    /**sets the robot to drive normally as a swerve
     * 
     * @param i_tx the translation joystick x value
     * @param i_ty the translation joystick y value
     * @param i_rot the rotation joystick value
     * @param i_gyro the gyro value, field centric, in RHR radians
     * @return driveSignal for normal driving, normalized
     */
    public SwerveSignal setDrive(double i_tx, double i_ty, double i_rot, double i_gyro) {
        //angle of front left rotation vector
        baseV = Math.atan(DriveConstants.ROBOT_LENGTH / DriveConstants.ROBOT_WIDTH);

        //find the translational vectors rotated to account for the gyro
        double xTrans = i_tx * Math.cos(i_gyro) + i_ty * Math.sin(i_gyro);
        double yTrans = - i_tx * Math.sin(i_gyro) + i_ty * Math.cos(i_gyro);

        //account for slight second order skew due to rotation and translation at the same time
        // xTrans += Math.cos(Math.atan2(xTrans,yTrans)) * i_rot * DriveConstants.ROT_CORRECTION_FACTOR * Math.hypot(i_tx, i_ty);
        // yTrans += -Math.sin(Math.atan2(xTrans,yTrans)) * i_rot * DriveConstants.ROT_CORRECTION_FACTOR * Math.hypot(i_tx, i_ty);

        //cartesian vector addition of translation and rotation vectors
        //FL->FR->RL->RR
        //note rotation vector angle advances in the cos -> sin -> -cos -> -sin fashion
        xCoords = new double[]{xTrans - i_rot * Math.sin(baseV), xTrans + i_rot * Math.sin(baseV), xTrans - i_rot * Math.sin(baseV), xTrans + i_rot * Math.sin(baseV)}; 
        yCoords = new double[]{yTrans + i_rot * Math.cos(baseV), yTrans + i_rot * Math.cos(baseV), yTrans - i_rot * Math.cos(baseV), yTrans - i_rot * Math.cos(baseV)};

        //create drivesignal, with magnitudes and directions of x and y
        swerveSignal = new SwerveSignal(new double[]{Math.hypot(xCoords[0], yCoords[0]), Math.hypot(xCoords[1], yCoords[1]), Math.hypot(xCoords[2], yCoords[2]), Math.hypot(xCoords[3], yCoords[3])}, 
            new double[]{getDirection(xCoords[0], yCoords[0]), getDirection(xCoords[1], yCoords[1]), getDirection(xCoords[2], yCoords[2]), getDirection(xCoords[3], yCoords[3])});
        swerveSignal.normalize();
        return swerveSignal;
        //slew rate limit
        // if (swerveSignal.isNotZeroed()) {
        //     return swerveSignal;
        // }
        // else {
        //     return this.setDrive(i_tx, i_ty, i_rot * 0.8, i_gyro);
        // }
    }

    /**automatically creates a rotational joystick value to rotate the robot towards a specific target
     * 
     * @param i_target target direction for the robot to face, field centric, radians
     * @param i_gyro the gyro value, field centric, in radians
     * @return double that indicates what the rotational joystick value should be
     */
    public double getRotControl(double i_target, double i_gyro) {
        rotErr = MathUtil.angleModulus(i_target - i_gyro);
        rotPID = rotErr * DriveConstants.ROT_P + (rotErr - prevRotErr) * DriveConstants.ROT_D;  // otherwise spin ccw
        prevRotErr = rotErr;
        return rotPID;  // saturate rotation control to range [-1.0, 1.0]
    }

    /**x,y inputs are cartesian, angle values are in radians, returns 0 - 2pi */
    public double getDirection(double x, double y) {
        return Math.atan2(y,x);  // this math ensures we always return a value between -pi and pi
    }
    
    public double scaleDeadband(double input, double deadband){
        if (Math.abs(input) < deadband) return 0.0;
        return Math.signum(input)*((Math.abs(input) - deadband) / (1.0 - deadband));
    }
    
}