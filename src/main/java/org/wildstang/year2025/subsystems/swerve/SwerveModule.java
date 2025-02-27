package org.wildstang.year2025.subsystems.swerve;

import com.revrobotics.spark.SparkAbsoluteEncoder;

import org.wildstang.hardware.roborio.outputs.WsSpark;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

    private double target;
    private double drivePower;
    private double chassisOffset;

    private WsSpark driveMotor;
    private WsSpark angleMotor;
    private SparkAbsoluteEncoder absEncoder;

    /** Class: SwerveModule
     *  controls a single swerve pod, featuring two motors and one offboard sensor
     * @param driveMotor canSparkMax of the drive motor
     * @param angleMotor canSparkMax of the angle motor
     * @param canCoder canCoder offboard encoder
     * @param offset double value of cancoder when module is facing forward
     */
    public SwerveModule(WsSpark driveMotor, WsSpark angleMotor, double offset) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.absEncoder = angleMotor.getController().getAbsoluteEncoder();
        this.driveMotor.setBrake();
        this.angleMotor.setBrake();

        chassisOffset = offset;
        
        //set up angle and drive with pid and kpid respectively
        driveMotor.initClosedLoop(ModuleConstants.DRIVE_P, ModuleConstants.DRIVE_I, ModuleConstants.DRIVE_D, 0);
        angleMotor.initClosedLoop(ModuleConstants.ANGLE_P, ModuleConstants.ANGLE_I, ModuleConstants.ANGLE_D, 0, true, true);
        angleMotor.setAbsEncConversion(2.0 * Math.PI, 2.0 * Math.PI / 60.0, true);

        driveMotor.setCurrentLimit(ModuleConstants.DRIVE_CURRENT_LIMIT, ModuleConstants.DRIVE_CURRENT_LIMIT, 0);
        angleMotor.setCurrentLimit(ModuleConstants.ANGLE_CURRENT_LIMIT, ModuleConstants.ANGLE_CURRENT_LIMIT, 0);

    }

    /** return double for cancoder position 
     * @return double for cancoder value (radians)
    */
    public double getAngle() {
        return (((absEncoder.getPosition() + chassisOffset) % (2.0 * Math.PI)) + 2.0 * Math.PI) % (2.0 * Math.PI);
    }

    /** displays module information, needs the module name from super 
     * @param name the name of this module
    */
    public void displayNumbers(String name) {
        SmartDashboard.putNumber(name + " true angle", getAngle());
        SmartDashboard.putNumber(name + " true target", target);
        SmartDashboard.putNumber(name + " raw angle", absEncoder.getPosition());
        SmartDashboard.putNumber(name + " raw target", (2.0 * Math.PI + (target - chassisOffset)) % (2.0 * Math.PI));
        SmartDashboard.putNumber(name + " NEO drive power", drivePower);
        SmartDashboard.putNumber(name + " NEO drive position", driveMotor.getPosition());
        SmartDashboard.putBoolean(name + " Drive Direction", getDirection(target));
    }

    /** resets drive encoder */
    public void resetDriveEncoders() {
        driveMotor.resetEncoder();
    }

    /**sets drive to brake mode if true, coast if false 
     * @param isBrake true for brake, false for coast
    */
    public void setDriveBrake(boolean isBrake) {
        if(isBrake) {
            driveMotor.setBrake();
        }
        else {
            driveMotor.setCoast();
        }
    }

    /** runs module at double power [0,1] and robot centric radian angle 
     * @param power power [0, 1] to run the module at
     * @param angle angle to run the robot at, radians
    */
    public void run(double power, double angle) {
        this.drivePower = power;
        this.target = angle;
        if (Math.abs(power) < 0.01) {
            runAtPower(power);
        }
        else if (getDirection(angle)) {
            runAtPower(power);
            runAtAngle(angle);
        }
        else {
            runAtPower(-power);
            runAtAngle(angle + Math.PI);
        }
    }

    // public void runCross(double position, double angle) {
    //     this.drivePower = position;
    //     this.target = angle;
    //     driveMotor.setSpeed(drivePower);
    //     if (getDirection(angle)) {
    //         runAtAngle(angle);
    //     }
    //     else {
    //         runAtAngle((angle + Math.PI) % (2.0 * Math.PI));
    //     }
    // }

    /**runs at specified robot centric angle 
     * @param angle angle to run the module at
    */
    private void runAtAngle(double angle) {
        angleMotor.setPosition((2.0 * Math.PI + (angle - chassisOffset)) % (2.0 * Math.PI));
    }

    /**runs module drive at specified power [-1, 1] 
     * @param power the power to run the module at, [-1, 1]
    */
    public void runAtPower(double power) {
        driveMotor.setSpeed(power);
    }

    /** returns drive encoder distance in meters 
     * @return double drive encoder distance in meters
    */
    public double getPosition() {
        return driveMotor.getPosition() * ModuleConstants.WHEEL_RADIUS * 2.0 * Math.PI / ModuleConstants.DRIVE_RATIO;
    }

    /**returns raw drive encoder value, rotations
     * @return drive encoder value, rotations
     */
    public double getRawEncoderValue() {
        return driveMotor.getPosition();
    }

    /**determines if it is faster to travel towards angle at positive power (true), or away from angle with negative power (false) 
     * @param angle the angle you are moving towards
     * @return boolean whether you should move towards that angle or the opposite
    */
    public boolean getDirection(double angle) {
        double goalAngle = ((angle - getAngle()) % (2.0 * Math.PI) + 2.0 * Math.PI) % (2.0 * Math.PI);
        return goalAngle < (Math.PI / 2.0) || goalAngle > (3.0 * Math.PI / 2.0);
    }

    public WsSpark getDriveMotor() {
        return driveMotor;
    }

    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(driveMotor.getVelocity() * 2.0 * Math.PI * (ModuleConstants.WHEEL_RADIUS) / 60.0, Rotation2d.fromRadians(getAngle()));
    }
    public SwerveModulePosition odoPosition(){
        return new SwerveModulePosition(getPosition(), Rotation2d.fromRadians(getAngle()));
    }
}