package org.wildstang.year2025.subsystems.swerve;

public class SwerveSignal {

    private double[] speed;
    private double[] angle;
    private double maxSpeed;

    /**contains motor speeds, robot relative angles in RHR radians 
     * @param i_speed double[] for the speed of each module, in [0,1] signal
     * @param i_angle double[] for the angle of the module, in radians
    */
    public SwerveSignal(double[] i_speed, double[] i_angle) {
        this.speed = i_speed;
        this.angle = i_angle;
    }

    /**ensures all speed values are below 1, and scales down if needed */
    public void normalize() {
        maxSpeed = getMaxSpeed();
        if (maxSpeed > 1.0){
            for (int i = 0; i < speed.length; i++){
                speed[i] /= maxSpeed;
            }
        }
    }

    public boolean isNotZeroed() {
        maxSpeed = getMaxSpeed();
        for (int i = 0; i < speed.length; i++){
            if (Math.abs(speed[i]) + 0.01 <= maxSpeed * 0.1){
                return false;
            }
        }
        return true;
    }

    /**speed is normalized value [0, 1] 
     * @param i_module the module to get the speed from (1 through 4)
     * @return double for the speed to set that module to
    */
    public double getSpeed(int i_module) {
        return speed[i_module];
    }

    /**returns speeds from the swerve signal
     * @return double array of 4 speeds, percent output
     */
    public double[] getSpeeds() {
        return speed;
    }

    /**returns max speed from the swerve signal
     * @return maximum speed across all four modules
     */
    public double getMaxSpeed() {
        double maxSpeed = 0.0;
        for (int i = 0; i < speed.length; i++){
            if (Math.abs(speed[i]) > maxSpeed){
                maxSpeed = Math.abs(speed[i]);
            }
        }
        return maxSpeed;
    }

    /**angle is robot centric, in radians
     * @param i_module the module to get the angle from (1 through 4)
     * @return double for the angle to set that module to
    */
    public double getAngle(int i_module) {
        return angle[i_module];
    }

    /**returns angles from the swerve signal
     * @return double array of 4 angles, radians
     */
    public double[] getAngles() {
        return angle;
    }
    
}