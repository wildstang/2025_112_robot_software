package org.wildstang.year2025.subsystems.arm_lift;

import edu.wpi.first.wpilibj.Timer;

public class PIDController {
    public static double P = 0, I = 0, D = 0;

    

    public static double dt;
    public static double tPrevious;
    private double t = Timer.getFPGATimestamp();
    private double de;

    public PIDController(double p, double i, double d){
        P = p;
        I = i;
        D = d;
        
    }

    public double PController(double setPoint, double currentPos){
        double error = setPoint - currentPos;
        double controlInput = error*P;
        return controlInput;
    }
    public double PDController(double setPoint, double currentPos){
        tPrevious = t;
        t = Timer.getFPGATimestamp();
        double error = setPoint - currentPos;
        double dt = t-tPrevious;
        double de = 0;
        double controlInput = (P * error) + (D*(de/dt));
        return controlInput;
    }
}
