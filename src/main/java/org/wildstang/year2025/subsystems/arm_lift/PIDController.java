package org.wildstang.year2025.subsystems.arm_lift;

import edu.wpi.first.wpilibj.Timer;

public class PIDController {
    public static double P = 0, I = 0, D = 0; 
    public static double kP = 0, kI = 0, kD = 0;

    

    public static double dt;
    public static double tPrevious;
    public static double errorPrevious = 0;
    private double t = Timer.getFPGATimestamp();
    private double de;


    public PIDController(double p, double i, double d){
        P = p;
        I = i;
        D = d;
        
    }

    public void PVal(double setPoint, double currentPos){
        double error = setPoint - currentPos;
        double controlInput = error*kP;
        P = controlInput;
    }
    public double IVal(double setPoint, double currentPos){
        tPrevious = t;
        t = Timer.getFPGATimestamp();
        double error = setPoint - currentPos;
        I += kI*error * (t-tPrevious);
        tPrevious = t;

        return I;
    }
    public void DVal(double setPoint, double currentPos){
        tPrevious = t;
        t = Timer.getFPGATimestamp();
        double error = setPoint - currentPos;
        dt = t-tPrevious;
        de = error-errorPrevious;
        double controlInput =  (kD*(de/dt));
        errorPrevious = error;
        D = controlInput;
        
    }
    public double PDController(double setPoint, double currentPos){
        tPrevious = t;
        t = Timer.getFPGATimestamp();
        double error = setPoint - currentPos;
        double dt = t-tPrevious;
        double de = error-errorPrevious;

        double controlInput = 
        errorPrevious = error;
        return controlInput;
    }
    
    public double PIController(double SetPoint, double currentPos){
        tPrevious = t;
        t = Timer.getFPGATimestamp();
        IVal()
        return controlInput;
    }
}
