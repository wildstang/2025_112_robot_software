package org.wildstang.year2025.subsystems.arm_lift;

import edu.wpi.first.wpilibj.Timer;

public class PIDController {
    public double kP = 0, kI = 0, kD = 0;
    public  double P = 0,I = 0,D = 0;
    public  double tPrevious;
    public double errorPrevious;


    public PIDController(double kP, double kI, double kD, double P, double I, double D){
       this.kP = kP;
       this.kI = kI;
       this.kD = kD;
       this.P = P;
       this.I = I;
       this.D = D;
       this.tPrevious = Timer.getFPGATimestamp();
       this.errorPrevious = 0;
    }

    private double getDeltaT(){
        double t = Timer.getFPGATimestamp();
        double dt = t - tPrevious;
        tPrevious = t;
        return dt;
    }

    public double PVal(double setPoint, double currentPos){
        double error = setPoint - currentPos;
        double controlInput = error*kP;
        return controlInput;
    }

    public double IVal(double setPoint, double currentPos){
        double dt = getDeltaT();
        double error = setPoint - currentPos;
        I += kI*error * (dt);
        return I;
    }

    public double DVal(double setPoint, double currentPos){
        double dt = getDeltaT();
        double error = setPoint - currentPos;
        double de = error-errorPrevious;
        double controlInput = (kD*(de/dt));
        errorPrevious = error;
        return controlInput;

    }

    public double PIDController(double setPoint, double currentPos){
        double pTerm = PVal(setPoint, currentPos);
        double iTerm = IVal(setPoint, currentPos);
        double dTerm = IVal(setPoint, currentPos);
        return pTerm + iTerm + dTerm;
    }
    public double PDController(double setPoint, double currentPos){
        double pTerm = PVal(setPoint, currentPos);
        double dTerm = DVal(setPoint, currentPos);
        return pTerm + dTerm;
    }
    
    public double PIController(double setPoint, double currentPos){
        double pTerm = PVal(setPoint, currentPos);
        double iTerm = IVal(setPoint, currentPos);
        return pTerm + iTerm;
    }
}
