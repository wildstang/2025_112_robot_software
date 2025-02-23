package org.wildstang.year2025.subsystems.arm_lift;

import edu.wpi.first.wpilibj.Timer;

public class PIDController {
    private double kpP, kpI; // Position Gains
    private double kvP; // Velecity Gains

    private double vP;
    private double pP, pI;
    private double tPrevious;
    private double maxIntegral;


    public PIDController(double kpP, double kpI, double kvP, double maxIntegral){
       this.kpP = kpP;
       this.kpI = kpI;
       this.kvP = kvP;
       this.tPrevious = Timer.getFPGATimestamp();
       this.maxIntegral = maxIntegral; 
    }

    private double getDeltaT(){
        double t = Timer.getFPGATimestamp();
        double dt = t - tPrevious;
        tPrevious = t;
        return dt;
    }

    public double positionPVal(double setPoint, double currentPos){
        double error = setPoint - currentPos;
        pP = error*kpP;
        return pP;
    }

    public double positionIVal(double setPoint, double currentPos){
        double dt = getDeltaT();
        double error = setPoint - currentPos;
        pI += kpI*error * (dt);

        if(pI > maxIntegral){
            pI = maxIntegral;
        }else if(pI < -maxIntegral){
            pI = -maxIntegral;
        }

        return pI;
    }

    public double positionPIController(double setPoint, double currentPos){
        double pTerm = positionPVal(setPoint, currentPos);
        double iTerm = positionIVal(setPoint, currentPos);
        return pTerm + iTerm;
    }

    public double velocityPController(double setPoint, double currentVel){
        double pTerm = velocityPVal(setPoint, currentVel);
        return pTerm;
    }


    public double velocityPVal(double setPoint, double currentVel){
        double error = setPoint - currentVel;
        vP = error*kvP;
        return vP;
    }

    public void resetIVal(){
        pI = 0;
    }




































    // public double positionDVal(double setPoint, double currentPos){
    //     double dt = getDeltaT();
    //     double error = setPoint - currentPos;
    //     double de = error-positionErrorPrevious;
    //     double controlInput = (kpD*(de/dt));
    //     positionErrorPrevious = error;
    //     return controlInput;

    // }

   

    // public double velocityIVal(double setPoint, double currentPos){
    //     double dt = getDeltaT();
    //     double error = setPoint - currentPos;
    //     vI += kvI*error * (dt);
    //     return vI;
    // }

    // public double velocityDVal(double setPoint, double currentPos){
    //     double dt = getDeltaT();
    //     double error = setPoint - currentPos;
    //     double de = error-errorPrevious;
    //     double controlInput = (kvD*(de/dt));
    //     errorPrevious = error;
    //     return controlInput;

    // }
    
    
}
