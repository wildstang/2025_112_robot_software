package org.wildstang.year2025.subsystems.arm_lift;

import edu.wpi.first.wpilibj.Timer;



public class MotionProfile {

    private Timer timer = new Timer();
    private double totalTime; // Total time to reach desired position
    private final double sampleTime = 0.02; //seconds
    private double[] posArray;
    private double[] velArray;
    private double[] accelArray;
    private int samples;
    
    private double maxAccelerationTime;
    private double cruiseTime;
    private int triangleSampleIndex;
    private int cruiseSampleIndex;

    public void calculate(double maxAcceleration, double maxVelocity, double curPos, double desPos){
        timer.reset();
        //Calculate the point at which acceleration is no more (not limited)
         double accelLimitEndpoint = (maxVelocity * maxVelocity) / maxAcceleration;

         double dP = desPos - curPos;

         
         if(dP <= accelLimitEndpoint){
            //Triangular Profile
            totalTime = Math.sqrt(2*dP/maxAcceleration);
            samples = (int)(totalTime/sampleTime);
            triangleSampleIndex = (int)(totalTime / (2*sampleTime));
            setArrayLengths();
            setTriangularArrays(maxAcceleration);
         }
         else{
            //Trapezoidal Profile
            maxAccelerationTime = maxVelocity/maxAcceleration; // time it takes to reach max velocity
            cruiseTime = (dP - (maxVelocity*maxVelocity)/maxAcceleration)/maxVelocity; //time of crusing at
            triangleSampleIndex = (int)(maxAccelerationTime/sampleTime);
            cruiseSampleIndex = (int)(cruiseTime/sampleTime);

            totalTime =  cruiseTime + 2*maxAccelerationTime;
            samples = (int)(totalTime/sampleTime);
            setArrayLengths();
            setTrapezoidArrays(maxAcceleration);
            
         }

    }

    private void setTrapezoidArrays(double maxAcceleration){
      double accelSum = 0;
      double velSum = 0;
      for(int i = 0; i < accelArray.length; i++){
         if(i < triangleSampleIndex){
            accelArray[i]=maxAcceleration;
         }else if(i< triangleSampleIndex + cruiseSampleIndex){
            accelArray[i] = 0;
         }else{
            accelArray[i] = -maxAcceleration;
         }
         accelSum += accelArray[i];
         velArray[i] = accelSum * sampleTime;
         velSum += velArray[i];
         posArray[i] = velSum * sampleTime;


      }
    }

    private void setTriangularArrays(double maxAcceleration){
      double accelSum = 0;
      double velSum = 0;
      for(int i = 0; i < accelArray.length; i++){
         if(i < triangleSampleIndex){
            accelArray[i]= maxAcceleration;
         }else{
            accelArray[i] = -maxAcceleration;
         }
         accelSum += accelArray[i];
         velArray[i] = accelSum * sampleTime;
         velSum += velArray[i];
         posArray[i] = velSum * sampleTime;


      }
    }

    
    public double[] getSamples(){
      if(!timer.isRunning()){
         timer.start();
      }
      int currIndex = (int)(timer.get()/sampleTime);
      double[] sampleArray = new double[3];
      sampleArray[0] = posArray[currIndex];
      sampleArray[1] = velArray[currIndex];
      sampleArray[2] = accelArray[currIndex];
      return sampleArray;
    }
    private void setArrayLengths(){
      accelArray = new double[samples];
      velArray = new double[samples];
      posArray = new double[samples];
    }
    

    
}
