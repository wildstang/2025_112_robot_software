package org.wildstang.year2025.subsystems.arm_lift;

import edu.wpi.first.wpilibj.Timer;



public class MotionProfile {

    private Timer timer = new Timer(); // Timer function (Sort of like a stopwatch/clock)
    private double totalTime; // Total time to reach desired position
    private final double sampleTime = 0.02; //seconds
    private double[] posArray; // The array that stores all of our calculates goal positions of the motion profile
    private double[] velArray; // Same thing as posArray just for velocity
    private double[] accelArray; // Same thing as posArray and velArray but for acceleration
    private int samples; // This is the size of the arrays. Samples is just a number
    

    private double maxAccelerationTime; // The time it takes to reach max velocity (Our highest Velocity)
    private double cruiseTime; // Stores the time when we do not accelerate or decelerate (We coast at max velocity for curiseTime)
    private int triangleSampleIndex; // Stores the index that we are on for triangle phase of motion profile
    private int cruiseSampleIndex; // Stores the index that we are on for the cruise phase of motion profile

    public static boolean profileDone;


    private final double maxAccel; // The maximum acceleration (Calculated using math, physics, and motor data sheets)
    private final double maxVel; // Maximum velocity (A value that we pass in)
    private final double accelLimitEndpoint;

   
   public MotionProfile(double maxAcceleration, double maxVelocity){
      this.maxAccel = maxAcceleration;
      this.maxVel = maxVelocity;
      this.accelLimitEndpoint = (Math.pow(maxVel,2)) / maxAccel;
      profileDone = false;

   }

    public void calculate(double curPos, double desPos){
        timer.reset();
        profileDone = false;

        //Calculate the point at which acceleration is no more (not limited)
         double dP = desPos - curPos;

         
         if(dP <= accelLimitEndpoint){
            //Triangular Profile
            totalTime = Math.sqrt(2*dP/maxAccel);
            samples = (int)(totalTime/sampleTime);
            triangleSampleIndex = (int)(totalTime / (2*sampleTime));
            setArrayLengths();
            setTriangularArrays(dP);
         }
         else{
            //Trapezoidal Profile
            maxAccelerationTime = maxVel/maxAccel; // time it takes to reach max velocity
            cruiseTime = (dP - accelLimitEndpoint)/maxVel; //time of crusing at
            triangleSampleIndex = (int)(maxAccelerationTime/sampleTime);
            cruiseSampleIndex = (int)(cruiseTime/sampleTime);

            totalTime =  cruiseTime + 2*maxAccelerationTime;
            samples = (int)(totalTime/sampleTime);
            setArrayLengths();
            setTrapezoidArrays(dP);
            
         }

    }

    private void setTrapezoidArrays(double dP){
      double accelSum = 0;
      double velSum = 0;
      for(int i = 0; i < accelArray.length; i++){
         if(i < triangleSampleIndex){
            accelArray[i]=maxAccel * Math.signum(dP);
         }else if(i< triangleSampleIndex + cruiseSampleIndex){
            accelArray[i] = 0;
         }else{
            accelArray[i] = -maxAccel * Math.signum(dP);
         }
         velArray[i] = accelSum * sampleTime;
         accelSum += accelArray[i];
         posArray[i] = velSum * sampleTime;
         velSum += velArray[i];



      }
    }

    private void setTriangularArrays(double dP){
      double accelSum = 0;
      double velSum = 0;
      for(int i = 0; i < accelArray.length; i++){
         if(i < triangleSampleIndex){
            accelArray[i]= maxAccel * Math.signum(dP);
         }else{
            accelArray[i] = -maxAccel * Math.signum(dP);
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

      if (currIndex >= samples) {
         currIndex = samples - 1;
         profileDone = true;
     }

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
