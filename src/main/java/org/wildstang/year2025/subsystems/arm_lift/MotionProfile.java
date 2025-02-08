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

    public boolean profileDone;


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

         if (dP == 0) { // No movement needed
            totalTime = 0;
            samples = 0;
            posArray = new double[]{0};
            velArray = new double[]{0};
            accelArray = new double[]{0};
            profileDone = true;
            return;
        }

         if(Math.abs(dP) <= accelLimitEndpoint){
            //Triangular Profile
           triangularProfile(dP);
         }
         else{
          trapezoidProfle(dP); 
         }

    }

    private void trapezoidProfle(double dP){
       //Trapezoidal Profile
       maxAccelerationTime = maxVel/maxAccel; // time it takes to reach max velocity
       cruiseTime = (dP - accelLimitEndpoint)/maxVel; //time of crusing at maxVelocity
       triangleSampleIndex = (int)(maxAccelerationTime/sampleTime); //index at which we reach max velocity
       cruiseSampleIndex = (int)(cruiseTime/sampleTime); //index at which we reach cruise time

       totalTime =  cruiseTime + 2*maxAccelerationTime; //time it takes to finish profile
       samples = (int)(totalTime/sampleTime);
       setArrayLengths();
       setTrapezoidArrays(dP);
       
    }


    //deals with the profile if it is a triangle
    private void triangularProfile(double dP){ 
      totalTime = Math.sqrt(2*dP/maxAccel); //time it takes to finish profile
      samples = (int)(totalTime/sampleTime);   //amount of samples needed for every 20 ms
      triangleSampleIndex = (int)(totalTime / (2*sampleTime));   //stores index of half way through the profile
      setArrayLengths();
      setTriangularArrays(dP);
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
      if (samples == 0) { // Prevents out-of-bounds access
         profileDone = true;
         return new double[]{0, 0, 0}; // No movement
     }

      if(!timer.isRunning()){
         timer.start();
      }
      int currIndex = (int)(timer.get()/sampleTime);

      if (currIndex >= samples) {
         currIndex = samples - 1;
         profileDone = true;
         timer.stop();
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
