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
    private int triangleSampleIndex; // Stores the index of the halfway point of array
    private int cruiseSampleIndex; // Stores the index that we are on for the cruise phase of motion profile

    public boolean profileDone; //if desired position is reached
    private boolean samplesIsZero;
    private double desPos; //desired position
    private double curPos; //current position

    private final double maxAccel; // The maximum acceleration (Calculated using math, physics, and motor data sheets)
    private final double maxVel; // Maximum velocity (A value that we pass in)
    private final double minDistanceForMaxVel; //minimum distance it takes to reach max velocity


   
   public MotionProfile(double maxAcceleration, double maxVelocity){
      this.maxAccel = maxAcceleration;
      this.maxVel = maxVelocity;
      this.minDistanceForMaxVel = (Math.pow(maxVel,2)) / maxAccel; //
      profileDone = false;
      samplesIsZero = false;
      desPos = 0.0;

   }

    public void calculate(double curPos, double desPos){

        timer.reset();
        profileDone = false;
        samplesIsZero = false;
        this.desPos = desPos;
        this.curPos = curPos;
        
         double dP = desPos - curPos;

         if (dP == 0) { // No movement needed
            totalTime = 0;
            samples = 0;
            posArray = new double[]{desPos};
            velArray = new double[]{0};
            accelArray = new double[]{0};
            profileDone = true;
            return;
        }

        //if change in position is less than minimum distance required to reach max velocity, use tringle profile
         if(Math.abs(dP) <= minDistanceForMaxVel){
            //Triangular Profile
           triangularProfile(dP, desPos);
         }
         else{
            //trapezoid profile
          trapezoidProfle(dP, desPos); 
         }

    }

    private void trapezoidProfle(double dP, double desPos){
       //Trapezoidal Profile
       maxAccelerationTime = maxVel/maxAccel; // time it takes to reach max velocity
       cruiseTime = (dP - minDistanceForMaxVel)/maxVel; //time of crusing at maxVelocity
       triangleSampleIndex = (int)(maxAccelerationTime/sampleTime); //index at which we reach max velocity
       cruiseSampleIndex = (int)(cruiseTime/sampleTime); //index at which we reach cruise time

       totalTime =  cruiseTime + 2*maxAccelerationTime; //time it takes to finish profile
       samples = (int)(totalTime/sampleTime);
       setArrayLengths(desPos);
       if (!samplesIsZero){
       setTrapezoidArrays(dP); //no need to set values to these arrays if samples is zero
       }
       
    }


    //deals with the profile if it is a triangle
    private void triangularProfile(double dP, double desPos){ 
      totalTime = 2* Math.sqrt(2*Math.abs(dP)/maxAccel); //time it takes to finish profile
      samples = (int)(totalTime/sampleTime);   //amount of samples (ever 20 ms) needed for the profile
      triangleSampleIndex = samples/2;  //stores index of half way through the profile
      setArrayLengths(desPos);
      if(!samplesIsZero){
      setTriangularArrays(dP); //no need to set values to these arrays if samples is zero
      }
    }

    //setting the arrays for position, velocity, and accleration
    private void setTrapezoidArrays(double dP){
      double velocity = 0;
      double position = curPos; 
      for(int i = 0; i < accelArray.length; i++){
         if(i <= triangleSampleIndex){ //before reaching max velocity (acceleration)
            accelArray[i]=maxAccel * Math.signum(dP); 
         }else if(i< triangleSampleIndex + cruiseSampleIndex){
            accelArray[i] = 0;   //during cruise time there is zero acceleration
         }else{
            accelArray[i] = -maxAccel * Math.signum(dP);    //deceleration
         }
          // Integrate acceleration to get velocity
        velocity += accelArray[i] * sampleTime;
        velArray[i] = velocity;

        // Integrate velocity to get position
        position += velArray[i] * sampleTime;
        posArray[i] = position;

      }
    }

    private void setTriangularArrays(double dP){
      double velocity = 0;
      double position = curPos;
      for(int i = 0; i < accelArray.length; i++){
         if(i < triangleSampleIndex){
            accelArray[i]= maxAccel * Math.signum(dP);
         }else{
            accelArray[i] = -maxAccel * Math.signum(dP);
         }

         // Integrate acceleration to get velocity
        velocity += accelArray[i] * sampleTime;
        velArray[i] = velocity;

        // Integrate velocity to get position
        position += velArray[i] * sampleTime;
        posArray[i] = position;


      }
    }

    
    public double[] getSamples(){
      if (samples == 0) { // Prevents out-of-bounds access
         profileDone = true;
         return new double[]{0, 0, desPos}; // No movement
     }
      if(!timer.isRunning()){
         timer.start();
      }
      int currIndex = (int)(timer.get()/sampleTime);

      //current index is bigger than how many samples there: profile is done
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

   
    private void setArrayLengths(double desPos){
      if(samples == 0){
         accelArray = new double[]{0};
         velArray = new double[]{0};
         posArray = new double[]{desPos};
         samplesIsZero = true;
      }
      else{
      accelArray = new double[samples];
      velArray = new double[samples];
      posArray = new double[samples];
      }
    

    
}
}
