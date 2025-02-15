package org.wildstang.year2025.subsystems.arm_lift;

import edu.wpi.first.wpilibj.Timer;

public class MotionProfile {
   private Timer timer = new Timer(); // Timer function (Sort of like a stopwatch/clock)
   private final double sampleTime = 0.02; //seconds
   private double[][] profileArray = new double[1][3];  // Array that stores an array containing the goal acceleration, velocity, and position at each timestep

   public boolean profileDone; //if desired position is reached
   private double smallDeltaP;

   private final double maxAccel; // The maximum acceleration (Calculated using math, physics, and motor data sheets)
   private final double maxVel; // Maximum velocity (A value that we pass in)
   private final double maxAccelerationTime; // The time it takes to reach max velocity (Our highest Velocity)
   private final double minDistanceForMaxVel; // minimum distance it takes to reach max velocity

   public MotionProfile(double maxAcceleration, double maxVelocity, double smallDeltaP){
      this.maxAccel = maxAcceleration;
      this.maxVel = maxVelocity;
      this.maxAccelerationTime = maxVelocity/maxAcceleration; // time it takes to reach max velocity
      this.minDistanceForMaxVel = 0.5 * maxAcceleration * Math.pow(maxAccelerationTime, 2.0); //mininum distance required to reach maxVelocity
      this.smallDeltaP = smallDeltaP;
      profileDone = false;
   }

   public void calculate(double curPos, double desPos){
      timer.reset();
      profileDone = false;
      
      if (Math.abs(desPos - curPos) < smallDeltaP) { // No profile needed
         profileArray = new double[][] {{0,0,desPos}};
         profileDone = true;
         return;
      } else if (Math.abs(desPos - curPos) <= 2 * minDistanceForMaxVel) {  //if change in position is less than minimum distance required to reach max velocity, use triangle profile
         //Triangular Profile
         triangularProfile(curPos, desPos);
      } else {
         //trapezoid profile
         trapezoidProfle(curPos, desPos);
      }
   }

   private void trapezoidProfle(double curPos, double desPos){
      //Trapezoidal Profile
      //cruiseTime = (dP - 2* minDistanceForMaxVel)/maxVel or time of crusing at maxVelocity
      //totalTime =  cruiseTime + 2*maxAccelerationTime or time it takes to finish profile
      //samples = (int)(totalTime/sampleTime)
      setArrayLengths((int) (((Math.abs(desPos - curPos) - 2 * minDistanceForMaxVel) / maxVel + 2 * maxAccelerationTime) / sampleTime));
      setTrapezoidArrays(curPos, desPos);
   }

   //deals with the profile if it is a triangle
   private void triangularProfile(double curPos, double desPos){ 
      //  totalTime = 2* Math.sqrt(2*Math.abs(dP)/maxAccel); //time it takes to finish profile
      //samples = (int)(totalTime/sampleTime);
      setArrayLengths((int) Math.abs(2 * Math.sqrt(2 * Math.abs(desPos - curPos) / maxAccel) / sampleTime));
      setTriangularArrays(curPos, desPos);
   }

   //setting the arrays for position, velocity, and accleration
   private void setTrapezoidArrays(double curPos, double desPos){
      double accel = 0;
      double velocity = 0;
      double position = curPos;
      int dir = (int) Math.signum(desPos - curPos);
      for(int i = 0; i < profileArray.length - 1; i++){
         if (i <= (int) (maxAccelerationTime / sampleTime)) {
            accel = maxAccel * dir;  // before reaching max velocity (acceleration)
         } else if (i < profileArray.length - (int) (maxAccelerationTime / sampleTime)) {
            accel = 0;  // during cruise time there is zero acceleration
         } else {
            accel = -maxAccel * dir;  // deceleration
         }

         // Integrate acceleration to get velocity
         velocity += accel * sampleTime;

         // Integrate velocity to get position
         position += velocity * sampleTime;

         profileArray[i] = new double[] {accel, velocity, position};
      }
      profileArray[profileArray.length-1] = new double[] {0, 0, desPos};  // ensure last sample has properly set values
   }

   private void setTriangularArrays(double curPos, double desPos){
      double accel = 0;
      double velocity = 0;
      double position = curPos;
      int dir = (int) Math.signum(desPos - curPos);
      for(int i = 0; i < profileArray.length - 1; i++){
         if (i < profileArray.length / 2.0) {
            accel = maxAccel * dir;
         } else {
            accel = -maxAccel * dir;
         }

         // Integrate acceleration to get velocity
         velocity += accel * sampleTime;

         // Integrate velocity to get position
         position += velocity * sampleTime;

         profileArray[i] = new double[] {accel, velocity, position};
      }
      profileArray[profileArray.length-1] = new double[] {0, 0, desPos};  // ensure last sample has properly set values
   }

   public double[] getSamples(){
      if (profileDone) {
         return profileArray[profileArray.length - 1];
      }

      if(!timer.isRunning()){
         timer.start();
      }

      int curIndex = (int)(timer.get()/sampleTime);
      //curent index is bigger than how many samples there: profile is done
      if (curIndex >= profileArray.length) {
         curIndex = profileArray.length - 1;
         profileDone = true;
         timer.stop();
      }

      return profileArray[curIndex];
   }

   private void setArrayLengths(int samples){
      profileArray = new double[samples][3];
   }
}
