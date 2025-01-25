package org.wildstang.year2025.subsystems.arm_lift;

public class MotionProfile {

    private double acceleration_dt;
    private double position;
    private double acceleration;
    private double velocity;


    public double[] calculate(double max_acceleration, double max_velocity, double distance, double elapsed_time){

        //Calculate the time it takes to accelerate to max velocity
        acceleration_dt = max_velocity / max_acceleration;

        /* If we can't accelerate to max
        *velocity in the given distance,
        *we'll accelerate as much as possible
        *
        */ 
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);

        if (acceleration_distance > halfway_distance){
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 *max_acceleration));
        }

        /* recalculate max velocity based on
        * the time we have to accelerate and
        * decelerate
         */
        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
         max_velocity = max_acceleration * acceleration_dt;

         // We decelerate at the same rate as we accelerate
         double deceleration_dt = acceleration_dt;

         // calculate the time that we're at max velocity
         double cruise_distance = distance - 2 * acceleration_distance;
         double cruise_dt = cruise_distance / max_velocity;
         double deceleration_time = acceleration_dt + cruise_dt;

         //check the current motion
         double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
         if(elapsed_time <= acceleration_dt){
            //Acceleration phase
            acceleration = max_acceleration;
            position = 0.5 * max_acceleration * Math.pow(elapsed_time,2);
            velocity = max_acceleration * elapsed_time;
         }else if(elapsed_time <= deceleration_time){
            // cruise phase
            acceleration = 0;
            double cruise_elapsed_time = elapsed_time - acceleration_dt;
            position = acceleration_distance + max_velocity * cruise_elapsed_time;
            velocity = max_velocity;
         }else if(elapsed_time >= deceleration_time){
              //Deceleration phase
            acceleration = -max_acceleration;
            double deceleration_elapsed_time = elapsed_time - deceleration_dt;
            position = acceleration_distance + cruise_distance + max_velocity * deceleration_elapsed_time;
         }
        
         double goals[] = {position, velocity, acceleration};
         return goals;


      
    }

    
}
