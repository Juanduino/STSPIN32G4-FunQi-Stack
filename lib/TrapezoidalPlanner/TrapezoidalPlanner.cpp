#include <Arduino.h>
#include <TrapezoidalPlanner.h>

#define __debug

TrapezoidalPlanner::TrapezoidalPlanner(int tickPeriod ){
    plannerPeriod = tickPeriod;
    isTrajectoryExecuting = false;
    return;
}

void TrapezoidalPlanner::linkMotor(StepperMotor *motorReference){
    motor = motorReference;
}

bool TrapezoidalPlanner::isPlannerMoving(){
    return isTrajectoryExecuting;
}


void TrapezoidalPlanner::doMCommand(char *MCommand){
  #ifdef __debug
        Serial.print("GGode command: M");
        Serial.println(MCommand);
    #endif


       
      // Parse this string for vals to int
        String commandSrt = String(MCommand);
        int commandValue_Int = 0;
        commandValue_Int = commandSrt.toInt();

  
  


        if (commandValue_Int == 115){
            // M115
            // Send firmware version and capabilities
            Serial.println("FIRMWARE_NAME:SimpleFOC");
            Serial.println("FIRMWARE_VERSION:1.0.0");
            Serial.println("FIRMWARE_URL:GIT_URL");
            Serial.println("PROTOCOL_VERSION:1.0");
            Serial.println("AVAILABLE_COMMANDS:M,V,A,L");
            Serial.println("CAPABILITY:MOTOR_VOLTAGE,INPUT_VOLTAGE,POWER_SUPPLY,POSITION_CONTROL,VELOCITY_CONTROL,VELOCITY_RAMP,TRAJECTORY_CONTROL");
            Serial.println("POWER_SUPPLY:24V");
            Serial.println("MOTOR_VOLTAGE:24V");
            Serial.println("INPUT_VOLTAGE:24V");
            Serial.println("POSITION_CONTROL:1");
            Serial.println("VELOCITY_CONTROL:1");
            Serial.println("VELOCITY_RAMP:1");
            Serial.println("TRAJECTORY_CONTROL:1");
            Serial.println("POSITION_MIN:-3.14159265359");
            Serial.println("POSITION_MAX:3.14159265359");
            Serial.println("VELOCITY_MIN:-12.5663706144");
            Serial.println("VELOCITY_MAX:12.5663706144");
            Serial.println("ACCELERATION_MIN:-12.5663706144");
            Serial.println("ACCELERATION_MAX:12.5663706144");


        }

        //The controller can report axes positions, including extra axes (A, B, C etc.), typically with the M114 command.

          if (commandValue_Int == 114){

        // M114
        // Send current position

        //MM per revolution movement 
        int mm_per_rev = 17;
        float angle_rapport = motor->shaft_angle;
        //convert to mm from 2 x radians per revolution using the mm_per_rev variable as the mm per revolution
        float mm =  map(angle_rapport, 0, 2*PI, 0, mm_per_rev);
        Serial.print("X:");
        Serial.println(mm);
 

       
          } 


         // The controller must be able to wait for motion completion, typically with the M400 command. Any further commands sent after the M400 must be suspended until motion completion. 
         // The controller must only acknowledge the command, when motion is complete i.e. the "ok" (COMMAND_CONFIRM_REGEX) response must be suspended until then, providing blocking 
         // synchronization to OpenPnP.

            if (commandValue_Int == 400){
                // M400
                // Wait for current move to complete



            }

            //The controller must support dynamic acceleration and/or jerk limits, typically by the M204 command for acceleration or the M201.3 command for jerk.
            float commandValue2;
             if (commandValue_Int == 204){
                // M204
                // Set acceleration
        
        
                // Remove M so can convert to a float
        
        commandValue2 = commandSrt.toFloat();
        Vmax_ = commandValue2;
        // Try calling the planner to use this new velocity value
        // We have to use the current pos, vel, and accel
        // calc_plan_trapezoidal_path_at_start(Xf_, Xi_, Vi_, Vmax_, Amax_, Dmax_);
        calculateTrapezoidalPathParameters(Xf_, motor->shaft_angle, motor->shaft_velocity, Vmax_, Amax_, Dmax_);
        #ifdef __debug
            Serial.print("User wants velocity change. Vmax_: ");
            Serial.println(Vmax_);
        #endif
      
            }

            if (commandValue_Int == 201.3){
                // M201.3
                // Set jerk

              // Remove M so can convert to a float
      
        commandValue2 = commandSrt.toFloat();
        Amax_ = commandValue2;
        Dmax_ = Amax_;
        // Try calling the planner to use this new acceleration value
        // calc_plan_trapezoidal_path_at_start(Xf_, Xi_, Vi_, Vmax_, Amax_, Dmax_);
        calculateTrapezoidalPathParameters(Xf_, motor->shaft_angle, motor->shaft_velocity, Vmax_, Amax_, Dmax_);
        #ifdef __debug
            Serial.print("User wants acceleration change. Amax_: ");
            Serial.println(Amax_);
        #endif 
       
            }

            // The controller must be able to home. The homing procedure must be configurable, typically with the M206 command. The controller must be able to home to a position other than 0.

            if (commandValue_Int == 206){
                // M206
                // Set current position, for homing to 0. 

                commandSrt = commandSrt.substring(4);
                commandValue2 = commandSrt.toFloat();
        
                motor->shaft_angle = commandValue2;

                Serial.print("Homing to: ");
                Serial.println(commandValue2);

                Serial.print("Chekking shaft_angle : ");
                Serial.println(motor->shaft_angle);


       
            }

           

            if (commandValue_Int == 203){
                // M203
                // Set maximum feedrate

                //  TODO  TODO TODO TODO
        //  TODO  TODO TODO TODO
      
            }

          

            if (commandValue_Int == 201){
                // M201
                // Set maximum acceleration

                //  TODO  TODO TODO TODO
        //  TODO  TODO TODO TODO
        
            }

            
            if (commandValue_Int == 202){
                // M202
                // Set maximum jerk

                //  TODO  TODO TODO TODO
        //  TODO  TODO TODO TODO
        
            }


            if (commandValue_Int == 205){
                // M205
                // Advanced settings

                //  TODO  TODO TODO TODO
        //  TODO  TODO TODO TODO


        
            }

            

           

                




        // Place different if statements like "fetch current position".

        #ifdef __debug
            Serial.print("M Command :  ");
            Serial.println(commandValue_Int);
        #endif
     

    


    

}


void TrapezoidalPlanner::doTrapezoidalPlannerCommand(char *gCodeCommand){
    #ifdef __debug
        Serial.print("GGode command: G");
        Serial.println(gCodeCommand);
    #endif
    
    // Parse this string for vals
    String commandSrt = String(gCodeCommand);
    float commandValue;
    switch (gCodeCommand[0]){

         

    case 'V':
        // Remove V so can convert to a float
        commandSrt = commandSrt.substring(1);
        commandValue = commandSrt.toFloat();
        Vmax_ = commandValue;
        // Try calling the planner to use this new velocity value
        // We have to use the current pos, vel, and accel
        // calc_plan_trapezoidal_path_at_start(Xf_, Xi_, Vi_, Vmax_, Amax_, Dmax_);
        calculateTrapezoidalPathParameters(Xf_, motor->shaft_angle, motor->shaft_velocity, Vmax_, Amax_, Dmax_);
        #ifdef __debug
            Serial.print("User wants velocity change. Vmax_: ");
            Serial.println(Vmax_);
        #endif
        break;
    case 'A':
        // Remove A so can convert to a float
        commandSrt = commandSrt.substring(1);
        commandValue = commandSrt.toFloat();
        Amax_ = commandValue;
        Dmax_ = Amax_;
        // Try calling the planner to use this new acceleration value
        // calc_plan_trapezoidal_path_at_start(Xf_, Xi_, Vi_, Vmax_, Amax_, Dmax_);
        calculateTrapezoidalPathParameters(Xf_, motor->shaft_angle, motor->shaft_velocity, Vmax_, Amax_, Dmax_);
        #ifdef __debug
            Serial.print("User wants acceleration change. Amax_: ");
            Serial.println(Amax_);
        #endif 
        break;
    case 'L':
        //  TODO  TODO TODO TODO
        //  TODO  TODO TODO TODO
        break;
    default:
        commandValue = commandSrt.toFloat();
        #ifdef __debug
            Serial.print("Move to new position (rads): ");
            Serial.println(commandValue);
        #endif 
        // We start moving to the new position
        startExecutionOfPlannerTo(commandValue);
        break;
    }
}

void TrapezoidalPlanner::runPlannerOnTick(){
    // This should get entered 100 times each second (100Hz)
    if ((unsigned long)(millis() - plannerTimeStap) > plannerPeriod){
        plannerTimeStap = millis();
        // see if we are in a move or not
        if (isTrajectoryExecuting){
            // we are in a move, let's calc the next position
            float timeSinceStartingTrajectoryInSeconds = (millis() - plannerStartingMovementTimeStamp) / 1000.0f;
            computeStepValuesForCurrentTime(timeSinceStartingTrajectoryInSeconds);
            motor->target = Y_;

            // see if we are done with our move
            if (timeSinceStartingTrajectoryInSeconds >= Tf_){
                // we are done with move
                // motor.monitor_downsample = 0; // disable monitor
                #ifdef __debug
                    Serial.println("Done with move");
                #endif 
                isTrajectoryExecuting = false;
            }
        }
    }
}

float TrapezoidalPlanner::sign(float val){
    if (val < 0)
        return -1.0f;
    if (val == 0)
        return 0.0f;
    // if val > 0:
    return 1.0f;
}

float TrapezoidalPlanner::sign_hard(float val){
    if (val < 0)
        return -1.0f;
    return 1.0f;
}

bool TrapezoidalPlanner::calculateTrapezoidalPathParameters(float Xf, float Xi, float Vi, float Vmax, float Amax, float Dmax){
    float dX = Xf - Xi;                          // Distance to travel
    float stop_dist = (Vi * Vi) / (2.0f * Dmax); // Minimum stopping distance
    float dXstop = sign(Vi) * stop_dist;         // Minimum stopping displacement
    float s = sign_hard(dX - dXstop);            // Sign of coast velocity (if any)
    Ar_ = s * Amax;                              // Maximum Acceleration (signed)
    Dr_ = -s * Dmax;                             // Maximum Deceleration (signed)
    Vr_ = s * Vmax;                              // Maximum Velocity (signed)

    // If we start with a speed faster than cruising, then we need to decel instead of accel
    // aka "double deceleration move" in the paper
    if ((s * Vi) > (s * Vr_)){
        Ar_ = -s * Amax;
    }

    // Time to accel/decel to/from Vr (cruise speed)
    Ta_ = (Vr_ - Vi) / Ar_;
    Td_ = -Vr_ / Dr_;

    // Integral of velocity ramps over the full accel and decel times to get
    // minimum displacement required to reach cuising speed
    float dXmin = 0.5f * Ta_ * (Vr_ + Vi) + 0.5f * Td_ * Vr_;

    // Are we displacing enough to reach cruising speed?
    if (s * dX < s * dXmin){
        // Short move (triangle profile)
        Vr_ = s * sqrt((Dr_ * sq(Vi) + 2 * Ar_ * Dr_ * dX) / (Dr_ - Ar_));
        Ta_ = max(0.0f, (Vr_ - Vi) / Ar_);
        Td_ = max(0.0f, -Vr_ / Dr_);
        Tv_ = 0.0f;
    }
    else{
        // Long move (trapezoidal profile)
        Tv_ = (dX - dXmin) / Vr_;
    }

    // Fill in the rest of the values used at evaluation-time
    Tf_ = Ta_ + Tv_ + Td_;
    Xi_ = Xi;
    Xf_ = Xf;
    Vi_ = Vi;
    yAccel_ = Xi + Vi * Ta_ + 0.5f * Ar_ * sq(Ta_); // pos at end of accel phase

    #ifdef __debug
        Serial.println("--------------------------------- ");
        Serial.println(" Calculated trapezoidal Values:   ");
        Serial.println("     Tf: " + String(Tf_));
        Serial.println("     Ta: " + String(Ta_));
        Serial.println("     Tv: " + String(Tv_));
        Serial.println("     Td: " + String(Td_));
        Serial.println("     --------------------- ");
        Serial.println("     Ar: " + String(Ar_));
        Serial.println("     Vr: " + String(Vr_));
        Serial.println("     Dr: " + String(Dr_));
        Serial.println("     --------------------- ");
        Serial.println("     Xf: " + String(Xf));
        Serial.println("     Xi: " + String(Xi));
    #endif

    return true;
}

void TrapezoidalPlanner::startExecutionOfPlannerTo(float newPos){
    
    // set our global of the new position
    Xf_ = newPos;
    // At this poin we are atarting to move following the trapezoidal profile
    plannerStartingMovementTimeStamp = millis();
    // take the position from SimpleFOC and set it to our start position
    Xi_ = motor->shaft_angle;
    // TODO: If we are being asked to move but are already in motion, we should go with the current velocity, position, and acceleration
    // and keep moving.
    // Now we need to do our calcs before we start moving
    calculateTrapezoidalPathParameters(Xf_, Xi_, Vi_, Vmax_, Amax_, Dmax_);
    motor->target = Y_; // could possibly put this in the method above
    // Tell user how long this move will take
    #ifdef __debug
        Serial.println("Starting to move to a new position");
        Serial.print("Time to complete move (secs):");
        Serial.println(Tf_);
        // Velocity and Accel of move
        Serial.print("Velocity for move: ");
        Serial.print(Vmax_);
        Serial.print(", Acceleration: ");
        Serial.println(Amax_);
    #endif
    // set our global bool so the tick method knows we have a move in motion
    isTrajectoryExecuting = true;
}
void TrapezoidalPlanner::computeStepValuesForCurrentTime(float currentTrajectoryTime){
    // Step_t trajStep;
    if (currentTrajectoryTime < 0.0f){ 
        // Initial Condition
        Y_ = Xi_;
        Yd_ = Vi_;
        Ydd_ = 0.0f;
        //Serial.println("--- Initial Stage ---");
        #ifdef __debug
            Serial.print(String(currentTrajectoryTime) + ",");
            Serial.print(Y_, 5);
            Serial.println();
        #endif
    }
    else if (currentTrajectoryTime < Ta_){ 
        // Accelerating
        Y_ = Xi_ + Vi_ * currentTrajectoryTime + 0.5f * Ar_ * sq(currentTrajectoryTime);
        Yd_ = Vi_ + Ar_ * currentTrajectoryTime;
        Ydd_ = Ar_;
        #ifdef __debug
        // Serial.print("Accelerating  ");
            Serial.print(String(currentTrajectoryTime) + ",");
            Serial.print(Y_, 5);
            Serial.println();
        #endif
    }
    else if (currentTrajectoryTime < Ta_ + Tv_){
        // Coasting
        Y_ = yAccel_ + Vr_ * (currentTrajectoryTime - Ta_);
        Yd_ = Vr_;
        #ifdef __debug
            // Serial.print("Coastting ");
            Serial.print(String(currentTrajectoryTime) + ",");
            Serial.print(Y_, 5);
            Serial.println();
        #endif
    }
    else if (currentTrajectoryTime < Tf_){ 
        // Deceleration
        float td = currentTrajectoryTime - Tf_;
        Y_ = Xf_ + 0.5f * Dr_ * sq(td);
        Yd_ = Dr_ * td;
        Ydd_ = Dr_;
        // Serial.print("Decelerating  ");
        #ifdef __debug
            // Serial.print("Accelerating  ");
            Serial.print(String(currentTrajectoryTime) + ",");
            Serial.print(Y_, 5);
            Serial.println();
        #endif
    }
    else if (currentTrajectoryTime >= Tf_){ // Final Condition
        Y_ = Xf_;
        Yd_ = 0.0f;
        Ydd_ = 0.0f;
        #ifdef __debug
            //Serial.print("--- Final Stage ---");
            Serial.print(String(currentTrajectoryTime) + ",");
            Serial.print(Y_, 5);
            Serial.println();
        #endif
    }
    else{
        // TODO: Error halnling here
    }
    return; 
}
