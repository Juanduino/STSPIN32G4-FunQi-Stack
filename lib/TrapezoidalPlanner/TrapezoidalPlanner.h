#ifndef __TRAPEZOIDAL_PLANNER__H
#define __TRAPEZOIDAL_PLANNER__H
#include <SimpleFOC.h>
#include "../CircularBuffer/CircularBuffer.h"

class TrapezoidalPlanner
{
public:
    TrapezoidalPlanner(int);
    void doTrapezoidalPlannerCommand(char *command);
    void doGcommandBuffer(char *command);
    void doMCommand(char *command);
    void linkMotor(StepperMotor*);
    void runPlannerOnTick();
    bool isPlannerMoving();
private:
    StepperMotor* motor;
    unsigned long plannerTimeStap;
    int plannerPeriod = 0.5; // 1000 / this number = Hz, i.e. 1000 / 100 = 10Hz, 1000 / 10 = 100Hz, 1000 / 5 = 200Hz, 1000 / 1 = 1000hZ
    float Vmax_ = 40.0f;    // # Velocity max (rads/s)
    float Amax_ = 30.0f;    // # Acceleration max (rads/s/s)
    float Dmax_ = 30.0f;    // # Decelerations max (rads/s/s)
    bool m400_flag = false;
    float Y_;
    float Yd_;
    float Ydd_;
    float Tf_, Xi_, Xf_, Vi_, Ar_, Dr_, Vr_, Ta_, Td_, Tv_, yAccel_;
    unsigned long plannerStartingMovementTimeStamp;
    bool isTrajectoryExecuting;
    float sign(float val);
    float sign_hard(float val);
    bool calculateTrapezoidalPathParameters(float Xf, float Xi, float Vi, float Vmax, float Amax, float Dmax);
    void startExecutionOfPlannerTo(float newPos);
    void computeStepValuesForCurrentTime(float currentTrajectoryTime);
    CircularBuffer buffer = CircularBuffer(100);
};

#endif