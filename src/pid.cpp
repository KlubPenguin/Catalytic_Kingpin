#include "main.h"
#include "numbers"
#include "autons.hpp"
#include "autons.cpp"

class PID{
    private: double maxLeft = 0;
    private : int totalTime = 0;

    PID(){
        
    }

    void startTime(int interval){
        while(finishTime != 0){
            totalTime += interval;
            pros::delay(interval);
        }
    }

    bool finishTime(){
        return 1;
    }

    void robotTasks(){
        default_constants();
    }
};
