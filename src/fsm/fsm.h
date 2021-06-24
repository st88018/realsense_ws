#ifndef FSM_H
#define FSM_H
#include <string>
#include <numeric>
#include <iostream>
#include "../utils/common.h"

using namespace std;

class fsm
{
    private:

    public:
        Vec6 EndPose;
        Vec7 UAV_lp;
        double velocity,angular_velocity;
        deque<Vec8> constantVtraj();
}

#endif