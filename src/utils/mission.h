#ifndef MISSION_H
#define MISSION_H

#include "common.h"

deque<Vec8> waypoints;

deque<Vec8> Finite_stage_mission(double velocity_mission,double velocity_angular){
    waypoints.clear();
    // Waypoints
    Vec8 stage; // state x y z yaw v av waittime
    stage << 1, 0, 0, 1, 0, velocity_mission, velocity_angular, 10;   // state = 1; takeoff
    waypoints.push_back(stage);
    stage << 2, 5, 5, 1, 0, velocity_mission, velocity_angular, 10;   // state = 2; constant velocity trajectory.
    waypoints.push_back(stage);
    stage << 2,-5, 5, 1, 0, velocity_mission, velocity_angular, 10;
    waypoints.push_back(stage);
    stage << 2,-5,-5, 1, 0, velocity_mission, velocity_angular, 10;
    waypoints.push_back(stage);
    stage << 2, 5,-5, 1, 0, velocity_mission, velocity_angular, 10;
    waypoints.push_back(stage);
    stage << 4, 0, 0, 1, 0, velocity_mission, velocity_angular, 20;  // state = 4; constant velocity RTL but with altitude.
    waypoints.push_back(stage);
    stage << 5, 0, 0, -10, 0, velocity_mission, velocity_angular, 60;  // state = 5; land.
    waypoints.push_back(stage);
    cout << " Mission generated!" << " Stage count: " << waypoints.size() << endl;
    return(waypoints);
}
#endif