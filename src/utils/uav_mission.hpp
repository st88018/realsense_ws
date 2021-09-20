#ifndef UAV_MISSION_H
#define UAV_MISSION_H

#include "common.h"

deque<Vec8> waypoints;
double velocity_mission = 0.5;
double velocity_angular = 1;

// 1 take off 2 constant velocity traj 4 RTL 5 Land 
// 6 PID single pose (step) 7 PID follow 8 PID land

// deque<Vec8> Finite_stage_mission(){
//     waypoints.clear();
//     // Waypoints
//     Vec8 stage; // state x y z yaw v av waittime
//     stage << 1, 0, 0, 1, 3.14, velocity_mission, velocity_angular, 5;   // state = 1; takeoff
//     waypoints.push_back(stage);
//     stage << 7, 0, 0, 1, 3.14, 0, 0, 30;
//     waypoints.push_back(stage);
//     stage << 8, 0, 0, 1, 3.14, 0, 0, 120;
//     waypoints.push_back(stage);
//     cout << " Mission generated!" << " Stage count: " << waypoints.size() << endl;
//     return(waypoints);
// }
deque<Vec8> Finite_stage_mission(){
    waypoints.clear();
    // Waypoints
    Vec8 stage; // state x y z yaw v av waittime
    stage << 1, 0, 0, 0.75, 3.14, velocity_mission, velocity_angular, 5;   // state = 1; takeoff
    waypoints.push_back(stage);
    stage << 6, 0, 0, 0.75, 3.14, 0, 0, 40;
    waypoints.push_back(stage);
    stage << 6, -5, 5, 0.75, 3.14, 0, 0, 40;
    waypoints.push_back(stage);
    stage << 6, -5, -5, 0.75, 3.14, 0, 0, 40;
    waypoints.push_back(stage);
    stage << 6, 5, -5, 0.75, 3.14, 0, 0, 40;
    waypoints.push_back(stage);
    stage << 6, 5, 5, 0.75, 3.14, 0, 0, 40;
    waypoints.push_back(stage);
    stage << 4, 0, 0, 0.75, 3.14, velocity_mission, velocity_angular, 10;  // state = 4; constant velocity RTL but with altitude.
    waypoints.push_back(stage);
    stage << 5, 0, 0, -10, 0, velocity_mission, velocity_angular, 60;  // state = 5; land.
    waypoints.push_back(stage);
    cout << " Mission generated!" << " Stage count: " << waypoints.size() << endl;
    return(waypoints);
}
// deque<Vec8> Finite_stage_mission(){
//     waypoints.clear();
//     // Waypoints
//     Vec8 stage; // state x y z yaw v av waittime
//     stage << 1, 0, 0, 1, 3.14, velocity_mission, velocity_angular, 10;   // state = 1; takeoff
//     waypoints.push_back(stage);
//     stage << 2, 1.5, 1.5, 1, 3.14, velocity_mission, velocity_angular, 0;   // state = 2; constant velocity trajectory.
//     waypoints.push_back(stage);
//     stage << 2,-1.5, 1.5, 1, 3.14, velocity_mission, velocity_angular, 0;
//     waypoints.push_back(stage);
//     stage << 2,-1.5,-1.5, 1, 3.14, velocity_mission, velocity_angular, 0;
//     waypoints.push_back(stage);
//     stage << 2, 1.5,-1.5, 1, 3.14, velocity_mission, velocity_angular, 0;
//     waypoints.push_back(stage);
//     stage << 4, 0, 0, 1, 3.14, velocity_mission, velocity_angular, 0;  // state = 4; constant velocity RTL but with altitude.
//     waypoints.push_back(stage);
//     stage << 5, 0, 0, -10, 3.14, velocity_mission, velocity_angular, 60;  // state = 5; land.
//     waypoints.push_back(stage);
//     cout << " Mission generated!" << " Stage count: " << waypoints.size() << endl;
//     return(waypoints);
// }
// deque<Vec8> Finite_stage_mission(){ //Normal mission
//     waypoints.clear();
//     // Waypoints
//     Vec8 stage; // state x y z yaw v av waittime
//     stage << 1, 0, 0, 1, 3.14, velocity_mission, velocity_angular, 10;   // state = 1; takeoff
//     waypoints.push_back(stage);
//     stage << 2, 1.5, 1.5, 1, 3.14, velocity_mission, velocity_angular, 0;   // state = 2; constant velocity trajectory.
//     waypoints.push_back(stage);
//     stage << 2,-1.5, 1.5, 1, 3.14, velocity_mission, velocity_angular, 0;
//     waypoints.push_back(stage);
//     stage << 2,-1.5,-1.5, 1, 3.14, velocity_mission, velocity_angular, 0;
//     waypoints.push_back(stage);
//     stage << 2, 1.5,-1.5, 1, 3.14, velocity_mission, velocity_angular, 0;
//     waypoints.push_back(stage);
//     stage << 4, 0, 0, 1, 3.14, velocity_mission, velocity_angular, 10;  // state = 4; constant velocity RTL but with altitude.
//     waypoints.push_back(stage);
//     stage << 5, 0, 0, -10, 3.14, velocity_mission, velocity_angular, 60;  // state = 5; land.
//     waypoints.push_back(stage);
//     cout << " Mission generated!" << " Stage count: " << waypoints.size() << endl;
//     return(waypoints);
// }
#endif