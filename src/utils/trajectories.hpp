#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H
#include "common.h"
#include "../am_traj/am_traj.hpp"

deque<Vec8> trajectory1;
double AM_traj_duration;
Vec2 traj1_information;
double Trajectory_timestep = 0.02;
Vec2 Twisttraj_information;

void constantVtraj( Vec7 StartPose, Vec7 EndPose, double velocity, double angular_velocity){
  Quaterniond localq(StartPose[3],StartPose[4],StartPose[5],StartPose[6]);
  Vec3 localrpy = Q2rpy(localq);
  Vec3 start_rpy = Vec3(0,0,localrpy[2]);
  Vec3 start_xyz(StartPose[0],StartPose[1],StartPose[2]);
  Quaterniond endq(EndPose[3],EndPose[4],EndPose[5],EndPose[6]);
  Vec3 des_rpy = Q2rpy(endq);
  double dist = sqrt(pow((EndPose[0]-start_xyz[0]),2)+pow((EndPose[1]-start_xyz[1]),2)+pow((EndPose[2]-start_xyz[2]),2));
  double dist_duration = dist/velocity; // In seconds
  double duration; //total duration in seconds
  Vec3 vxyz = Vec3(((EndPose[0]-start_xyz[0])/dist)*velocity,((EndPose[1]-start_xyz[1])/dist)*velocity,((EndPose[2]-start_xyz[2])/dist)*velocity);
  if (start_rpy[2]>=M_PI)  start_rpy[2]-=2*M_PI;
  if (start_rpy[2]<=-M_PI) start_rpy[2]+=2*M_PI;
  if (des_rpy[2]>=M_PI)    des_rpy[2]-=2*M_PI;
  if (des_rpy[2]<=-M_PI)   des_rpy[2]+=2*M_PI;
  double d_yaw = des_rpy[2] - start_rpy[2];
  if (d_yaw>=M_PI)  d_yaw-=2*M_PI;
  if (d_yaw<=-M_PI) d_yaw+=2*M_PI;
  double yaw_duration = sqrt(pow(d_yaw/angular_velocity,2));
  if(yaw_duration>=dist_duration){duration = yaw_duration;}else{duration = dist_duration;}
  //initialize trajectory
  trajectory1.clear();
  double traj1_init_time = ros::Time::now().toSec();
  int wpc = duration/Trajectory_timestep;
  for(int i=0; i<wpc; i++){
      double dt = Trajectory_timestep*i;
      Vec3 xyz;
      Quaterniond q;
      // RPY
      if(dt<=yaw_duration){
          q = rpy2Q(Vec3(0,0,start_rpy[2]+dt*angular_velocity));
      }else{
          q = rpy2Q(des_rpy);
      }
      // Position_xyz
      if(dt<=dist_duration){
          xyz = Vec3(start_xyz[0]+dt*vxyz[0],start_xyz[1]+dt*vxyz[1],start_xyz[2]+dt*vxyz[2]);
      }else{
          xyz << EndPose[0],EndPose[1],EndPose[2];
      }
      Vec8 traj1;
      traj1 << dt+traj1_init_time, xyz[0], xyz[1], xyz[2], q.w(), q.x(), q.y(), q.z();
      trajectory1.push_back(traj1);
  }
}

void AM_traj(vector<Vector3d> WPs,Vec7 UAV_lp){
    Quaterniond localq(UAV_lp[3],UAV_lp[4],UAV_lp[5],UAV_lp[6]);
    Vec3 localrpy = Q2rpy(localq);
    Vec3 desrpy(0,0,localrpy[2]);
    Quaterniond desq;
    Trajectory am_traj;
    desq = rpy2Q(desrpy);
    //(weightT,weightAcc,weightJerk,maxVelRate,maxAccRate,iterations,epsilon);
    AmTraj amTrajOpt(1024,16,0.4,0.8,2,100,0.02);
    Vector3d zero3(0.0, 0.0, 0.0);
    am_traj = amTrajOpt.genOptimalTrajDTC(WPs, zero3, zero3, zero3, zero3);
    cout<< "      WPs.size: " << WPs.size() << endl
        << "      Constrained Spatial Optimal Trajectory with Trapezoidal Time Allocation" << endl
        << "      Lap Time: " << am_traj.getTotalDuration() << " s" << std::endl
        << "      Cost: " << amTrajOpt.evaluateObjective(am_traj) << std::endl
        << "      Maximum Velocity Rate: " << am_traj.getMaxVelRate() << " m/s" << std::endl
        << "      Maximum Acceleration Rate: " << am_traj.getMaxAccRate() << " m/s^2" << std::endl;
    AM_traj_duration = am_traj.getTotalDuration();
    //initialize trajectory
    trajectory1.clear();
    double traj1_init_time = ros::Time::now().toSec();
    double T = 0.01;
    for  (double dt = 0; dt < am_traj.getTotalDuration(); dt += T){
        Vector3d xyz = am_traj.getPos(dt);
        Vec8 traj1;
        traj1 << dt+traj1_init_time, xyz[0], xyz[1], xyz[2], desq.w(), desq.x(), desq.y(), desq.z();
        trajectory1.push_back(traj1);
    }
}
#endif