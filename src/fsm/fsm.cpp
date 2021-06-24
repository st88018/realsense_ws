#include "fsm.h"

fsm::constantVtraj(){
  Quaterniond q(UAV_lp[3],UAV_lp[4],UAV_lp[5],UAV_lp[6]);
  Vec3 start_rpy = Q2rpy(q);
  Vec3 start_xyz(UAV_lp[0],UAV_lp[1],UAV_lp[2]);
  Vec3 des_rpy = Vec3(0,0,EndPose[5]);

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

  //initialize trajectory1
  deque<Vec8> trajectory1;
  double init_time = ros::Time::now().toSec();

  int wpc = duration/Trajectory_timestep;
  for (int i=0; i<wpc; i++){
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
    if(dt<=duration){
      xyz = Vec3(start_xyz[0]+dt*vxyz[0],start_xyz[1]+dt*vxyz[1],start_xyz[2]+dt*vxyz[2]);
    }else{
      xyz << EndPose[0],EndPose[1],EndPose[2];
    }

    Vec8 traj1;
    traj1 << dt+init_time, xyz[0], xyz[1], xyz[2], q.w(), q.x(), q.y(), q.z();
    trajectory1.push_back(traj1);
  }
  return(trajectory1);
}