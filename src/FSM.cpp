#include <ros/ros.h>
#include <string>
#include <numeric>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include "utils/kinetic_math.hpp"
#include "utils/uav_mission.hpp"
#include "utils/trajectories.hpp"

static mavros_msgs::State           current_state;
static mavros_msgs::AttitudeTarget  UAV_AttitudeTarget;
static geometry_msgs::PoseStamped   UAV_pose_sub,UGV_pose_sub,UAV_pose_pub,UGV_pose_pub;
static geometry_msgs::TwistStamped  UGV_twist_sub;
static geometry_msgs::Twist         UAV_twist_pub;
static Vec7 UAV_lp,UGV_lp;

/* System */
bool System_init = false;
double System_initT;
ros::Time init_time,last_request; /* Use at System start up */
static Vec7 Zero7;
static Vec4 Zero4;
static int coutcounter;
/* PID Position controller */
static Vec4   Pos_setpoint,UGV_twist;
static double PID_duration;
static double PID_InitTime;
static double Last_time = 0;
static Vec4 last_error,integral;
/* Fail safe (not ready)*/
static double safe_dist = 0.5;
bool   Failsafe_System_enable  = false;
bool   FailsafeFlag  = false;
/* FSM */
Vec7 UAV_desP,UAV_takeoffP;
Vec7 TargetPos;
int    FSM_state = 0;
bool   FSM_1_init = false;
Vec7   FSM_1_pose;
int    Mission_state = 0;
int    Mission_stage = 0;
int    Current_Mission_stage = 0;
Vec8   Current_stage_mission;
bool   FSMinit       = false;
bool   Mission8init  = false;
double M8start_alt;
bool   pubpose       = false;
bool   pubtwist      = false;
bool   Force_start   = false;
bool   Shut_down     = false;

void failsafe(){
    double dist = sqrt(pow((UAV_lp[0]-UGV_lp[0]),2)+pow((UAV_lp[1]-UGV_lp[1]),2)+pow((UAV_lp[2]-UGV_lp[2]),2));
    if (dist < safe_dist){
        FailsafeFlag = true;
    }else{FailsafeFlag = false;}
}
Vec7 ugv_pred_land_pose(Vec7 UGV_lp,Vec4 UGV_twist,double est_duration){
    Vec7 EstimatedPose;
    Quaterniond ugvq(UGV_lp[3],UGV_lp[4],UGV_lp[5],UGV_lp[6]);
    Vec3 ugvrpy = Q2rpy(ugvq);
    Vec4 ugv_XYZyaw = Vec4(UGV_lp[0],UGV_lp[1],UGV_lp[2],ugvrpy[2]);
    Vec4 ugv_pred_XYZyaw;
    for(int i=0; i<4; i++){
        ugv_pred_XYZyaw[i] = ugv_XYZyaw[i]+UGV_twist[i]*est_duration;
    }
    ugvq = rpy2Q(Vec3(0,0,ugv_pred_XYZyaw[3]));
    EstimatedPose << ugv_pred_XYZyaw[0],ugv_pred_XYZyaw[1],ugv_pred_XYZyaw[2],
                     ugvq.w(),ugvq.x(),ugvq.y(),ugvq.z();
    return(EstimatedPose);
}
void ugv_pose_sub(const geometry_msgs::PoseStamped::ConstPtr& pose){
    UGV_pose_sub.pose.position.x = pose->pose.position.x;
    UGV_pose_sub.pose.position.y = pose->pose.position.y;
    UGV_pose_sub.pose.position.z = pose->pose.position.z;
    UGV_pose_sub.pose.orientation.w = pose->pose.orientation.w;
    UGV_pose_sub.pose.orientation.x = pose->pose.orientation.x;
    UGV_pose_sub.pose.orientation.y = pose->pose.orientation.y;
    UGV_pose_sub.pose.orientation.z = pose->pose.orientation.z;
    UGV_lp << UGV_pose_sub.pose.position.x,UGV_pose_sub.pose.position.y,UGV_pose_sub.pose.position.z,
              UGV_pose_sub.pose.orientation.w,UGV_pose_sub.pose.orientation.x,UGV_pose_sub.pose.orientation.y,UGV_pose_sub.pose.orientation.z;
}
void ugv_twist_sub(const geometry_msgs::TwistStamped::ConstPtr& twist){
    UGV_twist_sub.twist.linear.x = twist->twist.linear.x;
    UGV_twist_sub.twist.linear.y = twist->twist.linear.y;
    UGV_twist_sub.twist.linear.z = twist->twist.linear.z;
    UGV_twist_sub.twist.angular.x = twist->twist.angular.x;
    UGV_twist_sub.twist.angular.y = twist->twist.angular.y;
    UGV_twist_sub.twist.angular.z = twist->twist.angular.z;
    UGV_twist << UGV_twist_sub.twist.linear.x,UGV_twist_sub.twist.linear.y,
                 UGV_twist_sub.twist.linear.z,UGV_twist_sub.twist.angular.z;
}
Vec4 uav_poistion_controller_PID(Vec4 pose, Vec4 setpoint){
    Vec4 error,u_p,u_i,u_d,output,derivative;
    double iteration_time = ros::Time::now().toSec() - Last_time;
    // cout << "iteration_time: " << iteration_time << endl;
    Vec4 K_p(2,2,1,1);
    Vec4 K_i(0.05,0.05,0.05,0.05);
    Vec4 K_d(1.5,1.5,0,0);
    error = setpoint-pose;
    if (error[3]>=M_PI){error[3]-=2*M_PI;}
    if (error[3]<=-M_PI){error[3]+=2*M_PI;}
    for (int i=0; i<4; i++){ //i = x,y,z
        integral[i] += (error[i]*iteration_time);
        if(integral[i] >  1){ integral[i]=  1;}
        if(integral[i] < -1){ integral[i]=-1;}
        derivative[i] = (error[i] - last_error[i])/(iteration_time + 1e-10);
        u_p[i] = error[i]*K_p[i];        //P controller
        u_i[i] = integral[i]*K_i[i];     //I controller
        u_d[i] = derivative[i]*K_d[i];   //D controller
        output[i] = u_p[i]+u_i[i]+u_d[i];
        // cout << "-----------------------------------------------------------------------" << endl;
        // cout << "error[" << i << "]=" << error[i] << " last_error[" << i << "]=" << last_error[i] << endl;
        // cout << "iteration_time=" << iteration_time << " integral[" << i << "]=" << integral[i] << endl;
        // cout << "output[" << i << "]=" << output[i] << " u_p[" << i << "]=" << u_p[i] << " u_i[" << i << "]=" << u_i[i] << " u_d[" << i << "]=" << u_d[i] << endl;
    }
    for (int i=0; i<3; i++){
        if(output[i] >  1){ output[i]=  1;}
        if(output[i] < -1){ output[i]= -1;}
    }
    last_error = error;
    Last_time = ros::Time::now().toSec();
    return(output);
}
void uav_state_sub(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void uav_pose_sub(const geometry_msgs::PoseStamped::ConstPtr& pose){
    UAV_pose_sub.pose.position.x = pose->pose.position.x;
    UAV_pose_sub.pose.position.y = pose->pose.position.y;
    UAV_pose_sub.pose.position.z = pose->pose.position.z;
    UAV_pose_sub.pose.orientation.w = pose->pose.orientation.w;
    UAV_pose_sub.pose.orientation.x = pose->pose.orientation.x;
    UAV_pose_sub.pose.orientation.y = pose->pose.orientation.y;
    UAV_pose_sub.pose.orientation.z = pose->pose.orientation.z;
    UAV_lp << UAV_pose_sub.pose.position.x,UAV_pose_sub.pose.position.y,UAV_pose_sub.pose.position.z,
              UAV_pose_sub.pose.orientation.w,UAV_pose_sub.pose.orientation.x,UAV_pose_sub.pose.orientation.y,UAV_pose_sub.pose.orientation.z;
}
void uav_pose_pub(Vec7 posepub){
    UAV_pose_pub.header.frame_id = "world";
    UAV_pose_pub.pose.position.x = posepub[0];
    UAV_pose_pub.pose.position.y = posepub[1];
    UAV_pose_pub.pose.position.z = posepub[2];
    UAV_pose_pub.pose.orientation.w = posepub[3];
    UAV_pose_pub.pose.orientation.x = posepub[4];
    UAV_pose_pub.pose.orientation.y = posepub[5];
    UAV_pose_pub.pose.orientation.z = posepub[6];
}
void uav_twist_pub(Vec4 vxyzaz){
    UAV_twist_pub.linear.x = vxyzaz(0);
    UAV_twist_pub.linear.y = vxyzaz(1);
    UAV_twist_pub.linear.z = vxyzaz(2);
    UAV_twist_pub.angular.z= vxyzaz(3);
}
void uav_pub(bool pubpose, bool pubtwist){
    if(pubpose){
        Vec8 traj1_deque_front = trajectory1.front();
        while (ros::Time::now().toSec() - traj1_deque_front[0] > 0){
            trajectory1.pop_front();
            traj1_deque_front = trajectory1.front();
        }
        Vec7 uavposepub;
        uavposepub << traj1_deque_front[1],traj1_deque_front[2],traj1_deque_front[3],
                        traj1_deque_front[4],traj1_deque_front[5],traj1_deque_front[6],traj1_deque_front[7];
        uav_pose_pub(uavposepub);
        if (traj1_deque_front[0] > traj1_information[1]){
            Mission_stage++;
            trajectory1.clear();
            uav_pose_pub(Zero7);
        }
    }
    if(pubtwist){  //Use PID position controller
        Quaterniond localq(UAV_lp[3],UAV_lp[4],UAV_lp[5],UAV_lp[6]);
        Vec3 localrpy = Q2rpy(localq);
        Vec4 xyzyaw;
        xyzyaw << UAV_pose_sub.pose.position.x,UAV_pose_sub.pose.position.y,UAV_pose_sub.pose.position.z,localrpy[2];
        if(Mission_state == 7){  //Follow the UGV
            Quaterniond UGVq(UGV_lp[3],UGV_lp[4],UGV_lp[5],UGV_lp[6]);
            Vec3 UGVrpy = Q2rpy(UGVq);
            Vec7 UGV_pred_lp = ugv_pred_land_pose(UGV_lp,UGV_twist,1);
            Pos_setpoint << UGV_pred_lp[0],UGV_pred_lp[1],Current_stage_mission[3],UGVrpy[2];
        }
        if(Mission_state == 8){  //PID landing
            if(!Mission8init){
                Mission8init = true;
                M8start_alt = xyzyaw[2];
            }
            Quaterniond UGVq(UGV_lp[3],UGV_lp[4],UGV_lp[5],UGV_lp[6]);
            Vec3 UGVrpy = Q2rpy(UGVq);
            Vec7 UGV_pred_lp = ugv_pred_land_pose(UGV_lp,UGV_twist,0.5);
            Pos_setpoint << UGV_pred_lp[0],UGV_pred_lp[1],M8start_alt-=0.001,UGVrpy[2];
            if( sqrt(pow((UAV_lp[0]-UGV_lp[0]),2)+pow((UAV_lp[1]-UGV_lp[1]),2)) < 0.15 && sqrt(pow((UAV_lp[2]-UGV_lp[2]),2)) < 0.1 ){
                Shut_down = true;
            }
        }
        if (PID_InitTime+PID_duration < ros::Time::now().toSec()){ // EndMission
            Mission_stage++;
            uav_twist_pub(Zero4);
        }
        uav_twist_pub(uav_poistion_controller_PID(xyzyaw,Pos_setpoint));
    }
}
string armstatus(){
    if(current_state.armed){
        return("Armed   ");
    }else{
        return("Disarmed");
    }
}
string statestatus(){
    if (Mission_state == 0){
        return("Not Initialized(0)");
    }else if(Mission_state == 1){
        return("TakeOff(1)");
    }else if(Mission_state == 2){
        return("constantVtraj(2)");
    }else if(Mission_state == 3){
        return("AM_traj(3)");
    }else if(Mission_state == 4){
        return("RTL(4)");
    }else if(Mission_state == 5){
        return("Landing(5)");
    }else if(Mission_state == 6){
        return("PID_step(6)");
    }else if(Mission_state == 7){
        return("PID(7)");
    }else if(Mission_state == 8){
        return("PID_landing(8)");
    }else{
        return("System error");
    }
}
void Finite_stage_mission(){  // Main FSM
    if (Mission_stage != Current_Mission_stage){// Generate trajectory while mission stage change
        Vec8 traj1;
        Vec4 traj2;
        trajectory1.clear();
        Current_Mission_stage = Mission_stage;      //Update Current_Mission_stage
        Current_stage_mission = waypoints.at(Mission_stage-1);
        Quaterniond Targetq;
        Targetq = rpy2Q(Vec3(0,0,Current_stage_mission[4]));
        Mission_state = Current_stage_mission[0];
        if (Mission_state == 1){ //state = 1 take off with no heading change
            pubpose = true;  pubtwist = false;
            TargetPos << UAV_lp[0],UAV_lp[1],Current_stage_mission[3],Targetq.w(),Targetq.x(),Targetq.y(),Targetq.z();
            constantVtraj(UAV_lp, TargetPos, 0.3, Current_stage_mission[6]);
        }
        if (Mission_state == 2){ //state = 2; constant velocity trajectory with desired heading.
            pubpose = true;  pubtwist = false;
            TargetPos << Current_stage_mission[1],Current_stage_mission[2],Current_stage_mission[3],Targetq.w(),Targetq.x(),Targetq.y(),Targetq.z();
            constantVtraj(UAV_lp, TargetPos, Current_stage_mission[5], Current_stage_mission[6]);
        }
        if (Mission_state == 3){ //state = 3 AM_traj;
            pubpose = true;  pubtwist = false;
            // Quaterniond localq(StartPose[3],StartPose[4],StartPose[5],StartPose[6]);
            // Vec3 localrpy = Q2rpy(localq);
            // Vec3 desrpy(0,0,localrpy[2]);
            // Quaterniond desq;
            // desq = rpy2Q(desrpy);
            vector<Vector3d> WPs;
            WPs.clear();
            Vector3d StartP(UAV_lp[0],UAV_lp[1],UAV_lp[2]);
            WPs.push_back(StartP);
            Vector3d WP(Current_stage_mission[1],Current_stage_mission[2],Current_stage_mission[3]);
            WPs.push_back(WP);
            AM_traj(WPs);
        }
        if (Mission_state == 4){ //state = 4; constant velocity RTL but with altitude
            pubpose = true;  pubtwist = false;
            TargetPos << UAV_takeoffP[0],UAV_takeoffP[1],Current_stage_mission[3],Targetq.w(),Targetq.x(),Targetq.y(),Targetq.z();
            constantVtraj(UAV_lp, TargetPos, Current_stage_mission[5], Current_stage_mission[6]);
        }
        if (Mission_state == 5){ //state = 5; land.
            pubpose = true;  pubtwist = false;
            TargetPos << UAV_takeoffP[0],UAV_takeoffP[1],UAV_takeoffP[2],Targetq.w(),Targetq.x(),Targetq.y(),Targetq.z();
            constantVtraj(UAV_lp, TargetPos, Current_stage_mission[5], Current_stage_mission[6]);
        }
        if (Mission_state == 6){ //state = 6; PID constant pose position control
            pubpose = false; pubtwist = true;
            trajectory1.clear();
            Pos_setpoint << Current_stage_mission[1],Current_stage_mission[2],Current_stage_mission[3],Current_stage_mission[4];
            PID_duration = Current_stage_mission[7];
            PID_InitTime = ros::Time::now().toSec();
        }
        if (Mission_state == 7){ //state = 7; PID position control
            pubpose = false; pubtwist = true;
            trajectory1.clear();
            PID_duration = Current_stage_mission[7];
            PID_InitTime = ros::Time::now().toSec();
        }
        if (Mission_state == 8){ //state = 8; PID position control with landing
            pubpose = false; pubtwist = true;
            trajectory1.clear();
            PID_duration = Current_stage_mission[7];
            PID_InitTime = ros::Time::now().toSec();
        }
        if (Mission_state == 9){ //state = 9;
            FSM_state = 2;
        }
        if (Mission_state < 6){
            if (Current_stage_mission[7] != 0){ //Wait after finish stage.
                traj1 = trajectory1.back();
                int wpc = Current_stage_mission[7]/Trajectory_timestep;
                for (double i=0; i<wpc; i++){
                    traj1[0] += Trajectory_timestep;
                    trajectory1.push_back(traj1);
                }
            }
            /*For CPP deque safety. Default generate 10 second of hover*/
            int hovertime = 10;
            if(trajectory1.size()>0){
                traj1 = trajectory1.back();
                for (int i=0; i<(hovertime/Trajectory_timestep); i++){
                    traj1[0] += Trajectory_timestep;
                    trajectory1.push_back(traj1);
                }
                traj1_information = Vec2(ros::Time::now().toSec(), traj1[0]-hovertime);
            }
        }
        /*For Debug section plot the whole trajectory*/
        // if(trajectory1.size()>0){
        //     for (unsigned int i = 0; i < trajectory1.size(); i++){
        //     Vec8 current_traj = trajectory1.at(i);
        //     cout << "dt: " << current_traj[0] << " x: " << current_traj[1] << " y: " << current_traj[2] << " z: " << current_traj[3] << endl;
        //     }
        // }
    }
    uav_pub(pubpose,pubtwist);
}
void Finite_state_machine(){
    if(FSM_state==1){ //Free Flying (position hold while triggered)
        if(!FSM_1_init){
            FSM_1_pose = UAV_lp;
            FSM_1_init = true;
        }
        Vec7 position_hold_pose;
        Quaterniond PHq(FSM_1_pose[3],FSM_1_pose[4],FSM_1_pose[5],FSM_1_pose[6]);
        Vec3 PHrpy = Q2rpy(PHq);
        PHq = rpy2Q(Vec3(0,0,PHrpy[2]));
        position_hold_pose << UAV_lp[0],UAV_lp[1],UAV_lp[2],PHq.w(),PHq.x(),PHq.y(),PHq.z();
        uav_pose_pub(position_hold_pose);
        pubpose = true; pubtwist = false;
        // if(){ //wait until have ugv estimated pose
        //     FSM_1_init = false;
        //     FSM_state++;
        // }
    }
    if(FSM_state==2){ //Follow 1 (using GPS or vicon) stay at horizontal_dist, vertical_dist
        Vec7 FSM_2_pose;
        Quaterniond FSM2q(UGV_lp[3],UGV_lp[4],UGV_lp[5],UGV_lp[6]);
        Vec3 FSM2rpy = Q2rpy(FSM2q);
        double horizontal_dist = 0.5;
        double vertical_dist = 0.5;
        Vec2 uavxy = Vec2(UGV_lp[0]-horizontal_dist*cos(FSM2rpy[2]),UGV_lp[1]-horizontal_dist*sin(FSM2rpy[2]));
        FSM_2_pose << uavxy[0],uavxy[1],UGV_lp[2]+vertical_dist,UGV_lp[3],UGV_lp[4],UGV_lp[5],UGV_lp[6];
        uav_pose_pub(FSM_2_pose);
        pubpose = true; pubtwist = false;
        if(Mission_state != 3){
            // FSM_state++;
        }
    }
    if(FSM_state==3){ //Follow 2 (using vision)

    }
    if(FSM_state==4){ //Land trajectory

    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "FSM");
    ros::NodeHandle nh;
    ros::Subscriber ugvpose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_car/pose", 5, ugv_pose_sub);
    ros::Subscriber ugvtwist_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node/gh034_car/twist", 5, ugv_twist_sub);
    ros::Subscriber uavpose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, uav_pose_sub);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, uav_state_sub);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Publisher uav_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 5); 
    ros::Publisher uav_vel_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 5);
    ros::Publisher uav_AttitudeTarget = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",5);
    mavros_msgs::SetMode offb_set_mode,posctl_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    posctl_set_mode.request.custom_mode = "POSCTL";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    nh.getParam("/FSM_node/Force_start", Force_start);
    Zero4 << 0,0,0,0;
    Zero7 << 0,0,0,0,0,0,0;
    ros::Rate loop_rate(50); /* ROS system Hz */

    while(ros::ok()){
        /* System initailize ***************************************************/
        if (!System_init){
            System_initT = ros::Time::now().toSec();
            init_time = ros::Time::now();
            waypoints = Mission_generator(); //Generate stages
            cout << " System Initialized" << " Force_start: " << Force_start << endl;
            /* Waypoints before starting */ 
            uav_pose_pub(Zero7);
            for(int i = 10; ros::ok() && i > 0; --i){
                uav_pos_pub.publish(UAV_pose_pub);
                ros::spinOnce();
                loop_rate.sleep();
            }
            System_init = true;
        }
        /* offboard and arm ****************************************************/
        if((ros::Time::now() - init_time < ros::Duration(20.0))){
            if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5))){
                //Set Offboard trigger duration here
                uav_pos_pub.publish(UAV_pose_pub);
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }else{
                if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.5))){
                    if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                last_request = ros::Time::now();
                }
            }
        }
        /* FSM initailize */
        if (current_state.mode == "OFFBOARD" && current_state.armed && !FSMinit){
            FSMinit = true;
            Mission_stage = 1;
            UAV_takeoffP = UAV_lp;
            cout << "UAV_takeoff_Position: " << UAV_takeoffP[0] << " " << UAV_takeoffP[1] << " " << UAV_takeoffP[2] << endl;
            cout << "Mission stage = 1 Mission start!" <<endl;
        }
        if (Force_start){
            FSMinit = true;
            Mission_stage = 1;
            UAV_takeoffP = UAV_lp;
            cout << "UAV_takeoff_Position: " << UAV_takeoffP[0] << " " << UAV_takeoffP[1] << " " << UAV_takeoffP[2] << endl;
            cout << "------------------Dangerous!------------------" << endl;
            cout << "Mission stage = 1 Mission start!" << endl;
            Force_start = false;
        }
        /* FSM *****************************************************************/
        Finite_stage_mission();
        if(Failsafe_System_enable){
            failsafe();
        }
        Finite_state_machine();
        if(Shut_down){ // UAV shut down
            cout << "Warning Vehicle Shut Down" << endl;
            pubtwist = false;
            pubpose = false;
            UAV_AttitudeTarget.thrust = 0; 
            uav_AttitudeTarget.publish(UAV_AttitudeTarget);
        }
        if(pubtwist){uav_vel_pub.publish(UAV_twist_pub);}
        if(pubpose){uav_pos_pub.publish(UAV_pose_pub);}
        /*Mission information cout**********************************************/
        if(coutcounter > 25 && FSMinit && !Shut_down){ //reduce cout rate
            cout << "Status: "<< armstatus() << "    Mode: " << current_state.mode <<endl;
            cout << "Mission_Stage: " << Mission_stage << "    Mission_total_stage: " << waypoints.size() << endl;
            cout << "Mission_State: " << statestatus() << endl;
            cout << "vicon__pos_x: " << UAV_lp[0] << " y: " << UAV_lp[1] << " z: "<< UAV_lp[2] << endl;
            if(pubpose){
                cout << "desiredpos___x: " << UAV_pose_pub.pose.position.x << " y: " << UAV_pose_pub.pose.position.y << " z: "<< UAV_pose_pub.pose.position.z << endl;
                cout << "Traj countdown: " << traj1_information[1] - ros::Time::now().toSec() << endl;
            }
            if(pubtwist){
                cout << "des__twist_x: " << UAV_twist_pub.linear.x << " y: " << UAV_twist_pub.linear.y << " z: "<< UAV_twist_pub.linear.z << " az: " << UAV_twist_pub.angular.z << endl;
                cout << "PID  countdown: " << PID_InitTime+PID_duration - ros::Time::now().toSec() << endl;
            }
            cout << "CAr____pos_x: " << UGV_pose_sub.pose.position.x << " y: " << UGV_pose_sub.pose.position.y << endl;
            cout << "Dist_UAVtoUGV_horizontal: " << sqrt(pow((UAV_lp[0]-UGV_lp[0]),2)+pow((UAV_lp[1]-UGV_lp[1]),2)) << endl;
            cout << "Dist_UAVtoUGV___vertical: " << sqrt(pow((UAV_lp[2]-UGV_lp[2]),2)) << endl;
            cout << "Failsafe_System_enable: " << Failsafe_System_enable <<  " Failsafe_state: " << FailsafeFlag << endl;
            cout << "---------------------------------------------------" << endl;
            coutcounter = 0;
        }else{coutcounter++;}
        ros::spinOnce();
        loop_rate.sleep();
    }
}