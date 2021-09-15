#include <ros/ros.h>
#include <string>
#include <numeric>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "utils/kinetic_math.h"
#include "utils/mission.h"
#include "utils/trajectories.h"
#include "utils/cv.h"

static mavros_msgs::State current_state;
static geometry_msgs::PoseStamped UAV_pose_vicon;
static geometry_msgs::PoseStamped UAV_pose_pub;
static geometry_msgs::PoseStamped UGV_pose_pub;
static geometry_msgs::Twist       UAV_twist_pub;
static Vec7 UAV_lp;

/* System */
bool System_init = false;
double System_initT;
ros::Time init_time,last_request; /* Use at System start up */
static Vec7 Zero7;
static Vec4 Zero4;
static int coutcounter;
/* PID Position controller */
Vec4   Pos_setpoint;
double PID_duration;
double PID_InitTime;
/* FSM */
Vec7 UAV_desP,UAV_takeoffP;
int    Mission_state = 0;
int    Mission_stage = 0;
int    Current_Mission_stage = 0;
Vec8   Current_stage_mission;
bool   FSMinit = false;
bool   pubtwist_traj = false;
bool   pubpose_traj  = false;
bool   pubtwist      = false;
bool   Force_start   = false;

Vec4 uav_poistion_controller_PID(Vec4 pose, Vec4 setpoint){ // From Depth calculate XYZ position and yaw
    Vec4 error,last_error,u_p,u_i,u_d,output; // Position Error
    double Last_time = ros::Time::now().toSec();
    double iteration_time = ros::Time::now().toSec() - Last_time;
    Vec4 K_p(0.5,0.5,0.5,0.1);
    Vec4 K_i(0,0,0,0);
    Vec4 K_d(0,0,0,0);
    error = setpoint-pose;
    last_error = error;
    Vec4 integral = integral+(error*iteration_time);
    Vec4 derivative = (error - last_error)/iteration_time;
    for (int i=0; i<4; i++){             //i = x,y,z
        u_p[i] = error[i]*K_p[i];        //P controller
        u_i[i] = integral[i]*K_i[i];     //I controller
        u_d[i] = derivative[i]*K_d[i];   //D controller
        output[i] = u_p[i]+u_i[i]+u_d[i];
    }
    for (int i=0; i<3; i++){
        if(output[i] >  1.5){ output[i]= 1.5;}
        if(output[i] < -1.5){ output[i]= -1.5;}
        if(output[i] >  3){ output[i]= 0;}
        if(output[i] < -3){ output[i]= 0;}
    }
    return(output);
}
void uav_state_sub(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void uav_pose_sub(const geometry_msgs::PoseStamped::ConstPtr& pose){
    UAV_pose_vicon.pose.position.x = pose->pose.position.x;
    UAV_pose_vicon.pose.position.y = pose->pose.position.y;
    UAV_pose_vicon.pose.position.z = pose->pose.position.z;
    UAV_pose_vicon.pose.orientation.w = pose->pose.orientation.w;
    UAV_pose_vicon.pose.orientation.x = pose->pose.orientation.x;
    UAV_pose_vicon.pose.orientation.y = pose->pose.orientation.y;
    UAV_pose_vicon.pose.orientation.z = pose->pose.orientation.z;
    UAV_lp << UAV_pose_vicon.pose.position.x,UAV_pose_vicon.pose.position.y,UAV_pose_vicon.pose.position.z,
              UAV_pose_vicon.pose.orientation.w,UAV_pose_vicon.pose.orientation.x,UAV_pose_vicon.pose.orientation.y,UAV_pose_vicon.pose.orientation.z;
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
void UAV_pub(bool pubtwist_traj, bool pubpose_traj, bool pubtwist){
    if(pubpose_traj){
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
    if(pubtwist_traj){
        Vec4 traj2_deque_front = Twisttraj.front();
        while (ros::Time::now().toSec() - traj2_deque_front[0] > 0){
            Twisttraj.pop_front();
            traj2_deque_front = Twisttraj.front();
        }
        uav_twist_pub(Vec4(traj2_deque_front[1],traj2_deque_front[2],traj2_deque_front[3],traj2_deque_front[4]));
        if (traj2_deque_front[0] > Twisttraj_information[1]){
            Mission_stage++;
            Twisttraj.clear();
            uav_twist_pub(Zero4);
        }
    }
    if(pubtwist){
        Quaterniond localq(UAV_lp[3],UAV_lp[4],UAV_lp[5],UAV_lp[6]);
        Vec3 localrpy = Q2rpy(localq);
        Vec4 xyzyaw;
        xyzyaw << UAV_pose_vicon.pose.position.x,UAV_pose_vicon.pose.position.y,UAV_pose_vicon.pose.position.z,localrpy[2];
        uav_twist_pub(uav_poistion_controller_PID(xyzyaw,Pos_setpoint));
        if (PID_InitTime+PID_duration < ros::Time::now().toSec()){
            Mission_stage++;
            uav_twist_pub(Zero4);
        }
    }
}
void pose_pub_ugv(Vec2 xy){
    UGV_pose_pub.header.frame_id = "world";
    UGV_pose_pub.pose.position.x = xy[0];
    UGV_pose_pub.pose.position.y = xy[1];
    UGV_pose_pub.pose.position.z = 0;
    UGV_pose_pub.pose.orientation.w = 0;
    UGV_pose_pub.pose.orientation.x = 0;
    UGV_pose_pub.pose.orientation.y = 0;
    UGV_pose_pub.pose.orientation.z = 0;
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
        return("Twist(3)");
    }else if(Mission_state == 4){
        return("RTL(4)");
    }else if(Mission_state == 5){
        return("Landing(5)");
    }else if(Mission_state == 6){
        return("PID(6)");
    }else{
        return("System error");
    }
}
void Finite_state_WP_mission(){ 
    // Generate trajectory while mission stage change
    if (Mission_stage != Current_Mission_stage){
        Vec8 traj1;
        Vec4 traj2;
        Vec7 TargetPos;
        Current_Mission_stage = Mission_stage;  //Update Current_Mission_stage
        Current_stage_mission = waypoints.at(Mission_stage-1);
        Quaterniond Targetq;
        Targetq = rpy2Q(Vec3(0,0,Current_stage_mission[4]));
        Mission_state = Current_stage_mission[0];
        if (Mission_state == 1){ //state = 1 take off with no heading change
            TargetPos << UAV_lp[0],UAV_lp[1],Current_stage_mission[3],Targetq.w(),Targetq.x(),Targetq.y(),Targetq.z();
            constantVtraj(UAV_lp, TargetPos, 0.1, Current_stage_mission[6]);
        }
        if (Mission_state == 2){ //state = 2; constant velocity trajectory with desired heading.
            TargetPos << Current_stage_mission[1],Current_stage_mission[2],Current_stage_mission[3],Targetq.w(),Targetq.x(),Targetq.y(),Targetq.z();
            constantVtraj(UAV_lp, TargetPos, Current_stage_mission[5], Current_stage_mission[6]);
        }
        if (Mission_state == 3){ //state = 3; For Twist test
            gen_twist_traj(Vec4(Current_stage_mission[1],Current_stage_mission[2],Current_stage_mission[3],Current_stage_mission[4]),Current_stage_mission[5]);
        }
        if (Mission_state == 4){ //state = 4; constant velocity RTL but with altitude
            TargetPos << UAV_takeoffP[0],UAV_takeoffP[1],UAV_lp[2],Targetq.w(),Targetq.x(),Targetq.y(),Targetq.z();
            constantVtraj(UAV_lp, TargetPos, Current_stage_mission[5], Current_stage_mission[6]);
        }
        if (Mission_state == 5){ //state = 5; land.
            TargetPos << UAV_takeoffP[0],UAV_takeoffP[1],UAV_takeoffP[2],Targetq.w(),Targetq.x(),Targetq.y(),Targetq.z();
            constantVtraj(UAV_lp, TargetPos, Current_stage_mission[5], Current_stage_mission[6]);
        }
        if (Mission_state == 6){ //state = 6; PID twist Aruco position hold.
            pubtwist_traj = false; pubpose_traj = false; pubtwist = true;
            Pos_setpoint << Current_stage_mission[1],Current_stage_mission[2],Current_stage_mission[3],Current_stage_mission[3];
            PID_duration = Current_stage_mission[5];
            PID_InitTime = ros::Time::now().toSec();
        }
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
            pubpose_traj = true; pubtwist_traj = false; pubtwist = false;
        }
        if(Twisttraj.size()>0){
            traj2 = Twisttraj.back();
            for (int i=0; i<(hovertime/Trajectory_timestep); i++){
                traj2[0] += Trajectory_timestep;
                Twisttraj.push_back(traj2);
            }
            Twisttraj_information = Vec2(ros::Time::now().toSec(), traj2[0]-hovertime);
            pubpose_traj = false; pubtwist_traj = true; pubtwist = false;
        }
        /*For Debug section plot the whole trajectory*/ 
        // int trajectorysize = trajectory1.size();
        // for (int i = 0; i < trajectorysize; i++){
        //   Vec8 current_traj = trajectory1.at(i);
        //   cout << "dt: " << current_traj[0] << " x: " << current_traj[1] << " y: " << current_traj[2] << " z: " << current_traj[3] << endl;
        // }
        /*For also contorl car section*/
        if(pubpose_traj){
            if(Mission_state != 1||Mission_state != 6){
               pose_pub_ugv(Vec2(TargetPos[0],TargetPos[1]));
            }
        }
    }
    
    UAV_pub(pubtwist_traj,pubpose_traj,pubtwist);
    /*Mission information cout*********************************************/
    if(coutcounter > 10){ //reduce cout rate
        cout << "------------------------------------------------------------------------------" << endl;
        cout << "Status: "<< armstatus() << "    Mode: " << current_state.mode <<endl;
        cout << "Mission_Stage: " << Mission_stage << "    Mission_total_stage: " << waypoints.size() << endl;
        cout << "Mission_State: " << statestatus() << endl;
        cout << "vicon__pos_x: " << UAV_lp[0] << " y: " << UAV_lp[1] << " z: "<< UAV_lp[2] << endl;
        if(pubpose_traj){
        cout << "desiredpos_x: " << UAV_pose_pub.pose.position.x << " y: " << UAV_pose_pub.pose.position.y << " z: "<< UAV_pose_pub.pose.position.z << endl;}
        if(pubtwist){
        cout << "desiredtwist_x: " << UAV_twist_pub.linear.x << " y: " << UAV_twist_pub.linear.y << " z: "<< UAV_twist_pub.linear.z << " az: " << UAV_twist_pub.angular.z << endl;}
        cout << "CAr____pos_x: " << UGV_pose_pub.pose.position.x << " y: " << UGV_pose_pub.pose.position.y << endl;
        cout << "Trajectory timer countdown: " << traj1_information[1] - ros::Time::now().toSec() << endl;
        cout << "ROS_time: " << fixed << ros::Time::now().toSec() << endl;
        cout << "traj1_size: " << trajectory1.size() << "  traj2_size: " << Twisttraj.size() << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        coutcounter = 0;
    }else{coutcounter++;}
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "FSM");
    ros::NodeHandle nh;
    ros::Subscriber uavpose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/gh034_led/mavros/local_position/pose", 1, uav_pose_sub);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/gh034_led/mavros/state", 10, uav_state_sub);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/gh034_led/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/gh034_led/mavros/set_mode");
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/gh034_led/mavros/setpoint_position/local", 10);
    ros::Publisher ugv_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/scout_wp/pose", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>("/gh034_led/mavros/setpoint_velocity/cmd_vel_unstamped", 100);
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::SetMode posctl_set_mode;
    posctl_set_mode.request.custom_mode = "POSCTL";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    Zero4 << 0,0,0,0;
    Zero7 << 0,0,0,0,0,0,0;

    ros::Rate loop_rate(50); /* ROS system Hz */
    
    while(ros::ok()){
        /* System initailize ***************************************************/
        if (!System_init){
            System_initT = ros::Time::now().toSec();
            init_time = ros::Time::now();
            waypoints = Finite_stage_mission(); //Generate stages
            cout << " System Initialized" << endl;
            /* Waypoints before starting */ 
            uav_pose_pub(Zero7);
            for(int i = 10; ros::ok() && i > 0; --i){
                local_pos_pub.publish(UAV_pose_pub);
                ros::spinOnce();
                loop_rate.sleep();
            }
            System_init = true;
        }
        /* offboard and arm ****************************************************/
        if((ros::Time::now() - init_time < ros::Duration(20.0))){
            if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5))){
                //Set Offboard trigger duration here
                local_pos_pub.publish(UAV_pose_pub);
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
        Finite_state_WP_mission();
        if(pubtwist_traj || pubtwist){local_vel_pub.publish(UAV_twist_pub);}
        if(pubpose_traj){local_pos_pub.publish(UAV_pose_pub);}
        ros::spinOnce();
        loop_rate.sleep();
    }
}