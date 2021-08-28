#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <chrono>
#include <iomanip>
#include <string>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <istream>
#include <sstream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <numeric>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "utils/kinetic_math.h"
#include "utils/mission.h"
#include "utils/trajectories.h"
#include "utils/cv.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

/*camera pramater*/
static double fx, fy, cx, cy; //focal length and principal point
static Vec4 CamParameters;
int ArucoLostcounter;
bool Aruco_found = false;
bool Aruco_init = false;
/*uav local parameter*/
static geometry_msgs::PoseStamped Aruco_pose_realsense;
static geometry_msgs::PoseStamped Depth_pose_realsense;
static geometry_msgs::PoseStamped UAV_pose_vicon;
static geometry_msgs::PoseStamped Camera_pose_vicon;
static geometry_msgs::PoseStamped UAV_pose_pub;
static geometry_msgs::Twist       UAV_twist_pub;
Vec7 UAV_lp;
/*IRR filter parameter*/
cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);
cv::Mat depthcameraMatrix = cv::Mat::eye(3,3, CV_64F);
cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
/* FSM */
Vec7 UAV_desP,UAV_takeoffP;
int    Mission_state = 0;
int    Mission_stage = 0;
int    Current_Mission_stage = 0;
Vec8   Current_stage_mission;
Vec8   Last_stage_mission;
bool   FSMinit = false;
bool   pubtwist_traj = false;
bool   pubpose_traj  = false;
bool   pubtwist      = false;
/* PID Position controller */
Vec4   Pos_setpoint;
double PID_duration;
double PID_InitTime;
/* System */
bool UAV = false;
bool ROS_init = true;
double System_initT,LastT;
ros::Time last_request;
ros::Time init_time;
mavros_msgs::State current_state;
int coutcounter = 0;
static Vec7 Zero7;
static Vec4 Zero4;

Vec4 Poistion_controller_PID(Vec4 pose, Vec4 setpoint){ // From Depth calculate XYZ position and yaw
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
void twist_pub(Vec4 vxyzaz){
    UAV_twist_pub.linear.x = vxyzaz(0);
    UAV_twist_pub.linear.y = vxyzaz(1);
    UAV_twist_pub.linear.z = vxyzaz(2);
    UAV_twist_pub.angular.z= vxyzaz(3);
}
void pose_pub(Vec7 posepub){
    UAV_pose_pub.header.frame_id = "world";
    UAV_pose_pub.pose.position.x = posepub[0];
    UAV_pose_pub.pose.position.y = posepub[1];
    UAV_pose_pub.pose.position.z = posepub[2];
    UAV_pose_pub.pose.orientation.w = posepub[3];
    UAV_pose_pub.pose.orientation.x = posepub[4];
    UAV_pose_pub.pose.orientation.y = posepub[5];
    UAV_pose_pub.pose.orientation.z = posepub[6];
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
        pose_pub(uavposepub);
        if (traj1_deque_front[0] > traj1_information[1]){
            Mission_stage++;
            trajectory1.clear();
            pose_pub(Zero7);
        }
    }
    if(pubtwist_traj){
        Vec4 traj2_deque_front = Twisttraj.front();
        while (ros::Time::now().toSec() - traj2_deque_front[0] > 0){
            Twisttraj.pop_front();
            traj2_deque_front = Twisttraj.front();
        }
        twist_pub(Vec4(traj2_deque_front[1],traj2_deque_front[2],traj2_deque_front[3],traj2_deque_front[4]));
        if (traj2_deque_front[0] > Twisttraj_information[1]){
            Mission_stage++;
            Twisttraj.clear();
            twist_pub(Zero4);
        }
    }
    if(pubtwist){
        Quaterniond localq(UAV_lp[3],UAV_lp[4],UAV_lp[5],UAV_lp[6]);
        Vec3 localrpy = Q2rpy(localq);
        Vec4 xyzyaw;
        xyzyaw << UAV_pose_vicon.pose.position.x,UAV_pose_vicon.pose.position.y,UAV_pose_vicon.pose.position.z,localrpy[2];
        twist_pub(Poistion_controller_PID(xyzyaw,Pos_setpoint));
        if (PID_InitTime+PID_duration < ros::Time::now().toSec()){
            Mission_stage++;
            twist_pub(Zero4);
        }
    }
}
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void camera_info_cb(const sensor_msgs::CameraInfoPtr& msg){
    fx = msg->K[0];
    fy = msg->K[4];
    cx = msg->K[2];
    cy = msg->K[5];
    cameraMatrix.at<double>(0,0) = fx;
    cameraMatrix.at<double>(1,1) = fy;
    cameraMatrix.at<double>(0,2) = cx;
    cameraMatrix.at<double>(1,2) = cy;
    CamParameters << fx,fy,cx,cy;
}
void CameraPose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
    Camera_pose_vicon.pose.position.x = pose->pose.position.x;
    Camera_pose_vicon.pose.position.y = pose->pose.position.y;
    Camera_pose_vicon.pose.position.z = pose->pose.position.z;
    Camera_pose_vicon.pose.orientation.x = pose->pose.orientation.x;
    Camera_pose_vicon.pose.orientation.y = pose->pose.orientation.y;
    Camera_pose_vicon.pose.orientation.z = pose->pose.orientation.z;
    Camera_pose_vicon.pose.orientation.w = pose->pose.orientation.w;
}
void UAVPose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
    UAV_pose_vicon.pose.position.x = pose->pose.position.x;
    UAV_pose_vicon.pose.position.y = pose->pose.position.y;
    UAV_pose_vicon.pose.position.z = pose->pose.position.z;
    UAV_pose_vicon.pose.orientation.x = pose->pose.orientation.x;
    UAV_pose_vicon.pose.orientation.y = pose->pose.orientation.y;
    UAV_pose_vicon.pose.orientation.z = pose->pose.orientation.z;
    UAV_pose_vicon.pose.orientation.w = pose->pose.orientation.w;
    UAV_lp << UAV_pose_vicon.pose.position.x,UAV_pose_vicon.pose.position.y,UAV_pose_vicon.pose.position.z,
              UAV_pose_vicon.pose.orientation.x,UAV_pose_vicon.pose.orientation.y,UAV_pose_vicon.pose.orientation.z,UAV_pose_vicon.pose.orientation.w;
}
void Aruco_PosePub(Vec6 rpyxyz){
    // Quaterniond Q = rpy2Q(Vec3(rpyxyz(0),rpyxyz(1),rpyxyz(2)));
    Quaterniond Q = rpy2Q(Vec3(0,0,3.14));
    Aruco_pose_realsense.header.stamp = ros::Time::now();
    Aruco_pose_realsense.header.frame_id = "world";
    Aruco_pose_realsense.pose.position.x = rpyxyz(3);
    Aruco_pose_realsense.pose.position.y = rpyxyz(4);
    Aruco_pose_realsense.pose.position.z = rpyxyz(5);
    Aruco_pose_realsense.pose.orientation.w = Q.w();
    Aruco_pose_realsense.pose.orientation.x = Q.x();
    Aruco_pose_realsense.pose.orientation.y = Q.y();
    Aruco_pose_realsense.pose.orientation.z = Q.z();
}
void Depth_PosePub(Vec6 rpyxyz){
    Quaterniond Q = rpy2Q(Vec3(rpyxyz(0),rpyxyz(1),rpyxyz(2)));
    Depth_pose_realsense.header.stamp = ros::Time::now();
    Depth_pose_realsense.header.frame_id = "world";
    Depth_pose_realsense.pose.position.x = rpyxyz(3);
    Depth_pose_realsense.pose.position.y = rpyxyz(4);
    Depth_pose_realsense.pose.position.z = rpyxyz(5);
    Depth_pose_realsense.pose.orientation.w = Q.w();
    Depth_pose_realsense.pose.orientation.x = Q.x();
    Depth_pose_realsense.pose.orientation.y = Q.y();
    Depth_pose_realsense.pose.orientation.z = Q.z();
}
Vec6 Pose_calc(const Vec3 rvecs, const Vec3 tvecs){ // camera coordinate to world coordinate
    Eigen::Quaterniond q;
    q.w() = Camera_pose_vicon.pose.orientation.w;
    q.x() = Camera_pose_vicon.pose.orientation.x;
    q.y() = Camera_pose_vicon.pose.orientation.y;
    q.z() = Camera_pose_vicon.pose.orientation.z;
    Eigen::Matrix3d Camera_Rotation_world = Eigen::Matrix3d::Identity();
    Camera_Rotation_world = q.matrix();
    Vec3 Cam_Translation_world(Camera_pose_vicon.pose.position.x, Camera_pose_vicon.pose.position.y, Camera_pose_vicon.pose.position.z);
    Vec3 translation_world = Camera_Rotation_world * tvecs + Cam_Translation_world;
    Vec3 rpy_world = Camera_Rotation_world*rvecs;
    Vec6 output;
    output << rpy_world[0],rpy_world[1],rpy_world[2],
              translation_world[0],translation_world[1],translation_world[2];
    return(output);
}
void callback(const sensor_msgs::CompressedImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth){
    // cout<<"hello callback "<<endl;
    cv::Mat image_rgb;
    try{
        image_rgb = cv::imdecode(cv::Mat(rgb->data),1);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    /* Aruco */
    if (Aruco_init == false){
        CVE_Corners.clear();
        Vec8I CVE_Corner;
        CVE_Corner << 0,0,0,0,0,0,0,0;
        for(int i=0; i<5; i++){CVE_Corners.push_back(CVE_Corner);}
    }
    cv::Mat ArucoOutput = image_rgb.clone();
    std::vector<int> markerIds;
    std::vector<Vec8I> markerConerABCDs;
    Vec2I markerCenter,last_markerCenter;
    Vec8I markerConerABCD;
    Vec8I last_markerConerABCD;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    std::vector<cv::Point2f> markerCorner;
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::Vec3d rvec, tvec;
    rvecs.clear();tvecs.clear();
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(image_rgb, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    if (markerIds.size() > 0){
        markerConerABCDs.clear();
        Aruco_init = true;
        Aruco_found = true;
        cv::aruco::drawDetectedMarkers(ArucoOutput, markerCorners, markerIds);
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.06, cameraMatrix, distCoeffs, rvecs, tvecs);
        for(unsigned int i=0; i<markerIds.size(); i++){
            cv::aruco::drawAxis(ArucoOutput, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
            markerCorner = markerCorners[i];
            for (unsigned int j=0; j<markerCorner.size();j++){
                cv::Point2f MC = markerCorner[j];
                markerConerABCD[j*2] = MC.x;
                markerConerABCD[j*2+1] = MC.y;
            }
            markerConerABCDs.push_back(markerConerABCD);
        }
    }else{Aruco_found = false; ArucoLostcounter++;}
    /* Pose in World Calc */
    if (Aruco_init){
        cv_bridge::CvImagePtr depth_ptr  = cv_bridge::toCvCopy(depth, depth->encoding);
        cv::Mat image_dep = depth_ptr->image;
        Vec3 Depthrvecs;
        if(Aruco_found){
            rvec = rvecs.front();
            tvec = tvecs.front();
            Vec3 Aruco_translation_camera(tvec(0),tvec(1),tvec(2));
            Vec3 Aruco_rpy_camera(rvec(0),rvec(1),rvec(2));
            Aruco_PosePub(Pose_calc(Aruco_rpy_camera,Aruco_translation_camera));
            Depthrvecs = Aruco_rpy_camera;
            double ArucoDepth = find_depth_avg(image_dep,markerConerABCDs.back());
            last_markerConerABCD = markerConerABCDs.back();
            Depth_PosePub(Pose_calc(Depthrvecs,camerapixel2tvec(Constant_velocity_predictor(last_markerConerABCD,ArucoLostcounter),ArucoDepth,CamParameters)));
            ArucoLostcounter = 0;
        }else{ //Aruco not found do constant-velocity predict
            double ArucoDepth = find_depth_avg(image_dep,last_markerConerABCD);
            Depth_PosePub(Pose_calc(Depthrvecs,camerapixel2tvec(Constant_velocity_predictor(last_markerConerABCD,ArucoLostcounter),ArucoDepth,CamParameters)));
        }
    }
    
    /* SolvePNP test */
    // cv::Vec3d PNPrvec, PNPtvec;
    // vector<cv::Point3f> PNPPoints3D;
    // vector<cv::Point2f> PNPPoints2D = markerCorner;
    // PNPPoints3D.clear();
    // PNPPoints3D.push_back(cv::Point3f( 0, 0, 0));
    // PNPPoints3D.push_back(cv::Point3f( 0,60, 0));
    // PNPPoints3D.push_back(cv::Point3f(60,60, 0));
    // PNPPoints3D.push_back(cv::Point3f(60, 0, 0));
    // if (Aruco_found){
    //     solvePnP(PNPPoints3D, PNPPoints2D, cameraMatrix, distCoeffs, PNPrvec, PNPtvec, false, SOLVEPNP_ITERATIVE);
    //     cout << "Aruco Tvec: " << tvec*1000 << endl;
    //     cout << "PNP   Tvec: " << PNPtvec << endl;
    // }


    // cv::imwrite("E1S100NEW.jpg",image_rgb);
    /* image plot */
    // cv::Mat depImage = image_dep.clone();
    // cv::imshow("dep_out", depImage);
    cv::imshow("Aruco_out", ArucoOutput);
    cv::waitKey(1);
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
    Last_stage_mission = Current_stage_mission;
    Current_stage_mission = waypoints.at(Mission_stage-1);
    Quaterniond Targetq;
    Mission_state = Current_stage_mission[0];
    if (Mission_state == 1){ //state = 1 take off with no heading change
        Targetq = rpy2Q(Vec3(0,0,Current_stage_mission[4]));
        TargetPos << UAV_lp[0],UAV_lp[1],Current_stage_mission[3],Targetq.w(),Targetq.x(),Targetq.y(),Targetq.z();
        constantVtraj(UAV_lp, TargetPos, 0.1, Current_stage_mission[6]);
    }
    if (Mission_state == 2){ //state = 2; constant velocity trajectory with desired heading.
        Targetq = rpy2Q(Vec3(0,0,Current_stage_mission[4]));
        TargetPos << Current_stage_mission[1],Current_stage_mission[2],Current_stage_mission[3],Targetq.w(),Targetq.x(),Targetq.y(),Targetq.z();
        constantVtraj(UAV_lp, TargetPos, Current_stage_mission[5], Current_stage_mission[6]);
    }
    if (Mission_state == 3){ //state = 3; For Twist test
        gen_twist_traj(Vec4(Current_stage_mission[1],Current_stage_mission[2],Current_stage_mission[3],Current_stage_mission[4]),Current_stage_mission[5]);
    }
    if (Mission_state == 4){ //state = 4; constant velocity RTL but with altitude
        Targetq = rpy2Q(Vec3(0,0,Current_stage_mission[4]));
        TargetPos << UAV_takeoffP[0],UAV_takeoffP[1],UAV_lp[2],Targetq.w(),Targetq.x(),Targetq.y(),Targetq.z();
        constantVtraj(UAV_lp, TargetPos, Current_stage_mission[5], Current_stage_mission[6]);
    }
    if (Mission_state == 5){ //state = 5; land.
        Targetq = rpy2Q(Vec3(0,0,Current_stage_mission[4]));
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
  }
  UAV_pub(pubtwist_traj,pubpose_traj,pubtwist);
  /*Mission information cout*********************************************/
    if(coutcounter > 10){ //reduce cout rate
        cout << "------------------------------------------------------------------------------" << endl;
        cout << "Status: "<< armstatus() << "    Mode: " << current_state.mode <<endl;
        cout << "Mission_Stage: " << Mission_stage << "    Mission_total_stage: " << waypoints.size() << endl;
        cout << "Mission_State: " << statestatus() << endl;
        cout << "aruco__pos_x: " << Aruco_pose_realsense.pose.position.x << " y: " << Aruco_pose_realsense.pose.position.y << " z: "<< Aruco_pose_realsense.pose.position.z << endl;
        // cout << "depth__pos_x: " << Depth_pose_realsense.pose.position.x << " y: " << Depth_pose_realsense.pose.position.y << " z: "<< Depth_pose_realsense.pose.position.z << endl;
        cout << "local__pos_x: " << UAV_lp[0] << " y: " << UAV_lp[1] << " z: "<< UAV_lp[2] << endl;
        cout << "desiredpos_x: " << UAV_pose_pub.pose.position.x << " y: " << UAV_pose_pub.pose.position.y << " z: "<< UAV_pose_pub.pose.position.z << endl;
        cout << "desiredtwist_x: " << UAV_twist_pub.linear.x << " y: " << UAV_twist_pub.linear.y << " z: "<< UAV_twist_pub.linear.z << " az: " << UAV_twist_pub.angular.z << endl;
        cout << "Trajectory timer countdown: " << traj1_information[1] - ros::Time::now().toSec() << endl;
        cout << "ROS_time: " << fixed << ros::Time::now().toSec() << endl;
        cout << "traj1_size: " << trajectory1.size() << "  traj2_size: " << Twisttraj.size() << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        coutcounter = 0;
    }else{coutcounter++;}
}
int main(int argc, char **argv){
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);
    ros::Subscriber camerapose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_l515/pose", 1, CameraPose_cb);
    ros::Subscriber uavpose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, UAVPose_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::Publisher ArucoPose_pub = nh.advertise<geometry_msgs::PoseStamped>("ArucoPose",1);
    ros::Publisher DepthPose_pub = nh.advertise<geometry_msgs::PoseStamped>("DepthPose",1);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 100);
    message_filters::Subscriber<CompressedImage> rgb_sub(nh, "/camera/color/image_raw/compressed", 1);
    message_filters::Subscriber<Image> dep_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    mavros_msgs::SetMode offb_set_mode; 
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::SetMode posctl_set_mode;
    posctl_set_mode.request.custom_mode = "POSCTL";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Rate loop_rate(50); /* ROS system Hz */
    typedef sync_policies::ApproximateTime<CompressedImage, Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, dep_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    Zero4 << 0,0,0,0;
    Zero7 << 0,0,0,0,0,0,0;

    

    while(ros::ok()){
        if (UAV){
        if (ROS_init){
            System_initT = ros::Time::now().toSec();
            init_time = ros::Time::now();
            ROS_init = false;
            waypoints = Finite_stage_mission(); //Generate stages
            cout << " System Initialized" << endl;
            /* Waypoints before starting */ 
            pose_pub(Zero7);
            for(int i = 10; ros::ok() && i > 0; --i){
                local_pos_pub.publish(UAV_pose_pub);
                ros::spinOnce();
                loop_rate.sleep();
            }
        }
        /*offboard and arm*****************************************************/
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5)) && (ros::Time::now() - init_time < ros::Duration(20.0))){ 
            //Set Offboard trigger duration here
            local_pos_pub.publish(UAV_pose_pub);
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }else{
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.5)) && (ros::Time::now() - init_time < ros::Duration(20.0))){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
            last_request = ros::Time::now();
            }
        }
        /*FSM******************************************************************/
        if (current_state.mode == "OFFBOARD" && current_state.armed && !FSMinit){
            FSMinit = true;
            Mission_stage = 1;
            UAV_takeoffP = UAV_lp;
            cout << "UAV_takeoff_Position: " << UAV_takeoffP[0] << " " << UAV_takeoffP[1] << " " << UAV_takeoffP[2] << endl;
            cout << "Mission stage = 1 Mission start!" <<endl;
        }
        Finite_state_WP_mission();
        if(pubtwist_traj || pubtwist){
            local_vel_pub.publish(UAV_twist_pub);
        }
        if(pubpose_traj){
            local_pos_pub.publish(UAV_pose_pub);
        }}
        
        imageprocess();
        ArucoPose_pub.publish(Aruco_pose_realsense);
        DepthPose_pub.publish(Depth_pose_realsense);
        /* ROS timer */
        // auto currentT = ros::Time::now().toSec();
        // cout << "System_Hz: " << 1/(currentT-LastT) << endl;
        // LastT = currentT;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}