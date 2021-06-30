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
#include <numeric>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "utils/kinetic_math.h"

using namespace std;
using namespace cv;
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
Vec7 UAV_lp;
/*IRR filter parameter*/
cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);
cv::Mat depthcameraMatrix = cv::Mat::eye(3,3, CV_64F);
cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
/* Constant velocity estimator */
std::deque<Vec8I> CVE_Corners;
/* FSM */
Vec7 UAV_desP,UAV_takeoffP;
int    Mission_state = 0;
int    Mission_stage = 0;
int    Current_Mission_stage = 0;
double velocity_takeoff,velocity_angular, velocity_mission, altitude_mission;
bool   UAV_flying = false;
/* Traj */
// std::deque<Vec8> trajectory1;
Vec2 traj1_information;
double Trajectory_timestep = 0.02;
/* System */
bool ROS_init = true;
double System_initT,callback_LastT,System_LastT;
ros::Time last_request;
ros::Time init_time;
int LowSpeedcounter;
mavros_msgs::State current_state;

/*void constantVtraj( Vec7 EndPose,double velocity,double angular_velocity){
  Quaterniond startq(UAV_lp[3],UAV_lp[4],UAV_lp[5],UAV_lp[6]);
  Vec3 start_rpy = Q2rpy(startq);
  Vec3 start_xyz(UAV_lp[0],UAV_lp[1],UAV_lp[2]);
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

  //initialize trajectory1
  trajectory1.clear();
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
}*/
/*void traj_pub(){
  double current_time = ros::Time::now().toSec();
  Vec8 traj1_deque_front = trajectory1.front();

  while (current_time - traj1_deque_front[0] > 0){
    trajectory1.pop_front();
    traj1_deque_front = trajectory1.front();
  }
  
  UAV_pose_pub.header.frame_id = "world";
  UAV_pose_pub.pose.position.x = traj1_deque_front[1];
  UAV_pose_pub.pose.position.y = traj1_deque_front[2];
  UAV_pose_pub.pose.position.z = traj1_deque_front[3];
  UAV_pose_pub.pose.orientation.w = traj1_deque_front[4];
  UAV_pose_pub.pose.orientation.x = traj1_deque_front[5];
  UAV_pose_pub.pose.orientation.y = traj1_deque_front[6];
  UAV_pose_pub.pose.orientation.z = traj1_deque_front[7];

  // Trajectory current time > duration than goes on to next stage
  if (traj1_deque_front[0] > traj1_information[1]){ Mission_stage++;}
}*/
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
    UAV_lp << UAV_pose_vicon.pose.position.x,UAV_pose_vicon.pose.position.y,UAV_pose_vicon.pose.position.z,UAV_pose_vicon.pose.orientation.x,UAV_pose_vicon.pose.orientation.y,UAV_pose_vicon.pose.orientation.z,UAV_pose_vicon.pose.orientation.w;
}
/*void depth_calc(cv::Mat rgbframe, cv::Mat depthframe){
    // auto depthsize = frame.size;
    // cout << "size: " << depthsize << endl;
    // auto depth = 0.001 * frame.at<ushort>(640,360);
    // cout << "depth: " << depth << endl;
    auto depthsizeR = depthframe.rows; // 720
    auto depthsizeC = depthframe.cols; // 1280
    std::vector<Point> target_point;
    cv::Mat depthframe2;
    depthframe2.create(depthsizeR,depthsizeC,CV_8U);

    // cout << "depthsizeR: " << depthsizeR << endl;
    // cout << "depthsizeC: " << depthsizeC << endl;
    
    for (auto i = 0; i<depthsizeR; i++){
        for (auto j = 0; j<depthsizeC; j++){ 
            auto depth = 0.001 * depthframe.at<ushort>(i,j);
            if ( depth > 0.2 && depth < 2 ){
                // target_point.push_back(Point(j,i));
                // depthframe2.ptr<int>(i)[j] = 1; 
            }else{
                // depthframe2.ptr<int>(i)[j] = 0;
            }
        }
    }
    // Create CSV
    // std::ofstream myFile("output.csv");
    // for( int i=0; i<depthsizeR; i++ ){  
    //     for( int j=0; j<depthsizeC; j++ ){  
    //         myFile << depthframe.at<ushort>(i,j);
    //         if(j != depthsizeC - 1) myFile << ","; 
    //     }
    //     myFile <<"\n";
    // }
    // myFile.close();
    // cv::imwrite("output.bmp", depthframe);
    // CSV end
    // cout << depthframe2 << endl;
    cv::Mat output = rgbframe.clone();
    cv::Scalar point_color(255, 255, 255);
    
    // for (auto j : target_point){
    //     cv::circle(output, j, 1, point_color, 4);
    // }

    // cv::imshow("out", depthframe2);
    // cv::waitKey(1);
}*/
void Aruco_PosePub(Vec6 rpyxyz)
// (Vec3 rpyW, Vec3 TranslationW)
{
    Quaterniond Q = rpy2Q(Vec3(rpyxyz(0),rpyxyz(1),rpyxyz(2)));
    Aruco_pose_realsense.header.frame_id = "world";
    Aruco_pose_realsense.pose.position.x = rpyxyz(3);
    Aruco_pose_realsense.pose.position.y = rpyxyz(4);
    Aruco_pose_realsense.pose.position.z = rpyxyz(5);
    Aruco_pose_realsense.pose.orientation.w = Q.w();
    Aruco_pose_realsense.pose.orientation.x = Q.x();
    Aruco_pose_realsense.pose.orientation.y = Q.y();
    Aruco_pose_realsense.pose.orientation.z = Q.z();
}
void Depth_PosePub(Vec6 rpyxyz)
// (Vec3 rpyW, Vec3 TranslationW)
{
    Quaterniond Q = rpy2Q(Vec3(rpyxyz(0),rpyxyz(1),rpyxyz(2)));
    Depth_pose_realsense.header.frame_id = "world";
    Depth_pose_realsense.pose.position.x = rpyxyz(3);
    Depth_pose_realsense.pose.position.y = rpyxyz(4);
    Depth_pose_realsense.pose.position.z = rpyxyz(5);
    Depth_pose_realsense.pose.orientation.w = Q.w();
    Depth_pose_realsense.pose.orientation.x = Q.x();
    Depth_pose_realsense.pose.orientation.y = Q.y();
    Depth_pose_realsense.pose.orientation.z = Q.z();
}
Vec6 Pose_calc(const Vec3 rvecs, const Vec3 tvecs)
{
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
    output << rpy_world[0],rpy_world[1],rpy_world[2],translation_world[0],translation_world[1],translation_world[2];
    return(output);
}
Vec6 Pose_calc_Aruco(const cv::Vec3d rvecs, const cv::Vec3d tvecs)
{
    Eigen::Quaterniond q;
    q.w() = Camera_pose_vicon.pose.orientation.w;
    q.x() = Camera_pose_vicon.pose.orientation.x;
    q.y() = Camera_pose_vicon.pose.orientation.y;
    q.z() = Camera_pose_vicon.pose.orientation.z;
    Eigen::Matrix3d Camera_Rotation_world = Eigen::Matrix3d::Identity();
    Camera_Rotation_world = q.matrix();
    Vec3 Camera_Translation_world(Camera_pose_vicon.pose.position.x, Camera_pose_vicon.pose.position.y, Camera_pose_vicon.pose.position.z);
    Vec3 Aruco_translation_camera(tvecs(0),tvecs(1),tvecs(2));
    Vec3 Aruco_rpy_camera(rvecs[0],rvecs[1],rvecs[2]);
    Vec3 Aruco_translation_world = Camera_Rotation_world * Aruco_translation_camera + Camera_Translation_world;
    Vec3 Aruco_rpy_world = Camera_Rotation_world*Aruco_rpy_camera;
    Vec6 output;
    output << Aruco_rpy_world[0],Aruco_rpy_world[1],Aruco_rpy_world[2],Aruco_translation_world[0],Aruco_translation_world[1],Aruco_translation_world[2];
    return(output);
}
/*void Pose_calc_Depth(const cv::Vec3d rvecs, const Vec3 Depth_translation_camera)
{
    Eigen::Quaterniond q;
    q.w() = Camera_pose_vicon.pose.orientation.w;
    q.x() = Camera_pose_vicon.pose.orientation.x;
    q.y() = Camera_pose_vicon.pose.orientation.y;
    q.z() = Camera_pose_vicon.pose.orientation.z;
    Eigen::Matrix3d Camera_Rotation_world = Eigen::Matrix3d::Identity();
    Camera_Rotation_world = q.matrix();
    Vec3 Camera_Translation_world(Camera_pose_vicon.pose.position.x, Camera_pose_vicon.pose.position.y, Camera_pose_vicon.pose.position.z);
    Vec3 Depth_rpy_camera(rvecs[0],rvecs[1],rvecs[2]);
    Vec3 Depth_translation_world = Camera_Rotation_world * Depth_translation_camera + Camera_Translation_world;
    Vec3 Depth_rpy_world = Camera_Rotation_world*Depth_rpy_camera;
    // Depth_PosePub(Depth_rpy_world,Depth_translation_world);
}*/
/*void estimatedpose_calc(){
    Vec3 PosefromAruco,PosefromDepth,PosefromVicon;
    PosefromAruco << Aruco_pose_realsense.pose.position.x,Aruco_pose_realsense.pose.position.y,Aruco_pose_realsense.pose.position.z;
    PosefromDepth << Depth_pose_realsense.pose.position.x,Depth_pose_realsense.pose.position.y,Depth_pose_realsense.pose.position.z;
    PosefromVicon << UAV_pose_vicon.pose.position.x,UAV_pose_vicon.pose.position.y,UAV_pose_vicon.pose.position.z;
    
    Vec3 DifferenceArucoDepth = PosefromAruco-PosefromDepth;
    Vec3 DifferenceArucoVicon = PosefromAruco-PosefromVicon;
    Vec3 DifferenceDepthVicon = PosefromDepth-PosefromVicon;

    cout << " PosefromAruco: " << PosefromAruco[0] << " " << PosefromAruco[1] << " " << PosefromAruco[2] << endl;
    cout << " PosefromDepth: " << PosefromDepth[0] << " " << PosefromDepth[1] << " " << PosefromDepth[2] << endl;
    cout << " PosefromVicon: " << PosefromVicon[0] << " " << PosefromVicon[1] << " " << PosefromVicon[2] << endl;
    
    // if(ArucoYes == false){
    //     cout << "Aruco Lost" << endl;
    // }else{ cout << " DifferenceAV: " << DifferenceArucoVicon[0] << " " << DifferenceArucoVicon[1] << " " << DifferenceArucoVicon[2] << endl; }
    // cout << " DifferenceDV: " << DifferenceDepthVicon[0] << " " << DifferenceDepthVicon[1] << " " << DifferenceDepthVicon[2] << endl;
    
}*/
double find_depth_avg(cv::Mat image_dep,Vec8I markerConerABCD){
    Vec2I markerCenterXY = FindMarkerCenter(markerConerABCD);
    double CornerLength1 = sqrt((pow(markerConerABCD[0]-markerConerABCD[2],2))+(pow(markerConerABCD[1]-markerConerABCD[3],2))); // in pixel
    double CornerLength2 = sqrt((pow(markerConerABCD[4]-markerConerABCD[6],2))+(pow(markerConerABCD[5]-markerConerABCD[7],2))); // in pixel
    double MidDepthTotal = 0;
    int valid_count = 0;
    for(int i = 0; i < max(CornerLength1,CornerLength2); i++){
        for(int j = 0; j < max(CornerLength1,CornerLength2); j++){
            double MidDepthij = image_dep.at<ushort>(markerCenterXY[1]-max(CornerLength1,CornerLength2)+i,markerCenterXY[0]-max(CornerLength1,CornerLength2)+j);
            if (MidDepthij > 0 ){
                MidDepthTotal += MidDepthij;
                valid_count++;
            }
        }
    }
    // double MidDepth = 0.001 * image_dep.at<ushort>(markerCenterXY[1],markerCenterXY[0]); 
    if (valid_count == 0){
        return(0);
    }else{
        // cout << " Depth: " << 0.001 * MidDepthTotal/valid_count << endl;
        // cout << " Original: " << 0.001 * image_dep.at<ushort>(markerCenterXY[1],markerCenterXY[0]) << endl;
        return(0.001 * MidDepthTotal/valid_count);
        // return(0.001 * image_dep.at<ushort>(markerCenterXY[1],markerCenterXY[0]));
    }
}
Vec2I Constant_velocity_estimator(const Vec8I last_markerConer,const int Lostcounter){
    if (Lostcounter == 0){
        CVE_Corners.push_back(last_markerConer);
        if (CVE_Corners.size() > 5){ //calculate last five
            CVE_Corners.pop_front();
        }
        return(FindMarkerCenter(last_markerConer));
    }else{
        Vec2I XY = Vec2I(0,0);
        for(unsigned int i=0; i < (CVE_Corners.size()-1) ;i++){
            Vec2I diff_xy = FindMarkerCenter(CVE_Corners.at(i)) - FindMarkerCenter(CVE_Corners.at(i+1));
            XY += diff_xy;
            XY /= CVE_Corners.size();
            XY *= Lostcounter;
        }
        return(FindMarkerCenter(CVE_Corners.back())+XY);
    }
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
    Vec8I markerConerABCD,last_markerConerABCD;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    std::vector<cv::Vec3d> rvecs, tvecs;
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
            std::vector<cv::Point2f> markerCorner = markerCorners[i];
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
            cv::Vec3d rvec = rvecs.front();
            cv::Vec3d tvec = tvecs.front();
            Vec3 Aruco_translation_camera(tvec(0),tvec(1),tvec(2));
            Vec3 Aruco_rpy_camera(rvec(0),rvec(1),rvec(2));
            Aruco_PosePub(Pose_calc(Aruco_rpy_camera,Aruco_translation_camera));
            Depthrvecs = Aruco_rpy_camera;
            double ArucoDepth = find_depth_avg(image_dep,markerConerABCDs.back());
            last_markerConerABCD = markerConerABCDs.back();
            Depth_PosePub(Pose_calc(Depthrvecs,camerapixel2tvec(Constant_velocity_estimator(last_markerConerABCD,ArucoLostcounter),ArucoDepth,CamParameters)));
            ArucoLostcounter = 0;
        }else{ //Aruco not found do constant-velocity estimate
            double ArucoDepth = find_depth_avg(image_dep,last_markerConerABCD);
            Depth_PosePub(Pose_calc(Depthrvecs,camerapixel2tvec(Constant_velocity_estimator(last_markerConerABCD,ArucoLostcounter),ArucoDepth,CamParameters)));
        }
    }
    /* ROS timer */
    // auto currentT = ros::Time::now().toSec();
    // cout << "callback_Hz: " << 1/(currentT-callback_LastT) << endl;
    // callback_LastT = currentT;

    /* image plot */
    // cv::Mat depImage = image_dep.clone();
    // cv::imshow("dep_out", depImage);
    cv::imshow("Aruco_out", ArucoOutput);
    cv::waitKey(1);
}
void Finite_stage_mission(){

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;
   
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);
    ros::Subscriber camerapose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_l515/pose", 1, CameraPose_cb);
    ros::Subscriber uavpose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_small/pose", 1, UAVPose_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ros::Publisher ArucoPose_pub = nh.advertise<geometry_msgs::PoseStamped>("ArucoPose",1);
    ros::Publisher DepthPose_pub = nh.advertise<geometry_msgs::PoseStamped>("DepthPose",1);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    message_filters::Subscriber<CompressedImage> rgb_sub(nh, "/camera/color/image_raw/compressed", 1);
    message_filters::Subscriber<Image> dep_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);

    typedef sync_policies::ApproximateTime<CompressedImage, Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, dep_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::SetMode posctl_set_mode;
    posctl_set_mode.request.custom_mode = "POSCTL";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Rate loop_rate(50); /* ROS system Hz */

    while(ros::ok()){

        // if (ROS_init){
        //     System_initT = ros::Time::now().toSec();
        //     init_time = ros::Time::now();
        //     UAV_takeoffP = UAV_lp;
        //     ROS_init = false;
        //     cout << " System Initialized" << endl;
        //     cout << "UAV_takeoffP: " << UAV_takeoffP[0] << " " << UAV_takeoffP[1] << " " << UAV_takeoffP[2] << endl;
        //     /* Waypoints before starting */
        //     UAV_pose_pub.header.frame_id = "world";
        //     UAV_pose_pub.pose.position.x = UAV_takeoffP[0];
        //     UAV_pose_pub.pose.position.y = UAV_takeoffP[1];
        //     UAV_pose_pub.pose.position.z = UAV_takeoffP[2];
        //     UAV_pose_pub.pose.orientation.w = UAV_takeoffP[3];
        //     UAV_pose_pub.pose.orientation.x = UAV_takeoffP[4];
        //     UAV_pose_pub.pose.orientation.y = UAV_takeoffP[5];
        //     UAV_pose_pub.pose.orientation.z = UAV_takeoffP[6];
        //     for(int i = 100; ros::ok() && i > 0; --i){
        //         local_pos_pub.publish(UAV_pose_pub);
        //         ros::spinOnce();
        //         loop_rate.sleep();
        //     }
        // }
        // /*offboard and arm*****************************************************/
        // if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)) && (ros::Time::now() - init_time < ros::Duration(10.0))){
        //     if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // }else{
        //     if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0)) && (ros::Time::now() - init_time < ros::Duration(10.0))){
        //         if( arming_client.call(arm_cmd) && arm_cmd.response.success){
        //         ROS_INFO("Vehicle armed");
        //         }
        //     }
        //     last_request = ros::Time::now();
        // }
        // Finite_stage_mission();
        
        ArucoPose_pub.publish(Aruco_pose_realsense);
        DepthPose_pub.publish(Depth_pose_realsense);
        local_pos_pub.publish(UAV_pose_pub);
        /* ROS timer */
        // auto currentT = ros::Time::now().toSec();
        // cout << "System_Hz: " << 1/(currentT-System_LastT) << endl;
        // System_LastT = currentT;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}