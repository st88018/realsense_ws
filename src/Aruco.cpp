#include <ros/ros.h>
#include <image_transport/image_transport.h>
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
#include <geometry_msgs/PoseStamped.h>
#include <numeric>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include "utils/kinetic_math.hpp"
#include "utils/cv.hpp"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

/*uav local parameter*/
geometry_msgs::PoseStamped Aruco_pose_realsense,Depth_pose_realsense,Camera_pose_sub,UGV_pose_sub,UAV_pose_sub,CAM_test;
Vec7 UAV_lp,UGV_lp,Camera_lp;
Quaterniond UAVq;
/* System */
Vec7 Zero7;
Vec4 Zero4;
double TimerLastT,logger_time,logger_time_last;


Vec3 uav_real_pose(Vec3 xyz){ // Move the surface pose backward a bit
    double horizontal_dist = 0.02;
    Vec3 UAVrpy = Q2rpy(UAVq);
    Vec2 uavxy = Vec2(UAV_lp[0]-horizontal_dist*cos(UAVrpy[2]),UAV_lp[1]-horizontal_dist*sin(UAVrpy[2]));
    return(Vec3(uavxy[0],uavxy[1],xyz[2]));
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
    UAVq = Quaterniond(UAV_lp[3],UAV_lp[4],UAV_lp[5],UAV_lp[6]);
}
// void camera_pose_sub(const geometry_msgs::PoseStamped::ConstPtr& pose){
//     Camera_pose_sub.pose.position.x = pose->pose.position.x;
//     Camera_pose_sub.pose.position.y = pose->pose.position.y;
//     Camera_pose_sub.pose.position.z = pose->pose.position.z;
//     Camera_pose_sub.pose.orientation.x = pose->pose.orientation.x;
//     Camera_pose_sub.pose.orientation.y = pose->pose.orientation.y;
//     Camera_pose_sub.pose.orientation.z = pose->pose.orientation.z;
//     Camera_pose_sub.pose.orientation.w = pose->pose.orientation.w;
//     Camera_lp << Camera_pose_sub.pose.position.x,Camera_pose_sub.pose.position.y,Camera_pose_sub.pose.position.z,
//                  Camera_pose_sub.pose.orientation.w,Camera_pose_sub.pose.orientation.x,Camera_pose_sub.pose.orientation.y,Camera_pose_sub.pose.orientation.z;
// }
void CAMtest_PosePub(Vec7 xyzq){
    CAM_test.header.stamp = ros::Time::now();
    CAM_test.header.frame_id = "world";
    CAM_test.pose.position.x = xyzq(0);
    CAM_test.pose.position.y = xyzq(1);
    CAM_test.pose.position.z = xyzq(2);
    CAM_test.pose.orientation.w = xyzq(3);
    CAM_test.pose.orientation.x = xyzq(4);
    CAM_test.pose.orientation.y = xyzq(5);
    CAM_test.pose.orientation.z = xyzq(6);
}
void ugv_pose_sub(const geometry_msgs::PoseStamped::ConstPtr& pose){
    UGV_pose_sub.pose.position.x = pose->pose.position.x;
    UGV_pose_sub.pose.position.y = pose->pose.position.y;
    UGV_pose_sub.pose.position.z = pose->pose.position.z;
    UGV_pose_sub.pose.orientation.x = pose->pose.orientation.x;
    UGV_pose_sub.pose.orientation.y = pose->pose.orientation.y;
    UGV_pose_sub.pose.orientation.z = pose->pose.orientation.z;
    UGV_pose_sub.pose.orientation.w = pose->pose.orientation.w;
    UGV_lp << UGV_pose_sub.pose.position.x,UGV_pose_sub.pose.position.y,UGV_pose_sub.pose.position.z,
                 UGV_pose_sub.pose.orientation.w,UGV_pose_sub.pose.orientation.x,UGV_pose_sub.pose.orientation.y,UGV_pose_sub.pose.orientation.z;
    Camera_lp = GenerateCameraLP(UGV_lp);
    CAMtest_PosePub(Camera_lp);
}
void Aruco_PosePub(Vec3 xyz){
    xyz = uav_real_pose(xyz);
    Aruco_pose_realsense.header.stamp = ros::Time::now();
    Aruco_pose_realsense.header.frame_id = "world";
    Aruco_pose_realsense.pose.position.x = xyz(0);
    Aruco_pose_realsense.pose.position.y = xyz(1);
    Aruco_pose_realsense.pose.position.z = xyz(2);
    Aruco_pose_realsense.pose.orientation.w = UAVq.w();
    Aruco_pose_realsense.pose.orientation.x = UAVq.x();
    Aruco_pose_realsense.pose.orientation.y = UAVq.y();
    Aruco_pose_realsense.pose.orientation.z = UAVq.z();
}
void Depth_PosePub(Vec3 xyz){
    xyz = uav_real_pose(xyz);
    Depth_pose_realsense.header.stamp = ros::Time::now();
    Depth_pose_realsense.header.frame_id = "world";
    Depth_pose_realsense.pose.position.x = xyz(0);
    Depth_pose_realsense.pose.position.y = xyz(1);
    Depth_pose_realsense.pose.position.z = xyz(2);
    Depth_pose_realsense.pose.orientation.w = UAVq.w();
    Depth_pose_realsense.pose.orientation.x = UAVq.x();
    Depth_pose_realsense.pose.orientation.y = UAVq.y();
    Depth_pose_realsense.pose.orientation.z = UAVq.z();
}
void Aruco_process(Mat image_rgb){
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
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.045, cameraMatrix, distCoeffs, rvecs, tvecs);
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
    if (Aruco_init){
        if(Aruco_found){
            rvec = rvecs.front();
            tvec = tvecs.front();
            Vec3 Aruco_translation_camera(tvec(0),tvec(1),tvec(2));
            Aruco_PosePub(Camera2World(Aruco_translation_camera,Camera_lp));
        }
    }
    cv::imshow("uav", ArucoOutput);
    cv::waitKey(1);
}
void datalogger(){ 
    logger_time = ros::Time::now().toSec();
    ofstream save("/home/patty/realsense_ws/src/realsense_ws/logs/Aruco_raw.csv", ios::app);
    save<<std::setprecision(20)<<logger_time<<","<<Aruco_pose_realsense.pose.position.x <<","<< Aruco_pose_realsense.pose.position.y <<","<< Aruco_pose_realsense.pose.position.z << endl;
    save.close();
}
void camera_rgb_cb(const sensor_msgs::CompressedImageConstPtr &rgb){
    /* Image initialize */
    cv::Mat image_rgb;
    try{
        image_rgb = cv::imdecode(cv::Mat(rgb->data),1);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Aruco_process(image_rgb);
    /* ROS timer */
    auto TimerT = ros::Time::now().toSec();
    cout << "---------------------------------------------------" << endl;
    cout << "Aruco_Hz: " << 1/(TimerT-TimerLastT) << endl;
    TimerLastT = TimerT;
    datalogger();
}
int main(int argc, char **argv){
    ros::init(argc, argv, "Aruco");
    ros::NodeHandle nh;
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);
    // ros::Subscriber camerapose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_d455/pose", 1, camera_pose_sub);
    ros::Subscriber ugvpose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/car/mavros/local_position/pose", 1, ugv_pose_sub);
    ros::Subscriber uavpose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, uav_pose_sub);
    ros::Publisher ArucoPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ArucoPose",1);
    ros::Publisher Camtest_pub = nh.advertise<geometry_msgs::PoseStamped>("/CamPose",1);
    ros::Subscriber camera_rgb_sub = nh.subscribe<CompressedImage>("/camera/color/image_raw/compressed",1,camera_rgb_cb);
    remove("/home/patty/realsense_ws/src/realsense_ws/logs/Aruco_raw.csv");

    while(ros::ok()){
        ros::spinOnce();
        Camtest_pub.publish(CAM_test);
        ArucoPose_pub.publish(Aruco_pose_realsense);
    }
    return 0;
}