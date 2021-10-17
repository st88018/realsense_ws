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
geometry_msgs::PoseStamped Aruco_pose_realsense,Depth_pose_realsense,LED_pose_realsense,Camera_pose_sub,UAV_pose_sub;
Vec7 UAV_lp,Camera_lp;
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
void camera_pose_sub(const geometry_msgs::PoseStamped::ConstPtr& pose){
    Camera_pose_sub.pose.position.x = pose->pose.position.x;
    Camera_pose_sub.pose.position.y = pose->pose.position.y;
    Camera_pose_sub.pose.position.z = pose->pose.position.z;
    Camera_pose_sub.pose.orientation.x = pose->pose.orientation.x;
    Camera_pose_sub.pose.orientation.y = pose->pose.orientation.y;
    Camera_pose_sub.pose.orientation.z = pose->pose.orientation.z;
    Camera_pose_sub.pose.orientation.w = pose->pose.orientation.w;
    Camera_lp << Camera_pose_sub.pose.position.x,Camera_pose_sub.pose.position.y,Camera_pose_sub.pose.position.z,
                 Camera_pose_sub.pose.orientation.w,Camera_pose_sub.pose.orientation.x,Camera_pose_sub.pose.orientation.y,Camera_pose_sub.pose.orientation.z;
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
void LED_PosePub(Vec6 rpyxyz){
    // Quaterniond Q = rpy2Q(Vec3(rpyxyz(0),rpyxyz(1),rpyxyz(2)));
    LED_pose_realsense.header.stamp = ros::Time::now();
    LED_pose_realsense.header.frame_id = "world";
    LED_pose_realsense.pose.position.x = rpyxyz(3);
    LED_pose_realsense.pose.position.y = rpyxyz(4);
    LED_pose_realsense.pose.position.z = rpyxyz(5);
    LED_pose_realsense.pose.orientation.w = UAVq.w();
    LED_pose_realsense.pose.orientation.x = UAVq.x();
    LED_pose_realsense.pose.orientation.y = UAVq.y();
    LED_pose_realsense.pose.orientation.z = UAVq.z();
}
void Aruco_Depth_process(Mat image_rgb, Mat image_dep){
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
        Vec3 Depthtvecs;
        if(Aruco_found){
            rvec = rvecs.front();
            tvec = tvecs.front();
            Vec3 Aruco_translation_camera(tvec(0),tvec(1),tvec(2));
            Vec3 Aruco_rpy_camera(rvec(0),rvec(1),rvec(2));
            Aruco_PosePub(Camera2World(Aruco_translation_camera,Camera_lp));
            double ArucoDepth = find_depth_avg(image_dep,markerConerABCDs.back());
            last_markerConerABCD = markerConerABCDs.back();
            Depthtvecs = camerapixel2tvec(FindMarkerCenter(last_markerConerABCD),ArucoDepth,CamParameters);
            Depth_PosePub(Camera2World(Depthtvecs,Camera_lp));
        }
    }
    // cv::imshow("uav", ArucoOutput);
    // cv::waitKey(1);
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
    // cv::imshow("uav", ArucoOutput);
    // cv::waitKey(1);
}
void callback(const sensor_msgs::CompressedImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth){
    /* Image initialize */
    cv::Mat image_rgb,image_dep;
    try{
        image_rgb = cv::imdecode(cv::Mat(rgb->data),1);
        cv_bridge::CvImagePtr depth_ptr  = cv_bridge::toCvCopy(depth, depth->encoding);
        image_dep = depth_ptr->image;
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    /* Aruco */
    Aruco_Depth_process(image_rgb,image_dep);
    /* LED PNP */
    // Vec6 LEDtvecrvec = LEDTvecRvec(image_rgb);  
    // LED_PosePub(Camera2World(Vec3(LEDtvecrvec[3],LEDtvecrvec[4],LEDtvecrvec[5]),Vec3(LEDtvecrvec[0],LEDtvecrvec[1],LEDtvecrvec[2])));
    // cout << "Aruco Tvec: " << tvec*1000 << endl;
    /* ROS timer */
    auto TimerT = ros::Time::now().toSec();
    cout << "---------------------------------------------------" << endl;
    cout << "Aruco_Hz: " << 1/(TimerT-TimerLastT) << endl;
    TimerLastT = TimerT;
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
    /* Aruco */
    Aruco_process(image_rgb);
    /* LED PNP */
    // Vec6 LEDtvecrvec = LEDTvecRvec(image_rgb);  
    // LED_PosePub(Camera2World(Vec3(LEDtvecrvec[3],LEDtvecrvec[4],LEDtvecrvec[5]),Vec3(LEDtvecrvec[0],LEDtvecrvec[1],LEDtvecrvec[2])));
    // cout << "Aruco Tvec: " << tvec*1000 << endl;
    /* ROS timer */
    auto TimerT = ros::Time::now().toSec();
    cout << "---------------------------------------------------" << endl;
    cout << "Aruco_Hz: " << 1/(TimerT-TimerLastT) << endl;
    TimerLastT = TimerT;
}
void datalogger(){ 
    logger_time = ros::Time::now().toSec();
    if(logger_time-logger_time_last > 0.01){
        ofstream save("/home/jeremy/realsense_ws/src/Aruco&Depth_raw.csv", ios::app);
        save<<logger_time<<","<<Aruco_pose_realsense.pose.position.x <<","<< Aruco_pose_realsense.pose.position.y <<","<< Aruco_pose_realsense.pose.position.z <<
                           ","<<Depth_pose_realsense.pose.position.x <<","<< Depth_pose_realsense.pose.position.y <<","<< Depth_pose_realsense.pose.position.z <<endl;
        save.close();
        logger_time_last = logger_time;
    }
}
int main(int argc, char **argv){
    ros::init(argc, argv, "Aruco");
    ros::NodeHandle nh;
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);
    ros::Subscriber camerapose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_d455/pose", 1, camera_pose_sub);
    ros::Subscriber uavpose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, uav_pose_sub);
    ros::Publisher ArucoPose_pub = nh.advertise<geometry_msgs::PoseStamped>("ArucoPose",1);
    ros::Publisher DepthPose_pub = nh.advertise<geometry_msgs::PoseStamped>("DepthPose",1);
    // ros::Publisher LEDPose_pub = nh.advertise<geometry_msgs::PoseStamped>("LEDPose",1);
    ros::Subscriber camera_rgb_sub = nh.subscribe<CompressedImage>("/camera/color/image_raw/compressed",1,camera_rgb_cb);
    // message_filters::Subscriber<CompressedImage> rgb_sub(nh, "/camera/color/image_raw/compressed", 1);
    // message_filters::Subscriber<Image> dep_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    // typedef sync_policies::ApproximateTime<CompressedImage, Image> MySyncPolicy;
    // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, dep_sub);
    // sync.registerCallback(boost::bind(&callback, _1, _2));
    // PNP3Dpoints();
    remove("/home/jeremy/realsense_ws/src/Aruco&Depth_raw.csv");

    while(ros::ok()){
        ros::spinOnce();
        ArucoPose_pub.publish(Aruco_pose_realsense);
        DepthPose_pub.publish(Depth_pose_realsense);
        // LEDPose_pub.publish(LED_pose_realsense);
        // datalogger();
    }
    return 0;
}