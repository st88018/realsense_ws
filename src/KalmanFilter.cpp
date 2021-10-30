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
#include <geometry_msgs/TwistStamped.h>
#include <numeric>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <fstream>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include "utils/kinetic_math.hpp"
#include "utils/cv.hpp"
#include "utils/run_yolo.hpp"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

/*uav local parameter*/
static geometry_msgs::PoseStamped   KF_pose_pub,Camera_pose_sub,UAV_pose_sub;
static geometry_msgs::TwistStamped  UGV_twist_sub,UAV_twist_sub;
static Vec6 UAV_twist;
static Vec7 UAV_lp,Camera_lp;
static Vec7 Depth_lp,Aruco_lp,Yolo_lp;
static Vec7 Depth_lp_last,Aruco_lp_last,Yolo_lp_last;
static bool Depth_updated,Aruco_updated,Yolo_updated;
Quaterniond UAVq;
/* System */
int coutcounter = 0;
static Vec7 Zero7;
static Vec4 Zero4;
double TimerLastT,logger_time,logger_time_last;
/* Kalman Filter */
double KFdT;
Vec7 KF_pub;
bool KF_init = false;
double CV_lost_timer;
bool CV_lost = false;
std_msgs::Bool KFok;
double Error_lp,dist2UAV;

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
void aruco_pose_sub(const geometry_msgs::PoseStamped::ConstPtr& pose){
    Aruco_lp[0] = pose->pose.position.x;
    Aruco_lp[1] = pose->pose.position.y;
    Aruco_lp[2] = pose->pose.position.z;
    Aruco_lp[3] = pose->pose.orientation.w;
    Aruco_lp[4] = pose->pose.orientation.x;
    Aruco_lp[5] = pose->pose.orientation.y;
    Aruco_lp[6] = pose->pose.orientation.z;
    if(Aruco_lp_last != Aruco_lp){
        Aruco_lp_last = Aruco_lp;
        Aruco_updated = true;
        KF_init = true;
    }
}
void depth_pose_sub(const geometry_msgs::PoseStamped::ConstPtr& pose){
    Depth_lp[0] = pose->pose.position.x;
    Depth_lp[1] = pose->pose.position.y;
    Depth_lp[2] = pose->pose.position.z;
    Depth_lp[3] = pose->pose.orientation.w;
    Depth_lp[4] = pose->pose.orientation.x;
    Depth_lp[5] = pose->pose.orientation.y;
    Depth_lp[6] = pose->pose.orientation.z;
    if(Depth_lp_last != Depth_lp){
        Depth_lp_last = Depth_lp;
        Depth_updated = true;
        CV_lost = false;
    }
}
void yolo_pose_sub(const geometry_msgs::PoseStamped::ConstPtr& pose){
    Yolo_lp[0] = pose->pose.position.x;
    Yolo_lp[1] = pose->pose.position.y;
    Yolo_lp[2] = pose->pose.position.z;
    Yolo_lp[3] = pose->pose.orientation.w;
    Yolo_lp[4] = pose->pose.orientation.x;
    Yolo_lp[5] = pose->pose.orientation.y;
    Yolo_lp[6] = pose->pose.orientation.z;
    if(Yolo_lp_last != Yolo_lp){
        Yolo_lp_last = Yolo_lp;
        Yolo_updated = true;
        KF_init = true;
        CV_lost = false;
    }
}
void uav_twist_sub(const geometry_msgs::TwistStamped::ConstPtr& twist){
    UAV_twist_sub.twist.linear.x = twist->twist.linear.x;
    UAV_twist_sub.twist.linear.y = twist->twist.linear.y;
    UAV_twist_sub.twist.linear.z = twist->twist.linear.z;
    UAV_twist_sub.twist.angular.x = twist->twist.angular.x;
    UAV_twist_sub.twist.angular.y = twist->twist.angular.y;
    UAV_twist_sub.twist.angular.z = twist->twist.angular.z;
    UAV_twist << UAV_twist_sub.twist.linear.x,UAV_twist_sub.twist.linear.y,UAV_twist_sub.twist.linear.z,
                 UAV_twist_sub.twist.angular.x,UAV_twist_sub.twist.angular.y,UAV_twist_sub.twist.angular.z;
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
void KF_PosePub(Vec7 KF_pub){
    KF_pose_pub.header.stamp = ros::Time::now();
    KF_pose_pub.header.frame_id = "world";
    KF_pose_pub.pose.position.x = KF_pub(0);
    KF_pose_pub.pose.position.y = KF_pub(1);
    KF_pose_pub.pose.position.z = KF_pub(2);
    KF_pose_pub.pose.orientation.w = KF_pub(3);
    KF_pose_pub.pose.orientation.x = KF_pub(4);
    KF_pose_pub.pose.orientation.y = KF_pub(5);
    KF_pose_pub.pose.orientation.z = KF_pub(6);
}
bool KFok_indicator(){
    bool output;
    if(CV_lost_timer - ros::Time::now().toSec() < -5){ //Visualize of UAV lost for 10 seceonds KF no OK
        KF_init = false;
        output = false;
        KF_PosePub(Zero7);
    }
    if(!CV_lost){
        CV_lost_timer = ros::Time::now().toSec();
        output = true;
        CV_lost = true;
    }
    return(output);
}
void datalogger(){ 
    logger_time = ros::Time::now().toSec();
    // if(logger_time-logger_time_last > 0.02){
        ofstream save("/home/patty/realsense_ws/src/realsense_ws/logs/KF.csv", ios::app);
        save<<std::setprecision(20)<<logger_time<<","<<KF_pub(0)<<","<<KF_pub(1)<<","<<KF_pub(2)<<endl;                                      
        save.close();
        ofstream save2("/home/patty/realsense_ws/src/realsense_ws/logs/Groundtruth.csv", ios::app);
        save2<<std::setprecision(20)<<logger_time<<","<<UAV_lp(0)<<","<<UAV_lp(1)<<","<<UAV_lp(2)<<endl;
        save2.close();
        // logger_time_last = logger_time;
    // }
}
int main(int argc, char **argv){
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);
    ros::Subscriber camerapose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_d455/pose", 1, camera_pose_sub);
    ros::Subscriber uavtwist_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 1, uav_twist_sub);
    ros::Subscriber uavpose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, uav_pose_sub);
    ros::Subscriber arucopose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/ArucoPose", 1, aruco_pose_sub);
    ros::Subscriber depthpose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/DepthPose", 1, depth_pose_sub);
    ros::Subscriber yolopose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/YoloPose", 1, yolo_pose_sub);
    ros::Publisher KFPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/KalmanFilterPose",1);
    ros::Publisher KFok_pub = nh.advertise<std_msgs::Bool>("/KFok",1);
    remove("/home/patty/realsense_ws/src/realsense_ws/logs/KF.csv");
    remove("/home/patty/realsense_ws/src/realsense_ws/logs/Groundtruth.csv");
    
    /* System Params */
    ros::Rate loop_rate(50); /* ROS system Hz */

    /* Kalman Filter */
    int stateSize = 6; // (x,y,z,vx,vy,vz)
    int measSize = 6;  // (x,y,z,vx,vy,vz)
    int contrSize = 0;
    cv::KalmanFilter KF(stateSize, measSize, contrSize);
    setIdentity(KF.transitionMatrix);
    //  A << 1,0,0,dt,0,0,
    //       0,1,0,0,dt,0, 
    //       0,0,1,0,0,dt,
    //       0,0,0,1,0,0,
    //       0,0,0,0,1,0,
    //       0,0,0,0,0,1);
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-2)); 
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    KF.measurementNoiseCov.at<float>(21) = 2e-1;
    KF.measurementNoiseCov.at<float>(28) = 2e-1;
    KF.measurementNoiseCov.at<float>(35) = 2e-1;
    setIdentity(KF.errorCovPost, Scalar::all(1));
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));
    KFok.data = false;

    while(ros::ok()){
        ros::spinOnce();
        /* ROS timer */
        auto TimerT = ros::Time::now().toSec();
        KFdT = TimerT-TimerLastT;
        cout << "---------------------------------------------------" << endl;
        cout << "System_Hz: " << 1/(TimerT-TimerLastT) << " dt: " << KFdT << endl;
        bool KF_ok = KFok.data;
        cout << "Visual_LOST_time: " << CV_lost_timer - ros::Time::now().toSec() << " KF_init: " << KF_init << " " << KF_ok << " " << CV_lost << endl;
        cout << "Aruco Flag: " << Aruco_updated << " Yolo Flag: " << Yolo_updated << endl;
        cout << "     X: " <<KF_pose_pub.pose.position.x << " Y: " <<KF_pose_pub.pose.position.y << " Z: " <<KF_pose_pub.pose.position.z << endl;
        Error_lp = sqrt(pow((KF_pub[0]-UAV_pose_sub.pose.position.x),2)+
                     pow((KF_pub[1]-UAV_pose_sub.pose.position.y),2)+
                     pow((KF_pub[2]-UAV_pose_sub.pose.position.z),2));
        cout << "error: " << Error_lp << " dist2UAV: " << dist2UAV << endl;
        TimerLastT = TimerT;
        KFok_indicator();
        KFok.data = KF_init;
        KFok_pub.publish(KFok);

        dist2UAV = sqrt(pow((KF_pub[0]-Camera_lp[0]),2)+
                     pow((KF_pub[1]-Camera_lp[1]),2)+
                     pow((KF_pub[2]-Camera_lp[2]),2));
        
        if(KF_init){
            /* Kalman Filter */
            KF.transitionMatrix.at<float>(3)  = KFdT; //update Mat A
            KF.transitionMatrix.at<float>(10) = KFdT; 
            KF.transitionMatrix.at<float>(17) = KFdT;
            /* KF prediction */
            Mat prediction = KF.predict();
            KF_pub << prediction.at<float>(0),prediction.at<float>(1),prediction.at<float>(2),UAV_lp[3],UAV_lp[4],UAV_lp[5],UAV_lp[6];
            KF_PosePub(KF_pub);
            /* update KF measurement */
            Mat measurement = Mat::zeros(measSize, 1, CV_32F);
            if(dist2UAV < 1.5){
                if(Aruco_updated){
                    measurement.at<float>(0) = Aruco_lp[0];
                    measurement.at<float>(1) = Aruco_lp[1];
                    measurement.at<float>(2) = Aruco_lp[2];
                    measurement.at<float>(3) = UAV_twist[0];
                    measurement.at<float>(4) = UAV_twist[1];
                    measurement.at<float>(5) = UAV_twist[2];
                    Aruco_updated = false; 
                }else if(Yolo_updated){
                    measurement.at<float>(0) = Yolo_lp[0];
                    measurement.at<float>(1) = Yolo_lp[1];
                    measurement.at<float>(2) = Yolo_lp[2];
                    measurement.at<float>(3) = UAV_twist[0];
                    measurement.at<float>(4) = UAV_twist[1];
                    measurement.at<float>(5) = UAV_twist[2];
                    Yolo_updated = false;
                }else{
                    measurement.at<float>(0) = prediction.at<float>(0);
                    measurement.at<float>(1) = prediction.at<float>(1);
                    measurement.at<float>(2) = prediction.at<float>(2);
                    measurement.at<float>(3) = UAV_twist[0];
                    measurement.at<float>(4) = UAV_twist[1];
                    measurement.at<float>(5) = UAV_twist[2];
                }
                KF.correct(measurement);
            }else{
                if(Yolo_updated){    
                    measurement.at<float>(0) = Yolo_lp[0];
                    measurement.at<float>(1) = Yolo_lp[1];
                    measurement.at<float>(2) = Yolo_lp[2];
                    measurement.at<float>(3) = UAV_twist[0];
                    measurement.at<float>(4) = UAV_twist[1];
                    measurement.at<float>(5) = UAV_twist[2];
                    Yolo_updated = false;
                }else if(Aruco_updated){
                    measurement.at<float>(0) = Aruco_lp[0];
                    measurement.at<float>(1) = Aruco_lp[1];
                    measurement.at<float>(2) = Aruco_lp[2];
                    measurement.at<float>(3) = UAV_twist[0];
                    measurement.at<float>(4) = UAV_twist[1];
                    measurement.at<float>(5) = UAV_twist[2];
                    Aruco_updated = false;
                }else{
                    measurement.at<float>(0) = prediction.at<float>(0);
                    measurement.at<float>(1) = prediction.at<float>(1);
                    measurement.at<float>(2) = prediction.at<float>(2);
                    measurement.at<float>(3) = UAV_twist[0];
                    measurement.at<float>(4) = UAV_twist[1];
                    measurement.at<float>(5) = UAV_twist[2];
                }
                KF.correct(measurement);
            }
        }
        KFPose_pub.publish(KF_pose_pub);
        datalogger();
        loop_rate.sleep();
    }
    return 0;
}