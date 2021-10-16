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
#include "utils/run_yolo.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

/*uav local parameter*/
static geometry_msgs::PoseStamped Aruco_pose_realsense,Depth_pose_realsense,LED_pose_realsense,YOLO_pose_realsense,KF_pose,Camera_pose_sub,UAV_pose_sub;
static geometry_msgs::TwistStamped  UGV_twist_sub,UAV_twist_sub;
static Vec6 UAV_twist;
static Vec7 UAV_lp,Camera_lp;
Quaterniond UAVq;
/* System */
cv::Mat image_rgb,image_dep;
int coutcounter = 0;
static Vec7 Zero7;
static Vec4 Zero4;
double TimerLastT,logger_time,logger_time_last;
int logger_counter = 0;
/* Kalman Filter */
double KFdT;
Vec7 KF_pub;
bool KF_init = false;
/* YOLO */
static cv::String weightpath ="/home/jeremy/realsense_ws/src/realsense_ws/src/utils/yolo/uav.weights";
static cv::String cfgpath ="/home/jeremy/realsense_ws/src/realsense_ws/src/utils/yolo/uav.cfg";
static cv::String classnamepath = "/home/jeremy/realsense_ws/src/realsense_ws/src/utils/yolo/uav.names";
static run_yolo Yolonet(cfgpath, weightpath, classnamepath, float(0.7));
bool YOLO_found = false;

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
    KF_pose.header.stamp = ros::Time::now();
    KF_pose.header.frame_id = "world";
    KF_pose.pose.position.x = KF_pub(0);
    KF_pose.pose.position.y = KF_pub(1);
    KF_pose.pose.position.z = KF_pub(2);
    KF_pose.pose.orientation.w = KF_pub(3);
    KF_pose.pose.orientation.x = KF_pub(4);
    KF_pose.pose.orientation.y = KF_pub(5);
    KF_pose.pose.orientation.z = KF_pub(6);
}
void Aruco_PosePub(Vec6 rpyxyz){
    // Quaterniond Q = rpy2Q(Vec3(rpyxyz(0),rpyxyz(1),rpyxyz(2)));
    // Quaterniond Q = rpy2Q(Vec3(0,0,3.14));
    Aruco_pose_realsense.header.stamp = ros::Time::now();
    Aruco_pose_realsense.header.frame_id = "world";
    Aruco_pose_realsense.pose.position.x = rpyxyz(3);
    Aruco_pose_realsense.pose.position.y = rpyxyz(4);
    Aruco_pose_realsense.pose.position.z = rpyxyz(5);
    Aruco_pose_realsense.pose.orientation.w = UAVq.w();
    Aruco_pose_realsense.pose.orientation.x = UAVq.x();
    Aruco_pose_realsense.pose.orientation.y = UAVq.y();
    Aruco_pose_realsense.pose.orientation.z = UAVq.z();
}
void YOLO_PosePub(Vec6 rpyxyz){
    // Quaterniond Q = rpy2Q(Vec3(rpyxyz(0),rpyxyz(1),rpyxyz(2)));
    // Quaterniond Q = rpy2Q(Vec3(0,0,3.14));
    YOLO_pose_realsense.header.stamp = ros::Time::now();
    YOLO_pose_realsense.header.frame_id = "world";
    YOLO_pose_realsense.pose.position.x = rpyxyz(3);
    YOLO_pose_realsense.pose.position.y = rpyxyz(4);
    YOLO_pose_realsense.pose.position.z = rpyxyz(5);
    YOLO_pose_realsense.pose.orientation.w = UAVq.w();
    YOLO_pose_realsense.pose.orientation.x = UAVq.x();
    YOLO_pose_realsense.pose.orientation.y = UAVq.y();
    YOLO_pose_realsense.pose.orientation.z = UAVq.z();
}
void Depth_PosePub(Vec6 rpyxyz){
    // Quaterniond Q = rpy2Q(Vec3(rpyxyz(0),rpyxyz(1),rpyxyz(2)));
    Depth_pose_realsense.header.stamp = ros::Time::now();
    Depth_pose_realsense.header.frame_id = "world";
    Depth_pose_realsense.pose.position.x = rpyxyz(3);
    Depth_pose_realsense.pose.position.y = rpyxyz(4);
    Depth_pose_realsense.pose.position.z = rpyxyz(5);
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
        // if (markerIds.size() > 1 ){cout << "Aruco Warning" << endl;}
    }else{Aruco_found = false; ArucoLostcounter++;}
    if (Aruco_init){
        if(Aruco_found){
            rvec = rvecs.front();
            tvec = tvecs.front();
            Vec3 Aruco_translation_camera(tvec(0),tvec(1),tvec(2));
            Vec3 Aruco_rpy_camera(rvec(0),rvec(1),rvec(2));
            Aruco_PosePub(Camera2World(Aruco_rpy_camera,Aruco_translation_camera,Camera_lp));
        }
    }
}
void Yolo_process(Mat image_rgb){
    if(!image_rgb.empty()){
        std_msgs::Bool got;
        Yolonet.rundarknet(image_rgb);
        if(Yolonet.obj_vector.size()!=0){
            got.data = true;
            YOLO_found = true;
            KF_init = true;
        }
        if(got.data){
            std_msgs::Int32 classname;
            double temp_depth = 1000;
            objectinfo temp;
            for (auto what : Yolonet.obj_vector){
                if(temp_depth > what.depth){
                    temp = what;
                    temp_depth = what.depth;
                }
            }
        if(temp.classnameofdetection == "nano"){
            classname.data = 0;
        }else if (temp.classnameofdetection == "talon"){
            classname.data = 1;
        }else if (temp.classnameofdetection == "F450"){
            classname.data = 2;
        }
        Vec3 Yolotvecs = camerapixel2tvec(Vec2I(temp.boundingbox.x + temp.boundingbox.width / 2,temp.boundingbox.y + temp.boundingbox.height / 2),temp.depth,CamParameters);
        YOLO_PosePub(Camera2World(Vec3(0,0,0),Yolotvecs,Camera_lp));
        cv::imshow("uav", image_rgb);
        cv::waitKey(1);
        }
    }
}
void callback(const sensor_msgs::CompressedImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth){
    /* Image initialize */
    try{
        image_rgb = cv::imdecode(cv::Mat(rgb->data),1);
        cv_bridge::CvImageConstPtr depth_ptr  = cv_bridge::toCvCopy(depth, depth->encoding);
        image_dep = depth_ptr->image;
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
    /* YOLO */
    Yolonet.getdepthdata(image_dep);
    Yolo_process(image_rgb);
    /* image plot */
    // cv::imshow("dep_out", image_dep);
}
void datalogger(){ 
    logger_time = ros::Time::now().toSec();
    if(logger_time-logger_time_last > 0.02){
        ofstream save("/home/jeremy/realsense_ws/src/log.csv", ios::app);
        save<<logger_time<<","<<KF_pub(0)<<","<<KF_pub(1)<<","<<KF_pub(2)<<endl;
        save.close();
        logger_time_last = logger_time;
    }
}
int main(int argc, char **argv){
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);
    ros::Subscriber camerapose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_d455/pose", 1, camera_pose_sub);
    ros::Subscriber uavtwist_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node/gh034_nano2/twist", 1, uav_twist_sub);
    ros::Subscriber uavpose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_nano2/pose", 1, uav_pose_sub);
    ros::Publisher ArucoPose_pub = nh.advertise<geometry_msgs::PoseStamped>("ArucoPose",1);
    ros::Publisher DepthPose_pub = nh.advertise<geometry_msgs::PoseStamped>("DepthPose",1);
    ros::Publisher YOLOPose_pub = nh.advertise<geometry_msgs::PoseStamped>("YoloPose",1);
    ros::Publisher LEDPose_pub = nh.advertise<geometry_msgs::PoseStamped>("LEDPose",1);
    ros::Publisher KFPose_pub = nh.advertise<geometry_msgs::PoseStamped>("KalmanFilterPose",1);
    message_filters::Subscriber<CompressedImage> rgb_sub(nh, "/camera/color/image_raw/compressed", 1);
    message_filters::Subscriber<Image> dep_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef sync_policies::ApproximateTime<CompressedImage, Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, dep_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    PNP3Dpoints();
    remove("/home/jeremy/realsense_ws/src/log.csv");
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
    Mat MeasureP = Mat::zeros(measSize, measSize, CV_32F);
    MeasureP.at<float>(0) = 1;
    MeasureP.at<float>(7) = 1;
    MeasureP.at<float>(14) = 1;
    Mat MeasureV = Mat::zeros(measSize, measSize, CV_32F);
    MeasureV.at<float>(21) = 1;
    MeasureV.at<float>(28) = 1;
    MeasureV.at<float>(35) = 1;
    setIdentity(KF.processNoiseCov, Scalar::all(1e-3)); 
    setIdentity(KF.measurementNoiseCov, Scalar::all(2e-1));
    // KF.measurementNoiseCov.at<float>(21) = 1e-1;
    // KF.measurementNoiseCov.at<float>(28) = 1e-1;
    // KF.measurementNoiseCov.at<float>(35) = 1e-1;
    setIdentity(KF.errorCovPost, Scalar::all(1));
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));

    while(ros::ok()){
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
            
            if(YOLO_found){    
                measurement.at<float>(0) = YOLO_pose_realsense.pose.position.x;
                measurement.at<float>(1) = YOLO_pose_realsense.pose.position.y;
                measurement.at<float>(2) = YOLO_pose_realsense.pose.position.z;
                measurement.at<float>(3) = UAV_twist[0];
                measurement.at<float>(4) = UAV_twist[1];
                measurement.at<float>(5) = UAV_twist[2];
                // KF.measurementMatrix = MeasureP;
                YOLO_found = false;
            }else{
                measurement.at<float>(0) = prediction.at<float>(0);
                measurement.at<float>(1) = prediction.at<float>(1);
                measurement.at<float>(2) = prediction.at<float>(2);
                measurement.at<float>(3) = UAV_twist[0];
                measurement.at<float>(4) = UAV_twist[1];
                measurement.at<float>(5) = UAV_twist[2];
                // KF.measurementMatrix = MeasureV;
            }
            /* update */
            KF.correct(measurement);
        }
        // ArucoPose_pub.publish(Aruco_pose_realsense);
        // DepthPose_pub.publish(Depth_pose_realsense);
        // LEDPose_pub.publish(LED_pose_realsense);
        YOLOPose_pub.publish(YOLO_pose_realsense);
        KFPose_pub.publish(KF_pose);
        datalogger();
        Yolonet.obj_vector.clear();
        /* ROS timer */
        auto TimerT = ros::Time::now().toSec();
        KFdT = TimerT-TimerLastT;
        cout << "System_Hz: " << 1/(TimerT-TimerLastT) << " dt: " << KFdT << endl;
        TimerLastT = TimerT;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}