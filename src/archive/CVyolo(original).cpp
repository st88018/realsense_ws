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
bool KF_use_Depth = false;
double CV_lost_timer;
bool CV_lost = false;
bool Do_Aruco,Do_Yolo;
std_msgs::Bool KFok;
double Error_lp;
/* YOLO */
// static run_yolo Yolonet(cfgpath, weightpath, classnamepath, float(0.7));
bool YOLO_found = false;


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
void YOLO_PosePub(Vec3 xyz){
    xyz = uav_real_pose(xyz);
    YOLO_pose_realsense.header.stamp = ros::Time::now();
    YOLO_pose_realsense.header.frame_id = "world";
    YOLO_pose_realsense.pose.position.x = xyz(0);
    YOLO_pose_realsense.pose.position.y = xyz(1);
    YOLO_pose_realsense.pose.position.z = xyz(2);
    YOLO_pose_realsense.pose.orientation.w = UAVq.w();
    YOLO_pose_realsense.pose.orientation.x = UAVq.x();
    YOLO_pose_realsense.pose.orientation.y = UAVq.y();
    YOLO_pose_realsense.pose.orientation.z = UAVq.z();
}
void Depth_PosePub(Vec3 xyz){
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
void Aruco_process(Mat image_rgb, Mat image_dep){
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
        rvec = rvecs.front();
        tvec = tvecs.front();
        Vec3 Aruco_translation_camera(tvec(0),tvec(1),tvec(2));
        Vec3 Aruco_rpy_camera(rvec(0),rvec(1),rvec(2));
        Aruco_PosePub(Camera2World(Aruco_translation_camera,Camera_lp));
        double ArucoDepth = find_depth_avg(image_dep,markerConerABCDs.back());
        last_markerConerABCD = markerConerABCDs.back();
        Vec3 Depthtvecs = camerapixel2tvec(FindMarkerCenter(last_markerConerABCD),ArucoDepth,CamParameters);
        Depth_PosePub(Camera2World(Depthtvecs,Camera_lp));
    }
    // cv::imshow("uav", ArucoOutput);
    // cv::waitKey(1);
}
// void Yolo_process(Mat image_rgb, Mat image_dep){
//     Yolonet.getdepthdata(image_dep);
//     cv::Mat YoloOutput = image_rgb.clone();
//     if(!YoloOutput.empty()){
//         std_msgs::Bool got;
//         Yolonet.rundarknet(YoloOutput);
//         if(Yolonet.obj_vector.size()!=0){
//             got.data = true;
//             YOLO_found = true;
//         }
//         if(got.data){
//             std_msgs::Int32 classname;
//             double temp_depth = 1000;
//             objectinfo temp;
//             for (auto what : Yolonet.obj_vector){
//                 if(temp_depth > what.depth){
//                     temp = what;
//                     temp_depth = what.depth;
//                 }
//             }
//             if(temp.classnameofdetection == "nano"){
//                 classname.data = 0;
//             }else if (temp.classnameofdetection == "talon"){
//                 classname.data = 1;
//             }else if (temp.classnameofdetection == "F450"){
//                 classname.data = 2;
//             }
//         Vec3 Yolotvecs = camerapixel2tvec(Vec2I(temp.boundingbox.x + temp.boundingbox.width / 2,temp.boundingbox.y + temp.boundingbox.height / 2),temp.depth,CamParameters);
//         YOLO_PosePub(Camera2World(Yolotvecs,Camera_lp));
//         }
//     }
//     // cv::imshow("uav", YoloOutput);
//     // cv::waitKey(1);
// }
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
    if(Do_Aruco){
        Aruco_process(image_rgb,image_dep);
    }
    /* YOLO */
    // if(Do_Yolo){
    //     Yolo_process(image_rgb,image_dep);
    // }
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
bool KFok_indicator(){
    bool output;
    if(Aruco_found){ //Visualize of UAV KF OK
        CV_lost = false;
        output = true;
        KF_init = true;
    }
    if(YOLO_found){ //Visualize of UAV KF OK
        CV_lost = false;
        output = true;
        KF_init = true;
    }
    
    if(UAV_twist[0]==0 && UAV_twist[1]==0 && UAV_twist[2]==0){ //No twist input KF no OK
        output = false;
    }

    if(!CV_lost){
        CV_lost_timer = ros::Time::now().toSec();
        CV_lost = true;
    }
    if(CV_lost_timer - ros::Time::now().toSec() < -10){ //Visualize of UAV lost for 10 seceonds KF no OK
        KF_init = false;
        output = false;
        KF_PosePub(Zero7);
    }
    cout << "Aruco_found: " << Aruco_found << " YOLO_found: " << YOLO_found << endl;
    cout << "---------------------------------------------------" << endl;
    return(output);
}
int main(int argc, char **argv){
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);
    ros::Subscriber camerapose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_d455/pose", 1, camera_pose_sub);
    ros::Subscriber uavtwist_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 1, uav_twist_sub);
    ros::Subscriber uavpose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, uav_pose_sub);
    ros::Publisher ArucoPose_pub = nh.advertise<geometry_msgs::PoseStamped>("ArucoPose",1);
    ros::Publisher DepthPose_pub = nh.advertise<geometry_msgs::PoseStamped>("DepthPose",1);
    ros::Publisher YOLOPose_pub = nh.advertise<geometry_msgs::PoseStamped>("YoloPose",1);
    ros::Publisher KFPose_pub = nh.advertise<geometry_msgs::PoseStamped>("KalmanFilterPose",1);
    ros::Publisher KFok_pub = nh.advertise<std_msgs::Bool>("/KFok",1);
    message_filters::Subscriber<CompressedImage> rgb_sub(nh, "/camera/color/image_raw/compressed", 1);
    message_filters::Subscriber<Image> dep_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef sync_policies::ApproximateTime<CompressedImage, Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, dep_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    PNP3Dpoints();
    remove("/home/jeremy/realsense_ws/src/log.csv");
    
    /* System Params */
    ros::Rate loop_rate(100); /* ROS system Hz */
    Do_Aruco = true;
    Do_Yolo = true;

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
    // Mat MeasureP = Mat::zeros(measSize, measSize, CV_32F);
    // MeasureP.at<float>(0) = 1;
    // MeasureP.at<float>(7) = 1;
    // MeasureP.at<float>(14) = 1;
    // Mat MeasureV = Mat::zeros(measSize, measSize, CV_32F);
    // MeasureV.at<float>(21) = 1;
    // MeasureV.at<float>(28) = 1;
    // MeasureV.at<float>(35) = 1;
    setIdentity(KF.processNoiseCov, Scalar::all(1e-3)); 
    setIdentity(KF.measurementNoiseCov, Scalar::all(2e-1));
    // KF.measurementNoiseCov.at<float>(21) = 1e-1;
    // KF.measurementNoiseCov.at<float>(28) = 1e-1;
    // KF.measurementNoiseCov.at<float>(35) = 1e-1;
    setIdentity(KF.errorCovPost, Scalar::all(1));
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));
    KFok.data = false;

    while(ros::ok()){
        ros::spinOnce();
        KFok.data = KFok_indicator();
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
            
            // if(YOLO_found){    
            //     measurement.at<float>(0) = YOLO_pose_realsense.pose.position.x;
            //     measurement.at<float>(1) = YOLO_pose_realsense.pose.position.y;
            //     measurement.at<float>(2) = YOLO_pose_realsense.pose.position.z;
            //     measurement.at<float>(3) = UAV_twist[0];
            //     measurement.at<float>(4) = UAV_twist[1];
            //     measurement.at<float>(5) = UAV_twist[2];
            //     YOLO_found = false;
            //     CV_lost = false;
            //     KF.correct(measurement);
            //     Yolonet.obj_vector.clear();
            // }

            if(Aruco_found){
                Aruco_found = false;
                measurement.at<float>(0) = Aruco_pose_realsense.pose.position.x;
                measurement.at<float>(1) = Aruco_pose_realsense.pose.position.y;
                measurement.at<float>(2) = Aruco_pose_realsense.pose.position.z;
                measurement.at<float>(3) = UAV_twist[0];
                measurement.at<float>(4) = UAV_twist[1];
                measurement.at<float>(5) = UAV_twist[2];
                KF.correct(measurement);
                Aruco_found = false;
            }else{
                measurement.at<float>(0) = prediction.at<float>(0);
                measurement.at<float>(1) = prediction.at<float>(1);
                measurement.at<float>(2) = prediction.at<float>(2);
                measurement.at<float>(3) = UAV_twist[0];
                measurement.at<float>(4) = UAV_twist[1];
                measurement.at<float>(5) = UAV_twist[2];
                KF.correct(measurement);
            }
            //     KF_use_Depth = !KF_use_Depth;
            // }else if(Aruco_found && KF_use_Depth){
            //     Aruco_found = false;
            //     measurement.at<float>(0) = Depth_pose_realsense.pose.position.x;
            //     measurement.at<float>(1) = Depth_pose_realsense.pose.position.y;
            //     measurement.at<float>(2) = Depth_pose_realsense.pose.position.z;
            //     measurement.at<float>(3) = UAV_twist[0];
            //     measurement.at<float>(4) = UAV_twist[1];
            //     measurement.at<float>(5) = UAV_twist[2];
            //     KF.correct(measurement);
            //     Aruco_found = false;
            //     KF_use_Depth = !KF_use_Depth;
            
        }
        KFok_pub.publish(KFok);
        ArucoPose_pub.publish(Aruco_pose_realsense);
        DepthPose_pub.publish(Depth_pose_realsense);
        YOLOPose_pub.publish(YOLO_pose_realsense);
        KFPose_pub.publish(KF_pose);
        datalogger();
        
        /* ROS timer */
        auto TimerT = ros::Time::now().toSec();
        KFdT = TimerT-TimerLastT;
        cout << "---------------------------------------------------" << endl;
        cout << "System_Hz: " << 1/(TimerT-TimerLastT) << " dt: " << KFdT << endl;
        cout << "KFLOST_time: " << CV_lost_timer - ros::Time::now().toSec() << " KF_init: " << KF_init << endl;
        cout << "     X: " <<KF_pose.pose.position.x << " Y: " <<KF_pose.pose.position.y << " Z: " <<KF_pose.pose.position.z << endl;
        cout << "diff_X: " <<KF_pose.pose.position.x - UAV_pose_sub.pose.position.x <<
                    " Y: " <<KF_pose.pose.position.y - UAV_pose_sub.pose.position.y << 
                    " Z: " <<KF_pose.pose.position.z -UAV_pose_sub.pose.position.z << endl;
        Error_lp = sqrt(pow((KF_pose.pose.position.x-UAV_pose_sub.pose.position.x),2)+
                     pow((KF_pose.pose.position.y-UAV_pose_sub.pose.position.y),2)+
                     pow((KF_pose.pose.position.z-UAV_pose_sub.pose.position.z),2));
        cout << "error: " << Error_lp << endl;
        TimerLastT = TimerT;
        loop_rate.sleep();

    }
    return 0;
}