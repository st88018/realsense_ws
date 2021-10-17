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
static geometry_msgs::PoseStamped YOLO_pose_realsense,Camera_pose_sub,UAV_pose_sub;
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
/* YOLO */
static run_yolo Yolonet(cfgpath, weightpath, classnamepath, float(0.7));
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
void Yolo_process(Mat image_rgb, Mat image_dep){
    Yolonet.getdepthdata(image_dep);
    cv::Mat YoloOutput = image_rgb.clone();
    if(!YoloOutput.empty()){
        std_msgs::Bool got;
        Yolonet.rundarknet(YoloOutput);
        if(Yolonet.obj_vector.size()!=0){
            got.data = true;
            YOLO_found = true;
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
        YOLO_PosePub(Camera2World(Yolotvecs,Camera_lp));
        }
    }
    // cv::imshow("uav", YoloOutput);
    // cv::waitKey(1);
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
    Yolo_process(image_rgb,image_dep);
    /* ROS timer */
    auto TimerT = ros::Time::now().toSec();
    cout << "---------------------------------------------------" << endl;
    cout << "Yolo_Hz: " << 1/(TimerT-TimerLastT) << endl;
    TimerLastT = TimerT;
}
void datalogger(){ 
    logger_time = ros::Time::now().toSec();
    if(logger_time-logger_time_last > 0.01){
        ofstream save("/home/jeremy/realsense_ws/src/Yolo_raw.csv", ios::app);
        save << std::setprecision(20) << logger_time <<","<< YOLO_pose_realsense.pose.position.x <<","<< YOLO_pose_realsense.pose.position.y <<","<< YOLO_pose_realsense.pose.position.z << endl;
        save.close();
        logger_time_last = logger_time;
    }
}
int main(int argc, char **argv){
    ros::init(argc, argv, "Yolo");
    ros::NodeHandle nh;
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);
    ros::Subscriber camerapose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_d455/pose", 1, camera_pose_sub);
    ros::Subscriber uavpose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, uav_pose_sub);
    ros::Publisher YOLOPose_pub = nh.advertise<geometry_msgs::PoseStamped>("YoloPose",1);
    message_filters::Subscriber<CompressedImage> rgb_sub(nh, "/camera/color/image_raw/compressed", 1);
    message_filters::Subscriber<Image> dep_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef sync_policies::ApproximateTime<CompressedImage, Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, dep_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    remove("/home/jeremy/realsense_ws/src/Yolo_raw.csv");

    while(ros::ok()){
        ros::spinOnce();
        YOLOPose_pub.publish(YOLO_pose_realsense);
        datalogger();
    }
    return 0;
}