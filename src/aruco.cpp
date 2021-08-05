#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <string>
#include <algorithm>
#include <fstream>
#include <iostream>
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

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

//camera intrinsic pramater
static double fx, fy, cx, cy; //focal length and principal point

// uav local parameter

static geometry_msgs::PoseStamped Aruco_pose;
static geometry_msgs::PoseStamped Aruco_pose_gt;
static geometry_msgs::PoseStamped Camera_pose_gt;

//IRR filter parameter

cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);
cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
void camera_info_cb(const sensor_msgs::CameraInfoPtr& msg){

    fx = msg->K[0];
    fy = msg->K[4];
    cx = msg->K[2];
    cy = msg->K[5];
    cameraMatrix.at<double>(0,0) = fx;
    cameraMatrix.at<double>(1,1) = fy;
    cameraMatrix.at<double>(0,2) = cx;
    cameraMatrix.at<double>(1,2) = cy;

}
void CameraPose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){

    //get Camera pose

    Camera_pose_gt.pose.position.x = pose->pose.position.x;
    Camera_pose_gt.pose.position.y = pose->pose.position.y;
    Camera_pose_gt.pose.position.z = pose->pose.position.z;
    Camera_pose_gt.pose.orientation.x = pose->pose.orientation.x;
    Camera_pose_gt.pose.orientation.y = pose->pose.orientation.y;
    Camera_pose_gt.pose.orientation.z = pose->pose.orientation.z;
    Camera_pose_gt.pose.orientation.w = pose->pose.orientation.w;
}

void ArucoPose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){

    //get Aruco pose

    Aruco_pose_gt.pose.position.x = pose->pose.position.x;
    Aruco_pose_gt.pose.position.y = pose->pose.position.y;
    Aruco_pose_gt.pose.position.z = pose->pose.position.z;
    Aruco_pose_gt.pose.orientation.x = pose->pose.orientation.x;
    Aruco_pose_gt.pose.orientation.y = pose->pose.orientation.y;
    Aruco_pose_gt.pose.orientation.z = pose->pose.orientation.z;
    Aruco_pose_gt.pose.orientation.w = pose->pose.orientation.w;
}
static cv::Vec3d tv;
void MarkerPose_(const cv::Vec3d rvecs, const cv::Vec3d tvecs){
    tv = tvecs;

    Eigen::Matrix3d camera_rotation = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Aruco_rotation = Eigen::Matrix3d::Identity();
    Eigen::Quaterniond q;
    q.w() = Camera_pose_gt.pose.orientation.w;
    q.x() = Camera_pose_gt.pose.orientation.x;
    q.y() = Camera_pose_gt.pose.orientation.y;
    q.z() = Camera_pose_gt.pose.orientation.z;
    camera_rotation = q.matrix();

    Eigen::Vector3d t_c(1,0,0);
    t_c(0) = tvecs(0);
    t_c(1) = tvecs(1);
    t_c(2) = tvecs(2);
    //cout << "point in camera frame " << t_c << endl;
   //cout << "camera_rotation" << camera_rotation << endl;

    Eigen::Vector3d t_w = camera_rotation * t_c;
    //cout << "point in world frame " << t_w << endl;



    Aruco_pose.header.frame_id = "world";
    Aruco_pose.pose.position.x = t_c(0);
    Aruco_pose.pose.position.y = t_c(1);
    Aruco_pose.pose.position.z = t_c(2);
    Aruco_pose.pose.orientation.w = 1.0;
    Aruco_pose.pose.orientation.x = 0.0;
    Aruco_pose.pose.orientation.y = 0.0;
    Aruco_pose.pose.orientation.z = 0.0;

}
void Depth_Cal(const geometry_msgs::PoseStamped& Aruco_pose_gt , const geometry_msgs::PoseStamped& Camera_pose_gt)
{
    //  relative pose between aruco and camera under vicon
    Eigen::Vector3d gt_;
    gt_(0) = Aruco_pose_gt.pose.position.x - Camera_pose_gt.pose.position.x;
    gt_(1) = Aruco_pose_gt.pose.position.y - Camera_pose_gt.pose.position.y;
    gt_(2) = Aruco_pose_gt.pose.position.z - Camera_pose_gt.pose.position.z;

    // relative estimated pose of aruco marker in camera frame
    Eigen::Vector3d est_;
    est_(0) = Aruco_pose.pose.position.x ;
    est_(1) = Aruco_pose.pose.position.y ;
    est_(2) = Aruco_pose.pose.position.z ;

    cout << "relative pose between aruco and camera under vicon \n"  << gt_ << endl;
    //cout << "relative pose of aruco old \n" << est_ << endl;
    //cout << "--------------\n " << endl;
    //cout << "relative pose of aruco update \n" << tv << endl;
    //cout << "-------------- \n" << endl;
}


void callback(const sensor_msgs::CompressedImageConstPtr & rgb, const sensor_msgs::ImageConstPtr & depth)
{
    //cout << "hi " << endl;

    cv::Mat image_rgb;
    try
    {
        image_rgb = cv::imdecode(cv::Mat(rgb->data),1);

    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat outputImage = image_rgb.clone();
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(image_rgb, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);


    // if at least one marker detected
    if (markerIds.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
        std::vector<cv::Vec3d> rvecs, tvecs;

        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.129, cameraMatrix, distCoeffs, rvecs, tvecs);
        MarkerPose_(rvecs[0], tvecs[0]);
        cout << "tvecs[0] \n" << tvecs[0] << endl;
        for(int i=0; i<markerIds.size(); i++){

            cv::aruco::drawAxis(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
            cout << "ith relative pose of aruco old \n" << tvecs[i] << endl;
        }


    }

    cv::imshow("out", outputImage);
    cv::waitKey(1);


    //depth_alignment call back
    cv_bridge::CvImagePtr depth_ptr  = cv_bridge::toCvCopy(depth, depth->encoding);
    cv::Mat image_dep = depth_ptr->image;

}
void rgb_cb(const sensor_msgs::CompressedImageConstPtr & rgb)
{
    cv::Mat image_rgb;
    try
    {
        image_rgb = cv::imdecode(cv::Mat(rgb->data),1);

    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat outputImage = image_rgb.clone();
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(image_rgb, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);


    // if at least one marker detected
    if (markerIds.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
        std::vector<cv::Vec3d> rvecs, tvecs;

        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.124, cameraMatrix, distCoeffs, rvecs, tvecs);


        for(int i=0; i<markerIds.size(); i++){

            cv::aruco::drawAxis(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
            cout << "translational \n" << tvecs[i] << endl;
        }
        MarkerPose_(rvecs[0], tvecs[0]);


    }

    cv::imshow("out", outputImage);
    cv::waitKey(1);

}

int main(int argc, char **argv)
{
    cout<<"hello camera "<<endl;
    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::drawMarker(dictionary, 23, 400, markerImage, 1);
    cv::imwrite("marker23.png", markerImage);
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/color/camera_info",1,camera_info_cb);
    ros::Subscriber camerapose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/D435i/pose", 1, CameraPose_cb);
    ros::Subscriber Arucopose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/Aruco/pose", 1, ArucoPose_cb);
    ros::Subscriber rgb_sub = nh.subscribe("/camera/color/image_raw/compressed", 1, rgb_cb);
    ros::Publisher MarkerPose_pub = nh.advertise<geometry_msgs::PoseStamped>("ArucoPose",1);

//    message_filters::Subscriber<CompressedImage> rgb_sub(nh, "/camera/color/image_raw/compressed", 1);
//    message_filters::Subscriber<Image> dep_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);

//    typedef sync_policies::ApproximateTime<CompressedImage, Image> MySyncPolicy;
//    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, dep_sub);
//    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        Depth_Cal(Aruco_pose_gt,Camera_pose_gt);
        MarkerPose_pub.publish(Aruco_pose);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



