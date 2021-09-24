#include <ros/ros.h>
#include <string>
#include <numeric>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include "utils/kinetic_math.hpp"

static geometry_msgs::PoseStamped UGV_pose_sub,UGV_estimated_pose_pub;
static Vec7 UGV_lp; // xyz yaw
Vec3 pose_XYyaw;

void ugv_pose_sub(const geometry_msgs::PoseStamped::ConstPtr& pose){
    UGV_pose_sub.pose.position.x = pose->pose.position.x;
    UGV_pose_sub.pose.position.y = pose->pose.position.y;
    UGV_pose_sub.pose.position.z = pose->pose.position.z;
    UGV_pose_sub.pose.orientation.w = pose->pose.orientation.w;
    UGV_pose_sub.pose.orientation.x = pose->pose.orientation.x;
    UGV_pose_sub.pose.orientation.y = pose->pose.orientation.y;
    UGV_pose_sub.pose.orientation.z = pose->pose.orientation.z;
    UGV_lp << UGV_pose_sub.pose.position.x,UGV_pose_sub.pose.position.y,UGV_pose_sub.pose.position.z,
              UGV_pose_sub.pose.orientation.w,UGV_pose_sub.pose.orientation.x,
              UGV_pose_sub.pose.orientation.y,UGV_pose_sub.pose.orientation.z;
    Vec3 localrpy = Q2rpy(localq);
    pose_XYyaw = Vec4(UGV_pose_sub.pose.position.x,UGV_pose_sub.pose.position.y,
                      UGV_pose_sub.pose.position.z,localrpy[2]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Failsafe");
    ros::NodeHandle nh;
    ros::Subscriber ugvpose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("/vrpn_client_node/gh034_car/pose", 5, ugv_pose_sub);
    ros::Publisher  ugv_estpos_pub = nh.advertise<geometry_msgs::PoseStamped>
                    ("/mavros/UGV_estimated_pose/pose", 5);
    ros::Rate loop_rate(20); /* ROS system Hz */

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}
