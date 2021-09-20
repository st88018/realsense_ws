#include <ros/ros.h>
#include <string>
#include <numeric>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/State.h>
#include "utils/kinetic_math.hpp"
#include "utils/trajectories.hpp"

static mavros_msgs::State current_state;
static geometry_msgs::PoseStamped UGV_pose_sub,UAV_pose_sub;
static geometry_msgs::Twist       UAV_twist_pub;
static Vec7 UAV_lp,UGV_lp;


void uav_twist_pub(Vec4 vxyzaz){
    UAV_twist_pub.linear.x = vxyzaz(0);
    UAV_twist_pub.linear.y = vxyzaz(1);
    UAV_twist_pub.linear.z = vxyzaz(2);
    UAV_twist_pub.angular.z= vxyzaz(3);
}
void ugv_pose_sub(const geometry_msgs::PoseStamped::ConstPtr& pose){
    UGV_pose_sub.pose.position.x = pose->pose.position.x;
    UGV_pose_sub.pose.position.y = pose->pose.position.y;
    UGV_pose_sub.pose.position.z = pose->pose.position.z;
    UGV_pose_sub.pose.orientation.w = pose->pose.orientation.w;
    UGV_pose_sub.pose.orientation.x = pose->pose.orientation.x;
    UGV_pose_sub.pose.orientation.y = pose->pose.orientation.y;
    UGV_pose_sub.pose.orientation.z = pose->pose.orientation.z;
    UGV_lp << UGV_pose_sub.pose.position.x,UGV_pose_sub.pose.position.y,UGV_pose_sub.pose.position.z,
              UGV_pose_sub.pose.orientation.w,UGV_pose_sub.pose.orientation.x,UGV_pose_sub.pose.orientation.y,UGV_pose_sub.pose.orientation.z;
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
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Failsafe");
    ros::NodeHandle nh;
    ros::Subscriber ugvpose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("/vrpn_client_node/gh034_car/pose", 5, ugv_pose_sub);
    ros::Subscriber uavpose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("mavros/local_position/pose", 1, uav_pose_sub);
    ros::Publisher uav_vel_pub = nh.advertise<geometry_msgs::Twist>
                    ("/FSM/cmd_vel_unstamped", 5);
    ros::Rate loop_rate(50); /* ROS system Hz */

    while(ros::ok()){
        
        uav_vel_pub.publish(UAV_twist_pub);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
