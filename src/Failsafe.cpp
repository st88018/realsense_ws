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
static geometry_msgs::PoseStamped UAV_pose_vicon,UGV_pose_vicon;
static geometry_msgs::Twist       UAV_twist_pub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Failsafe");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50); /* ROS system Hz */

    while(ros::ok()){
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}
