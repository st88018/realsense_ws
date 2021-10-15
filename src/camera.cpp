#include <sstream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/PointStamped.h"
#include <std_msgs/Int32.h>

#include "utils/run_yolo.h"
#include <string>

using namespace std;

double TimerLastT,TimerT;

static cv::Mat frame, res, gt;

static cv::String weightpath ="/home/jeremy/lly_ws/src/offb/src/include/yolo/uav.weights";
static cv::String cfgpath ="/home/jeremy/lly_ws/src/offb/src/include/yolo/uav.cfg";
static cv::String classnamepath = "/home/jeremy/lly_ws/src/offb/src/include/yolo/uav.names";

static run_yolo Yolonet(cfgpath, weightpath, classnamepath, float(0.1));


void callback(const sensor_msgs::CompressedImageConstPtr & rgbimage, const sensor_msgs::ImageConstPtr & depth)
{

    cv_bridge::CvImageConstPtr depth_ptr;
    try
    {
        depth_ptr  = cv_bridge::toCvCopy(depth, depth->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat image_dep = depth_ptr->image;

    Yolonet.getdepthdata(image_dep);

    try
    {
        frame = cv::imdecode(cv::Mat(rgbimage->data),1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
   // cout<<"test"<<frame.size<<endl;
}


int main(int argc, char** argv)
{

    cout<<"Object detection..."<<endl;

    ros::init(argc, argv, "yolotiny");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::CompressedImage> subimage(nh, "/camera/color/image_raw/compressed", 1);
    message_filters::Subscriber<sensor_msgs::Image> subdepth(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subimage, subdepth);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Publisher pub_yolo_boo = nh.advertise<std_msgs::Bool>("/object/yolo/torf", 1);
    ros::Publisher pub_yolo_pos = nh.advertise<geometry_msgs::Point>("/object/yolo/pos",1);
    ros::Publisher pub_yolo_cls = nh.advertise<std_msgs::Int32>("/object/yolo/class",1);

    while(ros::ok())
    {      
        if(!frame.empty())
        {
            std_msgs::Bool got;
            Yolonet.rundarknet(frame);
            if(Yolonet.obj_vector.size()!=0)
                got.data = true;

            if(got.data)
            {
                std_msgs::Int32 classname;

                double temp_depth = 1000;
                objectinfo temp;

                for (auto what : Yolonet.obj_vector)
                {
                    if(temp_depth > what.depth)
                    {
                        temp = what;
                        temp_depth = what.depth;
                    }
                }

                if(temp.classnameofdetection == "nano")
                    classname.data = 0;
                else if (temp.classnameofdetection == "talon")
                    classname.data = 1;
                else if (temp.classnameofdetection == "F450")
                    classname.data = 2;

                // cout<<temp.classnameofdetection<<endl<<endl;;
                geometry_msgs::Point tempp;
                tempp.x = temp.boundingbox.x + temp.boundingbox.width / 2;
                tempp.y = temp.boundingbox.y + temp.boundingbox.height / 2;
                tempp.z = temp.depth;

                pub_yolo_boo.publish(got);
                pub_yolo_pos.publish(tempp);
                pub_yolo_cls.publish(classname);

            }
            else
            {
                pub_yolo_boo.publish(got);
            }
            cv::imshow("uav", frame);
            cv::waitKey(1);
        }

        ros::spinOnce();
        /* ROS timer */
        auto TimerT = ros::Time::now().toSec();
        cout << "System_Hz: " << 1/(TimerT-TimerLastT) << endl;
        TimerLastT = TimerT;
    }
    ros::spin();
    return 0;
}
