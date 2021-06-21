#include <iostream>
#include <iomanip>
#include <ros/ros.h>
#include <sstream>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <algorithm>
#include <utility>
#include <deque>
#include <nmmintrin.h>
#include <chrono>
using namespace  std;
using namespace  cv;
using namespace  sensor_msgs;
deque <Mat> two_frame;

void callback(const sensor_msgs::CompressedImageConstPtr & rgb){
    //read image
    string file1="/home/sy/modern_ws/devel/lib/mode/1.png";
    string file2="/home/sy/modern_ws/devel/lib/mode/2.png";


    cv::Mat image_rgb;
  //  cv::Mat image_dep;
    try
    {
        image_rgb = cv::imdecode(cv::Mat(rgb->data),1);
        //cout << "seq: " << rgb->header.seq << endl;
        //cout << "stamp: " << rgb->header.stamp << endl;
        //cout << "frame_id: " << rgb->header.frame_id << endl;


  //      image_dep = cv::imdecode(cv::Mat(depth->data),1);

    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    two_frame.push_back(image_rgb);
    if (two_frame.size() == 1)
    {
        cout << "only one frame " << endl;
    }
    else
    {
        Mat img_1 = two_frame.front();
        Mat img_2 = two_frame.back();
        assert(img_1.data != nullptr && img_2.data != nullptr);

        //--intialization
        std::vector <KeyPoint> keypoints_1, keypoints_2;
        Mat descriptors_1, descriptors_2;
        Ptr<FeatureDetector> detector = ORB::create();
        Ptr<DescriptorExtractor> descriptor = ORB::create();
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

        //-first step: detect Oriented FAST corner position
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        detector->detect(img_1, keypoints_1);
        detector->detect(img_2, keypoints_2);

        //-second step: compute BRIEF descriptor according to corner position
        descriptor->compute(img_1, keypoints_1, descriptors_1);
        descriptor->compute(img_2, keypoints_2, descriptors_2);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> (t2-t1);

        cout << "extract ORB cost = " << time_used.count() << " seconds. " << endl;

        Mat outimg1;
        drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
        imshow("ORB features", outimg1);

        //third step: match BRIEF descriptor
        vector<DMatch> matches;
        t1 = chrono::steady_clock::now();
        matcher->match(descriptors_1, descriptors_2, matches);
        t2 = chrono::steady_clock::now();
        time_used = chrono::duration_cast<chrono::duration<double>> (t2 - t1);
        cout << "match ORB cost = " << time_used.count() << " seconds. " << endl;

        //fourth step: filter matching points
        //compute min distance and max distance
        auto min_max = minmax_element(matches.begin(), matches.end(),[](const DMatch &m1, const DMatch &m2) {return m1.distance < m2.distance;});
        double min_dist = min_max.first->distance;
        double max_dist = min_max.second->distance;

        cout << "--Max dist : " << max_dist << endl;
        cout << "--Min dist : " << min_dist << endl;
        cout << endl;
        //
        std::vector<DMatch> good_matches;
        for (int i = 0; i < descriptors_1.rows; i++){
            if (matches[i].distance <= max(2 * min_dist, 30.0)){
                    good_matches.push_back(matches[i]);
            }
        }

        // fifth step: plot
        Mat img_match;
        Mat img_goodmatch;
        drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
        drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
        imshow("all matches", img_match);
        imshow("good matches", img_goodmatch);
        waitKey(1);

        two_frame.pop_front();
    }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature");
    ros::NodeHandle nh;
    ros::Subscriber img_sub = nh.subscribe("/camera/color/image_raw/compressed", 1, callback);

    ros::Rate loop_rate(20);
    cout << "he" <<endl;
    while(ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }


    return 0;
}

