#ifndef CV_DEF_H
#define CV_DEF_H
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "common.h"

using namespace cv;

/* Constant velocity estimator */
std::deque<Vec8I> CVE_Corners;

inline Vec2I FindMarkerCenter(const Vec8I& markerConerABCD){
    Vec2I MarkerCenter;
    MarkerCenter << (markerConerABCD[0]+markerConerABCD[2]+markerConerABCD[4]+markerConerABCD[6])/4,
                    (markerConerABCD[1]+markerConerABCD[3]+markerConerABCD[5]+markerConerABCD[7])/4;
    return(MarkerCenter);
}
inline Vec3 camerapixel2tvec (const Vec2I xy, double depth, Vec4 campaprameters ){
    return Vec3 (
                ( xy ( 0,0 )-campaprameters[2] ) *depth/campaprameters[0],
                ( xy ( 1,0 )-campaprameters[3] ) *depth/campaprameters[1],
                depth
                );
}
double find_depth_avg(cv::Mat image_dep, Vec8I markerConerABCD){
    Vec2I markerCenterXY = FindMarkerCenter(markerConerABCD);
    double CornerLength1 = sqrt((pow(markerConerABCD[0]-markerConerABCD[2],2))
                                +(pow(markerConerABCD[1]-markerConerABCD[3],2))); // in pixel
    double CornerLength2 = sqrt((pow(markerConerABCD[4]-markerConerABCD[6],2))
                                +(pow(markerConerABCD[5]-markerConerABCD[7],2))); // in pixel
    double MidDepthTotal = 0;
    int valid_count = 0;
    for(int i = 0; i < max(CornerLength1,CornerLength2); i++){
        for(int j = 0; j < max(CornerLength1,CornerLength2); j++){
            double MidDepthij = 0.001 * image_dep.at<ushort>(markerCenterXY[1]-max(CornerLength1,CornerLength2)+i,
                                                     markerCenterXY[0]-max(CornerLength1,CornerLength2)+j);
            if (MidDepthij > 0 ){
                MidDepthTotal += MidDepthij;
                valid_count++;
            }
        }
    }
    if (valid_count == 0){
        return(0);
    }else{
        return(MidDepthTotal/valid_count);
    }
}
Vec2I Constant_velocity_predictor(const Vec8I last_markerConer,const int Lostcounter){
    if (Lostcounter == 0){
        CVE_Corners.push_back(last_markerConer);
        if (CVE_Corners.size() > 5){ //calculate last five
            CVE_Corners.pop_front();
        }
        return(FindMarkerCenter(last_markerConer));
    }else{
        Vec2I XY = Vec2I(0,0);
        for(unsigned int i=0; i < (CVE_Corners.size()-1) ;i++){
            Vec2I diff_xy = FindMarkerCenter(CVE_Corners.at(i)) - FindMarkerCenter(CVE_Corners.at(i+1));
            XY += diff_xy;
            XY /= CVE_Corners.size();
            XY *= Lostcounter;
        }
        // return(FindMarkerCenter(CVE_Corners.back())+XY);
        return(FindMarkerCenter(last_markerConer));
    }
}
Vec3I HSVaverage(cv::Mat BGRmat){
    cv::Mat HSVmat;
    int Haverage,Saverage,Vaverage;
    int Htotal = 0;
    int Stotal = 0;
    int Vtotal = 0;
    cvtColor(BGRmat, HSVmat, COLOR_BGR2HSV);
    for (int i=0; i<1280; i++){
        for (int j=0; j<720; j++){
            Htotal += HSVmat.at<Vec3b>(j,i)[0];
            Stotal += HSVmat.at<Vec3b>(j,i)[1];
            Vtotal += HSVmat.at<Vec3b>(j,i)[2];
        }
    }
    return(Vec3I(Htotal/921600,Stotal/921600,Vtotal/921600));
}
// void cameraconfig(cv::Mat BGRmat){ //Cam Auto config
//     if(HSVaverage(BGRmat)[2] < 50){
//         cout << "GO GO" << endl;
//     }
// }
Vec2I FindLEDCenter(cv::Mat SingleMarker){
    Vec2I PosXY;
    Moments oMoments = moments(SingleMarker);
    double dM01 = oMoments.m01;
    double dM10 = oMoments.m10;
    double dArea = oMoments.m00;

    cout << "dM01: " << dM01 << endl;
    cout << "dM10: " << dM10 << endl;
    cout << "dArea: " << dArea << endl;

    PosXY[1] = dM10 / dArea;
    PosXY[1] = dM01 / dArea;

    return(PosXY);
}
void imageprocess(){
    // system("./E10S50.sh");
    cv::Mat image_jpg = imread("./test.jpg");

    cv::imshow("image_jpg", image_jpg);

    cv::Mat image_hsv, image_Rthreshold, image_Gthreshold, image_Bthreshold,image_threshold;
    cvtColor(image_jpg, image_hsv, COLOR_BGR2HSV);
    inRange(image_hsv, Scalar(0, 0, 100), Scalar(255, 255, 255), image_threshold);
    inRange(image_hsv, Scalar(0, 0, 100), Scalar(60, 255, 255), image_Rthreshold); //Threshold the image
    // inRange(image_hsv, Scalar(38, 150, 150), Scalar(75, 255, 255), image_Gthreshold);
    // inRange(image_hsv, Scalar(75, 150, 150), Scalar(130, 255, 255), image_Bthreshold);

    // cout << "GreenRGB: " << image_jpg.at<Vec3b>(519,199) << endl;
    // cout << "GreenHSV: " << image_hsv.at<Vec3b>(519,199) << endl;
    // cout << "BlueRGB: " << image_jpg.at<Vec3b>(525,238) << endl;
    // cout << "BlueHSV: " << image_hsv.at<Vec3b>(525,238) << endl;
    // cout << "RedRGB: " << image_jpg.at<Vec3b>(537,300) << endl;
    // cout << "RedHSV: " << image_hsv.at<Vec3b>(537,300) << endl;
    // cout << "TestRGB: " << image_jpg.at<Vec3b>(53,85) << endl;
    // cout << "TestHSV: " << image_hsv.at<Vec3b>(53,85) << endl;

    FindLEDCenter(image_Rthreshold);

    // int Vcount = 0; 
    // for (int i=0; i<1280; i++){
    //     for (int j=0; j<720; j++){
    //         if (image_hsv.at<Vec3b>(j,i)[2] > 80){
    //             Vcount++;
    //         }
    //     }
    // }

    // cout << "Vcount: " << Vcount << endl;

    // cv::imwrite("image_rgb.jpg",image_rgb);

    //https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html

    cv::imshow("image_Rthreshold", image_Rthreshold);
    // cv::imshow("image_Gthreshold", image_Gthreshold);
    // cv::imshow("image_Bthreshold", image_Bthreshold);
    cv::imshow("image_hsv", image_hsv);
    cv::waitKey(1);
}
#endif 