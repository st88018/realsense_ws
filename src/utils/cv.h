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

/*camera pramater*/
static double fx, fy, cx, cy; //focal length and principal point
static Vec4 CamParameters;
int ArucoLostcounter;
bool Aruco_found = false;
bool Aruco_init = false;
/* Constant velocity estimator */
std::deque<Vec8I> CVE_Corners;
/*PNP*/
cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);
cv::Mat depthcameraMatrix = cv::Mat::eye(3,3, CV_64F);
cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
vector<cv::Point3f> PNPPoints3D;
vector<Point2f> PNPPoints2D; //red orange green blue

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
void PNP3Dpoints(){  //Determine the LED pos in real world
    PNPPoints3D.push_back(cv::Point3f( 0, 0, 0)); //red
    PNPPoints3D.push_back(cv::Point3f( 0,60, 0)); //orange
    PNPPoints3D.push_back(cv::Point3f(60,60, 0)); //green
    PNPPoints3D.push_back(cv::Point3f(60, 0, 0)); //blue
}
Vec8 Aruco(Mat image_rgb){
    Vec8 ArucoTvecrvec;
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
    ArucoTvecrvec << tvecs[0][0],tvecs[0][1],tvecs[0][2],rvecs[0][0],rvecs[0][1],rvecs[0][2],0,0;
    return(ArucoTvecrvec);
    // cv::imshow("Aruco_out", ArucoOutput);
    // cv::waitKey(1);
}
Vec6 LEDTvecRvec(Mat image_rgb){
    Vec6 PNPtvecrvec;
    Vec3d PNPrvec, PNPtvec;

    Mat image_jpg = imread("./test.jpg");
    imshow("image_jpg", image_jpg);
    Mat image_hsv,image_threshold;
    cvtColor(image_jpg, image_hsv, COLOR_BGR2HSV);

    inRange(image_hsv, Scalar(0, 0, 100), Scalar(255, 255, 255), image_threshold);  
    dilate(image_threshold, image_threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(image_threshold, image_threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    
    vector<vector<Point> > contours;
    findContours( image_threshold, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );
    unsigned int ledcounts = contours.size();
    vector<Moments> mu(ledcounts);
    vector<Point2f> mc(ledcounts); 
    for( size_t i = 0; i < ledcounts; i++ ){
        mu[i] = moments( contours[i] );
    }
    for( size_t i = 0; i < ledcounts; i++ ){ 
        mc[i] = Point2f( static_cast<float>(mu[i].m10 / (mu[i].m00 + 1e-5)), 
                         static_cast<float>(mu[i].m01 / (mu[i].m00 + 1e-5)) ); //add 1e-5 to avoid division by zero
        // cout << "mc[" << i << "]=" << mc[i] << endl;
    }
    vector<int> mc_hue(ledcounts);
    for (unsigned int i = 0; i < ledcounts; i++){
        mc_hue[i] = image_hsv.at<Vec3b>(mc[i])[0];
        // cout << "mc_hue[" << i << "]=" << mc_hue[i] << endl;
    }
    vector<int> mc_hue_sort = mc_hue;
    sort(mc_hue_sort.begin(), mc_hue_sort.end());
    
    PNPPoints2D.clear();
    for (unsigned int i = 0; i < ledcounts; i++){
        Point2f pos2D_temp;
        for (unsigned int j = 0; j < ledcounts; j++){
            if (mc_hue_sort[i]==mc_hue[j]){
                pos2D_temp = mc[j];
            }
        }
        PNPPoints2D.push_back(pos2D_temp);
        // cout << "PNPPoints2D[" << i << "]=" << pos2D_temp << endl;
    }
    solvePnP(PNPPoints3D, PNPPoints2D, cameraMatrix, distCoeffs, PNPrvec, PNPtvec, false, SOLVEPNP_ITERATIVE);

    PNPtvecrvec << PNPtvec[0]*0.001,PNPtvec[1]*0.001,PNPtvec[2]*0.001,PNPrvec[0],PNPrvec[1],PNPrvec[2];
    return(PNPtvecrvec);
    // imshow("image_threshold", image_threshold);
    // imshow("image_hsv", image_hsv);
    // waitKey(1);
}
#endif 