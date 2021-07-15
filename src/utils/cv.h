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

inline Vec2I FindMarkerCenter(const Vec8I& markerConerABCD){
    Vec2I MarkerCenter;
    MarkerCenter << (markerConerABCD[0]+markerConerABCD[2]+markerConerABCD[4]+markerConerABCD[6])/4,
                    (markerConerABCD[1]+markerConerABCD[3]+markerConerABCD[5]+markerConerABCD[7])/4;
    return(MarkerCenter);
}
inline Vec3 camerapixel2tvec (const Vec2I xy, double depth, Vec4 campaprameters )
{
    return Vec3 (
                ( xy ( 0,0 )-campaprameters[2] ) *depth/campaprameters[0],
                ( xy ( 1,0 )-campaprameters[3] ) *depth/campaprameters[1],
                depth
                );
}


#endif 