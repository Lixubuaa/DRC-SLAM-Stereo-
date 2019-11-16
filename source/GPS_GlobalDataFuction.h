#ifndef GPSGLOBALDATA_H
#define GPSGLOBALDATA_H

#include <string>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <boost/thread.hpp>

#include <Eigen/Dense>

#include "rs232/rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"
#include "InitialParameter.h"

using namespace std;
using namespace cv;

#define M_PI 3.1415926
#define RADIANS_TO_DEGREES (180.0/M_PI)
#define DEGREES_TO_RADIANS (M_PI/180.0)
#define Ra  6378137

extern Mat IniR;
extern bool initflag;
extern ORB_SLAM2::InitialParameter InitialRot;

int an_packet_transmit(an_packet_t *an_packet);
void set_sensor_ranges();
void Euler2Rotmateix(float& yaw, float& roll, float& pitch, cv::Mat& rot);
double RotMatrix2Euler(const Eigen::Matrix3d &rot);
double RotMatrix2EulerSLAM(const Eigen::Matrix3d &rot);

cv::Mat ConvertLatLon2RT(cv::Point3d Orign, cv::Point3d current, cv::Mat Rotation);
Point2d ConvertLatLon2XY(double originLat, double originLon, double goalLat, double goalLon);
bool ISArriveDestination(cv::Point2d goal, cv::Point2d orig, double radius);


#endif