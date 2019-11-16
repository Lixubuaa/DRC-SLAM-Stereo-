#ifndef BW_LOCATION_H_
#define BW_LOCATION_H_

#include "generalDataFuction.h"
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "math.h"
#include "string.h"

using namespace std;
using namespace cv;

//盲道定位类
class bw_location
{
private:
	//私有成员变量
	//鸟瞰图
	Mat bird_imL; 
	//原图像到鸟瞰图的转换矩阵
	Mat H_ob;

	//私有成员函数
	//计算鸟瞰图
	void GetBirdviewImage(Mat im, Point3f CPlane);

public:
	//公共成员函数
	//构造函数
	bw_location();
	//盲道中心线定位函数
	vector<Point3f> blindwaytest(Mat image1, Mat image2, Mat homography, Mat showimage, Mat CameraR, Mat CameraT, Point3f CPlane); 
};

#endif