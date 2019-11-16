#ifndef RL_TRACKING_H_
#define RL_TRACKING_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <fstream>
#include "generalDataFuction.h"
#include "linematcher.h"

using namespace cv;
using namespace std;

//道路线跟踪类
class RoadLineTrack
{
public:
	//公共成员函数
	//构造函数
	RoadLineTrack();
	//设置当前帧参数
	void setCurrentParams(Mat _image1, Mat _homography, Mat _cameraR, Mat _cameraT, Point3f _ground); 
	//未跟踪时重新寻找直线
	vector<vector<Point3f>> linesRefind(Mat& showimage); 
	//跟踪已挑选直线，outtype默认0输出世界坐标，1输出相机坐标
	vector<Point3f> linesTrack(vector<Point3f> w_points, Mat& showimage, int outtype = 0); 

private:
	//私有成员变量
	//文件存储类名
	ofstream trackfile;
	//当前帧的左图像、左右图像的单应矩阵、绝对坐标系到相机系的旋转矩阵、平移矩阵
	Mat currentLeftImage_, currentHomography_, currentCameraR_, currentCameraT_;
	//前一帧的左图像、绝对坐标系到相机系的旋转矩阵、平移矩阵
	Mat formerLeftImage_, formerCameraR_, formerCameraT_;
	//当前帧和前一帧的地面方程参数
	Point3f currentGroundParam_, formerGroundParam_;
	//前一帧到当前帧图像的旋转矩阵、平移矩阵
	Mat f2cCamerR_, f2cCamerT_;
	//当前帧和前一帧的最佳直线图像坐标
	vector<Point> current_ipoints, former_ipoints;

	//私有成员函数
	//图像处理检测所有直线左图像坐标
	vector<vector<Point>> lineDetect(Mat image, int threshold, double minLineLength, double maxLineGap, int flag = 0);
	//获取ROI的端点
	vector<Point> getROI(vector<Point> c_points, Mat image);
	//左图像坐标转换至左相机坐标
	vector<vector<Point3f>> image2camera(vector<vector<Point>> leftpoints, Mat homography);
	//世界坐标转换为左图像坐标
	vector<Point> world2image(vector<Point3f> w_points); 
	//世界坐标转换为左相机坐标
	vector<Point3f> world2camera(vector<Point3f> w_points); 
	//左相机坐标转为世界坐标
	vector<vector<Point3f>> camera2world(vector<vector<Point3f>> c_points); 
	//设置上一帧参数
	void setFormerParams(Mat _image1, Mat _cameraR, Mat _cameraT, Point3f _ground, vector<Point> _ipoints); 
	//特征点匹配评估法跟踪直线
	vector<Point3f> keylineMatch(vector<Point> theorypoints, Mat& showimage);
	//计算上一帧与当前帧的单应性矩阵
	Mat calf2cHomography(Mat& warp);
	//将上一帧的直线坐标转换到warp中
	vector<Point> calf2cpoints(vector<Point> fipoints, Mat f2cHomography);
	//计算公共区域直线的端点
	vector<Point> calcompoints(vector<Point> fipoints, Mat f2cHomography);
};

#endif
