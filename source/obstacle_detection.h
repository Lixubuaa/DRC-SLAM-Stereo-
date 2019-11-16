#ifndef OB_DETECTION_H_
#define OB_DETECTION_H_

#include "generalDataFuction.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/legacy/legacy.hpp>

using namespace std;
using namespace cv;

class ob_detection
{
private:			
	static Point3f planeguess(vector<Point3f>points);					//最小二乘法拟合平面方程ax+by+cz=1
	static bool injudge(Point3f point, Point3f modle, float t);			//判断点是否符合模型
	static float error_estimate(vector<Point3f>point, Point3f model);	//计算模型误差

public:
	ofstream fileground;
	Mat findH(Mat left, Mat right, Point3f& ransac_modle, vector<Point3f>& groundpoints, int filenum, cv::Mat WPlane);	//地面方程计算函数
	vector<pair<Point3f, int>> obstacletest(Mat left, Mat right, Point3f ransac_modle, Mat homography, Mat& disp, Mat showimage);  //SGBM障碍物检测函数
	Point3f ransac_Plane(vector<Point3f>points, vector<Point3f>&set, vector<int>&num, int iterat, float thresh);  //RANSAC平面估计函数
	static Point3f ransac_Plane(vector<Point3f>points, vector<Point3f>&set, vector<int>&num, int iterat, float t1, float t2, Point3f CPlane, float& r);  //RANSAC平面估计函数
};

#endif