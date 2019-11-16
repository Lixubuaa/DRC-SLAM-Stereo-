#ifndef GENERALDF_H_
#define GENERALDF_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/legacy/legacy.hpp>

using namespace std;
using namespace cv;

//左右相机标定参数
extern double _cameraMatrix1[3][3];
extern double _cameraMatrix2[3][3];
extern double _T[3];
extern double _R[3][3];
extern int filenum;
extern string data;
extern string mydataset;
extern vector<string> data_dir; //Left(0),Right(1),Disparity(2),Showimage(3),Road(4),Rightpoint(5),Line(6)
extern vector<string> mydataset_dir; //image_0,image_1

void SetCalibrateParams();                                 //设置标定参数
Point3f measureZ(Point2f Point1, Point2f Point2);							//三维坐标计算函数
void RandT(float A, float B, float C, Mat& R, Mat& T);					//坐标系转换计算函数
void trans(Mat R, Mat T, vector<Point3f>src, vector<Point3f>&dst);	//坐标转换函数
void trans(Mat R, Mat T, vector<pair<Point3f, int>>src, vector<Point3f>&dst);
template<class T>
string to_string(const T t) //模板函数转string
{
	ostringstream oss;//创建一个流
	oss << t;//把值传递如流中
	return oss.str();
}

#endif