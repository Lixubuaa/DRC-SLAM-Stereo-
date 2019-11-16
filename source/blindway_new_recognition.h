#ifndef BW_NEW_RECOGNITION_H_
#define BW_NEW_RECOGNITION_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "generalDataFuction.h"

using namespace std;
using namespace cv;

//盲道识别类
class bw_recognition
{
private:
	//私有成员变量
	//级联分类器检测类实例化
	CascadeClassifier blindway_cascade;
	//盲道分类器的名字
	string blindway_cascade_name; 
	//原图像的拷贝
	Mat frame;
	//分类器检测结果图像
	Mat detect_result;
	//粗略分割的二值图
	Mat cascade;
	//所有的分类器检测结果
	vector<Rect> allblindways;
	//设置角度x方向和y方向，以便适用于水平或垂直方向直线检测
	int directH, directV;
	
	//私有成员函数
	//canny检测垂直方向边缘
	void dxCanny(InputArray _src, OutputArray _dst, double low_thresh, double high_thresh);
	//质心法筛选最佳边界直线
	void drawDetectLines(Mat& image, const vector<Vec4i> lines, vector<Point2f>& endpoints, vector<Point> centroid);
	//RANSAC法剔除明显的局外盲道检测结果
	vector<int> ransacFindInliners(vector<int> centers, float thresh);

public:
	//公共成员函数
	//构造函数，传入原图像
	bw_recognition(Mat src);
	//分类器检测
	int cascadeDetect(string name, Scalar color);
	//对检测结果筛选剔除并显示结果
	void cascadeDisplay(int vBlindwayNum, int hBlindwayNum);
	//分水岭精确分割盲道
	vector<Point2f> watershedBlindway(int filenum); 
};

//分水岭类，其中主要是调用了opencv库中的分水岭各函数
class WatershedSegmenter{

private:
	//私有成员变量
	//分水岭的标记图像
	Mat markers;

public:
	//公共成员函数，由于函数定义位于类声明中，因而这些都是内联函数
	//将标记图像转换为整数图像
	void setMarkers(const Mat& markerImage){
		markerImage.convertTo(markers, CV_32S);
	}

	//使用分水岭算法
	Mat process(const Mat &image){
		watershed(image, markers);
		return markers;
	}

	//以图像的形式返回结果
	Mat getSegmentation(){
		Mat tmp;
		//标签高于255的分割一律赋值为255
		markers.convertTo(tmp, CV_8U);
		return tmp;
	}

	//以图像的形式返回分水岭
	Mat getWatersheds(){
		Mat tmp;
		//转换前每个像素都为255p+255
		markers.convertTo(tmp, CV_8U, 255, 255);
		return tmp;
	}
};


#endif