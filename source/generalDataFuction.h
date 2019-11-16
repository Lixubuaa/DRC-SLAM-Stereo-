#ifndef GENERALDF_H_
#define GENERALDF_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/legacy/legacy.hpp>

using namespace std;
using namespace cv;

//��������궨����
extern double _cameraMatrix1[3][3];
extern double _cameraMatrix2[3][3];
extern double _T[3];
extern double _R[3][3];
extern int filenum;
extern string data;
extern string mydataset;
extern vector<string> data_dir; //Left(0),Right(1),Disparity(2),Showimage(3),Road(4),Rightpoint(5),Line(6)
extern vector<string> mydataset_dir; //image_0,image_1

void SetCalibrateParams();                                 //���ñ궨����
Point3f measureZ(Point2f Point1, Point2f Point2);							//��ά������㺯��
void RandT(float A, float B, float C, Mat& R, Mat& T);					//����ϵת�����㺯��
void trans(Mat R, Mat T, vector<Point3f>src, vector<Point3f>&dst);	//����ת������
void trans(Mat R, Mat T, vector<pair<Point3f, int>>src, vector<Point3f>&dst);
template<class T>
string to_string(const T t) //ģ�庯��תstring
{
	ostringstream oss;//����һ����
	oss << t;//��ֵ����������
	return oss.str();
}

#endif