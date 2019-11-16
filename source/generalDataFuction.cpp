#include "stdafx.h"
#include "generalDataFuction.h"
#include "LineFinder.h"


string data, mydataset;
int filenum = 0;
vector<string> data_dir(0);
vector<string> mydataset_dir(0);

//广角相机参数
double _cameraMatrix1[3][3] = {};
double _cameraMatrix2[3][3] = {};
double _T[3] = {};
double _R[3][3] = {};

void SetCalibrateParams()
{
	// Load settings related to stereo calibration
	cv::FileStorage fsSettings("Settings.yaml", cv::FileStorage::READ);
	if (!fsSettings.isOpened())
	{
		cerr << "ERROR: Wrong path to settings" << endl;
		return;
	}

	double fx, fy, cx, cy, bf;

	fsSettings["Camera.fx"] >> fx;
	fsSettings["Camera.fy"] >> fy;
	fsSettings["Camera.cx"] >> cx;
	fsSettings["Camera.cy"] >> cy;
	fsSettings["Camera.bf"] >> bf;

	double cameraLMatrix[3][3] = { { fx / 2, 0., cx / 2 }, { 0., fy / 2, cy / 2 }, { 0, 0, 1 } };
	double cameraRMatrix[3][3] = { { fx / 2, 0., cx / 2 }, { 0., fy / 2, cy / 2 }, { 0, 0, 1 } };
	double T[3] = { -bf * 1000 / fx, 0, 0 };
	double R[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

	memcpy(_cameraMatrix1, cameraLMatrix, sizeof(cameraLMatrix));
	memcpy(_cameraMatrix2, cameraRMatrix, sizeof(cameraRMatrix));
	memcpy(_T, T, sizeof(T));
	memcpy(_R, R, sizeof(R));
}

//计算匹配点三维坐标
Point3f measureZ(Point2f Point1, Point2f Point2)
{
	Point3f P;
	float fenzi = _cameraMatrix2[0][0] * _T[0] + _cameraMatrix2[0][2] * _T[2] - _T[2] * Point2.x;
	float fenmu1 = (_R[2][0] * Point2.x - _cameraMatrix2[0][0] * _R[0][0] - _cameraMatrix2[0][2] * _R[2][0])*(Point1.x - _cameraMatrix1[0][2]) / _cameraMatrix1[0][0];
	float fenmu2 = (_R[2][1] * Point2.x - _cameraMatrix2[0][0] * _R[0][1] - _cameraMatrix2[0][2] * _R[2][1])*(Point1.y - _cameraMatrix1[1][2]) / _cameraMatrix1[1][1];
	float fenmu3 = _cameraMatrix2[0][0] * _R[0][2] + _cameraMatrix2[0][2] * _R[2][2] - _R[2][2] * Point2.x;
	P.z = fenzi / (fenmu1 + fenmu2 - fenmu3);
	P.y = P.z * (Point1.y - _cameraMatrix1[1][2]) / _cameraMatrix1[1][1];
	P.x = P.z * (Point1.x - _cameraMatrix1[0][2]) / _cameraMatrix1[0][0];
	return P;
};

//转换矩阵计算
void RandT(float A, float B, float C, Mat& R, Mat& T){
	float thetaZ, thetaX;

	if (A*B >= 0)
		thetaZ = -acos(abs(B) / sqrt(A*A + B*B));
	else
		thetaZ = acos(abs(B) / sqrt(A*A + B*B));

	if (B*C >= 0)
		thetaX = PI - acos(abs(C) / sqrt(B*B + C*C));
	else
		thetaX = acos(abs(C) / sqrt(B*B + C*C));

	cout << "绕z轴角度" << thetaZ * 180 / PI << endl;
	cout << "绕x轴角度" << thetaX * 180 / PI << endl;

	Mat RZ = Mat::zeros(3, 3, CV_32FC1);
	Mat RX = Mat::zeros(3, 3, CV_32FC1);

	RZ.at<float>(0, 0) = cos(thetaZ);
	RZ.at<float>(0, 1) = sin(thetaZ);
	RZ.at<float>(0, 2) = 0;
	RZ.at<float>(1, 0) = -sin(thetaZ);
	RZ.at<float>(1, 1) = cos(thetaZ);
	RZ.at<float>(1, 2) = 0;
	RZ.at<float>(2, 0) = 0;
	RZ.at<float>(2, 1) = 0;
	RZ.at<float>(2, 2) = 1;

	RX.at<float>(0, 0) = 1;
	RX.at<float>(0, 1) = 0;
	RX.at<float>(0, 2) = 0;
	RX.at<float>(1, 0) = 0;
	RX.at<float>(1, 1) = cos(thetaX);
	RX.at<float>(1, 2) = sin(thetaX);
	RX.at<float>(2, 0) = 0;
	RX.at<float>(2, 1) = -sin(thetaX);
	RX.at<float>(2, 2) = cos(thetaX);

	R = RX * RZ;

	T.at<float>(0) = 0;
	T.at<float>(1) = 0;
	T.at<float>(2) = 1 / sqrt(A*A + B*B + C*C);
}

//坐标系变换
void trans(Mat R, Mat T, vector<Point3f>src, vector<Point3f>&dst){
	for (int i = 0; i < src.size(); i++)
	{
		Point3f p;
		p.x = R.at<float>(0, 0)*src[i].x + R.at<float>(0, 1)*src[i].y + R.at<float>(0, 2)*src[i].z + T.at<float>(0);
		p.y = R.at<float>(1, 0)*src[i].x + R.at<float>(1, 1)*src[i].y + R.at<float>(1, 2)*src[i].z + T.at<float>(1);
		p.z = R.at<float>(2, 0)*src[i].x + R.at<float>(2, 1)*src[i].y + R.at<float>(2, 2)*src[i].z + T.at<float>(2);
		dst.push_back(p);
	}
}

void trans(Mat R, Mat T, vector<pair<Point3f, int>>src, vector<Point3f>&dst){
	for (int i = 0; i < src.size(); i++)
	{
		Point3f p;
		p.x = R.at<float>(0, 0)*src[i].first.x + R.at<float>(0, 1)*src[i].first.y + R.at<float>(0, 2)*src[i].first.z + T.at<float>(0);
		p.y = R.at<float>(1, 0)*src[i].first.x + R.at<float>(1, 1)*src[i].first.y + R.at<float>(1, 2)*src[i].first.z + T.at<float>(1);
		p.z = R.at<float>(2, 0)*src[i].first.x + R.at<float>(2, 1)*src[i].first.y + R.at<float>(2, 2)*src[i].first.z + T.at<float>(2);
		dst.push_back(p);
	}
}