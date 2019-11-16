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

//��·�߸�����
class RoadLineTrack
{
public:
	//������Ա����
	//���캯��
	RoadLineTrack();
	//���õ�ǰ֡����
	void setCurrentParams(Mat _image1, Mat _homography, Mat _cameraR, Mat _cameraT, Point3f _ground); 
	//δ����ʱ����Ѱ��ֱ��
	vector<vector<Point3f>> linesRefind(Mat& showimage); 
	//��������ѡֱ�ߣ�outtypeĬ��0����������꣬1����������
	vector<Point3f> linesTrack(vector<Point3f> w_points, Mat& showimage, int outtype = 0); 

private:
	//˽�г�Ա����
	//�ļ��洢����
	ofstream trackfile;
	//��ǰ֡����ͼ������ͼ��ĵ�Ӧ���󡢾�������ϵ�����ϵ����ת����ƽ�ƾ���
	Mat currentLeftImage_, currentHomography_, currentCameraR_, currentCameraT_;
	//ǰһ֡����ͼ�񡢾�������ϵ�����ϵ����ת����ƽ�ƾ���
	Mat formerLeftImage_, formerCameraR_, formerCameraT_;
	//��ǰ֡��ǰһ֡�ĵ��淽�̲���
	Point3f currentGroundParam_, formerGroundParam_;
	//ǰһ֡����ǰ֡ͼ�����ת����ƽ�ƾ���
	Mat f2cCamerR_, f2cCamerT_;
	//��ǰ֡��ǰһ֡�����ֱ��ͼ������
	vector<Point> current_ipoints, former_ipoints;

	//˽�г�Ա����
	//ͼ����������ֱ����ͼ������
	vector<vector<Point>> lineDetect(Mat image, int threshold, double minLineLength, double maxLineGap, int flag = 0);
	//��ȡROI�Ķ˵�
	vector<Point> getROI(vector<Point> c_points, Mat image);
	//��ͼ������ת�������������
	vector<vector<Point3f>> image2camera(vector<vector<Point>> leftpoints, Mat homography);
	//��������ת��Ϊ��ͼ������
	vector<Point> world2image(vector<Point3f> w_points); 
	//��������ת��Ϊ���������
	vector<Point3f> world2camera(vector<Point3f> w_points); 
	//���������תΪ��������
	vector<vector<Point3f>> camera2world(vector<vector<Point3f>> c_points); 
	//������һ֡����
	void setFormerParams(Mat _image1, Mat _cameraR, Mat _cameraT, Point3f _ground, vector<Point> _ipoints); 
	//������ƥ������������ֱ��
	vector<Point3f> keylineMatch(vector<Point> theorypoints, Mat& showimage);
	//������һ֡�뵱ǰ֡�ĵ�Ӧ�Ծ���
	Mat calf2cHomography(Mat& warp);
	//����һ֡��ֱ������ת����warp��
	vector<Point> calf2cpoints(vector<Point> fipoints, Mat f2cHomography);
	//���㹫������ֱ�ߵĶ˵�
	vector<Point> calcompoints(vector<Point> fipoints, Mat f2cHomography);
};

#endif
