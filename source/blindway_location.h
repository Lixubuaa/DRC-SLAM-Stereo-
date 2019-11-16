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

//ä����λ��
class bw_location
{
private:
	//˽�г�Ա����
	//���ͼ
	Mat bird_imL; 
	//ԭͼ�����ͼ��ת������
	Mat H_ob;

	//˽�г�Ա����
	//�������ͼ
	void GetBirdviewImage(Mat im, Point3f CPlane);

public:
	//������Ա����
	//���캯��
	bw_location();
	//ä�������߶�λ����
	vector<Point3f> blindwaytest(Mat image1, Mat image2, Mat homography, Mat showimage, Mat CameraR, Mat CameraT, Point3f CPlane); 
};

#endif