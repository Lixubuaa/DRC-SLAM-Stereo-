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

//ä��ʶ����
class bw_recognition
{
private:
	//˽�г�Ա����
	//���������������ʵ����
	CascadeClassifier blindway_cascade;
	//ä��������������
	string blindway_cascade_name; 
	//ԭͼ��Ŀ���
	Mat frame;
	//�����������ͼ��
	Mat detect_result;
	//���Էָ�Ķ�ֵͼ
	Mat cascade;
	//���еķ����������
	vector<Rect> allblindways;
	//���ýǶ�x�����y�����Ա�������ˮƽ��ֱ����ֱ�߼��
	int directH, directV;
	
	//˽�г�Ա����
	//canny��ⴹֱ�����Ե
	void dxCanny(InputArray _src, OutputArray _dst, double low_thresh, double high_thresh);
	//���ķ�ɸѡ��ѱ߽�ֱ��
	void drawDetectLines(Mat& image, const vector<Vec4i> lines, vector<Point2f>& endpoints, vector<Point> centroid);
	//RANSAC���޳����Եľ���ä�������
	vector<int> ransacFindInliners(vector<int> centers, float thresh);

public:
	//������Ա����
	//���캯��������ԭͼ��
	bw_recognition(Mat src);
	//���������
	int cascadeDetect(string name, Scalar color);
	//�Լ����ɸѡ�޳�����ʾ���
	void cascadeDisplay(int vBlindwayNum, int hBlindwayNum);
	//��ˮ�뾫ȷ�ָ�ä��
	vector<Point2f> watershedBlindway(int filenum); 
};

//��ˮ���࣬������Ҫ�ǵ�����opencv���еķ�ˮ�������
class WatershedSegmenter{

private:
	//˽�г�Ա����
	//��ˮ��ı��ͼ��
	Mat markers;

public:
	//������Ա���������ں�������λ���������У������Щ������������
	//�����ͼ��ת��Ϊ����ͼ��
	void setMarkers(const Mat& markerImage){
		markerImage.convertTo(markers, CV_32S);
	}

	//ʹ�÷�ˮ���㷨
	Mat process(const Mat &image){
		watershed(image, markers);
		return markers;
	}

	//��ͼ�����ʽ���ؽ��
	Mat getSegmentation(){
		Mat tmp;
		//��ǩ����255�ķָ�һ�ɸ�ֵΪ255
		markers.convertTo(tmp, CV_8U);
		return tmp;
	}

	//��ͼ�����ʽ���ط�ˮ��
	Mat getWatersheds(){
		Mat tmp;
		//ת��ǰÿ�����ض�Ϊ255p+255
		markers.convertTo(tmp, CV_8U, 255, 255);
		return tmp;
	}
};


#endif