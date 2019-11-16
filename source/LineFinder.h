#if!defined LINEF
#define LINEF

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#define PI 3.1415926

using namespace cv;

class LineFinder
{
private:
	//Դͼ��
	Mat img;
	//�����Ž������������
	std::vector<Vec4i> lines;
	//���۵ķֱ���
	double deltaRho;
	double deltaTheta;
	//�ж�Ϊ�ߵ���С������
	int minVote;
	//�ߵ���С����
	double minLength;
	//��������������
	double maxGap;
public:
	//Ĭ�����������Ϊ��1�����أ�1��Ϊ�뾶������û�м��Ҳû����С����
	LineFinder():deltaRho(1),deltaTheta(PI/180),minVote(10),minLength(0.),maxGap(0.){}

	//************��ص����ú���************//
	//���û������ķֱ���
	void setAccResolution(double dRho,double dTheta)
	{
		deltaRho = dRho;
		deltaTheta = dTheta;
	}

	//������С��ͶƱ��
	void setMinVote(int minv)
	{
		minVote = minv;
	}

	//�����߳��ͼ��
	void setLineLengthAndGap(double length,double gap)
	{
		minLength = length;
		maxGap = gap;
	}

	//��װ�ĸ���huogh�任����
	std::vector<Vec4i> findLines(Mat &binary)
	{
		lines.clear();
		HoughLinesP(binary,lines,deltaRho,deltaTheta,minVote,minLength,maxGap);
		return lines;
	}

	//��ͼ���ϻ���������
	void drawDetectedLines(Mat &image,Scalar color = Scalar(255,255,255))
	{
		//����
		std::vector<Vec4i>::const_iterator it2 = lines.begin();
		while(it2 != lines.end())
		{
			Point pt1((*it2)[0],(*it2)[1]);
			Point pt2((*it2)[2],(*it2)[3]);
			line(image,pt1,pt2,color);
			++it2;
		}
	}


};


#endif