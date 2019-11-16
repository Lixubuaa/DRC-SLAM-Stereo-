#if!defined LINEF
#define LINEF

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#define PI 3.1415926

using namespace cv;

class LineFinder
{
private:
	//源图像
	Mat img;
	//包含着结束点的向量：
	std::vector<Vec4i> lines;
	//积累的分辨率
	double deltaRho;
	double deltaTheta;
	//判定为线的最小的数量
	int minVote;
	//线的最小长度
	double minLength;
	//线内允许的最大间隔
	double maxGap;
public:
	//默认情况下设置为：1个像素，1度为半径搜索，没有间隔也没有最小长度
	LineFinder():deltaRho(1),deltaTheta(PI/180),minVote(10),minLength(0.),maxGap(0.){}

	//************相关的设置函数************//
	//设置积累器的分辨率
	void setAccResolution(double dRho,double dTheta)
	{
		deltaRho = dRho;
		deltaTheta = dTheta;
	}

	//设置最小的投票数
	void setMinVote(int minv)
	{
		minVote = minv;
	}

	//设置线长和间隔
	void setLineLengthAndGap(double length,double gap)
	{
		minLength = length;
		maxGap = gap;
	}

	//封装的概率huogh变换程序
	std::vector<Vec4i> findLines(Mat &binary)
	{
		lines.clear();
		HoughLinesP(binary,lines,deltaRho,deltaTheta,minVote,minLength,maxGap);
		return lines;
	}

	//在图像上画出检测的线
	void drawDetectedLines(Mat &image,Scalar color = Scalar(255,255,255))
	{
		//画线
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