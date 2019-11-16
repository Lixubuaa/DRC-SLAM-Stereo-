/*
*兼容任意线与盲道线跟踪
*HoughP检测、LBD描述、BFM匹配
*修改日期：2017.7.14
*/
#include "roadline_tracking.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <windows.h>

#define LINE_HEIGHT_SCALE 3

//构造函数
RoadLineTrack::RoadLineTrack()
{
	trackfile.open(data + "/TrackData.txt");
}

//设置当前帧参数，传入当前帧左图像、单应矩阵、世界系到相机系的旋转平移矩阵、地面方程参数
void RoadLineTrack::setCurrentParams(Mat _image1, Mat _homography, Mat _cameraR, Mat _cameraT, Point3f _ground)
{
	currentLeftImage_ = _image1.clone();
	currentHomography_ = _homography.clone();
	currentCameraR_ = _cameraR.clone();
	currentCameraT_ = _cameraT.clone();
	currentGroundParam_ = _ground;
}

//将当前帧参数存储作为下一帧跟踪匹配的参照，记为前一帧参数
void RoadLineTrack::setFormerParams(Mat _image1, Mat _cameraR, Mat _cameraT, Point3f _ground, vector<Point> _ipoints)
{
	formerLeftImage_ = _image1.clone();
	formerCameraR_ = _cameraR.clone();
	formerCameraT_ = _cameraT.clone();
	formerGroundParam_ = _ground;
	former_ipoints = _ipoints;
}

//左图像坐标转换为相机系坐标
vector<vector<Point3f>> RoadLineTrack::image2camera(vector<vector<Point>> leftpoints, Mat homography)
{
	//标定参数为1/2，因此需要将图像坐标缩小一半以计算三维坐标
	vector<vector<Point2f>> half_rightpoints(leftpoints.size());
	vector<vector<Point2f>> half_leftpoints(leftpoints.size());

	//遍历每个左图像坐标
	for (int i = 0; i < leftpoints.size(); i++)
	{
		//每一列代表一个左图像坐标，同时处理一条直线的两个端点
		Mat l = Mat::ones(3, 2, CV_64F);
		l.at<double>(0, 0) = (double)leftpoints[i][0].x / 2;
		l.at<double>(1, 0) = (double)leftpoints[i][0].y / 2;
		l.at<double>(0, 1) = (double)leftpoints[i][1].x / 2;
		l.at<double>(1, 1) = (double)leftpoints[i][1].y / 2;

		//单应性矩阵计算左图像坐标在右图中的对应坐标
		Mat r = homography*l;

		//转换为其次坐标式，[x/z,y/z,1]
		Point2f pt1(r.at<double>(0, 0) / r.at<double>(2, 0), r.at<double>(1, 0) / r.at<double>(2, 0));
		Point2f pt2(r.at<double>(0, 1) / r.at<double>(2, 1), r.at<double>(1, 1) / r.at<double>(2, 1));

		//存储对应的左右图像坐标
		half_leftpoints[i].push_back(Point2f(l.at<double>(0, 0), l.at<double>(1, 0)));
		half_leftpoints[i].push_back(Point2f(l.at<double>(0, 1), l.at<double>(1, 1)));
		half_rightpoints[i].push_back(pt1);
		half_rightpoints[i].push_back(pt2);
	}

	//对每一对左右图像点计算相机系坐标
	vector<vector<Point3f>> c_points(leftpoints.size());
	for (int i = 0; i < leftpoints.size(); i++)
	{
		c_points[i].push_back(measureZ(half_leftpoints[i][0], half_rightpoints[i][0]));
		c_points[i].push_back(measureZ(half_leftpoints[i][1], half_rightpoints[i][1]));
	}

	return c_points;
}

//世界系坐标转换为左图像坐标
vector<Point> RoadLineTrack::world2image(vector<Point3f> w_points)
{
	//根据旋转平移矩阵将世界系坐标转换为当前相机系坐标
	vector<Point3f> c_points(0);
	for (int i = 0; i < w_points.size(); i++)
	{
		Mat Xw(3, 1, CV_32F);
		Xw.at<float>(0) = w_points[i].x;
		Xw.at<float>(1) = w_points[i].y;
		Xw.at<float>(2) = w_points[i].z;
		Mat Xc(3, 1, CV_32F);
		Xc = currentCameraR_*Xw + currentCameraT_;
		c_points.push_back(Point3f(Xc.at<float>(0), Xc.at<float>(1), Xc.at<float>(2)));
	}

	//通过左相机的标定矩阵再将相机系坐标转至左图像坐标
	vector<Point2f> origin_ipoints(0);
	for (int i = 0; i < c_points.size(); i++)
	{
		float x = _cameraMatrix1[0][0] * 2 * c_points[i].x + _cameraMatrix1[0][2] * 2 * c_points[i].z;
		float y = _cameraMatrix1[1][1] * 2 * c_points[i].y + _cameraMatrix1[1][2] * 2 * c_points[i].z;
		float z = c_points[i].z;

		//转换为其次形式，[x/z,y/z,1]
		origin_ipoints.push_back(Point2f(x / z, y / z));
	}

	//计算直线的斜率和截距
	float k = (origin_ipoints[1].y - origin_ipoints[0].y + 0.1) / (origin_ipoints[1].x - origin_ipoints[0].x + 0.1);
	float b = origin_ipoints[1].y - k*origin_ipoints[1].x;

	//如果最低点小于1/4图像高度，则返回0值
	if (fmax(origin_ipoints[0].y, origin_ipoints[1].y) < currentLeftImage_.rows / LINE_HEIGHT_SCALE)
	{
		vector<Point> cut_ipoints = { Point(0, 0), Point(0, 0) };
		return cut_ipoints;
	}

	//在图像高度方向裁剪直线，裁剪或延迟直线至1/3——1图像高度
	Point ip1, ip2;
	//点1的纵坐标选取，判断最高点与1/3图像高度的关系，取大者
	ip1.y = fmax(fmin(origin_ipoints[0].y, origin_ipoints[1].y), currentLeftImage_.rows / LINE_HEIGHT_SCALE);
	//点2的纵坐标选取，直接选取图像高度
	ip2.y = currentLeftImage_.rows - 1;
	//计算相应的横坐标
	ip1.x = (ip1.y - b) / k;
	ip2.x = (ip2.y - b) / k;

	//在图像宽度方向裁剪直线，防止越界
	if (ip1.x < 0)
		ip1 = Point(0, b);
	if (ip1.x > currentLeftImage_.cols)
		ip1 = Point(currentLeftImage_.cols - 1, k*(currentLeftImage_.cols - 1) + b);
	if (ip2.x < 0)
		ip2 = Point(0, b);
	if (ip2.x > currentLeftImage_.cols)
		ip2 = Point(currentLeftImage_.cols - 1, k*(currentLeftImage_.cols - 1) + b);

	//返回裁剪结果
	vector<Point> cut_ipoints(0);
	cut_ipoints.push_back(ip1);
	cut_ipoints.push_back(ip2);

	return cut_ipoints;
}

//世界系坐标转换为相机系坐标
vector<Point3f> RoadLineTrack::world2camera(vector<Point3f> w_points)
{
	//根据旋转平移矩阵将世界系坐标转换到当前相机系坐标，并返回
	vector<Point3f> c_points(0);
	for (int i = 0; i < w_points.size(); i++)
	{
		Mat Xw(3, 1, CV_32F);
		Xw.at<float>(0) = w_points[i].x;
		Xw.at<float>(1) = w_points[i].y;
		Xw.at<float>(2) = w_points[i].z;
		Mat Xc(3, 1, CV_32F);
		Xc = currentCameraR_*Xw + currentCameraT_;
		c_points.push_back(Point3f(Xc.at<float>(0), Xc.at<float>(1), Xc.at<float>(2)));
	}

	return c_points;
}

//相机系坐标转换为世界系坐标
vector<vector<Point3f>> RoadLineTrack::camera2world(vector<vector<Point3f>> c_points)
{
	//根据旋转平移矩阵将当前相机系坐标转换为世界系坐标，并返回
	vector<vector<Point3f>> w_points(c_points.size());
	for (int i = 0; i < c_points.size(); i++)
	{
		for (int j = 0; j < c_points[i].size(); j++)
		{
			Mat Xc(3, 1, CV_32F);
			Xc.at<float>(0) = c_points[i][j].x;
			Xc.at<float>(1) = c_points[i][j].y;
			Xc.at<float>(2) = c_points[i][j].z;
			Mat X(3, 1, CV_32F);
			X = currentCameraR_.t()*(Xc - currentCameraT_);
			float x = X.at<float>(0);
			float y = X.at<float>(1);
			float z = X.at<float>(2);
			w_points[i].push_back(cv::Point3f(x, y, z));
		}
	}

	return w_points;
}

//区域内直线检测
vector<vector<Point>> RoadLineTrack::lineDetect(Mat image, int threshold, double minLineLength, double maxLineGap, int flag)
{
	//转换为灰度图
	Mat gray;
	cvtColor(image, gray, CV_BGR2GRAY);

	//canny检测边缘
	Mat edge;
	Canny(gray, edge, 20, 80);

	//霍夫概率变换检测直线
	vector<Vec4i> lines;
	HoughLinesP(edge, lines, 1, CV_PI / 180, threshold, minLineLength, maxLineGap);

	//对检测到的每条直线的端点选择性处理，原直线返回或延长至图像首尾高度返回
	vector<vector<Point>> leftpoints(lines.size());
	for (int i = 0; i < lines.size(); i++)
	{
		Point pt1(lines[i][0], lines[i][1]);
		Point pt2(lines[i][2], lines[i][3]);
		
		//若flag为1，则返回延长线的端点图像坐标
		if (flag == 1) 
		{
			//计算直线斜率和截距
			float k = (pt1.y - pt2.y + 0.1) / (pt1.x - pt2.x + 0.1);
			float b = pt2.y - k*pt2.x;
			
			//直线延长至图像首尾高度
			Point iph(-b / k, 0);
			Point ipl(((image.rows - 1) - b) / k, image.rows - 1);

			//在图像宽度方向裁剪直线，防止越界
			if (iph.x < 0)
				iph = Point(0, b);
			if (iph.x > image.cols)
				iph = Point(image.cols - 1, k*(image.cols - 1) + b);
			if (ipl.x < 0)
				ipl = Point(0, b);
			if (ipl.x > image.cols)
				ipl = Point(image.cols - 1, k*(image.cols - 1) + b);

			leftpoints[i].push_back(iph);
			leftpoints[i].push_back(ipl);
		}
		//若flag为0，则返回原线段端点图像坐标，默认
		else if (flag == 0)
		{
			leftpoints[i].push_back(pt1);
			leftpoints[i].push_back(pt2);
		}
	}

	return leftpoints;
}

//获取局部感兴趣区域的端点
vector<Point> RoadLineTrack::getROI(vector<Point> i_points, Mat image)
{
	//计算x方向和y方向的最大最小值
	int xmin = min(i_points[0].x, i_points[1].x);
	int xmax = max(i_points[0].x, i_points[1].x);
	int ymin = min(i_points[0].y, i_points[1].y);
	int ymax = max(i_points[0].y, i_points[1].y);
	//为防止接近水平或垂直，设置矩形最小的宽度和高度
	int xlength = (xmax - xmin > 30) ? 0 : 30;
	int ylength = (ymax - ymin > 30) ? 0 : 30;
	//在x方向或y方向扩展相应的大小
	Point highpoint(min(xmax + xlength, image.cols - 1), min(ymax + ylength, image.rows - 1));
	Point lowpoint(max(xmin - xlength, 0), max(ymin - ylength, 0));

	//返回ROI的左上和右下端点坐标
	vector<Point> ROIpoints = { highpoint, lowpoint };
	return ROIpoints;
}

//计算前一帧图像与当前帧图像地面间的单应矩阵，返回单应矩阵和单应性变换后的图像
Mat RoadLineTrack::calf2cHomography(Mat& warp)
{
	//以上一帧为世界坐标系，P_f = K[I|0]，P_c = K[f2cCamerR|f2cCamerT]
	//计算f2c的旋转和平移矩阵 f2cCamerR*X_f + f2cCamerT = X_c
	f2cCamerR_ = currentCameraR_*formerCameraR_.inv();
	f2cCamerT_ = -currentCameraR_*formerCameraR_.inv()*formerCameraT_ + currentCameraT_;

	//前后帧的标定矩阵相同
	Mat K = (Mat_<double>(3, 3) << _cameraMatrix1[0][0] * 2, _cameraMatrix1[0][1], _cameraMatrix1[0][2] * 2,
		_cameraMatrix1[1][0], _cameraMatrix1[1][1] * 2, _cameraMatrix1[1][2] * 2,
		_cameraMatrix1[2][0], _cameraMatrix1[2][1], _cameraMatrix1[2][2]);

	//地面的法向量 n.t*X+d=0
	Mat n = (Mat_<double>(3, 1) << formerGroundParam_.x, formerGroundParam_.y, formerGroundParam_.z);
	double d = -1.0;

	//计算f2c的单应性矩阵 H=K*(R-T*n.t/d)*K.inv
	f2cCamerR_.convertTo(f2cCamerR_, CV_64F);
	f2cCamerT_.convertTo(f2cCamerT_, CV_64F);
	Mat f2cHomography = K*(f2cCamerR_ - f2cCamerT_*n.t() / d)*K.inv();

	//单应变换
	warpPerspective(formerLeftImage_, // input image
		warp,			// output image
		f2cHomography,		// homography
		Size(formerLeftImage_.cols, formerLeftImage_.rows), CV_INTER_LINEAR); // size of output image

	/*Mat diff = current_image1 - warp;
	diff.convertTo(diff, CV_8UC1);
	imshow("diff", diff);*/

	return f2cHomography;
}

//根据单应性矩阵，将前一帧图像坐标转换为当前帧图像坐标
vector<Point> RoadLineTrack::calf2cpoints(vector<Point> fipoints, Mat f2cHomography)
{
	vector<Point> _ipoints;

	//遍历每个前一帧直线图像坐标
	for (int i = 0; i < fipoints.size(); i++)
	{
		Mat p = (Mat_<double>(3, 1) << fipoints[i].x, fipoints[i].y, 1.0);

		//单应矩阵计算当前帧图像坐标
		Mat r = f2cHomography*p;

		//转换为其次形式
		_ipoints.push_back(Point(r.at<double>(0) / r.at<double>(2), r.at<double>(1) / r.at<double>(2)));
	}

	//计算直线的斜率和截距，将直线最低点延长至图像高度
	float k = (_ipoints[1].y - _ipoints[0].y + 0.1) / (_ipoints[1].x - _ipoints[0].x + 0.1);
	float b = _ipoints[1].y - k*_ipoints[1].x;
	Point iph = _ipoints[0];
	Point ipl(((currentLeftImage_.rows - 1) - b) / k, currentLeftImage_.rows - 1);

	//返回延长后的当前帧图像坐标
	vector<Point> cipoints;
	cipoints.push_back(iph);
	cipoints.push_back(ipl);

	return cipoints;
}

//根据单应性矩阵，计算前一帧最佳直线位于前后帧公共区域内长度的端点图像坐标
vector<Point> RoadLineTrack::calcompoints(vector<Point> fipoints, Mat f2cHomography)
{
	//计算前一帧最近直线的斜率和截距
	float origink = (fipoints[1].y - fipoints[0].y + 0.1) / (fipoints[1].x - fipoints[0].x + 0.1);
	float originb = fipoints[1].y - origink*fipoints[1].x;

	vector<Point> _ipoints;
	//遍历前一帧直线的图像坐标，计算其在当前帧的图像坐标
	for (int i = 0; i < fipoints.size(); i++)
	{
		Mat p = (Mat_<double>(3, 1) << fipoints[i].x, fipoints[i].y, 1.0);
		//单应矩阵计算当前帧图像坐标
		Mat r = f2cHomography*p;
		//转换为其次形式
		_ipoints.push_back(Point(r.at<double>(0) / r.at<double>(2), r.at<double>(1) / r.at<double>(2)));
	}

	//根据转换得到的当前帧图像坐标求公共区域的端点
	//计算转换到当前帧的直线斜率和截距
	float warpk = (_ipoints[1].y - _ipoints[0].y + 0.1) / (_ipoints[1].x - _ipoints[0].x + 0.1);
	float warpb = _ipoints[1].y - warpk*_ipoints[1].x;
	//求出直线上纵坐标为图像高度的点，反变换到前一帧，得到端点
	Mat after = (Mat_<double>(3, 1) << ((currentLeftImage_.rows - 1) - warpb) / warpk, currentLeftImage_.rows - 1, 1.0);
	Mat before = f2cHomography.inv()*after;
	//转换为其次形式
	Point lowpoint = Point(before.at<double>(0) / before.at<double>(2), before.at<double>(1) / before.at<double>(2));

	//位于公共区域内的直线高度不能短于20
	if (lowpoint.y - fipoints[0].y < 20)
		lowpoint = Point((fipoints[0].y + 20 - originb) / origink, fipoints[0].y + 20);

	//返回直线在公共区域的端点图像坐标
	vector<Point> compoints;
	compoints.push_back(fipoints[0]);
	compoints.push_back(lowpoint);

	return compoints;
}

//直线匹配确定当前帧最佳直线
vector<Point3f> RoadLineTrack::keylineMatch(vector<Point> theorypoints, Mat& showimage)
{
	double t0, t1, t2, t3, t4, t5, t6;
	t0 = (double)clock() / CLOCKS_PER_SEC;

	//根据反投影图像坐标获取局部ROI端点坐标，从而获取ROI图像
	vector<Point> current_hlpoints = getROI(theorypoints, currentLeftImage_);
	Point ROIlowpoint = current_hlpoints[1];
	Mat current_ROI = currentLeftImage_(Rect(current_hlpoints[0], current_hlpoints[1]));

	//直线特征描绘类
	LineDescriptor ld;
	vector<KeyLine> keylinesL, keylinesR;
	Mat descriptorL, descriptorR;

	t1 = (double)clock() / CLOCKS_PER_SEC;

	//计算前后帧图像地面的单应矩阵及单应变换图像
	Mat warp;
	Mat f2cHomography = calf2cHomography(warp);
	//imshow("warp", warp);
	/*char fm0[50];
	sprintf_s(fm0, (data_dir[6] + "/Warp%d").c_str(), filenum);
	strcat_s(fm0, ".jpg");
	imwrite(fm0, warp);*/
	
	//计算前一帧最佳直线位于前后帧公共区域内长度的端点图像坐标
	vector<Point> compoints = calcompoints(former_ipoints, f2cHomography);

	//计算前一帧最佳直线在前一帧图像中的keyline
	keylinesL.push_back(LineDescriptor::calKeyline(formerLeftImage_, compoints[0], compoints[1], 0));

	//通过一个循环变化直线检测的投票数来保证局部区域内检测到的直线在3-10条之间
	int threshold = 100;
	int lastmark = 0, currentmark = 0;
	do 
	{
		//如果不满足跳出条件，则清空前一次计算的直线keyline
		keylinesR.clear();

		//局部区域内的直线检测
		vector<vector<Point>> ROIlineExtremes = lineDetect(current_ROI, threshold, 0.1*current_ROI.rows, 3, 1);
		
		//在原图下计算检测到直线的keyline
		vector<vector<Point>> lineExtremes(ROIlineExtremes.size());
		for (int i = 0; i < ROIlineExtremes.size(); i++)
		{
			ROIlineExtremes[i][0] += ROIlowpoint;
			ROIlineExtremes[i][1] += ROIlowpoint;

			//类似的延长和裁剪步骤，主要将最低点延长至图像高度
			float k = (ROIlineExtremes[i][1].y - ROIlineExtremes[i][0].y + 0.1) / (ROIlineExtremes[i][1].x - ROIlineExtremes[i][0].x + 0.1);
			float b = ROIlineExtremes[i][1].y - k*ROIlineExtremes[i][1].x;

			Point iph((theorypoints[0].y - b) / k, theorypoints[0].y);
			Point ipl(((currentLeftImage_.rows - 1) - b) / k, currentLeftImage_.rows - 1);
			//在图像宽度方向裁剪直线，防止越界
			if (ipl.x < 0)
				ipl = Point(0, b);
			if (ipl.x > currentLeftImage_.cols)
				ipl = Point(currentLeftImage_.cols - 1, k*(currentLeftImage_.cols - 1) + b);

			lineExtremes[i].push_back(iph);
			lineExtremes[i].push_back(ipl);
		}
		ld.calKeylineVec(currentLeftImage_, lineExtremes, keylinesR);

		//移除与前一帧最佳直线角度相差过大的检测直线
		vector<KeyLine>::iterator it = keylinesR.begin();
		int newID = 0;
		while (it != keylinesR.end())
		{
			float delta_theta = abs(keylinesL[0].angle - (*it).angle)*180. / M_PI;
			if (delta_theta > 30)
				it = keylinesR.erase(it);
			else
			{
				(*it).class_id = newID;
				++it;
				++newID;
			}
		}

		trackfile << "检测到的待选直线数为： " << lineExtremes.size() << endl;
		trackfile << "筛选后待选直线数为： " << keylinesR.size() << endl;

		//控制检测到的直线在3~10条之间
		if (keylinesR.size() < 3)
		{
			//投票数阈值减小10，递减状态，为-1
			threshold -= 10;
			currentmark = -1; 
		}
		else if (keylinesR.size() > 10)
		{
			//投票数阈值增加10，递加状态，为1
			threshold += 10;
			currentmark = 1;
		}
		else
			break;

		trackfile << "currentmark = " << currentmark << ", lastmark = " << lastmark << endl;

		//通过比较前后两次的状态，来保证递减或递加状态，避免陷入死循环
		if (currentmark*lastmark < 0)
			break;
		else
			lastmark = currentmark;

	} while (threshold > 0 && threshold < 200); //循环条件为投票数阈值在(0,200)之间

	//检测到直线仍少于2，则返回0值，即未找到匹配直线
	if (keylinesR.size() < 2)
	{
		vector<Point3f> zero = { Point3f(0, 0, 0) };
		return zero;
	}

	t2 = (double)clock() / CLOCKS_PER_SEC;

	//计算前一帧最佳直线在前一帧的直线描绘子，当前帧待匹配直线在当前帧的直线描绘子
	ld.compute(formerLeftImage_, keylinesL, descriptorL, true);
	ld.compute(currentLeftImage_, keylinesR, descriptorR, true);

	t3 = (double)clock() / CLOCKS_PER_SEC;

	//通过汉明距离计算描绘子的相似度，获取最佳的匹配直线
	LineMatcher lm;
	vector<DMatch> matches;
	BFMatcher bdm(NORM_L2);
	//距离最小的直线视为当前帧最佳匹配直线
	vector<vector<DMatch>> matches1;
	bdm.knnMatch(descriptorL, descriptorR, matches1, 2);
	vector<DMatch> symMatches = matches1[0];

	//画出直线匹配图
	Mat matchimg;
	lm.drawLineMatches(formerLeftImage_, keylinesL, currentLeftImage_, keylinesR, symMatches, matchimg, Scalar(255, 0, 0), Scalar(0, 0, 255));
	//imshow("match", matchimg);
	/*char fm1[50];
	sprintf_s(fm1, (data_dir[6] + "/Match%d").c_str(), filenum);
	strcat_s(fm1, ".jpg");
	imwrite(fm1, matchimg);*/

	//未有匹配直线，返回0值
	if (symMatches.empty())
	{
		vector<Point3f> zero = { Point3f(0, 0, 0) };
		return zero;
	}
	
	t4 = (double)clock() / CLOCKS_PER_SEC;

	//获取最佳匹配直线的端点图像坐标
	int id = symMatches[0].trainIdx;
	Point startpoint = Point(keylinesR[id].startPointX, keylinesR[id].startPointY);
	Point endpoint = Point(keylinesR[id].endPointX, keylinesR[id].endPointY);
	trackfile << "最终匹配的直线id为： " << id << endl;

	//延长匹配线至1/3-1图像高度
	float k = (endpoint.y - startpoint.y + 0.1) / (endpoint.x - startpoint.x + 0.1);
	float b = endpoint.y - k*endpoint.x;
	Point newEndpoint((currentLeftImage_.rows / LINE_HEIGHT_SCALE - b) / k, currentLeftImage_.rows / LINE_HEIGHT_SCALE);
	Point newStartpoint(((currentLeftImage_.rows - 1) - b) / k, currentLeftImage_.rows - 1);
	current_ipoints = { newEndpoint, newStartpoint };

	//计算最佳匹配直线的相机系坐标，并返回
	vector<vector<Point>> vector_ipoints = { current_ipoints };
	vector<vector<Point3f>> current_cpoints = image2camera(vector_ipoints, currentHomography_);
	
	line(showimage, current_ipoints[0], current_ipoints[1], Scalar(0, 0, 255), 2);

	t5 = (double)clock() / CLOCKS_PER_SEC;

	trackfile << "ROI区域内直线检测与keyline计算耗时： "<< t2 - t1 << " s" << endl;
	trackfile << "LBD直线描述子计算耗时： " << t3 - t2 << " s" << endl;
	trackfile << "直线匹配耗时： " << t4 - t3 << " s" << endl;
	trackfile << "匹配直线的后处理耗时： " << t5 - t4 << " s" << endl;
	trackfile << "跟踪部分总耗时： " << t5 - t0 << " s" << endl;

	return current_cpoints[0];
}


//*****道路线初始状态或跟踪失效时重新寻找待筛选直线*****
vector<vector<Point3f>> RoadLineTrack::linesRefind(Mat& showimage)
{
	trackfile << "===第 " << filenum << " 帧，状态为：重启===" << endl;
	double t0, t1, t2, t3;
	t0 = (double)clock() / CLOCKS_PER_SEC;

	//重启状态，清空存储的前一帧左图像
	formerLeftImage_.release();
	//在整幅图像里进行直线检测
	vector<vector<Point>> leftpoints = lineDetect(currentLeftImage_, 120, 50, 3);
	
	//将检测的直线延长至1/3-1图像高度
	vector<vector<Point>> newleftpoints(leftpoints.size());
	for (int i = 0; i < leftpoints.size();i++)
	{
		line(showimage, leftpoints[i][0], leftpoints[i][1], Scalar(0, 0, 255), 2);

		float k = (leftpoints[i][1].y - leftpoints[i][0].y + 0.1) / (leftpoints[i][1].x - leftpoints[i][0].x + 0.1);
		float b = leftpoints[i][1].y - k*leftpoints[i][1].x;
		
		Point iph((currentLeftImage_.rows / LINE_HEIGHT_SCALE - b) / k, currentLeftImage_.rows / LINE_HEIGHT_SCALE);
		Point ipl(((currentLeftImage_.rows - 1) - b) / k, currentLeftImage_.rows - 1);

		//在图像宽度方向裁剪直线，防止越界
		if (iph.x < 0)
			iph = Point(0, b);
		if (iph.x > currentLeftImage_.cols)
			iph = Point(currentLeftImage_.cols - 1, k*(currentLeftImage_.cols - 1) + b);
		if (ipl.x < 0)
			ipl = Point(0, b);
		if (ipl.x > currentLeftImage_.cols)
			ipl = Point(currentLeftImage_.cols - 1, k*(currentLeftImage_.cols - 1) + b);

		newleftpoints[i].push_back(iph);
		newleftpoints[i].push_back(ipl);
	}

	//剔除越界的直线
	vector<vector<Point>>::iterator it = newleftpoints.begin();
	while (it != newleftpoints.end())
	{
		if ((*it)[0] == (*it)[1])
			it = newleftpoints.erase(it);
		else
			++it;
	}

	t1 = (double)clock() / CLOCKS_PER_SEC;

	//计算待筛选直线的相机系坐标和世界系坐标
	vector<vector<Point3f>> c_points = image2camera(newleftpoints, currentHomography_);
	vector<vector<Point3f>> w_points = camera2world(c_points);
	
	t2 = (double)clock() / CLOCKS_PER_SEC;

	trackfile << "直线检测与剔除耗时： " << t1 - t0 << " s" << endl;
	trackfile << "坐标转换计算耗时： " << t2 - t1 << " s" << endl;
	trackfile << "重启部分总耗时： " << t2 - t0 << " s" << endl;
	trackfile << endl;
	
	//输出世界坐标
	return w_points;
}

//*****道路线跟踪主程序，传入前一帧最佳直线的世界坐标系坐标*****
vector<Point3f> RoadLineTrack::linesTrack(vector<Point3f> w_points, Mat& showimage, int outtype)
{
	//计算前一帧最佳直线在当前帧图像中的图像坐标，世界系坐标转换为左图像坐标
	vector<Point> theorypoints = world2image(w_points);
	trackfile << "===第 " << filenum << " 帧，状态为：跟踪==="<< endl;
	trackfile << "反投影到当前图像中的直线坐标： " << theorypoints << endl;

	//判断图像坐标是否越界，如果越界则清空存储的前一帧左图像，返回0值
	if ((theorypoints[0].x == 0 && theorypoints[1].x == 0) ||
		(theorypoints[0].x == currentLeftImage_.cols - 1 && theorypoints[1].x == currentLeftImage_.cols - 1))
	{
		formerLeftImage_.release();
		vector<Point3f> zero = { Point3f(0, 0, 0) };
		return zero;
	}

	//在showimage中画出反投影位置
	line(showimage, theorypoints[1], theorypoints[0], Scalar(0, 255, 0), 2);

	vector<Point3f> current_bestline;
	//如果存储的前一帧左图像为空，即初始状态，则认为反投影位置为当前帧最佳直线位置，直接将其在当前帧相机系的坐标返回
	if (formerLeftImage_.empty())
	{
		current_ipoints = theorypoints;	
		current_bestline = world2camera(w_points);
	}
	//如果存储的前一帧左图像不为空，则采用局部区域内直线匹配的方法得到当前帧最佳直线，返回其当前帧相机系坐标
	else		
	{
		current_bestline = keylineMatch(theorypoints, showimage);
	}

	//将当前帧的数据存储作为下一帧的跟踪依据
	if (current_bestline.size() > 1)
		setFormerParams(currentLeftImage_, currentCameraR_, currentCameraT_, currentGroundParam_, current_ipoints);
	else
		formerLeftImage_.release();
	
	//计算当前帧最佳直线的世界系坐标或相机系坐标
	vector<vector<Point3f>> bestline_cpoints = { current_bestline };
	vector<vector<Point3f>> bestline_wpoints = camera2world(bestline_cpoints);
	
	trackfile << endl;

	//若outtype标志位为0，则返回世界系坐标；否则返回相机系坐标
	if (!outtype)
		return bestline_wpoints[0];
	else
		return bestline_cpoints[0];
}