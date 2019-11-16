/*
*������������ä���߸���
*HoughP��⡢LBD������BFMƥ��
*�޸����ڣ�2017.7.14
*/
#include "roadline_tracking.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <windows.h>

#define LINE_HEIGHT_SCALE 3

//���캯��
RoadLineTrack::RoadLineTrack()
{
	trackfile.open(data + "/TrackData.txt");
}

//���õ�ǰ֡���������뵱ǰ֡��ͼ�񡢵�Ӧ��������ϵ�����ϵ����תƽ�ƾ��󡢵��淽�̲���
void RoadLineTrack::setCurrentParams(Mat _image1, Mat _homography, Mat _cameraR, Mat _cameraT, Point3f _ground)
{
	currentLeftImage_ = _image1.clone();
	currentHomography_ = _homography.clone();
	currentCameraR_ = _cameraR.clone();
	currentCameraT_ = _cameraT.clone();
	currentGroundParam_ = _ground;
}

//����ǰ֡�����洢��Ϊ��һ֡����ƥ��Ĳ��գ���Ϊǰһ֡����
void RoadLineTrack::setFormerParams(Mat _image1, Mat _cameraR, Mat _cameraT, Point3f _ground, vector<Point> _ipoints)
{
	formerLeftImage_ = _image1.clone();
	formerCameraR_ = _cameraR.clone();
	formerCameraT_ = _cameraT.clone();
	formerGroundParam_ = _ground;
	former_ipoints = _ipoints;
}

//��ͼ������ת��Ϊ���ϵ����
vector<vector<Point3f>> RoadLineTrack::image2camera(vector<vector<Point>> leftpoints, Mat homography)
{
	//�궨����Ϊ1/2�������Ҫ��ͼ��������Сһ���Լ�����ά����
	vector<vector<Point2f>> half_rightpoints(leftpoints.size());
	vector<vector<Point2f>> half_leftpoints(leftpoints.size());

	//����ÿ����ͼ������
	for (int i = 0; i < leftpoints.size(); i++)
	{
		//ÿһ�д���һ����ͼ�����꣬ͬʱ����һ��ֱ�ߵ������˵�
		Mat l = Mat::ones(3, 2, CV_64F);
		l.at<double>(0, 0) = (double)leftpoints[i][0].x / 2;
		l.at<double>(1, 0) = (double)leftpoints[i][0].y / 2;
		l.at<double>(0, 1) = (double)leftpoints[i][1].x / 2;
		l.at<double>(1, 1) = (double)leftpoints[i][1].y / 2;

		//��Ӧ�Ծ��������ͼ����������ͼ�еĶ�Ӧ����
		Mat r = homography*l;

		//ת��Ϊ�������ʽ��[x/z,y/z,1]
		Point2f pt1(r.at<double>(0, 0) / r.at<double>(2, 0), r.at<double>(1, 0) / r.at<double>(2, 0));
		Point2f pt2(r.at<double>(0, 1) / r.at<double>(2, 1), r.at<double>(1, 1) / r.at<double>(2, 1));

		//�洢��Ӧ������ͼ������
		half_leftpoints[i].push_back(Point2f(l.at<double>(0, 0), l.at<double>(1, 0)));
		half_leftpoints[i].push_back(Point2f(l.at<double>(0, 1), l.at<double>(1, 1)));
		half_rightpoints[i].push_back(pt1);
		half_rightpoints[i].push_back(pt2);
	}

	//��ÿһ������ͼ���������ϵ����
	vector<vector<Point3f>> c_points(leftpoints.size());
	for (int i = 0; i < leftpoints.size(); i++)
	{
		c_points[i].push_back(measureZ(half_leftpoints[i][0], half_rightpoints[i][0]));
		c_points[i].push_back(measureZ(half_leftpoints[i][1], half_rightpoints[i][1]));
	}

	return c_points;
}

//����ϵ����ת��Ϊ��ͼ������
vector<Point> RoadLineTrack::world2image(vector<Point3f> w_points)
{
	//������תƽ�ƾ�������ϵ����ת��Ϊ��ǰ���ϵ����
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

	//ͨ��������ı궨�����ٽ����ϵ����ת����ͼ������
	vector<Point2f> origin_ipoints(0);
	for (int i = 0; i < c_points.size(); i++)
	{
		float x = _cameraMatrix1[0][0] * 2 * c_points[i].x + _cameraMatrix1[0][2] * 2 * c_points[i].z;
		float y = _cameraMatrix1[1][1] * 2 * c_points[i].y + _cameraMatrix1[1][2] * 2 * c_points[i].z;
		float z = c_points[i].z;

		//ת��Ϊ�����ʽ��[x/z,y/z,1]
		origin_ipoints.push_back(Point2f(x / z, y / z));
	}

	//����ֱ�ߵ�б�ʺͽؾ�
	float k = (origin_ipoints[1].y - origin_ipoints[0].y + 0.1) / (origin_ipoints[1].x - origin_ipoints[0].x + 0.1);
	float b = origin_ipoints[1].y - k*origin_ipoints[1].x;

	//�����͵�С��1/4ͼ��߶ȣ��򷵻�0ֵ
	if (fmax(origin_ipoints[0].y, origin_ipoints[1].y) < currentLeftImage_.rows / LINE_HEIGHT_SCALE)
	{
		vector<Point> cut_ipoints = { Point(0, 0), Point(0, 0) };
		return cut_ipoints;
	}

	//��ͼ��߶ȷ���ü�ֱ�ߣ��ü����ӳ�ֱ����1/3����1ͼ��߶�
	Point ip1, ip2;
	//��1��������ѡȡ���ж���ߵ���1/3ͼ��߶ȵĹ�ϵ��ȡ����
	ip1.y = fmax(fmin(origin_ipoints[0].y, origin_ipoints[1].y), currentLeftImage_.rows / LINE_HEIGHT_SCALE);
	//��2��������ѡȡ��ֱ��ѡȡͼ��߶�
	ip2.y = currentLeftImage_.rows - 1;
	//������Ӧ�ĺ�����
	ip1.x = (ip1.y - b) / k;
	ip2.x = (ip2.y - b) / k;

	//��ͼ���ȷ���ü�ֱ�ߣ���ֹԽ��
	if (ip1.x < 0)
		ip1 = Point(0, b);
	if (ip1.x > currentLeftImage_.cols)
		ip1 = Point(currentLeftImage_.cols - 1, k*(currentLeftImage_.cols - 1) + b);
	if (ip2.x < 0)
		ip2 = Point(0, b);
	if (ip2.x > currentLeftImage_.cols)
		ip2 = Point(currentLeftImage_.cols - 1, k*(currentLeftImage_.cols - 1) + b);

	//���زü����
	vector<Point> cut_ipoints(0);
	cut_ipoints.push_back(ip1);
	cut_ipoints.push_back(ip2);

	return cut_ipoints;
}

//����ϵ����ת��Ϊ���ϵ����
vector<Point3f> RoadLineTrack::world2camera(vector<Point3f> w_points)
{
	//������תƽ�ƾ�������ϵ����ת������ǰ���ϵ���꣬������
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

//���ϵ����ת��Ϊ����ϵ����
vector<vector<Point3f>> RoadLineTrack::camera2world(vector<vector<Point3f>> c_points)
{
	//������תƽ�ƾ��󽫵�ǰ���ϵ����ת��Ϊ����ϵ���꣬������
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

//������ֱ�߼��
vector<vector<Point>> RoadLineTrack::lineDetect(Mat image, int threshold, double minLineLength, double maxLineGap, int flag)
{
	//ת��Ϊ�Ҷ�ͼ
	Mat gray;
	cvtColor(image, gray, CV_BGR2GRAY);

	//canny����Ե
	Mat edge;
	Canny(gray, edge, 20, 80);

	//������ʱ任���ֱ��
	vector<Vec4i> lines;
	HoughLinesP(edge, lines, 1, CV_PI / 180, threshold, minLineLength, maxLineGap);

	//�Լ�⵽��ÿ��ֱ�ߵĶ˵�ѡ���Դ���ԭֱ�߷��ػ��ӳ���ͼ����β�߶ȷ���
	vector<vector<Point>> leftpoints(lines.size());
	for (int i = 0; i < lines.size(); i++)
	{
		Point pt1(lines[i][0], lines[i][1]);
		Point pt2(lines[i][2], lines[i][3]);
		
		//��flagΪ1���򷵻��ӳ��ߵĶ˵�ͼ������
		if (flag == 1) 
		{
			//����ֱ��б�ʺͽؾ�
			float k = (pt1.y - pt2.y + 0.1) / (pt1.x - pt2.x + 0.1);
			float b = pt2.y - k*pt2.x;
			
			//ֱ���ӳ���ͼ����β�߶�
			Point iph(-b / k, 0);
			Point ipl(((image.rows - 1) - b) / k, image.rows - 1);

			//��ͼ���ȷ���ü�ֱ�ߣ���ֹԽ��
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
		//��flagΪ0���򷵻�ԭ�߶ζ˵�ͼ�����꣬Ĭ��
		else if (flag == 0)
		{
			leftpoints[i].push_back(pt1);
			leftpoints[i].push_back(pt2);
		}
	}

	return leftpoints;
}

//��ȡ�ֲ�����Ȥ����Ķ˵�
vector<Point> RoadLineTrack::getROI(vector<Point> i_points, Mat image)
{
	//����x�����y����������Сֵ
	int xmin = min(i_points[0].x, i_points[1].x);
	int xmax = max(i_points[0].x, i_points[1].x);
	int ymin = min(i_points[0].y, i_points[1].y);
	int ymax = max(i_points[0].y, i_points[1].y);
	//Ϊ��ֹ�ӽ�ˮƽ��ֱ�����þ�����С�Ŀ�Ⱥ͸߶�
	int xlength = (xmax - xmin > 30) ? 0 : 30;
	int ylength = (ymax - ymin > 30) ? 0 : 30;
	//��x�����y������չ��Ӧ�Ĵ�С
	Point highpoint(min(xmax + xlength, image.cols - 1), min(ymax + ylength, image.rows - 1));
	Point lowpoint(max(xmin - xlength, 0), max(ymin - ylength, 0));

	//����ROI�����Ϻ����¶˵�����
	vector<Point> ROIpoints = { highpoint, lowpoint };
	return ROIpoints;
}

//����ǰһ֡ͼ���뵱ǰ֡ͼ������ĵ�Ӧ���󣬷��ص�Ӧ����͵�Ӧ�Ա任���ͼ��
Mat RoadLineTrack::calf2cHomography(Mat& warp)
{
	//����һ֡Ϊ��������ϵ��P_f = K[I|0]��P_c = K[f2cCamerR|f2cCamerT]
	//����f2c����ת��ƽ�ƾ��� f2cCamerR*X_f + f2cCamerT = X_c
	f2cCamerR_ = currentCameraR_*formerCameraR_.inv();
	f2cCamerT_ = -currentCameraR_*formerCameraR_.inv()*formerCameraT_ + currentCameraT_;

	//ǰ��֡�ı궨������ͬ
	Mat K = (Mat_<double>(3, 3) << _cameraMatrix1[0][0] * 2, _cameraMatrix1[0][1], _cameraMatrix1[0][2] * 2,
		_cameraMatrix1[1][0], _cameraMatrix1[1][1] * 2, _cameraMatrix1[1][2] * 2,
		_cameraMatrix1[2][0], _cameraMatrix1[2][1], _cameraMatrix1[2][2]);

	//����ķ����� n.t*X+d=0
	Mat n = (Mat_<double>(3, 1) << formerGroundParam_.x, formerGroundParam_.y, formerGroundParam_.z);
	double d = -1.0;

	//����f2c�ĵ�Ӧ�Ծ��� H=K*(R-T*n.t/d)*K.inv
	f2cCamerR_.convertTo(f2cCamerR_, CV_64F);
	f2cCamerT_.convertTo(f2cCamerT_, CV_64F);
	Mat f2cHomography = K*(f2cCamerR_ - f2cCamerT_*n.t() / d)*K.inv();

	//��Ӧ�任
	warpPerspective(formerLeftImage_, // input image
		warp,			// output image
		f2cHomography,		// homography
		Size(formerLeftImage_.cols, formerLeftImage_.rows), CV_INTER_LINEAR); // size of output image

	/*Mat diff = current_image1 - warp;
	diff.convertTo(diff, CV_8UC1);
	imshow("diff", diff);*/

	return f2cHomography;
}

//���ݵ�Ӧ�Ծ��󣬽�ǰһ֡ͼ������ת��Ϊ��ǰ֡ͼ������
vector<Point> RoadLineTrack::calf2cpoints(vector<Point> fipoints, Mat f2cHomography)
{
	vector<Point> _ipoints;

	//����ÿ��ǰһֱ֡��ͼ������
	for (int i = 0; i < fipoints.size(); i++)
	{
		Mat p = (Mat_<double>(3, 1) << fipoints[i].x, fipoints[i].y, 1.0);

		//��Ӧ������㵱ǰ֡ͼ������
		Mat r = f2cHomography*p;

		//ת��Ϊ�����ʽ
		_ipoints.push_back(Point(r.at<double>(0) / r.at<double>(2), r.at<double>(1) / r.at<double>(2)));
	}

	//����ֱ�ߵ�б�ʺͽؾ࣬��ֱ����͵��ӳ���ͼ��߶�
	float k = (_ipoints[1].y - _ipoints[0].y + 0.1) / (_ipoints[1].x - _ipoints[0].x + 0.1);
	float b = _ipoints[1].y - k*_ipoints[1].x;
	Point iph = _ipoints[0];
	Point ipl(((currentLeftImage_.rows - 1) - b) / k, currentLeftImage_.rows - 1);

	//�����ӳ���ĵ�ǰ֡ͼ������
	vector<Point> cipoints;
	cipoints.push_back(iph);
	cipoints.push_back(ipl);

	return cipoints;
}

//���ݵ�Ӧ�Ծ��󣬼���ǰһ֡���ֱ��λ��ǰ��֡���������ڳ��ȵĶ˵�ͼ������
vector<Point> RoadLineTrack::calcompoints(vector<Point> fipoints, Mat f2cHomography)
{
	//����ǰһ֡���ֱ�ߵ�б�ʺͽؾ�
	float origink = (fipoints[1].y - fipoints[0].y + 0.1) / (fipoints[1].x - fipoints[0].x + 0.1);
	float originb = fipoints[1].y - origink*fipoints[1].x;

	vector<Point> _ipoints;
	//����ǰһֱ֡�ߵ�ͼ�����꣬�������ڵ�ǰ֡��ͼ������
	for (int i = 0; i < fipoints.size(); i++)
	{
		Mat p = (Mat_<double>(3, 1) << fipoints[i].x, fipoints[i].y, 1.0);
		//��Ӧ������㵱ǰ֡ͼ������
		Mat r = f2cHomography*p;
		//ת��Ϊ�����ʽ
		_ipoints.push_back(Point(r.at<double>(0) / r.at<double>(2), r.at<double>(1) / r.at<double>(2)));
	}

	//����ת���õ��ĵ�ǰ֡ͼ�������󹫹�����Ķ˵�
	//����ת������ǰ֡��ֱ��б�ʺͽؾ�
	float warpk = (_ipoints[1].y - _ipoints[0].y + 0.1) / (_ipoints[1].x - _ipoints[0].x + 0.1);
	float warpb = _ipoints[1].y - warpk*_ipoints[1].x;
	//���ֱ����������Ϊͼ��߶ȵĵ㣬���任��ǰһ֡���õ��˵�
	Mat after = (Mat_<double>(3, 1) << ((currentLeftImage_.rows - 1) - warpb) / warpk, currentLeftImage_.rows - 1, 1.0);
	Mat before = f2cHomography.inv()*after;
	//ת��Ϊ�����ʽ
	Point lowpoint = Point(before.at<double>(0) / before.at<double>(2), before.at<double>(1) / before.at<double>(2));

	//λ�ڹ��������ڵ�ֱ�߸߶Ȳ��ܶ���20
	if (lowpoint.y - fipoints[0].y < 20)
		lowpoint = Point((fipoints[0].y + 20 - originb) / origink, fipoints[0].y + 20);

	//����ֱ���ڹ�������Ķ˵�ͼ������
	vector<Point> compoints;
	compoints.push_back(fipoints[0]);
	compoints.push_back(lowpoint);

	return compoints;
}

//ֱ��ƥ��ȷ����ǰ֡���ֱ��
vector<Point3f> RoadLineTrack::keylineMatch(vector<Point> theorypoints, Mat& showimage)
{
	double t0, t1, t2, t3, t4, t5, t6;
	t0 = (double)clock() / CLOCKS_PER_SEC;

	//���ݷ�ͶӰͼ�������ȡ�ֲ�ROI�˵����꣬�Ӷ���ȡROIͼ��
	vector<Point> current_hlpoints = getROI(theorypoints, currentLeftImage_);
	Point ROIlowpoint = current_hlpoints[1];
	Mat current_ROI = currentLeftImage_(Rect(current_hlpoints[0], current_hlpoints[1]));

	//ֱ�����������
	LineDescriptor ld;
	vector<KeyLine> keylinesL, keylinesR;
	Mat descriptorL, descriptorR;

	t1 = (double)clock() / CLOCKS_PER_SEC;

	//����ǰ��֡ͼ�����ĵ�Ӧ���󼰵�Ӧ�任ͼ��
	Mat warp;
	Mat f2cHomography = calf2cHomography(warp);
	//imshow("warp", warp);
	/*char fm0[50];
	sprintf_s(fm0, (data_dir[6] + "/Warp%d").c_str(), filenum);
	strcat_s(fm0, ".jpg");
	imwrite(fm0, warp);*/
	
	//����ǰһ֡���ֱ��λ��ǰ��֡���������ڳ��ȵĶ˵�ͼ������
	vector<Point> compoints = calcompoints(former_ipoints, f2cHomography);

	//����ǰһ֡���ֱ����ǰһ֡ͼ���е�keyline
	keylinesL.push_back(LineDescriptor::calKeyline(formerLeftImage_, compoints[0], compoints[1], 0));

	//ͨ��һ��ѭ���仯ֱ�߼���ͶƱ������֤�ֲ������ڼ�⵽��ֱ����3-10��֮��
	int threshold = 100;
	int lastmark = 0, currentmark = 0;
	do 
	{
		//������������������������ǰһ�μ����ֱ��keyline
		keylinesR.clear();

		//�ֲ������ڵ�ֱ�߼��
		vector<vector<Point>> ROIlineExtremes = lineDetect(current_ROI, threshold, 0.1*current_ROI.rows, 3, 1);
		
		//��ԭͼ�¼����⵽ֱ�ߵ�keyline
		vector<vector<Point>> lineExtremes(ROIlineExtremes.size());
		for (int i = 0; i < ROIlineExtremes.size(); i++)
		{
			ROIlineExtremes[i][0] += ROIlowpoint;
			ROIlineExtremes[i][1] += ROIlowpoint;

			//���Ƶ��ӳ��Ͳü����裬��Ҫ����͵��ӳ���ͼ��߶�
			float k = (ROIlineExtremes[i][1].y - ROIlineExtremes[i][0].y + 0.1) / (ROIlineExtremes[i][1].x - ROIlineExtremes[i][0].x + 0.1);
			float b = ROIlineExtremes[i][1].y - k*ROIlineExtremes[i][1].x;

			Point iph((theorypoints[0].y - b) / k, theorypoints[0].y);
			Point ipl(((currentLeftImage_.rows - 1) - b) / k, currentLeftImage_.rows - 1);
			//��ͼ���ȷ���ü�ֱ�ߣ���ֹԽ��
			if (ipl.x < 0)
				ipl = Point(0, b);
			if (ipl.x > currentLeftImage_.cols)
				ipl = Point(currentLeftImage_.cols - 1, k*(currentLeftImage_.cols - 1) + b);

			lineExtremes[i].push_back(iph);
			lineExtremes[i].push_back(ipl);
		}
		ld.calKeylineVec(currentLeftImage_, lineExtremes, keylinesR);

		//�Ƴ���ǰһ֡���ֱ�߽Ƕ�������ļ��ֱ��
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

		trackfile << "��⵽�Ĵ�ѡֱ����Ϊ�� " << lineExtremes.size() << endl;
		trackfile << "ɸѡ���ѡֱ����Ϊ�� " << keylinesR.size() << endl;

		//���Ƽ�⵽��ֱ����3~10��֮��
		if (keylinesR.size() < 3)
		{
			//ͶƱ����ֵ��С10���ݼ�״̬��Ϊ-1
			threshold -= 10;
			currentmark = -1; 
		}
		else if (keylinesR.size() > 10)
		{
			//ͶƱ����ֵ����10���ݼ�״̬��Ϊ1
			threshold += 10;
			currentmark = 1;
		}
		else
			break;

		trackfile << "currentmark = " << currentmark << ", lastmark = " << lastmark << endl;

		//ͨ���Ƚ�ǰ�����ε�״̬������֤�ݼ���ݼ�״̬������������ѭ��
		if (currentmark*lastmark < 0)
			break;
		else
			lastmark = currentmark;

	} while (threshold > 0 && threshold < 200); //ѭ������ΪͶƱ����ֵ��(0,200)֮��

	//��⵽ֱ��������2���򷵻�0ֵ����δ�ҵ�ƥ��ֱ��
	if (keylinesR.size() < 2)
	{
		vector<Point3f> zero = { Point3f(0, 0, 0) };
		return zero;
	}

	t2 = (double)clock() / CLOCKS_PER_SEC;

	//����ǰһ֡���ֱ����ǰһ֡��ֱ������ӣ���ǰ֡��ƥ��ֱ���ڵ�ǰ֡��ֱ�������
	ld.compute(formerLeftImage_, keylinesL, descriptorL, true);
	ld.compute(currentLeftImage_, keylinesR, descriptorR, true);

	t3 = (double)clock() / CLOCKS_PER_SEC;

	//ͨ�����������������ӵ����ƶȣ���ȡ��ѵ�ƥ��ֱ��
	LineMatcher lm;
	vector<DMatch> matches;
	BFMatcher bdm(NORM_L2);
	//������С��ֱ����Ϊ��ǰ֡���ƥ��ֱ��
	vector<vector<DMatch>> matches1;
	bdm.knnMatch(descriptorL, descriptorR, matches1, 2);
	vector<DMatch> symMatches = matches1[0];

	//����ֱ��ƥ��ͼ
	Mat matchimg;
	lm.drawLineMatches(formerLeftImage_, keylinesL, currentLeftImage_, keylinesR, symMatches, matchimg, Scalar(255, 0, 0), Scalar(0, 0, 255));
	//imshow("match", matchimg);
	/*char fm1[50];
	sprintf_s(fm1, (data_dir[6] + "/Match%d").c_str(), filenum);
	strcat_s(fm1, ".jpg");
	imwrite(fm1, matchimg);*/

	//δ��ƥ��ֱ�ߣ�����0ֵ
	if (symMatches.empty())
	{
		vector<Point3f> zero = { Point3f(0, 0, 0) };
		return zero;
	}
	
	t4 = (double)clock() / CLOCKS_PER_SEC;

	//��ȡ���ƥ��ֱ�ߵĶ˵�ͼ������
	int id = symMatches[0].trainIdx;
	Point startpoint = Point(keylinesR[id].startPointX, keylinesR[id].startPointY);
	Point endpoint = Point(keylinesR[id].endPointX, keylinesR[id].endPointY);
	trackfile << "����ƥ���ֱ��idΪ�� " << id << endl;

	//�ӳ�ƥ������1/3-1ͼ��߶�
	float k = (endpoint.y - startpoint.y + 0.1) / (endpoint.x - startpoint.x + 0.1);
	float b = endpoint.y - k*endpoint.x;
	Point newEndpoint((currentLeftImage_.rows / LINE_HEIGHT_SCALE - b) / k, currentLeftImage_.rows / LINE_HEIGHT_SCALE);
	Point newStartpoint(((currentLeftImage_.rows - 1) - b) / k, currentLeftImage_.rows - 1);
	current_ipoints = { newEndpoint, newStartpoint };

	//�������ƥ��ֱ�ߵ����ϵ���꣬������
	vector<vector<Point>> vector_ipoints = { current_ipoints };
	vector<vector<Point3f>> current_cpoints = image2camera(vector_ipoints, currentHomography_);
	
	line(showimage, current_ipoints[0], current_ipoints[1], Scalar(0, 0, 255), 2);

	t5 = (double)clock() / CLOCKS_PER_SEC;

	trackfile << "ROI������ֱ�߼����keyline�����ʱ�� "<< t2 - t1 << " s" << endl;
	trackfile << "LBDֱ�������Ӽ����ʱ�� " << t3 - t2 << " s" << endl;
	trackfile << "ֱ��ƥ���ʱ�� " << t4 - t3 << " s" << endl;
	trackfile << "ƥ��ֱ�ߵĺ����ʱ�� " << t5 - t4 << " s" << endl;
	trackfile << "���ٲ����ܺ�ʱ�� " << t5 - t0 << " s" << endl;

	return current_cpoints[0];
}


//*****��·�߳�ʼ״̬�����ʧЧʱ����Ѱ�Ҵ�ɸѡֱ��*****
vector<vector<Point3f>> RoadLineTrack::linesRefind(Mat& showimage)
{
	trackfile << "===�� " << filenum << " ֡��״̬Ϊ������===" << endl;
	double t0, t1, t2, t3;
	t0 = (double)clock() / CLOCKS_PER_SEC;

	//����״̬����մ洢��ǰһ֡��ͼ��
	formerLeftImage_.release();
	//������ͼ�������ֱ�߼��
	vector<vector<Point>> leftpoints = lineDetect(currentLeftImage_, 120, 50, 3);
	
	//������ֱ���ӳ���1/3-1ͼ��߶�
	vector<vector<Point>> newleftpoints(leftpoints.size());
	for (int i = 0; i < leftpoints.size();i++)
	{
		line(showimage, leftpoints[i][0], leftpoints[i][1], Scalar(0, 0, 255), 2);

		float k = (leftpoints[i][1].y - leftpoints[i][0].y + 0.1) / (leftpoints[i][1].x - leftpoints[i][0].x + 0.1);
		float b = leftpoints[i][1].y - k*leftpoints[i][1].x;
		
		Point iph((currentLeftImage_.rows / LINE_HEIGHT_SCALE - b) / k, currentLeftImage_.rows / LINE_HEIGHT_SCALE);
		Point ipl(((currentLeftImage_.rows - 1) - b) / k, currentLeftImage_.rows - 1);

		//��ͼ���ȷ���ü�ֱ�ߣ���ֹԽ��
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

	//�޳�Խ���ֱ��
	vector<vector<Point>>::iterator it = newleftpoints.begin();
	while (it != newleftpoints.end())
	{
		if ((*it)[0] == (*it)[1])
			it = newleftpoints.erase(it);
		else
			++it;
	}

	t1 = (double)clock() / CLOCKS_PER_SEC;

	//�����ɸѡֱ�ߵ����ϵ���������ϵ����
	vector<vector<Point3f>> c_points = image2camera(newleftpoints, currentHomography_);
	vector<vector<Point3f>> w_points = camera2world(c_points);
	
	t2 = (double)clock() / CLOCKS_PER_SEC;

	trackfile << "ֱ�߼�����޳���ʱ�� " << t1 - t0 << " s" << endl;
	trackfile << "����ת�������ʱ�� " << t2 - t1 << " s" << endl;
	trackfile << "���������ܺ�ʱ�� " << t2 - t0 << " s" << endl;
	trackfile << endl;
	
	//�����������
	return w_points;
}

//*****��·�߸��������򣬴���ǰһ֡���ֱ�ߵ���������ϵ����*****
vector<Point3f> RoadLineTrack::linesTrack(vector<Point3f> w_points, Mat& showimage, int outtype)
{
	//����ǰһ֡���ֱ���ڵ�ǰ֡ͼ���е�ͼ�����꣬����ϵ����ת��Ϊ��ͼ������
	vector<Point> theorypoints = world2image(w_points);
	trackfile << "===�� " << filenum << " ֡��״̬Ϊ������==="<< endl;
	trackfile << "��ͶӰ����ǰͼ���е�ֱ�����꣺ " << theorypoints << endl;

	//�ж�ͼ�������Ƿ�Խ�磬���Խ������մ洢��ǰһ֡��ͼ�񣬷���0ֵ
	if ((theorypoints[0].x == 0 && theorypoints[1].x == 0) ||
		(theorypoints[0].x == currentLeftImage_.cols - 1 && theorypoints[1].x == currentLeftImage_.cols - 1))
	{
		formerLeftImage_.release();
		vector<Point3f> zero = { Point3f(0, 0, 0) };
		return zero;
	}

	//��showimage�л�����ͶӰλ��
	line(showimage, theorypoints[1], theorypoints[0], Scalar(0, 255, 0), 2);

	vector<Point3f> current_bestline;
	//����洢��ǰһ֡��ͼ��Ϊ�գ�����ʼ״̬������Ϊ��ͶӰλ��Ϊ��ǰ֡���ֱ��λ�ã�ֱ�ӽ����ڵ�ǰ֡���ϵ�����귵��
	if (formerLeftImage_.empty())
	{
		current_ipoints = theorypoints;	
		current_bestline = world2camera(w_points);
	}
	//����洢��ǰһ֡��ͼ��Ϊ�գ�����þֲ�������ֱ��ƥ��ķ����õ���ǰ֡���ֱ�ߣ������䵱ǰ֡���ϵ����
	else		
	{
		current_bestline = keylineMatch(theorypoints, showimage);
	}

	//����ǰ֡�����ݴ洢��Ϊ��һ֡�ĸ�������
	if (current_bestline.size() > 1)
		setFormerParams(currentLeftImage_, currentCameraR_, currentCameraT_, currentGroundParam_, current_ipoints);
	else
		formerLeftImage_.release();
	
	//���㵱ǰ֡���ֱ�ߵ�����ϵ��������ϵ����
	vector<vector<Point3f>> bestline_cpoints = { current_bestline };
	vector<vector<Point3f>> bestline_wpoints = camera2world(bestline_cpoints);
	
	trackfile << endl;

	//��outtype��־λΪ0���򷵻�����ϵ���ꣻ���򷵻����ϵ����
	if (!outtype)
		return bestline_wpoints[0];
	else
		return bestline_cpoints[0];
}