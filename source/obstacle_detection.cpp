#include "stdafx.h"
#include "obstacle_detection.h"
#include "generalDataFuction.h"
#include "matcher.h"
#include <time.h>


//计算地面方程
Mat ob_detection::findH(Mat left, Mat right, Point3f& ransac_modle, vector<Point3f>& groundpoints,int filenum, cv::Mat WPlane){
	//特征点提取
	RobustMatcher rmatcher;
	rmatcher.setConfidenceLevel(0.98);
	rmatcher.setMinDistanceToEpipolar(1.0);
	rmatcher.setRatio(0.65f);
	Ptr<FeatureDetector> pfd = new SurfFeatureDetector(20);
	rmatcher.setFeatureDetector(pfd);

	clock_t start1, finish1;
	double SURFtime;
	start1 = clock();
	vector<DMatch> matches;//全部的匹配点
	vector<KeyPoint> keypoints1, keypoints2;
	Mat fundemental = rmatcher.match(left, right, matches, keypoints1, keypoints2);//得到全部的匹配点对
	Mat empty;
	if (fundemental.empty()){
		cout << "Matched points aren't enough!" << endl;
		return empty;
	}
	
	finish1 = clock();
	SURFtime = (double)(finish1 - start1) / CLOCKS_PER_SEC;
	cout << "SURF运行时间为" << SURFtime << "秒" << endl;

	//// draw the matches for test
	//Mat imageMatches;
	//drawMatches(left, keypoints1,  // 1st image and its keypoints
	//right, keypoints2,  // 2nd image and its keypoints
	//matches,			// the matches
	//imageMatches,		// the image produced
	//cv::Scalar(255, 0, 0)); // color of the lines
	//namedWindow("Matches");
	//imshow("Matches", imageMatches);
	////imwrite("Matchess.jpg",imageMatches);
	
	vector<Point2f> points1, points2;
	for (vector<DMatch>::const_iterator it = matches.begin(); it != matches.end(); ++it)
	{
		float x = keypoints1[it->queryIdx].pt.x;
		float y = keypoints1[it->queryIdx].pt.y;
		points1.push_back(Point2f(x, y));
		//circle(old, Point(x, y), 1, Scalar(0, 0, 0), 2);//将匹配点画在原图old上
		x = keypoints2[it->trainIdx].pt.x;
		y = keypoints2[it->trainIdx].pt.y;
		points2.push_back(Point2f(x, y));
	}

	//计算特征点三维坐标
	vector<Point3f>testp;
	for (int i = 0; i < points1.size(); i++)
	{
		Point3f p;
		p = measureZ(points1[i], points2[i]);
		if (p.z < 5200 && p.x<2750 && p.x>-2750)
		{
			testp.push_back(p);
		}
	}
	
	//RANSAC方法拟合地面
	if (testp.size() < 4){
		cout << "Points for RANSAC are not enough!" << endl;
		return empty;
	}

	clock_t start, finish;
	double ransactime;
	start = clock();
	vector<Point3f>best_set;
	vector<int>num;
	if (WPlane.empty())
		ransac_modle = ransac_Plane(testp, best_set, num, 5000, 200);//参数：（数据集，局内点，迭代次数，阈值），返回值：模型
	else
		ransac_modle = ransac_Plane(testp, best_set, num, 500, 200);

	//存储地面特征点
	for (int i = 0; i < num.size(); i++)
	{
		int n = num[i];
		groundpoints.push_back(testp[n]);
	}
	finish = clock();
	ransactime = (double)(finish - start) / CLOCKS_PER_SEC;
	cout << "RANSAC运行时间为" << ransactime << "秒" << endl;

	//计算单应矩阵
	vector<Point2f>ground1;
	vector<Point2f>ground2;
	for (int i = 0; i < num.size(); i++)
	{
		int n = num[i];
		ground1.push_back(points1[n]);
		ground2.push_back(points2[n]);
	}
	if (ground1.size() < 4)
	{
		cout << "There isn't enough points in ground." << endl;
		return empty;
	}

	vector<uchar> inliers(ground1.size(), 0);
	Mat homography = findHomography(
		Mat(ground1), Mat(ground2), // corresponding points
		inliers,	// outputed inliers matches
		CV_RANSAC,	// RANSAC method
		1.);	    // max distance to reprojection point

	return homography;
}

//检测障碍物
vector<pair<Point3f,int>> ob_detection::obstacletest(Mat left, Mat right, Point3f ransac_modle, Mat homography, Mat& disp, Mat showimage){
	//SGBM处理
	StereoSGBM sgbm;
	sgbm.preFilterCap = 63; //预处理滤波器的截断值，预处理的输出值仅保留[-preFilterCap, preFilterCap]范围内的值，参数范围
	sgbm.SADWindowSize = 9;

	int cn = left.channels();

	sgbm.P1 = 8 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;//控制视差变化平滑性的参数。P1、P2的值越大，视差越平滑。P1是相邻像素点视差增/减 1 时的惩罚系数；P2是相邻像素点视差变化值大于1时的惩罚系数。P2必须大于P1
	sgbm.P2 = 32 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.minDisparity = 0;
	sgbm.numberOfDisparities = 32;
	sgbm.uniquenessRatio = 15;
	sgbm.speckleWindowSize = 100;
	sgbm.speckleRange = 10;
	sgbm.disp12MaxDiff = 1;
	sgbm.fullDP = 0; //当设置为 TRUE 时，运行双通道动态编程算法

	//单应变换
	Mat result;
	warpPerspective(left, // input image
		result,			// output image
		homography,		// homography
		Size(left.cols, left.rows)); // size of output image
	//imshow("warp",result);
	//imwrite("warp.jpg",result);

	int64 t = getTickCount();
	sgbm(result, right, disp);
	t = getTickCount() - t;
	printf("SGBM处理时间为: %fms\n", t * 1000 / getTickFrequency());
	disp.convertTo(disp, CV_8U);
	Mat disp_show;
	resize(disp,disp_show,Size(320,240));	//缩放
	imshow("disp", disp_show);
	HWND hwnd_disp = FindWindow(NULL,"disp");
	SetWindowPos(hwnd_disp,HWND_NOTOPMOST,640,661,320,240,SWP_ASYNCWINDOWPOS|SWP_DEFERERASE|SWP_NOSIZE|SWP_NOZORDER|SWP_NOACTIVATE);

	//提取灰度值为设定值的一系列点
	vector<Point2f> gray_ten;
	for (int i = 0; i < disp.rows; i++)
	{
		for (int j = 0; j < disp.cols; j++)
		{
			if ((disp.at<uchar>(i, j)) > 20 && (disp.at<uchar>(i, j)) < 200) //视差提取点阈值
			{
				Point2f gray10 = Point(j, i);
				gray_ten.push_back(gray10);
			}
		}
	}
	cout << "灰度值为额定值的点的数目：" << gray_ten.size() << endl;

	//gray_ten.size为0时，返回一个在视野外的点
	if (gray_ten.size() == 0)
	{
		vector<pair<Point3f,int>> zeropoint;
		Point3f zero;
		zero.x = 8000;
		zero.y = 8000;
		zero.z = 0;
		zeropoint.push_back(make_pair(zero, 0));
		return zeropoint;
	}

	//映射到左右图像以便计算三维坐标

	vector<Point2f> objl;
	vector<Point2f> objrr;
	for (int i = 0; i < gray_ten.size(); i++)
	{
		double x = 0, y = 0;
		x = gray_ten[i].x - (disp.at<uchar>(gray_ten[i].y, gray_ten[i].x) / 16);
		y = gray_ten[i].y;
		Point2f objl1 = Point2f(gray_ten[i].x, gray_ten[i].y);
		Point2f objr1 = Point2f(x, y);
		objl.push_back(objl1);
		objrr.push_back(objr1);
		//cout<<x<<"  "<<y<<endl;
	}

	Mat_<double> homo = homography;
	Mat homo_inv = homography.inv();
	//反向单应性变换，得到边缘点在左图中的坐标
	vector<Point2f> objll;
	perspectiveTransform(objl, objll, homo_inv);
	for (int i = 0; i < objll.size(); i++)
	{
		//int n = i * 10;
		circle(showimage, Point(2 * objll[i].x, 2 * objll[i].y), 5, Scalar(0), 2);
		circle(left, Point(objll[i].x, objll[i].y), 5, Scalar(0), 2);
	}
	//imshow("drawll", left);

	//计算特征点三维坐标
	cout << "左相机坐标系三维坐标：" << endl;
	vector<pair<Point3f,int>> edge;
	for (int i = 0; i < objll.size(); i++)
	{
		Point3f p;
		p = measureZ(objll[i], objrr[i]);
		//	cout<<p.x<<"  "<<p.y<<"  "<<p.z<<endl;
		if (p.z < 5200 && p.x<2750 && p.x>-2750)
		{
			edge.push_back(make_pair(p, disp.at<uchar>(objl[i].y, objl[i].x) / 16));
		}
	}
	cout << "edge数目为（限定5m*5m）：" << edge.size() << endl;

	return edge;
}

//最小二乘法拟合平面方程ax+by+cz=1
Point3f ob_detection::planeguess(vector<Point3f>points){
	Mat a = Mat::zeros(points.size(), 3, CV_32F);    //初始化全0矩阵
	Mat y = Mat::zeros(points.size(), 1, CV_32F);    //初始化全0矩阵
	Mat x = Mat::zeros(3, 1, CV_32F);    //初始化全0矩阵
	for (int j = 0; j<points.size(); j++)
	{
		a.at<float>(j, 0) = points[j].x;
		a.at<float>(j, 1) = points[j].y;
		a.at<float>(j, 2) = points[j].z;
		y.at<float>(j) = 1;
	}
	x = (a.t()*a).inv();
	x = x*a.t()*y;
	Point3f result;
	result.x = x.at<float>(0);
	result.y = x.at<float>(1);
	result.z = x.at<float>(2);
	return result;
}

//判断点是否符合模型
bool ob_detection::injudge(Point3f point, Point3f modle, float t){

	float dis;
	dis = abs(point.x*modle.x + point.y*modle.y + point.z*modle.z - 1) / sqrt(pow(modle.x, 2) + pow(modle.y, 2) + pow(modle.z, 2));

	//cout << "dis=" << dis << endl;

	if (dis > t) return false;
	else{
		return true;
	}
}

//计算模型误差
float ob_detection::error_estimate(vector<Point3f>points, Point3f modle){
	float sum = 0;
	for (int i = 0; i < points.size(); i++){
		float dis = 0;
		dis = abs(points[i].x*modle.x + points[i].y*modle.y + points[i].z*modle.z - 1) / sqrt(pow(modle.x, 2) + pow(modle.y, 2) + pow(modle.z, 2));
		sum = sum + dis;
	}
	sum = sum / points.size();
	return sum;
}

//RANSAC平面估计
Point3f ob_detection::ransac_Plane(vector<Point3f>points, vector<Point3f>&set, vector<int>&num, int iterat, float thresh){

	srand((unsigned)time(NULL));

	vector<Point3f>data = points;//数据集
	Point3f model;
	int size = data.size();
	int n = 3;//适用于模型的最少数据个数
	float p = 0.8;//算法有用的概率
	int k = iterat;//算法迭代次数*
	float t = thresh;//数据是否适应于模型的阈值,200mm
	int d1 = data.size()*0.1;//判定模型是否适用于数据集的数目个数，20%
	int d2 = 0;

	Point3f best_model;
	best_model.x = 0; best_model.y = 0; best_model.z = 0;
	vector<Point3f>best_consensus_set;
	vector<int>best_num;
	float best_error = 1e38;// 无穷;

	int iterations = 0;
	while (iterations < k){
		vector<Point3f>maybe_in;
		vector<Point3f>consensus_set;
		vector<int>maybe_num;
		Point3f maybe_model;
		Point3f better_model;
		//随机选取三个点
		int a, b, c;
		do{
			a = rand() % data.size();
			b = rand() % data.size();
			c = rand() % data.size();
		} while (a == b || a == c || b == c);
		maybe_in.push_back(data[a]);
		maybe_in.push_back(data[b]);
		maybe_in.push_back(data[c]);

		//求出其对应平面参数
		Mat m = Mat::zeros(3, 3, CV_32F);    //初始化全0矩阵
		Mat n = Mat::zeros(3, 1, CV_32F);    //初始化全0矩阵
		Mat x = Mat::zeros(3, 1, CV_32F);    //初始化全0矩阵
		for (int j = 0; j < 3; j++)
		{
			m.at<float>(j, 0) = maybe_in[j].x;
			m.at<float>(j, 1) = maybe_in[j].y;
			m.at<float>(j, 2) = maybe_in[j].z;
			n.at<float>(j) = 1;
		}
		x = m.inv();
		x = x*n;
		maybe_model.x = x.at<float>(0);
		maybe_model.y = x.at<float>(1);
		maybe_model.z = x.at<float>(2);
		//consensus_set = maybe_in;
		//测试所有点
		for (int i = 0; i < data.size(); i++){
			bool in = injudge(data[i], maybe_model, t);
			if (in) {
				consensus_set.push_back(data[i]);
				maybe_num.push_back(i);
			}
		}
		if ((consensus_set.size()>d2)){// && (consensus_set.size() > d1)){//
			better_model = planeguess(consensus_set);
			best_error = error_estimate(consensus_set, better_model);
			best_model = better_model;
			best_consensus_set = consensus_set;
			best_num = maybe_num;
			d2 = consensus_set.size();
		}
		iterations++;
	}
	set = best_consensus_set;
	num = best_num;
	float u = best_consensus_set.size();
	float v = data.size();
	float persent = u / v;
	cout << u << "  " << v << endl;
	cout << "模型符合率：" << persent << endl;
	return best_model;
}

Point3f ob_detection::ransac_Plane(vector<Point3f>points, vector<Point3f>&set, vector<int>&num, int iterat, float t1, float t2, Point3f CPlane, float &r){

	srand((unsigned)time(NULL));

	vector<Point3f>data = points;//数据集
	Point3f model;
	int size = data.size();
	float p = 0.8;//算法有用的概率
	int k = iterat;//算法迭代次数*
	int d1 = data.size()*0.1;//判定模型是否适用于数据集的数目个数，20%
	int d2 = 0;

	Point3f best_model;
	best_model.x = 0; best_model.y = 0; best_model.z = 0;
	vector<Point3f>best_consensus_set;
	vector<int>best_num;
	float best_error = 1e38;// 无穷;
	int iterations = 0;
	int num_last = 0;
	while (iterations < k)
	{
		vector<Point3f>maybe_in;
		vector<Point3f>consensus_set;
		vector<int>maybe_num;
		Point3f better_model;
		if (iterations == 0)
		{
			for (int i = 0; i < data.size(); i++){
				bool in = injudge(data[i], CPlane, t1);
				if (in) {
					consensus_set.push_back(data[i]);
					maybe_num.push_back(i);
				}
			}
			CPlane = best_model = planeguess(consensus_set);
			iterations++;
		}
		else
		{
			//测试所有点
			for (int i = 0; i < data.size(); i++){
				bool in = injudge(data[i], CPlane, t2);
				if (in) {
					consensus_set.push_back(data[i]);
					maybe_num.push_back(i);
				}
			}
			if (maybe_num.size()>num_last)
			{
				CPlane = best_model = planeguess(consensus_set);
				best_error = error_estimate(consensus_set, best_model);
				best_consensus_set = consensus_set;
				best_num = maybe_num;
				num_last = maybe_num.size();
				iterations++;
			}
			else
				break;
		}
		

		
		
	}

	set = best_consensus_set;
	num = best_num;
	r = best_error / best_consensus_set.size();
	float u = best_consensus_set.size();
	float v = data.size();
	float persent = u / v;
	cout << u << "  " << v << endl;
	cout << "模型符合率：" << persent << endl;
	return best_model;
}