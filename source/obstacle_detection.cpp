#include "stdafx.h"
#include "obstacle_detection.h"
#include "generalDataFuction.h"
#include "matcher.h"
#include <time.h>


//������淽��
Mat ob_detection::findH(Mat left, Mat right, Point3f& ransac_modle, vector<Point3f>& groundpoints,int filenum, cv::Mat WPlane){
	//��������ȡ
	RobustMatcher rmatcher;
	rmatcher.setConfidenceLevel(0.98);
	rmatcher.setMinDistanceToEpipolar(1.0);
	rmatcher.setRatio(0.65f);
	Ptr<FeatureDetector> pfd = new SurfFeatureDetector(20);
	rmatcher.setFeatureDetector(pfd);

	clock_t start1, finish1;
	double SURFtime;
	start1 = clock();
	vector<DMatch> matches;//ȫ����ƥ���
	vector<KeyPoint> keypoints1, keypoints2;
	Mat fundemental = rmatcher.match(left, right, matches, keypoints1, keypoints2);//�õ�ȫ����ƥ����
	Mat empty;
	if (fundemental.empty()){
		cout << "Matched points aren't enough!" << endl;
		return empty;
	}
	
	finish1 = clock();
	SURFtime = (double)(finish1 - start1) / CLOCKS_PER_SEC;
	cout << "SURF����ʱ��Ϊ" << SURFtime << "��" << endl;

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
		//circle(old, Point(x, y), 1, Scalar(0, 0, 0), 2);//��ƥ��㻭��ԭͼold��
		x = keypoints2[it->trainIdx].pt.x;
		y = keypoints2[it->trainIdx].pt.y;
		points2.push_back(Point2f(x, y));
	}

	//������������ά����
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
	
	//RANSAC������ϵ���
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
		ransac_modle = ransac_Plane(testp, best_set, num, 5000, 200);//�����������ݼ������ڵ㣬������������ֵ��������ֵ��ģ��
	else
		ransac_modle = ransac_Plane(testp, best_set, num, 500, 200);

	//�洢����������
	for (int i = 0; i < num.size(); i++)
	{
		int n = num[i];
		groundpoints.push_back(testp[n]);
	}
	finish = clock();
	ransactime = (double)(finish - start) / CLOCKS_PER_SEC;
	cout << "RANSAC����ʱ��Ϊ" << ransactime << "��" << endl;

	//���㵥Ӧ����
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

//����ϰ���
vector<pair<Point3f,int>> ob_detection::obstacletest(Mat left, Mat right, Point3f ransac_modle, Mat homography, Mat& disp, Mat showimage){
	//SGBM����
	StereoSGBM sgbm;
	sgbm.preFilterCap = 63; //Ԥ�����˲����Ľض�ֵ��Ԥ��������ֵ������[-preFilterCap, preFilterCap]��Χ�ڵ�ֵ��������Χ
	sgbm.SADWindowSize = 9;

	int cn = left.channels();

	sgbm.P1 = 8 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;//�����Ӳ�仯ƽ���ԵĲ�����P1��P2��ֵԽ���Ӳ�Խƽ����P1���������ص��Ӳ���/�� 1 ʱ�ĳͷ�ϵ����P2���������ص��Ӳ�仯ֵ����1ʱ�ĳͷ�ϵ����P2�������P1
	sgbm.P2 = 32 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.minDisparity = 0;
	sgbm.numberOfDisparities = 32;
	sgbm.uniquenessRatio = 15;
	sgbm.speckleWindowSize = 100;
	sgbm.speckleRange = 10;
	sgbm.disp12MaxDiff = 1;
	sgbm.fullDP = 0; //������Ϊ TRUE ʱ������˫ͨ����̬����㷨

	//��Ӧ�任
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
	printf("SGBM����ʱ��Ϊ: %fms\n", t * 1000 / getTickFrequency());
	disp.convertTo(disp, CV_8U);
	Mat disp_show;
	resize(disp,disp_show,Size(320,240));	//����
	imshow("disp", disp_show);
	HWND hwnd_disp = FindWindow(NULL,"disp");
	SetWindowPos(hwnd_disp,HWND_NOTOPMOST,640,661,320,240,SWP_ASYNCWINDOWPOS|SWP_DEFERERASE|SWP_NOSIZE|SWP_NOZORDER|SWP_NOACTIVATE);

	//��ȡ�Ҷ�ֵΪ�趨ֵ��һϵ�е�
	vector<Point2f> gray_ten;
	for (int i = 0; i < disp.rows; i++)
	{
		for (int j = 0; j < disp.cols; j++)
		{
			if ((disp.at<uchar>(i, j)) > 20 && (disp.at<uchar>(i, j)) < 200) //�Ӳ���ȡ����ֵ
			{
				Point2f gray10 = Point(j, i);
				gray_ten.push_back(gray10);
			}
		}
	}
	cout << "�Ҷ�ֵΪ�ֵ�ĵ����Ŀ��" << gray_ten.size() << endl;

	//gray_ten.sizeΪ0ʱ������һ������Ұ��ĵ�
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

	//ӳ�䵽����ͼ���Ա������ά����

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
	//����Ӧ�Ա任���õ���Ե������ͼ�е�����
	vector<Point2f> objll;
	perspectiveTransform(objl, objll, homo_inv);
	for (int i = 0; i < objll.size(); i++)
	{
		//int n = i * 10;
		circle(showimage, Point(2 * objll[i].x, 2 * objll[i].y), 5, Scalar(0), 2);
		circle(left, Point(objll[i].x, objll[i].y), 5, Scalar(0), 2);
	}
	//imshow("drawll", left);

	//������������ά����
	cout << "���������ϵ��ά���꣺" << endl;
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
	cout << "edge��ĿΪ���޶�5m*5m����" << edge.size() << endl;

	return edge;
}

//��С���˷����ƽ�淽��ax+by+cz=1
Point3f ob_detection::planeguess(vector<Point3f>points){
	Mat a = Mat::zeros(points.size(), 3, CV_32F);    //��ʼ��ȫ0����
	Mat y = Mat::zeros(points.size(), 1, CV_32F);    //��ʼ��ȫ0����
	Mat x = Mat::zeros(3, 1, CV_32F);    //��ʼ��ȫ0����
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

//�жϵ��Ƿ����ģ��
bool ob_detection::injudge(Point3f point, Point3f modle, float t){

	float dis;
	dis = abs(point.x*modle.x + point.y*modle.y + point.z*modle.z - 1) / sqrt(pow(modle.x, 2) + pow(modle.y, 2) + pow(modle.z, 2));

	//cout << "dis=" << dis << endl;

	if (dis > t) return false;
	else{
		return true;
	}
}

//����ģ�����
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

//RANSACƽ�����
Point3f ob_detection::ransac_Plane(vector<Point3f>points, vector<Point3f>&set, vector<int>&num, int iterat, float thresh){

	srand((unsigned)time(NULL));

	vector<Point3f>data = points;//���ݼ�
	Point3f model;
	int size = data.size();
	int n = 3;//������ģ�͵��������ݸ���
	float p = 0.8;//�㷨���õĸ���
	int k = iterat;//�㷨��������*
	float t = thresh;//�����Ƿ���Ӧ��ģ�͵���ֵ,200mm
	int d1 = data.size()*0.1;//�ж�ģ���Ƿ����������ݼ�����Ŀ������20%
	int d2 = 0;

	Point3f best_model;
	best_model.x = 0; best_model.y = 0; best_model.z = 0;
	vector<Point3f>best_consensus_set;
	vector<int>best_num;
	float best_error = 1e38;// ����;

	int iterations = 0;
	while (iterations < k){
		vector<Point3f>maybe_in;
		vector<Point3f>consensus_set;
		vector<int>maybe_num;
		Point3f maybe_model;
		Point3f better_model;
		//���ѡȡ������
		int a, b, c;
		do{
			a = rand() % data.size();
			b = rand() % data.size();
			c = rand() % data.size();
		} while (a == b || a == c || b == c);
		maybe_in.push_back(data[a]);
		maybe_in.push_back(data[b]);
		maybe_in.push_back(data[c]);

		//������Ӧƽ�����
		Mat m = Mat::zeros(3, 3, CV_32F);    //��ʼ��ȫ0����
		Mat n = Mat::zeros(3, 1, CV_32F);    //��ʼ��ȫ0����
		Mat x = Mat::zeros(3, 1, CV_32F);    //��ʼ��ȫ0����
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
		//�������е�
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
	cout << "ģ�ͷ����ʣ�" << persent << endl;
	return best_model;
}

Point3f ob_detection::ransac_Plane(vector<Point3f>points, vector<Point3f>&set, vector<int>&num, int iterat, float t1, float t2, Point3f CPlane, float &r){

	srand((unsigned)time(NULL));

	vector<Point3f>data = points;//���ݼ�
	Point3f model;
	int size = data.size();
	float p = 0.8;//�㷨���õĸ���
	int k = iterat;//�㷨��������*
	int d1 = data.size()*0.1;//�ж�ģ���Ƿ����������ݼ�����Ŀ������20%
	int d2 = 0;

	Point3f best_model;
	best_model.x = 0; best_model.y = 0; best_model.z = 0;
	vector<Point3f>best_consensus_set;
	vector<int>best_num;
	float best_error = 1e38;// ����;
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
			//�������е�
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
	cout << "ģ�ͷ����ʣ�" << persent << endl;
	return best_model;
}