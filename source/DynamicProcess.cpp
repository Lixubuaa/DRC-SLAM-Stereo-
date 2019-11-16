#include "DynamicProcess.h"
#include "Converter.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "dynamic/header.hpp"
#include "dynamic/SPSStereo.h"
#include "dynamic/defParameter.h"
#include <stdint.h> 
#include <time.h>

#include<iomanip>
using namespace std;
using namespace cv;



namespace ORB_SLAM2{
	
	void makeSegmentBoundaryImage(const cv::Mat & inputImage,
		const /*png::image<png::gray_pixel_16>*/ cv::Mat & segmentImage,
		std::vector< std::vector<int> >& boundaryLabels,
		cv::Mat& segmentBoundaryImage, std::vector<cv::Point2f>, cv::Mat disp);
	Mat otsuGray(const Mat& src);
	vector<Rect> draw_dynamic_object(const Mat&, std::vector<Point2f>&, const Mat& left, const Mat& segment);
	int width;
	int height;
	//Mat preframe;

	//DynamicProcess::DynamicProcess(ORB_SLAM2::Map* World) :
	//	sWorld(World)/*, obstacle(0), blindwaypar(1, { 0, 0, 0 }), mbObstacle(true), mbBlindway(true), meLineTrackMode(eLineTrackMode::Close),
	//	mbRestartNeed(true), mbSelectNeed(false), mvCandilinesWpoints(0), mvBestlineWpoints(0), mbStopped(false), mbStopRequested(false)*/
	//{
	//	//SetCalibrateParams();
	//}
	DynamicProcess::DynamicProcess() 
		/*, obstacle(0), blindwaypar(1, { 0, 0, 0 }), mbObstacle(true), mbBlindway(true), meLineTrackMode(eLineTrackMode::Close),
					 mbRestartNeed(true), mbSelectNeed(false), mvCandilinesWpoints(0), mvBestlineWpoints(0), mbStopped(false), mbStopRequested(false)*/
	{
		//SetCalibrateParams();
	}


	std::vector<cv::Rect> DynamicProcess::Process(cv::Mat imLeft, cv::Mat imRight, cv::Mat pre, cv::Mat& dynamic_mask)
	{
		int num = 0;
		//ramesNum = sWorld->KeyFramesInMap();
		//while (1){

			double start_time, end_time, t0, t1, t2, t3, t4, t5, t6;
			start_time = (double)clock() / CLOCKS_PER_SEC;
			
			//if (FramesNum){
			im = imLeft;
			im_r = imRight;
				if (im.rows != 0){
					//cv::imshow("Dynamicinput", im);
					cv::waitKey(1);
				}
				MOD mod;
				vector<Point2f> DynamicPoint;
				int _width, _height;
				_width = im.cols;
				_height = im.rows;
				_width = _width / 2;
				_height = _height / 2;

				resize(im, im, Size(_width, _height));
				resize(im_r, im_r, Size(_width, _height));

				Mat frame;
				cv::cvtColor(im_r, frame, CV_BGR2GRAY);
				t6 = (double)clock() / CLOCKS_PER_SEC;
				//Mat im_result;
				//im_result = im.clone();
				//if (!preframe.empty()){
				Mat preframe = pre;
				cv::cvtColor(preframe, preframe, CV_BGR2GRAY);
					resize(preframe, preframe, Size(_width, _height));
					t5 = (double)clock() / CLOCKS_PER_SEC;

					t0 = (double)clock() / CLOCKS_PER_SEC;
					//SGBM
					StereoSGBM sgbm;
					sgbm.preFilterCap = 63; //预处理滤波器的截断值，预处理的输出值仅保留[-preFilterCap, preFilterCap]范围内的值，参数范围
					sgbm.SADWindowSize = 9;

					int cn = im.channels();

					sgbm.P1 = 8 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;//控制视差变化平滑性的参数。P1、P2的值越大，视差越平滑。P1是相邻像素点视差增/减 1 时的惩罚系数；P2是相邻像素点视差变化值大于1时的惩罚系数。P2必须大于P1
					sgbm.P2 = 32 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
					sgbm.minDisparity = 0;
					sgbm.numberOfDisparities = 32;
					sgbm.uniquenessRatio = 15;
					sgbm.speckleWindowSize = 100;
					sgbm.speckleRange = 10;
					sgbm.disp12MaxDiff = 1;
					sgbm.fullDP = 0; //当设置为 TRUE 时，运行双通道动态编程算法
					Mat disp, normdisp, vdisp, disp_filter9;
					//cout << "before cut size = " << im.size() << endl;
					
					sgbm(im, im_r, disp);
					disp.convertTo(vdisp, CV_8U, 255 / (sgbm.numberOfDisparities * 16.));
					/*cv::Rect vdisp_rect(20, 0, leftImage.cols - 40, leftImage.rows);
					Mat vdisp_roi = vdisp(vdisp_rect);*/
					
					//medianBlur(vdisp, disp_filter9, 9);
					
				//	cv::imshow("vdisp", vdisp);
				//	cv::waitKey(3);
					Mat threhold_vdisp;
					threhold_vdisp = otsuGray(vdisp);
					/*cv::imshow("threhold_vdisp", threhold_vdisp);
					cv::waitKey(1);*/

					t1 = (double)clock() / CLOCKS_PER_SEC;

					//MOD
				//	cout << "start MOD" << endl;
					DynamicPoint = mod.process(preframe, frame, threhold_vdisp);
					//SPS
					SPSStereo sps;
					sps.setIterationTotal(outerIterationTotal, innerIterationTotal);
					sps.setWeightParameter(lambda_pos, lambda_depth, lambda_bou, lambda_smo);
					sps.setInlierThreshold(lambda_d);
					sps.setPenaltyParameter(lambda_hinge, lambda_occ, lambda_pen);

					cv::Mat segmentImage;
					cv::Mat disparityImage;
					std::vector< std::vector<double> > disparityPlaneParameters;
					std::vector< std::vector<int> > boundaryLabels;
				//	cout << "start SPS" << endl;
					sps.compute(superpixelTotal, im, im_r, segmentImage, disparityImage, disparityPlaneParameters, boundaryLabels, vdisp);
				//	cout << "end SPS" << endl;
					vector<int> dynamicflag(static_cast<int>(disparityPlaneParameters.size()));
					
					int DynaSize = DynamicPoint.size();
					vector<Point2f> DynamicPoint_object;
					Mat point_image = im.clone();
					DynamicPoint_object = DynamicPoint;//注释
					//for (int i = 0; i < DynaSize; i++){
					//	if (!(threhold_vdisp.at<uchar>(DynamicPoint[i].y, DynamicPoint[i].x) == 0)){
					//		//circle(segmentBoundaryImage, DynamicPoint[i], 1, Scalar(0, 0, 255),1);
					//		DynamicPoint_object.push_back(DynamicPoint[i]);
					//	}
					//}
					/*cv::imshow("point_image", point_image);
					cv::waitKey(1);*/
					t2 = (double)clock() / CLOCKS_PER_SEC;
					cv::Mat segmentBoundaryImage;
					makeSegmentBoundaryImage(im, segmentImage, boundaryLabels, segmentBoundaryImage, DynamicPoint_object, threhold_vdisp);
					//cvtColor(segmentBoundaryImage, segmentBoundaryImage, CV_BGR2GRAY);
					//threshold(segmentBoundaryImage, segmentBoundaryImage, 145, 255, THRESH_BINARY);
					t3 = (double)clock() / CLOCKS_PER_SEC;
					Mat DynamicObjectMask, DynamicContour;
					//DynamicContour = drawDynamicObject(segmentBoundaryImage, leftImage, boundaryLabels, DynamicPoint, DynamicObjectMask);
					//drawDynamicObject(segmentBoundaryImage, leftImage, boundaryLabels, DynamicPoint, DynamicObjectMask);
					//drawDynamicObject(segmentBoundaryImage, segmentImage, boundaryLabels, DynamicPoint, DynamicObjectMask);

					vector<Rect> dynamic_rect;
					//Mat dynamic_mask;
				//	cout << "start DRAW" << endl;
					dynamic_rect = draw_dynamic_object(segmentBoundaryImage, DynamicPoint_object, im, segmentImage);
					Mat out_image = im.clone();
					for (int i = 0; i < dynamic_rect.size(); i++){
						cv::rectangle(out_image, dynamic_rect[i], cv::Scalar(0, 0, 255), 2, 8);
					}
					imshow("out_image", out_image);

					waitKey(3);
					t4 = (double)clock() / CLOCKS_PER_SEC;
					Mat result_image(_height, _width, CV_8UC1, Scalar(1));
					//Vec3b white, black;
					//white.val[2] = 255; white.val[1] = 255; white.val[0] = 255;
					//black.val[2] = 0; black.val[1] = 0; black.val[0] = 0;
					//加KCF跟踪，if动态框大小在连续帧中相似
					//cout << "test " << dynamic_rect.size()<< endl;
					for (int i = 0; i < dynamic_rect.size(); i++){
						//cout << "dynamic rect size = " << dynamic_rect.size() << endl;
						//rectangle(im, dynamic_rect[i].tl(), dynamic_rect[i].br(), Scalar(0, 0, 255), 2, 8, 0);
						for (int m = 0; m < _width; m++){
							for (int n = 0; n < _height; n++){
								if (result_image.at<uchar>(n, m) != 0){
									if (m>dynamic_rect[i].tl().x && m<dynamic_rect[i].br().x && n>dynamic_rect[i].tl().y && n<dynamic_rect[i].br().y){
										result_image.at<uchar>(n, m) = 0;
									}
									/*else{
										result_image.at<uchar>(n, m) = 1;
									}*/
								}
							
							}
						}

					}
					/*cv::imshow("disparityImage", disparityImage);
					cv::waitKey(3);*/
					//stringstream ss;
					//ss << setfill('0') << setw(6) << num;
					//std::string saveresult = "resultrect/";
					//cv::imwrite(saveresult + ss.str() + ".png", segmentBoundaryImage);
				
					end_time = (double)clock() / CLOCKS_PER_SEC;
				/*	std::cout << "-1st 耗时：" << t6 - start_time << "s" << std::endl;
					std::cout << "0st 耗时：" << t5 - t6 << "s" << std::endl;
					std::cout << "1st 耗时：" << t0 - t5 << "s" << std::endl;
					std::cout << "2nd 耗时：" << t1 - t0 << "s" << std::endl;
					std::cout << "3rd 耗时：" << t2 - t1 << "s" << std::endl;
					std::cout << "4th 耗时：" << t3 - t2 << "s" << std::endl;

					std::cout << "5th 耗时：" << t4 - t3 << "s" << std::endl;
					std::cout << "6th 耗时：" << end_time - t4 << "s" << std::endl;*/
				//	std::cout << "dynamicprocess total time-comsuming：" << end_time - start_time << "s" << std::endl;
					/*cv::imshow("im", im);
					waitKey(1);*/
					//cv::imshow("result", result_image);
					//cv::imwrite(saveresult + ss.str() + ".png", im_result);
					//cvWaitKey(0);
					num++;
			//	}
			//	std::swap(preframe, frame);
					dynamic_mask = result_image;
					return dynamic_rect;
			
				//cout << "DynamicProcess start" << endl;
			//}

		//}
	}
	void makeSegmentBoundaryImage(const cv::Mat & inputImage,
		const /*png::image<png::gray_pixel_16>*/ cv::Mat & segmentImage,
		std::vector< std::vector<int> >& boundaryLabels,
		cv::Mat& segmentBoundaryImage, std::vector<Point2f> dynamic_point, cv::Mat disp)
	{
		width = static_cast<int>(inputImage.cols);
		height = static_cast<int>(inputImage.rows);
		int boundaryTotal = static_cast<int>(boundaryLabels.size());
		cv::Mat whiteImage(height, width, CV_8UC1, Scalar(255));
		/*imshow("outImage",whiteImage);
		waitKey(3);*/
		segmentBoundaryImage.create(height, width, CV_8UC1);
		segmentBoundaryImage = whiteImage;
		/*for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
		segmentBoundaryImage.at<cv::Vec3b>(y, x) = whiteImage.at<cv::Vec3b>(y, x);
		}
		}*/

		//int boundaryWidth = 0;
		/*for (int y = 0; y < height - 1; ++y) {
		for (int x = 0; x < width - 1; ++x) {
		if (!(disp.at<uchar>(y, x) == 0)){
		int pixelLabelIndex = segmentImage.at<uint16_t>(y, x);

		if (segmentImage.at<uint16_t>(y, x + 1) != pixelLabelIndex) {
		for (int w = 0; w < boundaryWidth - 1; ++w) {
		if (x - w >= 0) segmentBoundaryImage.at<cv::Vec3b>(y, x - w) = cv::Vec3b(128, 128, 128);
		}
		for (int w = 1; w < boundaryWidth; ++w) {
		if (x + w < width) segmentBoundaryImage.at<cv::Vec3b>(y, x + w) = cv::Vec3b(128, 128, 128);
		}
		}
		if (segmentImage.at<uint16_t>(y + 1, x) != pixelLabelIndex) {
		for (int w = 0; w < boundaryWidth - 1; ++w) {
		if (y - w >= 0) segmentBoundaryImage.at<cv::Vec3b>(y - w, x) = cv::Vec3b(128, 128, 128);
		}
		for (int w = 1; w < boundaryWidth; ++w) {
		if (y + w < height) segmentBoundaryImage.at<cv::Vec3b>(y + w, x) = cv::Vec3b(128, 128, 128);
		}
		}
		}

		}
		}*/

		int boundaryWidth = 2;
		//边缘部分不做分割
		for (int y = 5; y < height - 5; ++y) {
			for (int x = 5; x < width - 5; ++x) {

				int pixelLabelIndex = segmentImage.at<uint16_t>(y, x);
				cv::Vec3b negativeSideColor, positiveSideColor;
				uchar gray_color = 0;
				if (segmentImage.at<uint16_t>(y, x + 1) != pixelLabelIndex) {

					int pixelBoundaryIndex = -1;
					for (int boundaryIndex = 0; boundaryIndex < boundaryTotal; ++boundaryIndex) {
						if ((boundaryLabels[boundaryIndex][0] == pixelLabelIndex && boundaryLabels[boundaryIndex][1] == segmentImage.at<uint16_t>(y, x + 1))
							|| (boundaryLabels[boundaryIndex][0] == segmentImage.at<uint16_t>(y, x + 1) && boundaryLabels[boundaryIndex][1] == pixelLabelIndex))
						{
							pixelBoundaryIndex = boundaryIndex;
							break;
						}
					}
					if (boundaryLabels[pixelBoundaryIndex][2] == 3) continue;
					//else if (boundaryLabels[pixelBoundaryIndex][2] == 2) {
					//	//negativeSideColor.val[2] = 0;  negativeSideColor.val[1] = 225;  negativeSideColor.val[0] = 0;
					//	//positiveSideColor.val[2] = 0;  positiveSideColor.val[1] = 225;  positiveSideColor.val[0] = 0;
					//	gray_color = 0;
					//}
					//else if (pixelLabelIndex == boundaryLabels[pixelBoundaryIndex][boundaryLabels[pixelBoundaryIndex][2]]) {
					//	//negativeSideColor.val[2] = 225;  negativeSideColor.val[1] = 0;  negativeSideColor.val[0] = 0;
					//	//positiveSideColor.val[2] = 0;  positiveSideColor.val[1] = 0;  positiveSideColor.val[0] = 225;
					//	gray_color = 0;
					//}
					//else {
					//	//negativeSideColor.val[2] = 0;  negativeSideColor.val[1] = 0;  negativeSideColor.val[0] = 225;
					//	//positiveSideColor.val[2] = 225;  positiveSideColor.val[1] = 0;  positiveSideColor.val[0] = 0;
					//	gray_color = 0;
					//}

					/*for (int w = 0; w < boundaryWidth - 1; ++w) {
					if (x - w >= 0) segmentBoundaryImage.at<cv::Vec3b>(y, x - w) = negativeSideColor;
					}*/
					for (int w = 1; w < boundaryWidth; ++w) {
						//if (x + w < width) segmentBoundaryImage.at<cv::Vec3b>(y, x + w) = positiveSideColor;
						if (x + w < width) segmentBoundaryImage.at<uchar>(y, x + w) = gray_color;
					}
				}
				if (segmentImage.at<uint16_t>(y + 1, x) != pixelLabelIndex) {
					cv::Vec3b negativeSideColor, positiveSideColor;
					int pixelBoundaryIndex = -1;
					for (int boundaryIndex = 0; boundaryIndex < boundaryTotal; ++boundaryIndex) {
						if ((boundaryLabels[boundaryIndex][0] == pixelLabelIndex && boundaryLabels[boundaryIndex][1] == segmentImage.at<uint16_t>(y + 1, x))
							|| (boundaryLabels[boundaryIndex][0] == segmentImage.at<uint16_t>(y + 1, x) && boundaryLabels[boundaryIndex][1] == pixelLabelIndex))
						{
							pixelBoundaryIndex = boundaryIndex;
							break;
						}
					}
					if (boundaryLabels[pixelBoundaryIndex][2] == 3) continue;
					//else if (boundaryLabels[pixelBoundaryIndex][2] == 2) {
					//	//negativeSideColor.val[2] = 0;  negativeSideColor.val[1] = 225;  negativeSideColor.val[0] = 0;
					//	//positiveSideColor.val[2] = 0;  positiveSideColor.val[1] = 225;  positiveSideColor.val[0] = 0;
					//	gray_color = 0;
					//}
					//else if (pixelLabelIndex == boundaryLabels[pixelBoundaryIndex][boundaryLabels[pixelBoundaryIndex][2]]) {
					//	//negativeSideColor.val[2] = 225;  negativeSideColor.val[1] = 0;  negativeSideColor.val[0] = 0;
					//	//positiveSideColor.val[2] = 0;  positiveSideColor.val[1] = 0;  positiveSideColor.val[0] = 225;
					//	gray_color = 0;
					//}
					//else {
					//	//negativeSideColor.val[2] = 0;  negativeSideColor.val[1] = 0;  negativeSideColor.val[0] = 225;
					//	//positiveSideColor.val[2] = 225;  positiveSideColor.val[1] = 0;  positiveSideColor.val[0] = 0;
					//	gray_color = 0;
					//}

					/*	for (int w = 0; w < boundaryWidth - 1; ++w) {
					if (y - w >= 0) segmentBoundaryImage.at<cv::Vec3b>(y - w, x) = negativeSideColor;
					}*/
					for (int w = 1; w < boundaryWidth; ++w) {
						//if (y + w < height) segmentBoundaryImage.at<cv::Vec3b>(y + w, x) = positiveSideColor;
						if (y + w < height) segmentBoundaryImage.at<uchar>(y + w, x) = gray_color;
					}
				}

			}
		}
	}


Mat otsuGray(const Mat& src) {
		Mat img = src.clone();
		int c = img.cols; //图像列数
		int r = img.rows; //图像行数
		int T = 0; //阈值
		uchar* data = img.data; //数据指针
		int ftNum = 0; //前景像素个数
		int bgNum = 0; //背景像素个数
		int NN = c*r; //总像素个数
		int ftSum = 0; //前景总灰度值
		int bgSum = 0; //背景总灰度值
		int graySum = 0;
		double w0 = 0; //前景像素个数占比
		double w1 = 0; //背景像素个数占比
		double u0 = 0; //前景平均灰度
		double u1 = 0; //背景平均灰度
		double Histogram[256] = { 0 }; //灰度直方图
		double temp = 0; //临时类间方差
		double g = 0; //类间方差

		//灰度直方图
		for (int i = 0; i < r; i++) {
			for (int j = 0; j <c; j++) {
				Histogram[img.at<uchar>(i, j)]++;
			}
		}
		//求总灰度值
		for (int i = 0; i < 256; i++) {
			graySum += Histogram[i] * i;
		}

		for (int i = 0; i < 256; i++) {
			ftNum += Histogram[i];  //阈值为i时前景个数
			bgNum = NN - ftNum;      //阈值为i时背景个数
			w0 = (double)ftNum / NN; //前景像素占总数比
			w1 = (double)bgNum / NN; //背景像素占总数比
			if (ftNum == 0) continue;
			if (bgNum == 0) break;
			//前景平均灰度
			ftSum += i*Histogram[i];
			u0 = ftSum / ftNum;

			//背景平均灰度
			bgSum = graySum - ftSum;
			u1 = bgSum / bgNum;

			g = w0*w1*(u0 - u1)*(u0 - u1);
			if (g > temp) {
				temp = g;
				T = i+30 ;
			}
		}
		cout << "threhold = " << T << endl;
		for (int i = 0; i<img.rows; i++)
		{
			for (int j = 0; j<img.cols; j++)
			{
				if ((int)img.at<uchar>(i, j)>T || (int)img.at<uchar>(i, j) == 0)
					img.at<uchar>(i, j) = 255;
				else
					img.at<uchar>(i, j) = 0;
			}
		}
		return img;
	}

vector<Rect> draw_dynamic_object(const Mat& segment, std::vector<Point2f>& dynamicpoint, const Mat& leftimage, const Mat&segmentimage){
	vector<Rect> result;
	vector<vector<cv::Point>> result_contours;
	Mat dynamicmask;
	Point2f sorted_point;
	Point2f padding = Point2f(0.0, -1.0);

	//cvtColor(segment, segment, CV_BGR2GRAY);
	threshold(segment, segment, 200, 255, CV_THRESH_BINARY);
	Mat erode_element;
	erode_element = getStructuringElement(MORPH_RECT, Size(2, 2));
	erode(segment, segment, erode_element);

	/*imshow("segment", segment);
	waitKey(1);*/
	vector<Point2f>::iterator it;
	vector<Point> _border;
	/// 画多边形轮廓 + 包围的矩形框 + 圆形框


	Mat drawing = Mat::zeros(segment.size(), CV_8UC3);
	for (it = dynamicpoint.begin(); it != dynamicpoint.end();)
	{
		Point2f borderpoint = *it;
		while (segment.at<uchar>(int(borderpoint.y), int(borderpoint.x)) != 0 && floor(borderpoint.y) > 0){

			borderpoint = borderpoint + padding;
		}
		//std::cout << "stop zuobiao " << int(borderpoint.y) << endl;
		if (int(borderpoint.y) == 0 || int(borderpoint.y) == 1){
			dynamicpoint.erase(it);

		}
		else{
			_border.push_back(Point(borderpoint));

			//line(segment, *it, borderpoint, Scalar(0), 1);
			++it;
		}

	}
	//for (int i = 0; i < _border.size(); i++){
	//	//cout << "border point = " << _border[i] << endl;
	//	circle(leftimage, _border[i], 3, Scalar(0, 255, 0), 2);
	//}

	//for (int i = 0; i < dynamicpoint.size(); i++){
	//	//cout << "border point = " << _border[i] << endl;
	//	circle(leftimage, dynamicpoint[i], 3, Scalar(0, 0, 255), 2);
	//}

	/*imshow("leftimage_point", leftimage);
	waitKey(1);

	imshow("segment", segment);
	waitKey(1);*/

	vector<vector<cv::Point>>contours;   //轮廓数组（非矩形数组），每个轮廓是一个Point型的vector
	vector<Vec4i>hierarchy;                //见下面findContours的解释
	findContours(segment, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	vector<vector<cv::Point>> contours_poly(contours.size());         //近似后的轮廓点集
	vector<Rect>boundRect(contours.size());                          //包围点集的最小矩形vector
	vector<Point2f>center(contours.size());                              //包围点集的最小圆形vector
	vector<float>radius(contours.size());                                  //包围点集的最小圆形半径vector



	int dyna_flag;
	int dynamic_height;
	for (int i = 0; i<contours.size(); i++)
	{

		//minEnclosingCircle(contours_poly[i], center[i], radius[i]);    //计算并返回包围轮廓点集的最小圆形及其半径
		vector<Point> this_dynamicpoint;
		vector<Point> this_borderpoint;
		vector<Point> _contour;
		_contour = contours[i];
		dyna_flag = 0;
		Point bottom_point, left_point, right_point, top_point;
		//cout << "border size = " << _border.size() << endl;
		for (int j = 0; j < _border.size(); j++){
			if (abs(pointPolygonTest(contours[i], _border[j], true)) <= 2.0){
				if (dyna_flag == 0){
					bottom_point = dynamicpoint[j];

				}
				else{
					if (dynamicpoint[j].y>bottom_point.y){
						bottom_point = dynamicpoint[j];

					}
				}
				this_dynamicpoint.push_back(Point(dynamicpoint[j]));
				this_borderpoint.push_back(_border[j]);
				for (int m = 0; m < contours[i].size(); m++){
					if (dyna_flag == 0){
						top_point = _contour[0];

					}
					else{
						if (contours[i][m].y<top_point.y){
							top_point = contours[i][m];
						}
					}
				}


				dyna_flag = dyna_flag + 1;
			}

		}
		//cout << "top_point = " << top_point << endl;
		//cout << "dyna_flag = " << dyna_flag << endl;

		dynamic_height = bottom_point.y - top_point.y;

		//cout << " dyna_flag = " << dyna_flag << endl;
		//cout << " _border point size " << _border.size() << endl;
		//cout << " dynamicpoint size " << dynamicpoint.size() << endl;
		Scalar color = Scalar(0, 255, 0);
		if (dyna_flag>15){
			for (int n = 0; n < contours[i].size(); n++){
				if (contours[i][n].x == boundRect[i].tl().x){
					left_point = contours[i][n];
					continue;
				}
			}
			for (int n = 0; n < contours[i].size(); n++){
				if (contours[i][n].x > right_point.x){
					right_point = contours[i][n];
				}
			}
			if (contours[i].size() > 300 || boundRect[i].width > 160 || boundRect[i].width < 10 || boundRect[i].height < 10){
				vector<Point> multi_point;

				vector<Point> approx_multi;
				Rect Rect_multi;

				multi_point.insert(multi_point.end(), this_dynamicpoint.begin(), this_dynamicpoint.end());
				multi_point.insert(multi_point.end(), this_borderpoint.begin(), this_borderpoint.end());
				if (multi_point.size() >= 4){
					approxPolyDP(Mat(multi_point), approx_multi, 1, false);
					Rect_multi = boundingRect(Mat(approx_multi));

					if (/*Rect_multi.width<150 || */Rect_multi.width>10){
						//cout << "rect_factor = " << rect_factor << endl;
						rectangle(drawing, Rect_multi.tl(), Rect_multi.br(), color, 2, 8, 0);
						result.push_back(Rect_multi);
					}

				}
				continue;
			}
			approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);     //对多边形曲线做适当近似，contours_poly[i]是输出的近似点集
			boundRect[i] = boundingRect(Mat(contours_poly[i]));        //计算并返回包围轮廓点集的最小矩形
			//drawContours(drawing, contours_poly, i, color, 2, 8, hierarchy, 0, cv::Point());        //根据轮廓点集contours_poly和轮廓结构hierarchy画出轮廓
			boundRect[i].height = dynamic_height;
			//rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
			//circle(drawing, bottom_point, 6, Scalar(0, 0, 255), 6);
			//circle(drawing, top_point, 6, Scalar(255, 0, 0), 6);
			//circle(drawing, left_point, 3, Scalar(255, 0, 255), 3);
			//circle(drawing, right_point, 3, Scalar(255, 255, 0), 3);


			result.push_back(boundRect[i]);
			//circle(drawing, contours[i][contours[i].size() - 1], 6, Scalar(0, 255, 255), 6);

		}

	}
	//cout << "dynamicpoint.size()" << dynamicpoint.size() << endl;

	//cv::imshow("drawing", drawing);
	//cv::waitKey(3);
	return result;
}


}