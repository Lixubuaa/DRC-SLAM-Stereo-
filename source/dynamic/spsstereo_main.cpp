/*
    Copyright (C) 2014  Koichiro Yamaguchi

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "header.hpp"
#include "SPSStereo.h"
#include "defParameter.h"
#include <stdint.h> 
#include <time.h>

#include<iomanip>
using namespace std;
void Rectify(cv::Mat im_l, cv::Mat im_r, cv::Mat& imLeft, cv::Mat& imRight);
void makeSegmentBoundaryImage(const cv::Mat & inputImage,
							  const /*png::image<png::gray_pixel_16>*/ cv::Mat & segmentImage,
							  std::vector< std::vector<int> >& boundaryLabels,
							  cv::Mat& segmentBoundaryImage,std::vector<cv::Point2f>,cv::Mat disp);
void writeDisparityPlaneFile(const std::vector< std::vector<double> >& disparityPlaneParameters, const std::string outputDisparityPlaneFilename);
void writeBoundaryLabelFile(const std::vector< std::vector<int> >& boundaryLabels, const std::string outputBoundaryLabelFilename);
void drawDynamicObject(const cv::Mat & inputImage,
	const cv::Mat & segmentImage,
	std::vector< std::vector<int> >& boundaryLabels, vector<Point2f> Point,
	cv::Mat& DynamicObjectMask);
bool isBoundary(const int x, const int y, const cv::Mat & segmentImage);
bool ContourIsDynamicMask(vector<cv::Point> contour, vector<cv::Point2f> dynamic, vector<bool> dynamic_flag);
Mat otsuGray(const Mat src);
std::vector<cv::Rect> draw_dynamic_object(cv::Mat, std::vector<cv::Point2f>, cv::Mat left, cv::Mat segment);
Point2f sort_point(vector<Point2f> point);
Mat draw_mask(vector<Rect>, Mat, Mat, vector<Point2f>, std::vector< std::vector<int>>);
int width;
int height;
int dynamicflag_size;
Mat preframe;
vector<vector<Point>> dynamic_point_vector;
int main() {

	string dir_path = "E:/dataset/dynamic_data/20190301_lc/";
	

	string left_dir_path = dir_path+"image_0/";
	string right_dir_path = dir_path + "image_1/";

	Directory left_dir;
	  
	Directory right_dir;
	vector<string> left_fileNames = left_dir.GetListFiles(left_dir_path, "*.png", false);
	vector<string> right_fileNames = left_dir.GetListFiles(right_dir_path, "*.png", false);
	for (int i = 0; i < left_fileNames.size()-1; i++)	{
		double start_time, end_time, t0, t1, t2, t3, t4, t5, t6;
		start_time = (double)clock() / CLOCKS_PER_SEC;
		
		string left_fileFullName = left_dir_path + left_fileNames[i];

		Mat leftImage = imread(left_fileFullName);
		MOD mod;
		vector<Point2f> DynamicPoint;
		
		string right_fileFullName = right_dir_path + right_fileNames[i];
		cv::Mat rightImage = cv::imread(right_fileFullName);
		
		
		int _width, _height;
		_width = leftImage.cols;
		_height = leftImage.rows;
		_width = _width / 2;
		_height = _height / 2;
		resize(leftImage, leftImage, Size(_width, _height));
		resize(rightImage, rightImage, Size(_width, _height));
		
		Mat frame;
		cv::cvtColor(leftImage, frame, CV_BGR2GRAY);
		t6 = (double)clock() / CLOCKS_PER_SEC;
		if (!preframe.empty()){
			resize(preframe, preframe, Size(_width, _height));
			t5 = (double)clock() / CLOCKS_PER_SEC;
			t0 = (double)clock() / CLOCKS_PER_SEC;
			//SGBM
			StereoSGBM sgbm;
			sgbm.preFilterCap = 63; //预处理滤波器的截断值，预处理的输出值仅保留[-preFilterCap, preFilterCap]范围内的值，参数范围
			sgbm.SADWindowSize = 9;

			int cn = leftImage.channels();

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
			//cout << "before cut size = " << leftImage.size() << endl;
			
			sgbm(leftImage, rightImage, disp);
			disp.convertTo(vdisp, CV_8U, 255 / (sgbm.numberOfDisparities * 16.));
			
			/*cv::imshow("vdisp", vdisp);
			cv::waitKey(3);*/
			//medianBlur(vdisp, disp_filter9, 9);
			/*cv::imshow("disp_filter9", disp_filter9);
			cv::waitKey(3);*/
			Mat threhold_vdisp;
			threhold_vdisp = otsuGray(vdisp);

			t1 = (double)clock() / CLOCKS_PER_SEC;
			
			//MOD
			DynamicPoint = mod.process(preframe, frame, vdisp);
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

			sps.compute(superpixelTotal, leftImage, rightImage, segmentImage, disparityImage, disparityPlaneParameters, boundaryLabels, vdisp);
			
			int DynaSize = DynamicPoint.size();
			vector<Point2f> DynamicPoint_object;
			Mat point_image = leftImage.clone();
			for (int i = 0; i < DynaSize; i++){
				if (!(threhold_vdisp.at<uchar>(DynamicPoint[i].y, DynamicPoint[i].x) == 0)){
					//circle(segmentBoundaryImage, DynamicPoint[i], 1, Scalar(0, 0, 255),1);
					DynamicPoint_object.push_back(DynamicPoint[i]);
					//circle(point_image, DynamicPoint[i], 3, Scalar(0, 0, 255), 2);
				}
			}
			//cv::imshow("point_image", point_image);
			//cv::waitKey(1);
			t2 = (double)clock() / CLOCKS_PER_SEC;
			cv::Mat segmentBoundaryImage;
			makeSegmentBoundaryImage(leftImage, segmentImage, boundaryLabels, segmentBoundaryImage, DynamicPoint_object, threhold_vdisp);
			//cvtColor(segmentBoundaryImage, segmentBoundaryImage, CV_BGR2GRAY);
			//threshold(segmentBoundaryImage, segmentBoundaryImage, 145, 255, THRESH_BINARY);
			
			Mat DynamicObjectMask, DynamicContour;
			//DynamicContour = drawDynamicObject(segmentBoundaryImage, leftImage, boundaryLabels, DynamicPoint, DynamicObjectMask);
			//drawDynamicObject(segmentBoundaryImage, leftImage, boundaryLabels, DynamicPoint, DynamicObjectMask);
			//drawDynamicObject(segmentBoundaryImage, segmentImage, boundaryLabels, DynamicPoint, DynamicObjectMask);

			vector<Rect> dynamic_rect;
			Mat dynamic_mask;
			dynamic_point_vector.clear();
			dynamic_rect = draw_dynamic_object(segmentBoundaryImage, DynamicPoint_object, leftImage, segmentImage);
			Mat result_image(_height,_width,CV_8UC3,Scalar(255,255,255));
			
			
			//result_image = leftImage.clone();
			//dynamic_mask = draw_mask(dynamic_rect, leftImage, segmentImage, DynamicPoint_object, boundaryLabels);
			t3 = (double)clock() / CLOCKS_PER_SEC;
			Vec3b white, black;
			white.val[2] = 255; white.val[1] = 255; white.val[0] = 255;
			black.val[2] = 0; black.val[1] = 0; black.val[0] = 0;
			for (int i = 0; i < dynamic_rect.size(); i++){
				rectangle(leftImage, dynamic_rect[i].tl(), dynamic_rect[i].br(), Scalar(0, 0, 255), 2, 8, 0);
				for (int m = 0; m < _width;m++){
					for (int n = 0; n < _height;n++){
						if (m>dynamic_rect[i].tl().x && m<dynamic_rect[i].br().x && n>dynamic_rect[i].tl().y && n<dynamic_rect[i].br().y){
							result_image.at<Vec3b>(n, m) = black;
						}
						else{
							result_image.at<Vec3b>(n, m) = white;
						}
					}
				}
				
			}
			t4 = (double)clock() / CLOCKS_PER_SEC;
			
			//Mat DynamicObjectMask;
			//drawDynamicObject(leftImage, segmentImage, boundaryLabels, DynamicPoint, DynamicObjectMask);

			/*std::string outputBaseFilename = "E:/李绪/LX的编程练习/sps_stereo/sps/Release/";
			size_t slashPosition = outputBaseFilename.rfind('/');
			if (slashPosition != std::string::npos) outputBaseFilename.erase(0, slashPosition + 1);
			size_t dotPosition = outputBaseFilename.rfind('.');
			if (dotPosition != std::string::npos) outputBaseFilename.erase(dotPosition);
			std::string outputDisparityImageFilename = outputBaseFilename + "_left_disparity.png";
			std::string outputSegmentImageFilename = outputBaseFilename + "_segment.png";
			std::string outputBoundaryImageFilename = outputBaseFilename + "_boundary.png";
			std::string outputDisparityPlaneFilename = outputBaseFilename + "_plane.txt";
			std::string outputBoundaryLabelFilename = outputBaseFilename + "_label.txt";
			cv::imwrite(outputBoundaryImageFilename, segmentBoundaryImage);
			cv::imwrite(outputDisparityImageFilename, disparityImage);
			cv::imwrite(outputSegmentImageFilename, segmentImage);*/
			//cv::imshow("disparityImage", disparityImage);
			//cv::waitKey(1);
			
			//writeDisparityPlaneFile(disparityPlaneParameters, outputDisparityPlaneFilename);
			//writeBoundaryLabelFile(boundaryLabels, outputBoundaryLabelFilename);

			/*cv::imwrite(outputDisparityImageFilename, disparityImage);
			cv::imwrite(outputSegmentImageFilename, segmentImage);
			cv::imwrite(outputBoundaryImageFilename, segmentBoundaryImage);
			writeDisparityPlaneFile(disparityPlaneParameters, outputDisparityPlaneFilename);
			writeBoundaryLabelFile(boundaryLabels, outputBoundaryLabelFilename);*/
			//std::getline(images, leftImageFilename);
			//std::getline(images, rightImageFilename);
			// }
			end_time = (double)clock() / CLOCKS_PER_SEC;
			std::cout << "-1st 耗时：" << t6 - start_time << "s" << std::endl;
			std::cout << "0st 耗时：" << t5 - t6 << "s" << std::endl;
			std::cout << "1st 耗时：" << t0 - t5 << "s" << std::endl;
			std::cout << "2nd 耗时：" << t1 - t0 << "s" << std::endl;
			std::cout << "3rd 耗时：" << t2 - t1 << "s" << std::endl;
			std::cout << "4th 耗时：" << t3 - t2 << "s" << std::endl;

			std::cout << "5th 耗时：" << t4 - t3 << "s" << std::endl;
			std::cout << "6th 耗时：" << end_time - t4 << "s" << std::endl;
			std::cout << "总耗时：" << end_time - start_time << "s" << std::endl;
			//stringstream ss;
			//ss << setfill('0') << setw(6) << i;
			//std::string saveresult = "resultrect/";
			//cv::imwrite(saveresult + ss.str() + ".png", segmentBoundaryImage);
			cv::imshow("result", result_image);
			waitKey(1);
			cv::imshow("leftImage", leftImage);
			//cv::imwrite(saveresult + ss.str() + ".png", leftImage);
			cvWaitKey(0);
		}
		std::swap(preframe, frame);
		
		
	}
	return 0;
}

//int main(/*int argc, char* argv[]*/) {
//	VideoCapture cap0;
//	cap0.open(0);
//	VideoCapture cap1;
//	cap1.open(1);
//	namedWindow("RightCam", CV_WINDOW_AUTOSIZE);//创建窗口，（名字，默认大小）
//	namedWindow("LeftCam", CV_WINDOW_AUTOSIZE); 
//	Mat  img_1,img_2;
//	int frame_num;
//	frame_num = 0;
//	while (1){
//		cap0.read(img_1);
//		cap1.read(img_2);
//		if (cap1.isOpened()&&cap0.isOpened())
//			break;
//	}
//	while (1){
//		if (cap1.isOpened() && cap0.isOpened()){
//			cap1.read(img_2);
//			cap0.read(img_1);
//			if (img_2.rows == 0||img_1.rows == 0) continue;
//			//break;
//			Rectify(img_1, img_2, img_1, img_2);
//			int _width, _height;
//			_width = img_1.cols;
//			_height = img_1.rows;
//			cv::resize(img_1, img_1, Size(_width / 2, _height / 2));
//			cv::resize(img_2, img_2, Size(_width / 2, _height / 2));
//			
//			cv::imshow("RightCam", img_2);
//			cv::imshow("LeftCam", img_1);
//			
//			
//			SPSStereo sps;
//			sps.setIterationTotal(outerIterationTotal, innerIterationTotal);
//			sps.setWeightParameter(lambda_pos, lambda_depth, lambda_bou, lambda_smo);
//			sps.setInlierThreshold(lambda_d);
//			sps.setPenaltyParameter(lambda_hinge, lambda_occ, lambda_pen);
//
//			cv::Mat segmentImage;
//			cv::Mat disparityImage;
//			std::vector< std::vector<double> > disparityPlaneParameters;
//			std::vector< std::vector<int> > boundaryLabels;
//
//			sps.compute(superpixelTotal, img_1, img_2, segmentImage, disparityImage, disparityPlaneParameters, boundaryLabels);
//			
//			vector<int> dynamicflag(static_cast<int>(disparityPlaneParameters.size()));
//			dynamicflag_size = dynamicflag.size();
//			cv::Mat segmentBoundaryImage;
//			makeSegmentBoundaryImage(img_1, segmentImage, boundaryLabels, segmentBoundaryImage);
//			cv::imshow("segmentBoundaryImage", segmentBoundaryImage);
//			waitKey(3);
//			
//			string saveresult = "result/segment";
//			stringstream ss;
//			ss << setfill('0') << setw(6) << frame_num;
//			imwrite(saveresult + ss.str() + ".png", segmentBoundaryImage);
//			frame_num++;
//		}
//	}
//
//	return 0;
//}

//void makeSegmentBoundaryImage(const cv::Mat & inputImage,
//							  const /*png::image<png::gray_pixel_16>*/ cv::Mat & segmentImage,
//							  std::vector< std::vector<int> >& boundaryLabels,
//							  cv::Mat& segmentBoundaryImage)
//{
//	width = static_cast<int>(inputImage.cols);
//	height = static_cast<int>(inputImage.rows);
//	int boundaryTotal = static_cast<int>(boundaryLabels.size());
//	cv::Mat whiteImage(height, width, CV_8UC3, Scalar(255,255,255));
//	/*imshow("outImage",whiteImage);
//	waitKey(3);*/
//	segmentBoundaryImage.create(height, width, CV_8UC3);
//	for (int y = 0; y < height; ++y) {
//		for (int x = 0; x < width; ++x) {
//			segmentBoundaryImage.at<cv::Vec3b>(y, x) = whiteImage.at<cv::Vec3b>(y, x);
//		}
//	}
//
//	int boundaryWidth = 0;
//	for (int y = 0; y < height - 1; ++y) {
//		for (int x = 0; x < width - 1; ++x) {
//			int pixelLabelIndex = segmentImage.at<uint16_t>(y, x);
//
//			if (segmentImage.at<uint16_t>(y, x + 1) != pixelLabelIndex) {
//				for (int w = 0; w < boundaryWidth - 1; ++w) {
//					if (x - w >= 0) segmentBoundaryImage.at<cv::Vec3b>(y, x - w) = cv::Vec3b(128, 128, 128);
//				}
//				for (int w = 1; w < boundaryWidth; ++w) {
//					if (x + w < width) segmentBoundaryImage.at<cv::Vec3b>(y, x + w) =  cv::Vec3b(128, 128, 128);
//				}
//			}
//			if (segmentImage.at<uint16_t>( y + 1, x) != pixelLabelIndex) {
//				for (int w = 0; w < boundaryWidth - 1; ++w) {
//					if (y - w >= 0) segmentBoundaryImage.at<cv::Vec3b>(y - w, x) =  cv::Vec3b(128, 128, 128);
//				}
//				for (int w = 1; w < boundaryWidth; ++w) {
//					if (y + w < height) segmentBoundaryImage.at<cv::Vec3b>(y + w, x) = cv::Vec3b(128, 128, 128);
//				}
//			}
//		}
//	}
//
//	boundaryWidth = 4;
//	for (int y = 0; y < height - 1; ++y) {
//		for (int x = 0; x < width - 1; ++x) {
//			int pixelLabelIndex = segmentImage.at<uint16_t>(y, x);
//
//			if (segmentImage.at<uint16_t>(y, x + 1) != pixelLabelIndex) {
//				cv::Vec3b negativeSideColor, positiveSideColor;
//				int pixelBoundaryIndex = -1;
//				for (int boundaryIndex = 0; boundaryIndex < boundaryTotal; ++boundaryIndex) {
//					if ((boundaryLabels[boundaryIndex][0] == pixelLabelIndex && boundaryLabels[boundaryIndex][1] == segmentImage.at<uint16_t>(y, x + 1))
//						|| (boundaryLabels[boundaryIndex][0] == segmentImage.at<uint16_t>(y, x + 1) && boundaryLabels[boundaryIndex][1] == pixelLabelIndex))
//					{
//						pixelBoundaryIndex = boundaryIndex;
//						break;
//					}
//				}
//				if (boundaryLabels[pixelBoundaryIndex][2] == 3) continue;
//				else if (boundaryLabels[pixelBoundaryIndex][2] == 2) {
//					negativeSideColor.val[2] = 0;  negativeSideColor.val[1] = 225;  negativeSideColor.val[0] = 0;
//					positiveSideColor.val[2] = 0;  positiveSideColor.val[1] = 225;  positiveSideColor.val[0] = 0;
//				} else if (pixelLabelIndex == boundaryLabels[pixelBoundaryIndex][boundaryLabels[pixelBoundaryIndex][2]]) {
//					negativeSideColor.val[2] = 225;  negativeSideColor.val[1] = 0;  negativeSideColor.val[0] = 0;
//					positiveSideColor.val[2] = 0;  positiveSideColor.val[1] = 0;  positiveSideColor.val[0] = 225;
//				} else {
//					negativeSideColor.val[2] = 0;  negativeSideColor.val[1] = 0;  negativeSideColor.val[0] = 225;
//					positiveSideColor.val[2] = 225;  positiveSideColor.val[1] = 0;  positiveSideColor.val[0] = 0;
//				}
//
//				for (int w = 0; w < boundaryWidth - 1; ++w) {
//					if (x - w >= 0) segmentBoundaryImage.at<cv::Vec3b>(y, x - w) = negativeSideColor;
//				}
//				for (int w = 1; w < boundaryWidth; ++w) {
//					if (x + w < width) segmentBoundaryImage.at<cv::Vec3b>(y, x + w) = positiveSideColor;
//				}
//			}
//			if (segmentImage.at<uint16_t>(y + 1, x) != pixelLabelIndex) {
//                cv::Vec3b negativeSideColor, positiveSideColor;
//				int pixelBoundaryIndex = -1;
//				for (int boundaryIndex = 0; boundaryIndex < boundaryTotal; ++boundaryIndex) {
//					if ((boundaryLabels[boundaryIndex][0] == pixelLabelIndex && boundaryLabels[boundaryIndex][1] == segmentImage.at<uint16_t>(y + 1, x))
//						|| (boundaryLabels[boundaryIndex][0] == segmentImage.at<uint16_t>(y + 1, x) && boundaryLabels[boundaryIndex][1] == pixelLabelIndex))
//					{
//						pixelBoundaryIndex = boundaryIndex;
//						break;
//					}
//				}
//				if (boundaryLabels[pixelBoundaryIndex][2] == 3) continue;
//				else if (boundaryLabels[pixelBoundaryIndex][2] == 2) {
//					negativeSideColor.val[2] = 0;  negativeSideColor.val[1] = 225;  negativeSideColor.val[0] = 0;
//					positiveSideColor.val[2] = 0;  positiveSideColor.val[1] = 225;  positiveSideColor.val[0] = 0;
//				} else if (pixelLabelIndex == boundaryLabels[pixelBoundaryIndex][boundaryLabels[pixelBoundaryIndex][2]]) {
//					negativeSideColor.val[2] = 225;  negativeSideColor.val[1] = 0;  negativeSideColor.val[0] = 0;
//					positiveSideColor.val[2] = 0;  positiveSideColor.val[1] = 0;  positiveSideColor.val[0] = 225;
//				} else {
//					negativeSideColor.val[2] = 0;  negativeSideColor.val[1] = 0;  negativeSideColor.val[0] = 225;
//					positiveSideColor.val[2] = 225;  positiveSideColor.val[1] = 0;  positiveSideColor.val[0] = 0;
//				}
//
//				for (int w = 0; w < boundaryWidth - 1; ++w) {
//					if (y - w >= 0) segmentBoundaryImage.at<cv::Vec3b>(y - w, x) = negativeSideColor;
//				}
//				for (int w = 1; w < boundaryWidth; ++w) {
//					if (y+ w < height) segmentBoundaryImage.at<cv::Vec3b>(y + w, x) = positiveSideColor;
//				}
//			}
//		}
//	}
//}

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
	for (int y = 3; y < height - 3; ++y) {
		for (int x =3; x < width - 3; ++x) {
			
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

void writeDisparityPlaneFile(const std::vector< std::vector<double> >& disparityPlaneParameters, const std::string outputDisparityPlaneFilename) {
	std::ofstream outputFileStream(outputDisparityPlaneFilename.c_str(), std::ios_base::out);
	if (outputFileStream.fail()) {
		std::cerr << "error: can't open file (" << outputDisparityPlaneFilename << ")" << std::endl;
		exit(0);
	}

	int segmentTotal = static_cast<int>(disparityPlaneParameters.size());
	for (int segmentIndex = 0; segmentIndex < segmentTotal; ++segmentIndex) {
		outputFileStream << disparityPlaneParameters[segmentIndex][0] << " ";
		outputFileStream << disparityPlaneParameters[segmentIndex][1] << " ";
		outputFileStream << disparityPlaneParameters[segmentIndex][2] << std::endl;
	}

	outputFileStream.close();
}

void writeBoundaryLabelFile(const std::vector< std::vector<int> >& boundaryLabels, const std::string outputBoundaryLabelFilename) {
	std::ofstream outputFileStream(outputBoundaryLabelFilename.c_str(), std::ios_base::out);
	if (outputFileStream.fail()) {
		std::cerr << "error: can't open output file (" << outputBoundaryLabelFilename << ")" << std::endl;
		exit(1);
	}

	int boundaryTotal = static_cast<int>(boundaryLabels.size());
	for (int boundaryIndex = 0; boundaryIndex < boundaryTotal; ++boundaryIndex) {
		outputFileStream << boundaryLabels[boundaryIndex][0] << " ";
		outputFileStream << boundaryLabels[boundaryIndex][1] << " ";
		outputFileStream << boundaryLabels[boundaryIndex][2] << std::endl;
	}
	outputFileStream.close();
}

void drawDynamicObject(const cv::Mat & inputImage, const cv::Mat & segmentImage,
	std::vector< std::vector<int> >& boundaryLabels, vector<Point2f> Point, cv::Mat& DynamicObjectMask){
	//width = static_cast<int>(inputImage.cols);
	//int height = static_cast<int>(inputImage.rows);
	int boundaryTotal = static_cast<int>(boundaryLabels.size());
	//vector<int> DynaFlag(dynamicflag_size);
	vector<int> dynamicSegement;
	cv::Vec3b MaskColor;
	DynamicObjectMask.create(height, width, CV_8UC3);
	DynamicObjectMask = inputImage;
	/*for (int y = 0; y < height; ++y) {
	 	for (int x = 0; x < width; ++x) {
			DynamicObjectMask.at<cv::Vec3b>(y, x) = inputImage.at<cv::Vec3b>(y, x);
		}
	}*/
	MaskColor.val[2] = 255; MaskColor.val[1] =255; MaskColor.val[0] = 0;
	int Point_size = Point.size();
	int dynamicSeg;
	int pixelLabelIndex;
	for (int i = 0; i < Point_size; i++){
		dynamicSeg = segmentImage.at<uint16_t>(Point[i].y, Point[i].x);
		dynamicSegement.push_back(dynamicSeg);
		for (int y = 0; y < height - 1; ++y) {
			for (int x = 0; x < width - 1; ++x) {
				pixelLabelIndex = segmentImage.at<uint16_t>(y, x);
				
				if (pixelLabelIndex == dynamicSeg&&isBoundary(x,y,segmentImage)==0){
					
					DynamicObjectMask.at<cv::Vec3b>(y, x) = MaskColor;
				}
			}
		}
	}
	//扩充动态区域
	/*for (int i = 0; i < Point_size; i++){
		
	}*/
	cv::imshow("Mask", DynamicObjectMask);
	cv::waitKey(3);
	
}

bool isBoundary(const int x, const int y, const cv::Mat & segmentImage){
	const int _fourNeighborTotal = 4;
	const int _fourNeighborOffsetX[4] = { -1, 0, 1, 0 };
	const int _fourNeighborOffsetY[4] = { 0, -1, 0, 1 };
	int width_, height_;
	width_ = width;
	height_ = height;
	int pixelSegmentIndex = segmentImage.at<uint16_t>(y, x);
	for (int neighborIndex = 0; neighborIndex < _fourNeighborTotal; ++neighborIndex) {
		int neighborX = x + _fourNeighborOffsetX[neighborIndex];
		if (neighborX < 0 || neighborX >= width_) continue;
		int neighborY = y + _fourNeighborOffsetY[neighborIndex];
		if (neighborY < 0 || neighborY >= height_) continue;

		if (segmentImage.at<uint16_t>(y, x) != pixelSegmentIndex) return true;
	}

	return false;
}

//Mat drawDynamicObject(const cv::Mat & inputImage, const cv::Mat  &leftImage,
//	std::vector< std::vector<int> >& boundaryLabels, vector<Point2f> DynamicPoint, cv::Mat& DynamicObjectMask){
//	//width = static_cast<int>(inputImage.cols);
//	//int height = static_cast<int>(inputImage.rows);
//	int boundaryTotal = static_cast<int>(boundaryLabels.size());
//	//vector<int> DynaFlag(dynamicflag_size);
//	vector<int> dynamicSegement;
//	cv::Vec3b MaskColor;
//	DynamicObjectMask.create(height, width, CV_8UC1);
//	DynamicObjectMask = inputImage;
//	Mat threshold_output, Closed_result;
//	threshold_output.create(height, width, CV_8UC1);
//	vector<vector<cv::Point>>contours;   //轮廓数组（非矩形数组），每个轮廓是一个Point型的vector
//	vector<Vec4i>hierarchy;                //见下面findContours的解释
//	int thresh = 200;
//	RNG rng(12345);
//	///使用Threshold二值
//	threshold(DynamicObjectMask, threshold_output, thresh, 255, THRESH_BINARY);
//
//	imshow("threshold_output", threshold_output);
//	waitKey(3);
//	cout << "threshold_output :" << threshold_output.size() << endl;
//	/// 找到轮廓
//	//contours参数为检测的轮廓数组，每一个轮廓用一个point类型的vector表示
//	//hiararchy参数和轮廓个数相同，每个轮廓contours[ i ]对应4个hierarchy元素hierarchy[ i][ 0 ] ~hierarchy[ i ][ 3 ]，
//	//分别表示后一个轮廓、前一个轮廓、父轮廓、内嵌轮廓的索引编号，如果没有对应项，该值设置为负数。
//	//CV_RETR_TREE：建立一个等级树结构的轮廓
//	//
//	findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
//	vector<vector<cv::Point>> contours_poly(contours.size());         //近似后的轮廓点集
//	vector<Rect>boundRect(contours.size());                          //包围点集的最小矩形vector
//	vector<Point2f>center(contours.size());                              //包围点集的最小圆形vector
//	vector<float>radius(contours.size());                                  //包围点集的最小圆形半径vector
//	cout << "contours.size()" << contours.size() << endl;
//	for (int i = 0; i< contours.size(); i++)
//	{
//		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);     //对多边形曲线做适当近似，contours_poly[i]是输出的近似点集
//		boundRect[i] = boundingRect(Mat(contours_poly[i]));        //计算并返回包围轮廓点集的最小矩形
//		minEnclosingCircle(contours_poly[i], center[i], radius[i]);    //计算并返回包围轮廓点集的最小圆形及其半径
//	}
//	vector<bool> dynamic_flag(DynamicPoint.size());
//	/// 画多边形轮廓 + 包围的矩形框 + 圆形框
//	Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
//	drawing = leftImage;
//
//	Mat drawing2 = Mat::zeros(threshold_output.size(), CV_8UC3);
//	//drawing2 = leftImage;
//	for (int i = 0; i<contours.size(); i++)
//	{
//		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));  //随机颜色
//		//if (!ContourIsDynamicMask(contours[i], DynamicPoint, dynamic_flag)) continue;
//		drawContours(drawing2, contours_poly, i, color, 2, 8, hierarchy, 0, cv::Point());        //根据轮廓点集contours_poly和轮廓结构hierarchy画出轮廓
//
//	}
//	imshow("drawing2", drawing2);
//	waitKey(3);
//	for (int i = 0; i<contours.size(); i++)
//	{
//		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));  //随机颜色
//		if (!ContourIsDynamicMask(contours[i], DynamicPoint, dynamic_flag)) continue;
//		cout << "contours[i].size: " << contours[i].size() << endl;
//		drawContours(drawing, contours_poly, i, color, 2, 8, hierarchy, 0, cv::Point());        //根据轮廓点集contours_poly和轮廓结构hierarchy画出轮廓
//
//	}
//
//	return drawing;
//}


//bool ContourIsDynamicMask(vector<cv::Point> contour, vector<cv::Point2f> dynamic, vector<bool> dynamic_flag) {
//	int DynamicPoint_size = dynamic.size();
//	int contour_size = contour.size();
//	int dyna_num;
//	dyna_num = 0;
//	for (int i = 0; i < DynamicPoint_size; i++){
//		bool measureDist;
//		measureDist = false;
//		double dynaflag;
//		dynaflag = pointPolygonTest(contour, dynamic[i], measureDist);
//		if ((dynaflag == 1 || dynaflag == 0)) dyna_num++;
//	}
//	if (dyna_num>2/*&& contour_size / dyna_num>50*/&& contour_size>100) return true;
//	else return false;
//}

bool ContourIsDynamicMask(vector<cv::Point> contour, vector<cv::Point2f> dynamic, vector<bool> dynamic_flag) {
	int DynamicPoint_size = dynamic.size();
	
	int dyna_num;
	dyna_num = 0;
	for (int i = 0; i < DynamicPoint_size; i++){
		bool measureDist;
		measureDist = false;
		double dynaflag;
		dynaflag = pointPolygonTest(contour, dynamic[i], measureDist);
		if ((dynaflag == 1 || dynaflag == 0)) dyna_num++;
	}
	if (dyna_num>2) return true;
	else return false;
}
void Rectify(cv::Mat im_l, cv::Mat im_r, cv::Mat& imLeft, cv::Mat& imRight)
{
	// Load settings related to stereo calibration
	cv::FileStorage fsSettings("EuRoC.yaml", cv::FileStorage::READ);
	if (!fsSettings.isOpened())
	{
		cerr << "ERROR: Wrong path to settings" << endl;
		return;
	}

	cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
	cv::Size imageSize, newImageSize_l, newImageSize_r;
	fsSettings["LEFT.K"] >> K_l;
	fsSettings["RIGHT.K"] >> K_r;

	fsSettings["LEFT.P"] >> P_l;
	fsSettings["RIGHT.P"] >> P_r;

	fsSettings["LEFT.R"] >> R_l;
	fsSettings["RIGHT.R"] >> R_r;

	fsSettings["LEFT.D"] >> D_l;
	fsSettings["RIGHT.D"] >> D_r;

	int rows_l = fsSettings["LEFT.height"];
	int cols_l = fsSettings["LEFT.width"];
	int rows_r = fsSettings["RIGHT.height"];
	int cols_r = fsSettings["RIGHT.width"];

	imageSize.height = rows_l;
	imageSize.width = cols_l;

	if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
		rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
	{
		cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
		return;
	}
	cv::Mat M1l, M2l, M1r, M2r;
	cv::fisheye::initUndistortRectifyMap(K_l, D_l, R_l, P_l, imageSize, CV_32F, M1l, M2l);
	cv::fisheye::initUndistortRectifyMap(K_r, D_r, R_r, P_r, imageSize, CV_32F, M1r, M2r);
	cv::remap(im_l, imLeft, M1l, M2l, cv::INTER_LINEAR);
	cv::remap(im_r, imRight, M1r, M2r, cv::INTER_LINEAR);
}

Mat otsuGray(const Mat src) {
	
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
			T = i+40;
		}
	}
	cout << "threhold = " <<T << endl;
	for (int i = 0; i<img.rows; i++)
	{
		for (int j = 0; j<img.cols; j++)
		{
			if ((int)img.at<uchar>(i, j)>T || (int)img.at<uchar>(i, j) == 0)
				img.at<uchar>(i, j) = 0;
			else
				img.at<uchar>(i, j) = 255;
		}
	}
	return img;
}

vector<Rect> draw_dynamic_object(Mat segment, std::vector<Point2f> dynamicpoint, Mat leftimage,Mat segmentimage){
	vector<Rect> result;
	vector<vector<cv::Point>> result_contours;
	Mat dynamicmask;
	Point2f sorted_point;
	Point2f padding = Point2f(0.0, -1.0);
	
	//cvtColor(segment, segment,CV_BGR2GRAY);
	threshold(segment, segment, 200, 255, CV_THRESH_BINARY);
	Mat erode_element;
	erode_element = getStructuringElement(MORPH_RECT, Size(2, 2));
	erode(segment, segment, erode_element);
	
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
	//
	//imshow("leftimage_point", leftimage);
	//waitKey(1);

	//imshow("segment", segment);
	//waitKey(1);

	vector<vector<cv::Point>>contours;   //轮廓数组（非矩形数组），每个轮廓是一个Point型的vector
	vector<Vec4i>hierarchy;                //见下面findContours的解释
	findContours(segment, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	


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
		Point bottom_point, left_point, right_point,top_point;
		//cout << "border size = " << _border.size() << endl;
		for (int j = 0; j < _border.size();j++){
			if (abs(pointPolygonTest(contours[i], _border[j], true))<=2.0){
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
		if (dyna_flag>2){
			vector<vector<cv::Point>> contours_poly(contours.size());         //近似后的轮廓点集
			vector<Rect>boundRect(contours.size());                          //包围点集的最小矩形vector
			vector<Point2f>center(contours.size());                              //包围点集的最小圆形vector
			vector<float>radius(contours.size());                                  //包围点集的最小圆形半径vector

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
					
					if (Rect_multi.width<150 || Rect_multi.width>10){
						//cout << "rect_factor = " << rect_factor << endl;
						//rectangle(drawing, Rect_multi.tl(), Rect_multi.br(), color, 2, 8, 0);
						result.push_back(Rect_multi);
					}
					dynamic_point_vector.push_back(multi_point);
					
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

Point2f sort_point(vector<Point2f>dynamicpoint){
	vector<Point2f> sorted;
	cout << "start sorted" << endl;
	int dynamic_size = dynamicpoint.size();
	Point2f tmp1 = dynamicpoint[0];
	
	for (int i = 1; i < dynamic_size;i++){
		if (dynamicpoint[i].x)
		if (dynamicpoint[i].y>tmp1.y) tmp1 = dynamicpoint[i];
	}
	return tmp1;
}

Mat draw_mask(vector<Rect> dynamic_rect, Mat leftImage, Mat segment, vector<Point2f> point, std::vector< std::vector<int>> boundarylabel){
	Mat dynamic_mask;
	dynamic_mask = leftImage;
	cv::Vec3b MaskColor;
	MaskColor.val[2] = 255; MaskColor.val[1] = 255; MaskColor.val[0] = 0;
	for (int i = 0; i < dynamic_rect.size();i++){
		int start_x, end_x;
		int start_y, end_y;
		start_x = dynamic_rect[i].tl().x;
		end_x = start_x + dynamic_rect[i].width;
		start_y = dynamic_rect[i].tl().y;
		end_y = start_y + dynamic_rect[i].height;
		int point_size = dynamic_point_vector[i].size();
		int pixelLabelIndex, dynamicSeg;
		//vector<int> dynamic_index;
		for (int j = 0; j < point_size;j++){
			dynamicSeg = segment.at<uint16_t>(dynamic_point_vector[i][j].y, dynamic_point_vector[i][j].x);
			for (int m = start_x; m < end_x; m++){
				for (int n = start_y; n < end_y;n++){
					pixelLabelIndex = segment.at<uint16_t>(n, m);
					if (pixelLabelIndex == dynamicSeg/*&&isBoundary(m, n, segment) == 0*/){
						dynamic_mask.at<cv::Vec3b>(n, m) = MaskColor;
						//dynamic_index.push_back(dynamicSeg);
					}
			
				}
			}
		}
	}


	return dynamic_mask;
}