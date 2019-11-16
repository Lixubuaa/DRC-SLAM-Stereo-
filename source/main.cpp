/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Ra��l Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "stdafx.h"
#include <iostream>
#include <algorithm>
#include <fstream>
#include <opencv2/core/core.hpp>
#include "System.h"
#include <Windows.h>
#include <cstring>
#include <direct.h>
#include <strstream>

#include "generalDataFuction.h"
//#include "GPS_GlobalDataFuction.h"
#include "InitialParameter.h"

using namespace std;
vector<cv::Point3f> dynamic_center_vector;
ORB_SLAM2::InitialParameter InitialRot;
void Rotate(cv::Mat& im_l, cv::Mat& im_r);
void Rectify(cv::Mat im_l, cv::Mat im_r, cv::Mat& imLeft, cv::Mat& imRight);
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
	vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps, vector<cv::Mat> &vZwc);
string destination;
int iMode;
char* ComNo;

extern int ChoseSchoolDestination(char buf[100]);
extern int destiChose;
int thres;
float thre_dist2epipolar;
bool eliminate_dynamic;
string load_path;
int main(int argc, char *argv[])
{
	iMode = 321;
	
	
	//�����洢�ļ���
	SYSTEMTIME time = { 0 };
	GetLocalTime(&time);
	int serial = 0;
	do
	{
		serial++;
		data = "./DATA_" + to_string(time.wMonth) + to_string(time.wDay) + "_" + to_string(serial);
		mydataset = "./mydataset_" + to_string(time.wMonth) + to_string(time.wDay) + "_" + to_string(serial);
	} while (_mkdir(data.c_str()) != 0 || _mkdir(mydataset.c_str()) != 0); //�����ɹ�����ֵΪ0
	data_dir = { data + "/Left", data + "/Right", data + "/Disparity", data + "/Showimage", data + "/Road", data + "/Rightpoint", data + "/Line", data + "/training" ,data+"/Frame"};
	mydataset_dir = { mydataset + "/image_0", mydataset + "/image_1" };
	for (int i = 0; i < data_dir.size(); i++)
	{
		_mkdir(data_dir[i].c_str());
		if (i < mydataset_dir.size())
			_mkdir(mydataset_dir[i].c_str());
	}

	string strPathTimeFile = mydataset + "/times.txt";
	string strPathPoseFile = mydataset + "/poses.txt";
	string strPathGPSFile = mydataset + "/GPS.txt";
	string strPathIMUFile = mydataset + "/IMU.txt";
	string strPathCOVFile = mydataset + "/COV.txt";
	string strPathFixtypeFile = mydataset + "/Fixtype.txt";
	string strPrefixLeft = mydataset_dir[0] + "/";
	string strPrefixRight = mydataset_dir[1] + "/";
	ofstream fTimes, fGPS, fIMU, fPoses, fRawSensor, fCOV, fFixtype;
	fTimes.open(strPathTimeFile.c_str(), ios::app);
	fPoses.open(strPathPoseFile.c_str(), ios::app);
	fGPS.open(strPathGPSFile.c_str(), ios::app);
	fIMU.open(strPathIMUFile.c_str(), ios::app);
	fCOV.open(strPathCOVFile.c_str(), ios::app);
	fFixtype.open(strPathFixtypeFile.c_str(), ios::app);

	

	cout << "please input threshold:";
	cin >> thres;
	/*cout << "please input threshold for MOD:";
	cin >> thre_dist2epipolar;*/
	cout << " please input image path";
	cin >> load_path;
	cout << "Choose eliminate(1) of not eliminate(0)";
	cin >> eliminate_dynamic;
	////Choose Mode
	//int iMode;
	//cout << "Choose Mode(1-8):";
	//cin >> iMode;

	//if (iMode == 2)
	//{   //�����յ�
	//	cout << "Please Input Your Destination: ";
	//	cin >> destination;
	//}

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	string strSettingsFile = "Settings.yaml";
	string strVocFile = "ORBvoc.bin";
	ORB_SLAM2::System sys(strVocFile, strSettingsFile, ORB_SLAM2::System::STEREO, true, iMode);

	//Main loop
	/*cv::VideoCapture Camera(0);  
	cv::VideoCapture Camera_r(1); 
	if (!Camera.isOpened() || !Camera_r.isOpened())
	{
		cout << "Camera opened error!" << endl;
		cvWaitKey();
	}*/
	string dir_path = "E:/dataset/dynamic_data/" + load_path+"/";
	string left_dir_path = dir_path + "image_0/";
	string right_dir_path = dir_path + "image_1/";
	//string left_dir_path = "E:/dataset/object_tracking/tracking_dataset/data_tracking_image_2/training/image_02/0019/";
	//string right_dir_path = "E:/dataset/object_tracking/tracking_dataset/data_tracking_image_3/training/image_03/0019/";
	Directory left_dir;

	Directory right_dir;
	vector<string> left_fileNames = left_dir.GetListFiles(left_dir_path, "*.png", false);
	vector<string> right_fileNames = left_dir.GetListFiles(right_dir_path, "*.png", false);
	
	int n = 0;
	int k = 0;
	cv::Point3d IniLatLonHeight;
	cv::Mat Zcw = cv::Mat(4, 4, CV_32F);
	
	for (int i = 0; i < left_fileNames.size() - 1; i++)	{
		//while (1)
		
			string fileName = left_fileNames[i];
			string left_fileFullName = left_dir_path + fileName;
			//std::cout << "file name:" << fileName << endl;
			//std::cout << "file path:" << left_fileFullName << endl;
			//MOD
			Mat im_l = imread(left_fileFullName);
			//cout << " image size" << leftImage.size() << endl;
			string right_fileName = right_fileNames[i];
			string right_fileFullName = right_dir_path + right_fileName;
			cv::Mat im_r = cv::imread(right_fileFullName);

		/*	string fileName_pre = left_fileNames[i - 1];
			string fileFullName_pre = left_dir_path + fileName_pre;
			Mat preframe = imread(fileFullName_pre, 0);*/
			//��ʱ
			LARGE_INTEGER litmp;
			double qt, qf;
			double tframe;

			//cv::Mat im_l, im_r, imLeft, imRight;
			cv::Point3d LatLonHeight;
			cv::Mat Rcw = cv::Mat(3, 3, CV_32F);
			cv::Mat tcw = cv::Mat(3, 1, CV_32F);
			cv::Mat Cov = cv::Mat(6, 1, CV_32F);
			unsigned int Fixtype;

			//��ȡͼ��
			//Camera >> im_l;
			//Camera_r >> im_r;
			if (im_l.empty() || im_r.empty()/* || i++ < 100*/)
			{
				cout << "None image got!!!" << endl;
				continue;
			}

			//��ȡ�ߵ�����
			//GPSIMU CurrentData = sys.mpReader->GetGPSIMU();
			//if (!initflag)
			//{
			//	IniLatLonHeight.x = CurrentData.latitude;
			//	IniLatLonHeight.y = CurrentData.longitude;
			//	IniLatLonHeight.z = CurrentData.height;
			//	Euler2Rotmateix(CurrentData.yaw, CurrentData.roll, CurrentData.pitch, IniR);
			//	InitialRot.SetInitialRot(IniR);
			//	Fixtype = CurrentData.fixtype;
			//	Cov.at<float>(0) = CurrentData.position_standard_deviation[0];
			//	Cov.at<float>(1) = CurrentData.position_standard_deviation[1];
			//	Cov.at<float>(2) = CurrentData.position_standard_deviation[2];
			//	Cov.at<float>(3) = CurrentData.euler_orientation_standard_deviation[0];
			//	Cov.at<float>(4) = CurrentData.euler_orientation_standard_deviation[1];
			//	Cov.at<float>(5) = CurrentData.euler_orientation_standard_deviation[2];
			//	//cv::Mat trans = cv::Mat::eye(6, 6, CV_32F);
			//	//trans.rowRange(0, 3).colRange(0, 3) = IniR;
			//	//Cov = trans*Cov;
			//	Zcw = cv::Mat::eye(4, 4, CV_32F);
			//	fGPS << setprecision(15) << CurrentData.latitude << "," << CurrentData.longitude << "," << CurrentData.height << endl;;//γ�ȡ����ȡ��߶�
			//	fIMU << setprecision(6) << CurrentData.roll << "," << CurrentData.pitch << "," << CurrentData.yaw << endl; //����(roll)������(pitch)��ƫ��(yaw),������
			//	fCOV << setprecision(6) << Cov.at<float>(0) << "," << Cov.at<float>(1) << "," << Cov.at<float>(2) << "," << Cov.at<float>(3) << "," <<
			//		Cov.at<float>(4) << "," << Cov.at<float>(5) << endl;
			//	fFixtype << CurrentData.fixtype << endl;
			//	initflag = true;
			//}
			//else
			//{
			//	Fixtype = CurrentData.fixtype;
			//	LatLonHeight.x = CurrentData.latitude;
			//	LatLonHeight.y = CurrentData.longitude;
			//	LatLonHeight.z = CurrentData.height;
			//	cv::Mat R = cv::Mat(3, 3, CV_32F);
			//	Euler2Rotmateix(CurrentData.yaw, CurrentData.roll, CurrentData.pitch, R);
			//	Cov.at<float>(0) = CurrentData.position_standard_deviation[0];
			//	Cov.at<float>(1) = CurrentData.position_standard_deviation[1];
			//	Cov.at<float>(2) = CurrentData.position_standard_deviation[2];
			//	Cov.at<float>(3) = CurrentData.euler_orientation_standard_deviation[0];
			//	Cov.at<float>(4) = CurrentData.euler_orientation_standard_deviation[1];
			//	Cov.at<float>(5) = CurrentData.euler_orientation_standard_deviation[2];
			//	//cv::Mat trans = cv::Mat::eye(6, 6, CV_32F);
			//	//trans.rowRange(0, 3).colRange(0, 3) = IniR;
			//	//Cov = trans*Cov;
			//	Rcw = R*IniR.t();
			//	tcw = ConvertLatLon2RT(IniLatLonHeight, LatLonHeight, R);
			//	Rcw.copyTo(Zcw.rowRange(0, 3).colRange(0, 3));
			//	tcw.copyTo(Zcw.rowRange(0, 3).col(3));
			//	fGPS << setprecision(15) << CurrentData.latitude << "," << CurrentData.longitude<< "," << CurrentData.height << endl; //γ�ȡ����ȡ��߶�
			//	fIMU << setprecision(6) << CurrentData.roll << "," << CurrentData.pitch << "," << CurrentData.yaw << endl; //����(roll)������(pitch)��ƫ��(yaw),������
			//	fCOV << setprecision(6) << Cov.at<float>(0) << "," << Cov.at<float>(1) << "," << Cov.at<float>(2) << "," << Cov.at<float>(3) << "," <<
			//		Cov.at<float>(4) << "," << Cov.at<float>(5) << endl;
			//	fFixtype << CurrentData.fixtype << endl;
			//}

			//Rectify(im_l, im_r, imLeft, imRight);

			QueryPerformanceFrequency(&litmp);//���ʱ��Ƶ��
			qf = (double)litmp.QuadPart;
			QueryPerformanceCounter(&litmp);//��ó�ʼֵ
			qt = (double)litmp.QuadPart;
			tframe = qt / qf;     //����timestamp��for win

			//��������
			fTimes << setprecision(15) << tframe << endl;
			fPoses << setprecision(6) << Zcw.at<float>(0, 0) << " " << Zcw.at<float>(0, 1) << " " << Zcw.at<float>(0, 2) << " " << Zcw.at<float>(0, 3) << " " <<
				Zcw.at<float>(1, 0) << " " << Zcw.at<float>(1, 1) << " " << Zcw.at<float>(1, 2) << " " << Zcw.at<float>(1, 3) << " " <<
				Zcw.at<float>(2, 0) << " " << Zcw.at<float>(2, 1) << " " << Zcw.at<float>(2, 2) << " " << Zcw.at<float>(2, 3) << endl;
			stringstream ss;
			ss << setfill('0') << setw(6) << n;
			cv::imwrite(strPrefixLeft + ss.str() + ".png", im_l);
			cv::imwrite(strPrefixRight + ss.str() + ".png", im_r);
			double start, end;
			start = (double)clock() / CLOCKS_PER_SEC;
			// Pass the images to the SLAM system
			sys.TrackStereo(im_l, im_r, tframe, Zcw, Cov, Fixtype);
			end = (double)clock() / CLOCKS_PER_SEC;
			cout << "one frame cost = " << end - start << " s " << endl;
			//Mat imLeft_show;
			//resize(im_r, imLeft_show, Size(320, 240));	//����
			//imshow("left", imLeft_show);
			//HWND hwnd_imleft = FindWindow(NULL, "left");
			//SetWindowPos(hwnd_imleft, HWND_NOTOPMOST, 1280, 661, 320, 240, SWP_ASYNCWINDOWPOS | SWP_DEFERERASE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE);
			//cv::waitKey(3);

			//k = cv::waitKey(30);
			//if (k == 13)
			//	break;
			n++;
			cout << "process frame = " << n << endl;
			if (n == left_fileNames.size() - 2)
				cout << "process end" << endl;
			string dyna_flag;
			if (eliminate_dynamic){
				dyna_flag = "ti";
			}
			else
				dyna_flag = "buti";
			
			sys.SaveTrajectoryKITTI(data + "/" + dyna_flag + " CameraTrajectory.txt");
	}
	
	sys.Shutdown();
	
	return 0;
}

void Rotate(cv::Mat& im_l, cv::Mat& im_r)
{
	cv::transpose(im_l, im_l);
	cv::flip(im_l, im_l, 1);
	cv::transpose(im_r, im_r);
	cv::flip(im_r, im_r, 0);
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

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
	vector<string> &vstrImageRight, vector<double> &vTimestamps, vector<cv::Mat> &vZwc)
{
	ofstream FilePoses("FilePose.txt", ios::out);
	ifstream fTimes;
	ifstream fPoses;
	string strPathTimeFile = strPathToSequence + "/times.txt";
	string strPathPoseFile = strPathToSequence + "/poses.txt";
	fTimes.open(strPathTimeFile.c_str());
	fPoses.open(strPathPoseFile.c_str());
	while (!fTimes.eof())
	{
		string s;
		getline(fTimes, s);
		if (!s.empty())
		{
			stringstream ss;
			ss << s;
			double t;
			ss >> t;
			vTimestamps.push_back(t);
		}
	}
	while (!fPoses.eof())
	{
		cv::Mat Zwc = cv::Mat::eye(4, 4, CV_32F);
		string s;
		getline(fPoses, s);
		if (!s.empty())
		{
			int n = 0;
			stringstream ss;
			for (int i = 0; i < s.size(); i++)
			{
				
				
				if (s[i] == ' ')
				{
					ss >> Zwc.at<float>(n++);
					ss.clear();
				}
				else
					ss << s[i];
			}
			ss >> Zwc.at<float>(n);
		}
		vZwc.push_back(Zwc);
		FilePoses << "Zwc:" << Zwc << endl;
	}
	string strPrefixLeft = strPathToSequence + "/image_0/";
	string strPrefixRight = strPathToSequence + "/image_1/";

	const int nTimes = vTimestamps.size();
	vstrImageLeft.resize(nTimes);
	vstrImageRight.resize(nTimes);

	for (int i = 0; i<nTimes; i++)
	{
		stringstream ss;
		ss << setfill('0') << setw(6) << i;
		vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
		vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
	}
}