#include "stdafx.h"
#include "ObstacleReconstruction.h"
#include "generalDataFuction.h"
#include "obstacle_detection.h"
#include "path_planning.h"
#include "walkingcontrol.h"
#include "GPS_GlobalDataFuction.h"
#include "baidumapGPSpath.h"
#include "Converter.h"
#include "roadline_tracking.h"

extern string destination;
extern int thres;

//����㡢���T���ϰ��ä����Ҫ��mmΪ��λ
namespace ORB_SLAM2{
	ObstacleReconstruction::ObstacleReconstruction(ORB_SLAM2::Map* World) :
		sWorld(World), mYaw(-90), mCurrentGPSXY({ 0, 0 }), mCurrentPathplanXY({ 0, 0 }), Originbaidu({ 0, 0 }),
		mbSLAMWork(false), mbGPSWork(false), ReconstructInitflag(false),
		g_obstacle(0), g_blindwaypar(0), mvBlindwaypar(0), mvObstacle(0), Dynamic_Obstacle(0),
		FramesNumLast(0), FramesNum(0), mbStopped(false), mbStopRequested(false)

	{
		mDeltaXYZ = Mat::zeros(3, 1, CV_32F);
		mSLAMXYZ = Mat::zeros(3, 1, CV_32F);
		R_cg = Mat::zeros(3, 3, CV_32F);
		T_cg = Mat::zeros(3, 1, CV_32F);
		R_wg = Mat::zeros(3, 3, CV_32F);
		T_wg = Mat::zeros(3, 1, CV_32F);
		X_i.setZero();
		P_i << 2, 0, 0,
			0, 2, 0,
			0, 0, 2;
		Q << 0.01, 0, 0,
			0, 0.01, 0,
			0, 0, 0.01;
	}
	
	void ObstacleReconstruction::Run()
	{	
		//GPS ״̬����
		const char *GPSState[8] = { "0 No GNSS Fix ����λ����...", "1 2D GNSS Fix ����λ����...", "2 3D GNSS Fix ����λ����...", "3 SBAS GNSS Fix ����λ����...", "4 Differential GNSS Fix ����λ����...", "5 Omnistar GNSS Fix ����λ����...", "6 RTK Float GNSS Fix ����λ����...", "7 RTK Fixed GNSS Fix ����λ����..." };

		//�������߿���ģ���ಢ����д�߳�
		wk_control wk_control;

		fstream datafile, timefile;
		datafile.open(data + "/ReconstructionData.txt", ios::app);   //����һ���ļ�
		timefile.open(data + "/ReconstructionTime.txt", ios::app);

		while (1){
			if (Stop())
			{
				while (isStopped())
					Sleep(3);
			}
			//ÿ�ζ���Ҫ���³�ʼ���Ĳ���
			int flagBarrier[ROOM][ROOM] = { 0 };
			Point3f blindwayPar = { 0, 0, 0 };

			double start_time, end_time;
			start_time = (double)clock() / CLOCKS_PER_SEC;
			double t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10;
			int DetectFramesNum = 0;
			int savenum = 0;
			
			GetCurrentCameraPose();

			vector<ORB_SLAM2::KeyFrame*> KFs = sWorld->GetAllKeyFrames();
			sort(KFs.begin(), KFs.end(), ORB_SLAM2::KeyFrame::lId);
			FramesNum = KFs.size();
			for (int i = 0; i < KFs.size(); i++){
				if (KFs[i]->GetObstacle().size()>0 || KFs[i]->GetBlindwaypar().size()>1)
					DetectFramesNum++;
				if (KFs[i]->GetSaveNum() > savenum)
					savenum = KFs[i]->GetSaveNum();
			}

			t0 = (double)clock() / CLOCKS_PER_SEC;			

			//�ж�SLAM��GPS״̬����ȡ����
			GetSlamGpsWorkStatusAndData(KFs);

			//��GPS��SLAM������
			if (!mbSLAMWork && !mbGPSWork)
			{
				cout << "��GPS�ź���SLAM���ݲ�����!!!����..." << endl;
				continue;
			}

			t1 = (double)clock() / CLOCKS_PER_SEC;

			//�Ƿ���Ƶı�־
			bool controlIf = false;
			//��SLAM�����Ҹ��¹ؼ�֡ʱ����
			if (mbSLAMWork){
				t2 = (double)clock() / CLOCKS_PER_SEC;

				datafile << "SaveNum = " << savenum << endl;
				datafile << "FramesNum = " << FramesNum << endl;
				
				if (!sWorld->GetWPlane().empty())
					WPlane = sWorld->GetWPlane();
				if (WPlane.empty())
				{
					//cout << "WPlane is empty!" << endl;
					continue;
				}

				//������������ϵ����������ϵ��RT�����Ƶ�λ
				CalculateWorld2GroundRT();
				
				//Dynamic_Obstacle.clear();
				//ֻȡ15֡���㣬����ϰ���㣬SLAM��Ϊ����
				int startFrame = (KFs.size() > 5) ? (KFs.size() - 6) : 0;
				CalculateObstacleWorldPoints(KFs, startFrame, mvObstacle, Dynamic_Obstacle);
				
				//ֻȡ20֡���㣬���ä��������������
				startFrame = (KFs.size() > 20) ? (KFs.size() - 21) : 0;
				mvBlindwaypar = CalculateBlindwayWorldPars(KFs, startFrame);
				
				t3 = (double)clock() / CLOCKS_PER_SEC;

				sWorld->RemoveObstaclePoints();
				for (int i = 0; i < mvObstacle.size(); i++){
					sWorld->AddObstaclePoint(mvObstacle[i].first);
				}
				for (int i = 0; mvBlindwaypar.size() == 2 && i < mvBlindwaypar.size(); i++){
					sWorld->AddBlindwayPoint(mvBlindwaypar[i]);
				}

				//�ϰ�������ת�����涯����ϵ��T����Ϊ���Ƶ�λ��mvObstacleΪ���Ƶ�λ
				trans(R_wg, T_wg, mvObstacle, g_obstacle);
				trans(R_wg, T_wg, Dynamic_Obstacle, g_dynamic_obstacle);
				//ä������ת�����涯����ϵ��T����Ϊ���Ƶ�λ��mvBlindwayparΪ�����Ƶ�λ
				trans(R_wg, 1000 * T_wg, mvBlindwaypar, g_blindwaypar);
				t4 = (double)clock() / CLOCKS_PER_SEC;

				//�ϰ���������תΪ�����ƣ���ת����դ������ϵ��ȥ���ӵ�
				TransferObstaclePoints2Grid(flagBarrier);

				//ä���������Ǻ����ƣ�ֱ��ת����դ������ϵ
				if (g_blindwaypar.size() == 2)
				{
					TransferBlindwayPar2Grid(blindwayPar);
				}
				

				datafile << "���������� " << WGroundPoints.size() << endl;
				datafile << "���淽�̣� " << WPlane << endl;

				t5 = (double)clock() / CLOCKS_PER_SEC;

				controlIf = true;
			}	
			//��SLAM�����õ�GPS����ʱ
			else if (!mbSLAMWork && mbGPSWork)
				controlIf = true;

			//ֻ����������������²Ź滮
			if (controlIf)
			{
				t6 = (double)clock() / CLOCKS_PER_SEC;

				double heading;
				path_plan.GetCameraHeadingFromRot_C(mCurrentCameraR, heading);
				//////////����ļ�����//////////
				fstream headingfile;
				headingfile.open(data + "/headingfile.txt", ios::app);
				headingfile << heading << endl;
				headingfile.close();

				//��GPS�Ҳ�����Բ��ֱ��ģʽ�£�������GPS��������·��
				if (mbGPSWork && (int)meRouteMode < 3)
					mCurrentPathplanXY = mCurrentGPSXY;//��ǰ������
				//��GPS��SLAM���ã�����Բ��ֱ��ģʽ�£�����SLAM��������·��
				else if (mbSLAMWork)
				{
					Matrix3d R = Converter::toMatrix3d(mCurrentCameraR);
					mYaw = RotMatrix2EulerSLAM(R)* RADIANS_TO_DEGREES - 90;
					mCurrentPathplanXY.x += mDeltaXYZ.at<float>(0);//���㵱ǰ������
					mCurrentPathplanXY.y += mDeltaXYZ.at<float>(1);
				}

				//**********path_plan����*************
				path_plan.OverallPathPlanning(mbRoute, meRouteMode, mCurrentPathplanXY, mYaw, blindwayPar);

				double err = path_plan.LocalPathPlanning(flagBarrier, blindwayPar);
			
				t7 = (double)clock() / CLOCKS_PER_SEC;
				//*************PID����***************
				wk_control.adaptivePID(err);
				t8 = (double)clock() / CLOCKS_PER_SEC;

				//***********path_plan��ͼ*************
				path_plan.DrawingPathPlanWin(mbRoute, meRouteMode, R_cg, T_cg, wk_control.voltage, wk_control.scope, g_blindwaypar);
				RECT r = { 0, 580, 350, 600 };
				setorigin(SCREEN_X, 0);
				settextcolor(BLACK);
				settextstyle(14, 0, _T("����"));
				if (mbGPSWork && (int)meRouteMode < 3)
					//drawtext(_T("GPS����λ����..."), &r, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
					drawtext(*(GPSState+mGPSState), &r, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
				else
					drawtext(_T("SLAM����λ����..."), &r, DT_CENTER | DT_VCENTER | DT_SINGLELINE);

				//�л�Ŀ���
				if (path_plan.aim_change)
				{
					//����Ѱ�Ҹ���ֱ��
					mpDetection->mbRestartNeed = true;

					char drawtxt[50];
					sprintf_s(drawtxt, "Arrive Goal %d !!!", path_plan.pathcount);
					r = { 450, 580, 600, 600 };
					drawtext(drawtxt, &r, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
				}

				//�洢����
				char f[100];
				sprintf_s(f, (data_dir[4] + "/Road%d").c_str(), savenum);
				strcat_s(f, ".jpg");
				saveimage(f);//��·���滮���������Ӳ����
				t9 = (double)clock() / CLOCKS_PER_SEC;

				datafile << "����ǣ� " << mYaw << endl;
				datafile << "��ǰ�㣺 " << mCurrentPathplanXY << endl;
				datafile << "SLAM state: " << mbSLAMWork << ", GPS state: " << mbGPSWork << endl;
				datafile << "·���滮��Ϊ: " << err << "����" << endl;
				datafile << endl;
				
			}
			
			FramesNumLast = FramesNum;
			mvObstacle.clear();
			Dynamic_Obstacle.clear();
			mvBlindwaypar.clear();
			g_obstacle.clear();
			g_dynamic_obstacle.clear();
			g_blindwaypar.clear();

			//ʱ�����
			
			timefile << "FramesNum = " << FramesNum << endl;
			timefile << "��ǰλ�ü����ʱ�� " << t1 - t0 << " s" << endl;
			timefile << "�ϰ���ä������������ϵ��������ʱ�� " << t3 - t2 << " s" << endl;
			timefile << "һϵ������ϵת����ʱ�� " << t4 - t3 << " s" << endl;
			timefile << "�ϰ���ä��ת����դ������ϵ��ʱ�� " << t5 - t4 << " s" << endl;
			timefile << "·���滮�����ܺ�ʱ�� " << t7 - t6 << " s" << endl;
			timefile << "PID���Ʋ����ܺ�ʱ�� " << t8 - t7 << " s" << endl;
			timefile << "��ͼ�� " << t9 - t8 << " s" << endl;
			end_time = (double)clock() / CLOCKS_PER_SEC;
			timefile << "�ؽ�ģ���ܺ�ʱ�� " << end_time - start_time << " s" << endl;
			timefile << endl;
		} //while
		datafile.close();
		timefile.close();
	}

	void ObstacleReconstruction::SetReader(ReadNAV982* pReader)
	{
		mpReader = pReader;
	}

	void ObstacleReconstruction::SetDetector(Detection* pDetection)
	{
		mpDetection = pDetection;
	}

	void ObstacleReconstruction::RequestStop()
	{

		boost::mutex::scoped_lock lock(mMutexStop);
		mbStopRequested = true;



	}

	bool ObstacleReconstruction::Stop()
	{
		boost::mutex::scoped_lock lock(mMutexStop);
		if (mbStopRequested)
		{
			FramesNumLast = 0;
			mSLAMXYZ = Mat::zeros(3, 1, CV_32F);
			WPlane.release();
			mbStopped = true;
			mbStopRequested = false;
		}

		return true;
	}

	bool ObstacleReconstruction::isStopped()
	{
		boost::mutex::scoped_lock lock(mMutexStop);
		return mbStopped;
	}
	void ObstacleReconstruction::Release()
	{
		boost::mutex::scoped_lock lock(mMutexStop);
		mbStopped = false;
	}

	vector<cv::Point3f> ObstacleReconstruction::GetWGroundPoints(vector<ORB_SLAM2::MapPoint*> KPs)
	{
		//�������㲢ת��Ϊmm��λ
		vector<cv::Point3f> WGroundPoints;

		////SLAM���������ϰ��﷨
		//for (int i = 0; i < KPs.size(); i++){
		//	float x = 1000 * KPs[i]->GetWorldPos().at<float>(0);
		//	float y = 1000 * KPs[i]->GetWorldPos().at<float>(1);
		//	float z = 1000 * KPs[i]->GetWorldPos().at<float>(2);
		//	WGroundPoints.push_back(cv::Point3f(x, y, z));
		//}

		if (KPs.size() > 1000){
			for (int i = 0; i < 1000; i++){
			float x = 1000 * KPs[i]->GetWorldPos().at<float>(0);
			float y = 1000 * KPs[i]->GetWorldPos().at<float>(1);
			float z = 1000 * KPs[i]->GetWorldPos().at<float>(2);
			WGroundPoints.push_back(cv::Point3f(x, y, z));
			}
			}
			else{
			for (int i = 0; i < KPs.size(); i++){
			float x = 1000 * KPs[i]->GetWorldPos().at<float>(0);
			float y = 1000 * KPs[i]->GetWorldPos().at<float>(1);
			float z = 1000 * KPs[i]->GetWorldPos().at<float>(2);
			WGroundPoints.push_back(cv::Point3f(x, y, z));
			}
			}
		return WGroundPoints;
	}

	void ObstacleReconstruction::GetWGroundPoints(ORB_SLAM2::KeyFrame* KFs, vector<cv::Point3f>& WGroundPoints)
	{
		set<ORB_SLAM2::MapPoint*> KPs;
		KPs = KFs->GetMapPoints();
		for (set<ORB_SLAM2::MapPoint*>::const_iterator itMP = KPs.begin(), itEndMP = KPs.end(); itMP != itEndMP; itMP++)
		{
			auto pMP = *itMP;
			float x = 1000 * pMP->GetWorldPos().at<float>(0);
			float y = 1000 * pMP->GetWorldPos().at<float>(1);
			float z = 1000 * pMP->GetWorldPos().at<float>(2);
			WGroundPoints.push_back(cv::Point3f(x, y, z));
		}
	}

	int ObstacleReconstruction::Otsu(int data[ROOM][ROOM])
	{
		//histogram
		vector<int> histogram;
		for (int i = 0; i < ROOM; i++)
		{	
			for (int j = 0; j < ROOM; j++)
			{
				histogram.push_back(data[i][j]);
			}
		}
		sort(histogram.begin(), histogram.end(),greater<int>());
		float Size;
		for (int i = 0; i < histogram.size(); i++){
			if (histogram[i] == 0){
				Size = i + 1;
				break;
			}
		}

		//average pixel value    
		float avgValue = 0;
		
		for (int i = 0; i < Size; i++)
		{
			avgValue += histogram[i] / Size;  //����ͼ���ƽ���Ҷ�  
		}
		int threshold;
		float maxVariance = 0;
		double U, U1,N;
		for (int i = 0; i <50; i++)
		{
			float W = 0;
			float w = 0, u = 0, u1 = 0;
			W = i + 1;
			w = W / Size;
			for (int j = 0; j<W; j++){
				u += histogram[j] / W;
			}
			u1 = (avgValue*Size - u*W) / (Size - W);
			float t = u - u1;
			float variance = t * t * w * (1 - w);
			if (variance > maxVariance)
			{
				maxVariance = variance;
				threshold = histogram[i];
				N = W;
				U = u;
				U1 = u1;
			}
		}
		float rate = U / U1;
		//if (rate<10)
		//	threshold = histogram[0] + 1;
		//else{
		//	if (N > 4 && N<10)
		//		threshold = (threshold + U) / 2;
		//	if (N > 10 || N == 10)
		//		threshold = U;
		//}
		
		return threshold;
	}

	void ObstacleReconstruction::SetCurrentCameraPose(const cv::Mat& Tcw)
	{
		boost::mutex::scoped_lock lock(mMutexCurrentCamera);
		mCurrentCameraPose = Tcw.clone();
	}

	void ObstacleReconstruction::GetCurrentCameraPose()
	{
		if (!mCurrentCameraPose.empty())
		{
			cv::Mat Rwc(3, 3, CV_32F);
			cv::Mat twc(3, 1, CV_32F);
			{
				boost::mutex::scoped_lock lock(mMutexCurrentCamera);
				mCurrentCameraR = mCurrentCameraPose.rowRange(0, 3).colRange(0, 3);
				mCurrentCamerat = mCurrentCameraPose.rowRange(0, 3).col(3);
			}
		}
		else
		{
			mCurrentCameraR = cv::Mat::eye(3, 3, CV_32F);
			mCurrentCamerat = cv::Mat::zeros(3, 1, CV_32F);
		}
	}

	void ObstacleReconstruction::GetSlamGpsWorkStatusAndData(vector<ORB_SLAM2::KeyFrame*> KFs)
	{
		//�ж�SLAM�����Ƿ���ã��ɿ����Ƿ������ɹ��ı�־��
		mbSLAMWork = (KFs.size() > 0) ? true : false;

		//�ж�GPS����,��ȡGPS����
		mbGPSWork = mpReader->GetGNSSFixFlag();
		mGPSState = mpReader->GetGNSSFixState();

		if (mbSLAMWork)
		{
			if (mbRoute) //�ڵ���ģʽ��äУģʽ������ģʽ��Բģʽ��ֱ��ģʽ�»�ȡ��Ϻ��SLAM����
			{
				Matrix3d R = InitialRot.GetInitialRot().matrix();
				cv::Mat InitialR = Converter::toCvMat(R);
				mDeltaXYZ = (-InitialR.t()*(mCurrentCameraR.t()*mCurrentCamerat)) - mSLAMXYZ; //������������֮��������
				mSLAMXYZ = -InitialR.t()*(mCurrentCameraR.t()*mCurrentCamerat);
			}
			else //��ɢ��ģʽ��ä��ģʽ���ϰ���ģʽ�»�ȡԭSLAM����
			{
				float yaw =  90 / DEGREES_TO_RADIANS;
				float roll = 180/ DEGREES_TO_RADIANS;
				float pitch = 0;
				cv::Mat R(3, 3, CV_32F);
				Euler2Rotmateix(yaw, roll, pitch, R);
				mDeltaXYZ = (-R.t()*(mCurrentCameraR.t()*mCurrentCamerat)) - mSLAMXYZ; //������������֮��������
				mSLAMXYZ = -R.t()*(mCurrentCameraR.t()*mCurrentCamerat);
			}
		}

		if (mbGPSWork && mbRoute && (int)meRouteMode < 3) //ֻ�ڵ���ģʽ��äУģʽ������ģʽ�»�ȡ�ٶ�·��
		{
			GPSIMU CurrentData = mpReader->GetGPSIMU();
			if (!ReconstructInitflag)
			{
				Point3d IniLatLonHeight = { CurrentData.latitude, CurrentData.longitude, CurrentData.height };
				Originbaidu = path_plan.GetOriginbaiduPoint(IniLatLonHeight, destination, meRouteMode);
				ReconstructInitflag = true;
			}

			Point3d CurrentGPS = { CurrentData.latitude, CurrentData.longitude, CurrentData.height };
			Matrix3d R = Converter::toMatrix3d(mCurrentCameraR);
			mYaw = RotMatrix2Euler(R*InitialRot.GetInitialRot().matrix())* RADIANS_TO_DEGREES;
			cout << "now yaw is: " << mYaw << "!!!!!!!!!!!!!!!!!!!!!!" << endl;
			Point2d Currentbaidu = getBaiduLatLon(CurrentGPS);
			mCurrentGPSXY = ConvertLatLon2XY(Originbaidu.x*DEGREES_TO_RADIANS, Originbaidu.y*DEGREES_TO_RADIANS, Currentbaidu.x*DEGREES_TO_RADIANS, Currentbaidu.y*DEGREES_TO_RADIANS);
		}
	}

	//vector<pair<cv::Point3f, int>> ObstacleReconstruction::CalculateObstacleWorldPoints(vector<ORB_SLAM2::KeyFrame*> KFs, int startFrame)
	//{
	//	vector<pair<cv::Point3f, int>> ObPoints(0);
	//	for (int i = startFrame; i < KFs.size() - 1; i++){
	//		//cout << "�������������ϵ�����꣺" << KFs[i]->GetCameraCenter() << endl;
	//		cv::Mat CameraR = KFs[i]->GetPose().rowRange(0, 3).colRange(0, 3).clone();
	//		cv::Mat CameraT = KFs[i]->GetPose().rowRange(0, 3).col(3).clone();
	//		vector<pair<cv::Point3f, int>> CurrentObstacle = KFs[i]->obstacle;
	//		for (int i = 0; i < CurrentObstacle.size(); i++){
	//			cv::Mat Xc(3, 1, CV_32F);
	//			Xc.at<float>(0) = CurrentObstacle[i].first.x*0.001;
	//			Xc.at<float>(1) = CurrentObstacle[i].first.y*0.001;
	//			Xc.at<float>(2) = CurrentObstacle[i].first.z*0.001;
	//			cv::Mat X(3, 1, CV_32F);
	//			X = CameraR.t()*(Xc - CameraT);
	//			float x = X.at<float>(0);
	//			float y = X.at<float>(1);
	//			float z = X.at<float>(2);
	//			if (-(WPlane.at<float>(0)*x + WPlane.at<float>(1)*y + WPlane.at<float>(2)*z - 1) / sqrt(pow(WPlane.at<float>(0), 2) + pow(WPlane.at<float>(1), 2) + pow(WPlane.at<float>(2), 2))>0.1){
	//				ObPoints.push_back(make_pair(cv::Point3f(x, y, z), CurrentObstacle[i].second));
	//			}
	//		}
	//	}
	//
	//	return ObPoints;
	//}
	
	void ObstacleReconstruction::CalculateObstacleWorldPoints(vector<ORB_SLAM2::KeyFrame*> KFs, int startFrame, vector<pair<cv::Point3f, int>>& ObPoints, vector<cv::Point3f>& Dynamic_ObPoints)
	{
		set<ORB_SLAM2::MapPoint*> MPs;
		vector<Point3f> DynamicPoints,DynamicPoints_all;
		KeyFrame* mCurrentKeyFrame = KFs[FramesNum - 1];
		DynamicPoints = mCurrentKeyFrame->GetDynamicPoints();
		for (int i = startFrame; i < KFs.size(); i++)
		{
			set<ORB_SLAM2::MapPoint*> KPs;
			KPs = KFs[i]->GetMapPoints();
			
			MPs.insert(KPs.begin(), KPs.end());
			//DynamicPoints_all.insert(DynamicPoints_all.end(), DynamicPoints.begin(), DynamicPoints.end());
		}

		cout << "**************************************" << FramesNum<<"   " << DynamicPoints.size() << endl;
		vector<Point3f> MapPoints;
		for (set<ORB_SLAM2::MapPoint*>::const_iterator itMP = MPs.begin(), itEndMP = MPs.end(); itMP != itEndMP; itMP++)
		{
			auto pMP = *itMP;

			float x = pMP->GetWorldPos().at<float>(0);
			float y = pMP->GetWorldPos().at<float>(1);
			float z = pMP->GetWorldPos().at<float>(2);
			if (-(WPlane.at<float>(0)*x + WPlane.at<float>(1)*y + WPlane.at<float>(2)*z - 1) / sqrt(pow(WPlane.at<float>(0), 2) + pow(WPlane.at<float>(1), 2) + pow(WPlane.at<float>(2), 2)) < 0.2)
				MapPoints.push_back(cv::Point3f(x, y, z));
		}
		vector<Point3f>best_set;
		vector<int>num;
		Point3f Aw_b, Aw_a;
		Aw_b.x = WPlane.at<float>(0);
		Aw_b.y = WPlane.at<float>(1);
		Aw_b.z = WPlane.at<float>(2);
		Aw_a = ob_detection::ransac_Plane(MapPoints, best_set, num, 500, 0.5, 0.2, Aw_b, r);
		R << r, 0, 0,
			0, r, 0,
			0, 0, r;
		cv::Mat WPlane_m(1, 3, CV_32F);
		WPlane_m.at<float>(0) = Aw_a.x;
		WPlane_m.at<float>(1) = Aw_a.y;
		WPlane_m.at<float>(2) = Aw_a.z;
		cv::Mat Norm_Mat = WPlane_m.cross(WPlane);
		float Norm = sqrt(Norm_Mat.dot(Norm_Mat));
		float Norm_1 = sqrt(WPlane.dot(WPlane));
		float Norm_2 = sqrt(WPlane_m.dot(WPlane_m));
		float theta = asin(Norm / Norm_1 / Norm_2);
		if (theta > CV_PI / 18)
			mpDetection->ResetWPlane();
		Z = Converter::toVector3d(WPlane_m);
		X_j_prime = X_i;
		P_j_prime = P_i + Q;
		K = P_j_prime*(P_j_prime + R).inverse();
		X_j = X_j_prime + K*(Z - X_j_prime);
		WPlane = Converter::toCvMat(X_j).t();
		sWorld->SetWPlane(WPlane);
		P_j = (Matrix3d::Identity() - K)*P_j_prime;

		X_i = X_j;
		P_i = P_j;


		for (set<ORB_SLAM2::MapPoint*>::const_iterator itMP = MPs.begin(), itEndMP = MPs.end(); itMP != itEndMP; itMP++)
		{
			auto pMP = *itMP;
			int Num = pMP->GetObservations().size();
			float x = pMP->GetWorldPos().at<float>(0);
			float y = pMP->GetWorldPos().at<float>(1);
			float z = pMP->GetWorldPos().at<float>(2);
			if (-(WPlane.at<float>(0)*x + WPlane.at<float>(1)*y + WPlane.at<float>(2)*z - 1) / sqrt(pow(WPlane.at<float>(0), 2) + pow(WPlane.at<float>(1), 2) + pow(WPlane.at<float>(2), 2)) > 0.2)
				ObPoints.push_back(make_pair(cv::Point3f(x, y, z), Num));

		}
		
		for (int i = 0; i < DynamicPoints.size();i++)
		{
			auto pDP = DynamicPoints[i];
			//int Num = 100;//100��ʾ��̬�㣿
			float x = pDP.x;
			float y = pDP.y;
			float z = pDP.z;
			if (-(WPlane.at<float>(0)*x + WPlane.at<float>(1)*y + WPlane.at<float>(2)*z - 1) / sqrt(pow(WPlane.at<float>(0), 2) + pow(WPlane.at<float>(1), 2) + pow(WPlane.at<float>(2), 2)) > 0.2)
				Dynamic_ObPoints.push_back(cv::Point3f(x, y, z));

		}
	}

	vector<cv::Point3f> ObstacleReconstruction::CalculateBlindwayWorldPars(vector<ORB_SLAM2::KeyFrame*> KFs, int startFrame)
	{
		vector<cv::Point3f> currentblindwaypar(0);

		//��ѡ�����ä����������ϵ����
		for (int i = KFs.size() - 1; i >= startFrame; i--)
		{
			currentblindwaypar = KFs[i]->GetBlindwaypar();
			if (currentblindwaypar.size() > 1)
				break;
		}

		//�����Ҫ��ֱ�߽���ɸѡ
		if (mpDetection->getSelectNeedBool())
		{
			vector<vector<Point3f>> candilines_wpoints = mpDetection->mvCandilinesWpoints;
			//��ѡֱ�߲���ת���涯����ϵ
			vector<vector<Point3f>> candilines_gpoints(0);
			for (int i = 0; i < candilines_wpoints.size(); i++)
			{
				vector<Point3f> g_points(0); //�涯����ϵ��ä������
				//ֱ�߲���ת�����涯����ϵ��T����Ϊ���Ƶ�λ��candilines_wpointsΪ�����Ƶ�λ
				trans(R_wg, 1000*T_wg, candilines_wpoints[i], g_points);
				candilines_gpoints.push_back(g_points);
			}
			//ɸѡ
			int bestline_id = path_plan.ParallelLineJudge(candilines_gpoints, false);
			
			if (bestline_id == -1)
				mpDetection->setRestartNeedBool(true);
			else
			{
				vector<Point3f> bestline_wpoints = candilines_wpoints[bestline_id];

				mpDetection->mvBestlineWpoints = bestline_wpoints;
				currentblindwaypar = bestline_wpoints;

				mpDetection->setSelectNeedBool(false);
			}
		}

		return currentblindwaypar;
	}

	void ObstacleReconstruction::CalculateWorld2GroundRT()
	{
		//ƽ�����ת�Ƶ��������ϵ��
		cv::Mat Ac(1, 3, CV_32F);
		cv::Mat ART(1, 1, CV_32F);
		ART = WPlane*mCurrentCameraR.t()*mCurrentCamerat;
		Ac = WPlane*mCurrentCameraR.t() / (1 + ART.at<float>(0));
		//�����������ϵ���涯����ϵת������
		R_cg = Mat::zeros(3, 3, CV_32F);
		T_cg = Mat::zeros(3, 1, CV_32F);
		RandT(Ac.at<float>(0), Ac.at<float>(1), Ac.at<float>(2), R_cg, T_cg);
		//������������ϵ���涯����ϵת������
		R_wg = R_cg*mCurrentCameraR;
		T_wg = R_cg*mCurrentCamerat + T_cg;
	}

	void ObstacleReconstruction::TransferObstaclePoints2Grid(int flagBarrier[ROOM][ROOM])
	{
		for (int i = 0; i < mvObstacle.size(); i++)
		{
			cv::Point3f m = 1000 * g_obstacle[i]; //�ϰ����תΪ������
			cv::Point3f m1 = path_plan.PointTransform(m);
			//cout << "�ϰ����ڵ����ϵĶ�ά����;" << m1 << endl;
			if ((int)m1.x >= 0 && (int)m1.x <= ROOM && (int)m1.y >= 0 && (int)m1.y <= ROOM)
			{
				flagBarrier[(int)m1.x][(int)m1.y] = flagBarrier[(int)m1.x][(int)m1.y] + mvObstacle[i].second;
			}
		}// ��ȡ�ϰ����
		int dynamic_flag_barrier[ROOM][ROOM];
		for (int i = 0; i < ROOM; i++)
		{
			for (int j = 0; j < ROOM; j++)
			{
				dynamic_flag_barrier[i][j] = 0;
			}
		}
		for (int i = 0; i < g_dynamic_obstacle.size(); i++)
		{
			cv::Point3f md = 1000 * g_dynamic_obstacle[i]; //�ϰ����תΪ������
			cv::Point3f md1 = path_plan.PointTransform(md);
			//cout << "�ϰ����ڵ����ϵĶ�ά����;" << m1 << endl;
			if ((int)md1.x >= 0 && (int)md1.x <= ROOM && (int)md1.y >= 0 && (int)md1.y <= ROOM)
			{
				dynamic_flag_barrier[(int)md1.x][(int)md1.y]++;
			}
		}
		// ��ȡ��̬�ϰ����
		/*for (int i = 0; i < ROOM; i++){
			for (int j = 0; j < ROOM; j++){
				if (dynamic_flag_barrier[i][j] > 5)
					flagBarrier[i][j] = 2;
			}
		}*/
		//����Ӧդ�������ӵ�
		int Threshold = max(Otsu(flagBarrier), 20);
		cout << "Threshold=" << Threshold << endl;
		for (int i = 0; i < ROOM; i++){
			for (int j = 0; j < ROOM; j++){
				if (flagBarrier[i][j] <= thres)  //�Ӳ����ϰ��﷨ȡThreshold��SLAM���������ϰ��﷨ȡ10
					flagBarrier[i][j] = 0;
				else
					flagBarrier[i][j] = 1;
			}
		}
		for (int i = 0; i < ROOM; i++){
			for (int j = 0; j < ROOM; j++){
				
				if (dynamic_flag_barrier[i][j] > 10)
					flagBarrier[i][j] = 2;
			}
		}
		
	}

	void ObstacleReconstruction::TransferBlindwayPar2Grid(Point3f &blindwayPar)
	{
		vector<Point3f> tmp_blindwaypar(0);	//������Գ߶Ⱥ����ʱä������
		Point3f blindwaypoint1 = g_blindwaypar[0];//ä��ʵ������
		Point3f blindwaypoint2 = g_blindwaypar[1];
		//�����涯����ϵ��ä�������߲���
		float centalK = (float)(blindwaypoint2.y - blindwaypoint1.y) / (blindwaypoint2.x - blindwaypoint1.x + 0.001);
		float centalB = blindwaypoint2.y - centalK*blindwaypoint2.x;
		Point3f par;
		par.x = centalK;
		par.y = -1;
		par.z = centalB;

		if (meRouteMode == eRouteMode::Blindway)
		{
			//�����涯ԭ�㵽ä�������ߵĴ�ֱ����
			ofstream blindwayDistance(data + "/BlindwayDistance.txt", ios::app);
			static float sumdis = 0;
			static int num = 0;
			static int serial_dis = 0;
			float dis = 0.;
			float absdis = abs(centalB) / sqrt(1 + centalK*centalK);
			//k��b��ţ�ԭ������࣬��Ϊ��
			if (centalB*centalK<0)
				dis = -absdis;
			//k��bͬ�ţ�ԭ�����Ҳ࣬��Ϊ��
			else
				dis = absdis;
			if (abs(dis / 10) <= 50)
			{
				sumdis += absdis;
				num++;
				blindwayDistance << "FramesNum = " << /*FramesNum*/++serial_dis << endl;
				blindwayDistance << "distance = " << setprecision(5) << dis / 10 << " cm��������������" << endl;
				blindwayDistance << "����ֵƽ���� " << setprecision(5) << sumdis / num / 10 << " cm" << endl;
				blindwayDistance << endl;
			}
			blindwayDistance.close();
		}

		//�洢���Գ߶Ⱥ�Ĳ���
		tmp_blindwaypar.push_back(blindwaypoint1);
		tmp_blindwaypar.push_back(blindwaypoint2);
		tmp_blindwaypar.push_back(par);
		//ת����դ������ϵ
		blindwayPar = path_plan.BlindLineTransform(tmp_blindwaypar);
	}

}
