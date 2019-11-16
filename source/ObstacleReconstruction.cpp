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

//地面点、相机T阵、障碍物、盲道均要以mm为单位
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
		//GPS 状态常量
		const char *GPSState[8] = { "0 No GNSS Fix 计算位置中...", "1 2D GNSS Fix 计算位置中...", "2 3D GNSS Fix 计算位置中...", "3 SBAS GNSS Fix 计算位置中...", "4 Differential GNSS Fix 计算位置中...", "5 Omnistar GNSS Fix 计算位置中...", "6 RTK Float GNSS Fix 计算位置中...", "7 RTK Fixed GNSS Fix 计算位置中..." };

		//调用行走控制模块类并创建写线程
		wk_control wk_control;

		fstream datafile, timefile;
		datafile.open(data + "/ReconstructionData.txt", ios::app);   //创建一个文件
		timefile.open(data + "/ReconstructionTime.txt", ios::app);

		while (1){
			if (Stop())
			{
				while (isStopped())
					Sleep(3);
			}
			//每次都需要重新初始化的参数
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

			//判断SLAM和GPS状态并获取数据
			GetSlamGpsWorkStatusAndData(KFs);

			//无GPS且SLAM不可用
			if (!mbSLAMWork && !mbGPSWork)
			{
				cout << "无GPS信号且SLAM数据不可用!!!请检查..." << endl;
				continue;
			}

			t1 = (double)clock() / CLOCKS_PER_SEC;

			//是否控制的标志
			bool controlIf = false;
			//当SLAM可用且更新关键帧时操作
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

				//计算世界坐标系到地面坐标系的RT，米制单位
				CalculateWorld2GroundRT();
				
				//Dynamic_Obstacle.clear();
				//只取15帧计算，添加障碍物点，SLAM法为米制
				int startFrame = (KFs.size() > 5) ? (KFs.size() - 6) : 0;
				CalculateObstacleWorldPoints(KFs, startFrame, mvObstacle, Dynamic_Obstacle);
				
				//只取20帧计算，添加盲道参数，毫米制
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

				//障碍物坐标转换到随动坐标系，T矩阵为米制单位，mvObstacle为米制单位
				trans(R_wg, T_wg, mvObstacle, g_obstacle);
				trans(R_wg, T_wg, Dynamic_Obstacle, g_dynamic_obstacle);
				//盲道参数转换到随动坐标系，T矩阵为米制单位，mvBlindwaypar为毫米制单位
				trans(R_wg, 1000 * T_wg, mvBlindwaypar, g_blindwaypar);
				t4 = (double)clock() / CLOCKS_PER_SEC;

				//障碍物坐标先转为毫米制，再转换到栅格坐标系并去除杂点
				TransferObstaclePoints2Grid(flagBarrier);

				//盲道参数已是毫米制，直接转换到栅格坐标系
				if (g_blindwaypar.size() == 2)
				{
					TransferBlindwayPar2Grid(blindwayPar);
				}
				

				datafile << "地面点个数： " << WGroundPoints.size() << endl;
				datafile << "地面方程： " << WPlane << endl;

				t5 = (double)clock() / CLOCKS_PER_SEC;

				controlIf = true;
			}	
			//当SLAM不可用但GPS可用时
			else if (!mbSLAMWork && mbGPSWork)
				controlIf = true;

			//只有在以上两种情况下才规划
			if (controlIf)
			{
				t6 = (double)clock() / CLOCKS_PER_SEC;

				double heading;
				path_plan.GetCameraHeadingFromRot_C(mCurrentCameraR, heading);
				//////////输出文件数据//////////
				fstream headingfile;
				headingfile.open(data + "/headingfile.txt", ios::app);
				headingfile << heading << endl;
				headingfile.close();

				//有GPS且不处在圆或直线模式下，则利用GPS计算行走路径
				if (mbGPSWork && (int)meRouteMode < 3)
					mCurrentPathplanXY = mCurrentGPSXY;//当前点坐标
				//无GPS但SLAM可用，或处在圆或直线模式下，利用SLAM计算行走路径
				else if (mbSLAMWork)
				{
					Matrix3d R = Converter::toMatrix3d(mCurrentCameraR);
					mYaw = RotMatrix2EulerSLAM(R)* RADIANS_TO_DEGREES - 90;
					mCurrentPathplanXY.x += mDeltaXYZ.at<float>(0);//计算当前点坐标
					mCurrentPathplanXY.y += mDeltaXYZ.at<float>(1);
				}

				//**********path_plan计算*************
				path_plan.OverallPathPlanning(mbRoute, meRouteMode, mCurrentPathplanXY, mYaw, blindwayPar);

				double err = path_plan.LocalPathPlanning(flagBarrier, blindwayPar);
			
				t7 = (double)clock() / CLOCKS_PER_SEC;
				//*************PID控制***************
				wk_control.adaptivePID(err);
				t8 = (double)clock() / CLOCKS_PER_SEC;

				//***********path_plan绘图*************
				path_plan.DrawingPathPlanWin(mbRoute, meRouteMode, R_cg, T_cg, wk_control.voltage, wk_control.scope, g_blindwaypar);
				RECT r = { 0, 580, 350, 600 };
				setorigin(SCREEN_X, 0);
				settextcolor(BLACK);
				settextstyle(14, 0, _T("宋体"));
				if (mbGPSWork && (int)meRouteMode < 3)
					//drawtext(_T("GPS计算位置中..."), &r, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
					drawtext(*(GPSState+mGPSState), &r, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
				else
					drawtext(_T("SLAM计算位置中..."), &r, DT_CENTER | DT_VCENTER | DT_SINGLELINE);

				//切换目标点
				if (path_plan.aim_change)
				{
					//重新寻找跟踪直线
					mpDetection->mbRestartNeed = true;

					char drawtxt[50];
					sprintf_s(drawtxt, "Arrive Goal %d !!!", path_plan.pathcount);
					r = { 450, 580, 600, 600 };
					drawtext(drawtxt, &r, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
				}

				//存储操作
				char f[100];
				sprintf_s(f, (data_dir[4] + "/Road%d").c_str(), savenum);
				strcat_s(f, ".jpg");
				saveimage(f);//将路径规划结果保存在硬盘里
				t9 = (double)clock() / CLOCKS_PER_SEC;

				datafile << "航向角： " << mYaw << endl;
				datafile << "当前点： " << mCurrentPathplanXY << endl;
				datafile << "SLAM state: " << mbSLAMWork << ", GPS state: " << mbGPSWork << endl;
				datafile << "路径规划角为: " << err << "弧度" << endl;
				datafile << endl;
				
			}
			
			FramesNumLast = FramesNum;
			mvObstacle.clear();
			Dynamic_Obstacle.clear();
			mvBlindwaypar.clear();
			g_obstacle.clear();
			g_dynamic_obstacle.clear();
			g_blindwaypar.clear();

			//时间计算
			
			timefile << "FramesNum = " << FramesNum << endl;
			timefile << "当前位置计算耗时： " << t1 - t0 << " s" << endl;
			timefile << "障碍物盲道点世界坐标系坐标计算耗时： " << t3 - t2 << " s" << endl;
			timefile << "一系列坐标系转换耗时： " << t4 - t3 << " s" << endl;
			timefile << "障碍物盲道转换到栅格坐标系耗时： " << t5 - t4 << " s" << endl;
			timefile << "路径规划部分总耗时： " << t7 - t6 << " s" << endl;
			timefile << "PID控制部分总耗时： " << t8 - t7 << " s" << endl;
			timefile << "绘图： " << t9 - t8 << " s" << endl;
			end_time = (double)clock() / CLOCKS_PER_SEC;
			timefile << "重建模块总耗时： " << end_time - start_time << " s" << endl;
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
		//计算地面点并转换为mm单位
		vector<cv::Point3f> WGroundPoints;

		////SLAM特征点检测障碍物法
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
			avgValue += histogram[i] / Size;  //整幅图像的平均灰度  
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
		//判断SLAM数据是否可用（可看作是否重启成功的标志）
		mbSLAMWork = (KFs.size() > 0) ? true : false;

		//判断GPS质量,读取GPS数据
		mbGPSWork = mpReader->GetGNSSFixFlag();
		mGPSState = mpReader->GetGNSSFixState();

		if (mbSLAMWork)
		{
			if (mbRoute) //在到达模式、盲校模式、北航模式、圆模式、直线模式下获取组合后的SLAM数据
			{
				Matrix3d R = InitialRot.GetInitialRot().matrix();
				cv::Mat InitialR = Converter::toCvMat(R);
				mDeltaXYZ = (-InitialR.t()*(mCurrentCameraR.t()*mCurrentCamerat)) - mSLAMXYZ; //计算相邻两次之间的坐标差
				mSLAMXYZ = -InitialR.t()*(mCurrentCameraR.t()*mCurrentCamerat);
			}
			else //在散步模式、盲道模式、障碍物模式下获取原SLAM数据
			{
				float yaw =  90 / DEGREES_TO_RADIANS;
				float roll = 180/ DEGREES_TO_RADIANS;
				float pitch = 0;
				cv::Mat R(3, 3, CV_32F);
				Euler2Rotmateix(yaw, roll, pitch, R);
				mDeltaXYZ = (-R.t()*(mCurrentCameraR.t()*mCurrentCamerat)) - mSLAMXYZ; //计算相邻两次之间的坐标差
				mSLAMXYZ = -R.t()*(mCurrentCameraR.t()*mCurrentCamerat);
			}
		}

		if (mbGPSWork && mbRoute && (int)meRouteMode < 3) //只在到达模式、盲校模式、北航模式下获取百度路径
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
	//		//cout << "相机在世界坐标系下坐标：" << KFs[i]->GetCameraCenter() << endl;
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
			//int Num = 100;//100表示动态点？
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

		//挑选最近的盲道世界坐标系参数
		for (int i = KFs.size() - 1; i >= startFrame; i--)
		{
			currentblindwaypar = KFs[i]->GetBlindwaypar();
			if (currentblindwaypar.size() > 1)
				break;
		}

		//如果需要对直线进行筛选
		if (mpDetection->getSelectNeedBool())
		{
			vector<vector<Point3f>> candilines_wpoints = mpDetection->mvCandilinesWpoints;
			//待选直线参数转到随动坐标系
			vector<vector<Point3f>> candilines_gpoints(0);
			for (int i = 0; i < candilines_wpoints.size(); i++)
			{
				vector<Point3f> g_points(0); //随动坐标系下盲道参数
				//直线参数转换到随动坐标系，T矩阵为米制单位，candilines_wpoints为毫米制单位
				trans(R_wg, 1000*T_wg, candilines_wpoints[i], g_points);
				candilines_gpoints.push_back(g_points);
			}
			//筛选
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
		//平面参数转移到相机坐标系下
		cv::Mat Ac(1, 3, CV_32F);
		cv::Mat ART(1, 1, CV_32F);
		ART = WPlane*mCurrentCameraR.t()*mCurrentCamerat;
		Ac = WPlane*mCurrentCameraR.t() / (1 + ART.at<float>(0));
		//计算相机坐标系到随动坐标系转换矩阵
		R_cg = Mat::zeros(3, 3, CV_32F);
		T_cg = Mat::zeros(3, 1, CV_32F);
		RandT(Ac.at<float>(0), Ac.at<float>(1), Ac.at<float>(2), R_cg, T_cg);
		//计算世界坐标系到随动坐标系转换矩阵
		R_wg = R_cg*mCurrentCameraR;
		T_wg = R_cg*mCurrentCamerat + T_cg;
	}

	void ObstacleReconstruction::TransferObstaclePoints2Grid(int flagBarrier[ROOM][ROOM])
	{
		for (int i = 0; i < mvObstacle.size(); i++)
		{
			cv::Point3f m = 1000 * g_obstacle[i]; //障碍物点转为毫米制
			cv::Point3f m1 = path_plan.PointTransform(m);
			//cout << "障碍物在地面上的二维坐标;" << m1 << endl;
			if ((int)m1.x >= 0 && (int)m1.x <= ROOM && (int)m1.y >= 0 && (int)m1.y <= ROOM)
			{
				flagBarrier[(int)m1.x][(int)m1.y] = flagBarrier[(int)m1.x][(int)m1.y] + mvObstacle[i].second;
			}
		}// 获取障碍物点
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
			cv::Point3f md = 1000 * g_dynamic_obstacle[i]; //障碍物点转为毫米制
			cv::Point3f md1 = path_plan.PointTransform(md);
			//cout << "障碍物在地面上的二维坐标;" << m1 << endl;
			if ((int)md1.x >= 0 && (int)md1.x <= ROOM && (int)md1.y >= 0 && (int)md1.y <= ROOM)
			{
				dynamic_flag_barrier[(int)md1.x][(int)md1.y]++;
			}
		}
		// 获取动态障碍物点
		/*for (int i = 0; i < ROOM; i++){
			for (int j = 0; j < ROOM; j++){
				if (dynamic_flag_barrier[i][j] > 5)
					flagBarrier[i][j] = 2;
			}
		}*/
		//自适应栅格消除杂点
		int Threshold = max(Otsu(flagBarrier), 20);
		cout << "Threshold=" << Threshold << endl;
		for (int i = 0; i < ROOM; i++){
			for (int j = 0; j < ROOM; j++){
				if (flagBarrier[i][j] <= thres)  //视差检测障碍物法取Threshold，SLAM特征点检测障碍物法取10
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
		vector<Point3f> tmp_blindwaypar(0);	//定义乘以尺度后的临时盲道参数
		Point3f blindwaypoint1 = g_blindwaypar[0];//盲道实际两点
		Point3f blindwaypoint2 = g_blindwaypar[1];
		//计算随动坐标系下盲道中心线参数
		float centalK = (float)(blindwaypoint2.y - blindwaypoint1.y) / (blindwaypoint2.x - blindwaypoint1.x + 0.001);
		float centalB = blindwaypoint2.y - centalK*blindwaypoint2.x;
		Point3f par;
		par.x = centalK;
		par.y = -1;
		par.z = centalB;

		if (meRouteMode == eRouteMode::Blindway)
		{
			//计算随动原点到盲道中心线的垂直距离
			ofstream blindwayDistance(data + "/BlindwayDistance.txt", ios::app);
			static float sumdis = 0;
			static int num = 0;
			static int serial_dis = 0;
			float dis = 0.;
			float absdis = abs(centalB) / sqrt(1 + centalK*centalK);
			//k、b异号，原点在左侧，记为负
			if (centalB*centalK<0)
				dis = -absdis;
			//k、b同号，原点在右侧，记为正
			else
				dis = absdis;
			if (abs(dis / 10) <= 50)
			{
				sumdis += absdis;
				num++;
				blindwayDistance << "FramesNum = " << /*FramesNum*/++serial_dis << endl;
				blindwayDistance << "distance = " << setprecision(5) << dis / 10 << " cm（距离左负右正）" << endl;
				blindwayDistance << "绝对值平均误差： " << setprecision(5) << sumdis / num / 10 << " cm" << endl;
				blindwayDistance << endl;
			}
			blindwayDistance.close();
		}

		//存储乘以尺度后的参数
		tmp_blindwaypar.push_back(blindwaypoint1);
		tmp_blindwaypar.push_back(blindwaypoint2);
		tmp_blindwaypar.push_back(par);
		//转换到栅格坐标系
		blindwayPar = path_plan.BlindLineTransform(tmp_blindwaypar);
	}

}
