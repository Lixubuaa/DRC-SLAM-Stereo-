#ifndef DETECTION_H
#define DETECTION_H

#include "Map.h"
#include "generalDataFuction.h"
#include "obstacle_detection.h"
#include "blindway_location.h"
#include "path_planning.h"
#include "roadline_tracking.h"

enum class eLineTrackMode{ Close, Blindway, BrickLine };

namespace ORB_SLAM2
{
	class KeyFrame;
	class Detection
	{
	public:
		ORB_SLAM2::Map* sWorld;

		//调用盲道识别定位模块类
		bw_location bw_location;
		RoadLineTrack rl_tracking;

		bool mbRestartNeed;
		bool mbSelectNeed;
		vector<vector<Point3f>> mvCandilinesWpoints;
		vector<Point3f> mvBestlineWpoints;

		//Mode Choose
		bool mbObstacle, mbBlindway;
		eLineTrackMode meLineTrackMode;

		int FramesNum,FramesNumLast;

		//左右图像匹配
		vector<int> vnMatches12;

		//障碍物坐标
		std::vector<pair<cv::Point3f, int>> obstacle;

		Mat im, im_r;
		Mat showimage;
		Mat homography;
		Mat CameraR, CameraT;

		//相机坐标系下地面方程
		cv::Point3f CPlane;

		std::vector<MapPoint*> mvpMapPoints;

		KeyFrame* mCurrentKeyFrame;

		Detection(ORB_SLAM2::Map* World);
		void Run();
		//相机坐标系下地面坐标
		std::vector<cv::Point3f> CGroundPoints;
		//相机坐标系下盲道参数
		std::vector<cv::Point3f> blindwaypar;

		//Stop
		void RequestStop();
		bool isStopped();
		bool Stop();
		void Release();
		bool mbStopRequested;
		bool mbStopped;
		boost::mutex mMutexStop;

		vector<cv::Point3f> GetBlindwayTrackLine();
		vector<cv::Point3f> GetBrickTrackLine();

		void ResetWPlane();
		bool mbResetWPlane;
		boost::mutex mMutexResetWPlane;

		void setSelectNeedBool(bool SelectNeed_);
		bool getSelectNeedBool();
		boost::mutex mMutexSelectNeedBool;

		void setRestartNeedBool(bool RestartNeed_);
		bool getRestartNeedBool();
		boost::mutex mMutexRestartNeedBool;
	};
}
#endif