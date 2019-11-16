#ifndef OBSTACLERECONSTRUCTION_H
#define OBSTACLERECONSTRUCTION_H

#include "path_planning.h"
#include "Map.h"
#include "ReadNAV982.h"
#include "Detection.h"

namespace ORB_SLAM2
{
	class KeyFrame;
	class ObstacleReconstruction
	{
	public:
		PathPlan path_plan;

		Vector3d X_i, X_j_prime, X_j, Z;
		Matrix3d P_i, P_j_prime, P_j, R, Q, K;
		float r;

		//��Ҫ��־λ

		eRouteMode meRouteMode;
		bool mbDestination;
		bool mbRoute;

		bool ReconstructInitflag;
		Point2d Originbaidu;

		double mYaw;

		bool mbSLAMWork;
		bool mbGPSWork;
		unsigned int mGPSState;

		Point2d mCurrentGPSXY;
		Point2d mCurrentPathplanXY;
		
		Mat mDeltaXYZ;
		Mat mSLAMXYZ;
		Mat R_cg, T_cg;
		Mat R_wg, T_wg;

		vector<Point3f> g_obstacle;	//�涯����ϵ���ϰ�������
		vector<Point3f> g_dynamic_obstacle;	//�涯����ϵ�¶�̬�ϰ�������
		vector<Point3f> g_blindwaypar; //�涯����ϵ��ä������
		vector<cv::Point3f> Dynamic_Obstacle;
		//ReadNAV982
		ReadNAV982* mpReader;
		Detection* mpDetection;
		ORB_SLAM2::Map* sWorld;

		vector<cv::Point3f> WGroundPoints;
		vector<pair<cv::Point3f, int>> mvObstacle;
		
		vector<Point3f> mvBlindwaypar;

		int FramesNum;
		int FramesNumLast;

		cv::Mat CPlane;
		cv::Mat WPlane;

		//��ǰ֡λ��
		cv::Mat mCurrentCameraPose;
		cv::Mat mCurrentCameraR;
		cv::Mat mCurrentCamerat;

		//AbsScale();
		ObstacleReconstruction(ORB_SLAM2::Map* World);
        //void Run();
		void Run(/*ORB_SLAM::FramePublisher* FramePub, ORB_SLAM::Map* World, vector<cv::Point3f> &Obstacle, vector<cv::Point3f> &WGroundPoints*/);

		void SetReader(ReadNAV982* pReader);
		void SetDetector(Detection* pDetection);

		//Stop
		void RequestStop();
		bool isStopped();
		bool Stop();
		void Release();
		bool mbStopRequested;
		bool mbStopped;
		boost::mutex mMutexStop;

		void SetCurrentCameraPose(const cv::Mat &Tcw);
		void GetCurrentCameraPose();

		//����Ӧ��ֵ����
		int Otsu(int data[ROOM][ROOM]);

		//Get WGroundPoints
		vector<cv::Point3f> GetWGroundPoints(vector<ORB_SLAM2::MapPoint*> KPs);

		//Get WGroundPoints
		void GetWGroundPoints(ORB_SLAM2::KeyFrame* KFs, vector<cv::Point3f>& WGroundPoints);

		//�ж�slam��gps�Ƿ������Լ���ȡ����
		void GetSlamGpsWorkStatusAndData(vector<ORB_SLAM2::KeyFrame*> KFs);

		//�����ϰ��������������ϵ�е�����
		//vector<pair<cv::Point3f, int>> CalculateObstacleWorldPoints(vector<ORB_SLAM2::KeyFrame*> KFs, int startFrame);
		void CalculateObstacleWorldPoints(vector<ORB_SLAM2::KeyFrame*> KFs, int startFrame, vector<pair<cv::Point3f, int>>& ObPoints,vector<cv::Point3f>& Dynamic_ObPoints);

		//����ä������������ϵ�еĲ���
		vector<cv::Point3f> CalculateBlindwayWorldPars(vector<ORB_SLAM2::KeyFrame*> KFs, int startFrame);

		//������������ϵ���涯����ϵ��ת��RT
		void CalculateWorld2GroundRT();

		//���ϰ����ת����դ������ϵ��ȥ�ӵ�
		void TransferObstaclePoints2Grid(int flagBarrier[ROOM][ROOM]);

		//��ä��ת����դ������ϵ
		void TransferBlindwayPar2Grid(Point3f &blindwayPar);

		boost::mutex mMutexCurrentCamera;
	};
}
#endif