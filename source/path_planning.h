#ifndef PH_PLAN_H_
#define PH_PLAN_H_

#include <graphics.h>
#include <conio.h>
#include <iostream>
#include <stdlib.h> 
#include <math.h>
#include <fstream>
#include <Eigen/Dense>
#include "GPS_GlobalDataFuction.h"
#include "generalDataFuction.h"

using namespace std;
using namespace cv;

//����դ��ߴ硢���ص�λ
#define ROOM 27//դ�����--����
#define LENGTH 200//250//150//250  //դ��߳�����λΪmm   27*20mm=5.4m
#define SCREEN_X 621//��ͼ�߳�����
#define SCREEN_Y 621
#define SCREEN_GAP (SCREEN_X/ROOM)//��ͼһդ��߳������ظ���
#define SCREEN_XX 621//��ͼ�߳�����
#define SCREEN_YY 621
#define SCREEN_GAP2 (3)//��ͼС������һդ��߳������ظ���
#define SCREEN_SIDE (ROOM*SCREEN_GAP2)//��ͼС����ı߳�������
#define BLAND_SIDE  floor(400/LENGTH)//ä��������Χԣ��
#define INF 1000
#define PI 3.14159265
#define hu2du(x) (x*180/PI)
#define du2hu(x) (x*PI/180)
enum class eRouteMode{ Input, School, Beihang, Circle, Straight, Blindway };
extern Mat pathimage;

class PathPlan
{
public:
	int pGrid;// 621/2/5/12/2--240Լ2.5m�л�һ��  //13Լ47.7m�л�һ��//30Լ20.7�л�һ��//16Լ18*2���л�һ��
	int PlanningMode;
	bool transfer_map;
	bool drawpath;

	//���Բ��֡���·���켣�����趨
	Point2d g_d1;//��ǰ���������
	double g_circle_R;//�뾶10m	
	double g_line_R;
	Point2d g_d2;
	Point2d d_south;//��
	Point2d d_north;//��
	Point2d d_west;//��
	Point2d d_east;
	Point2d d_line_west;//��
	Point2d d_line_east;
	//Point2d{ circle_R, 0 };//��
	Point2d g_d1_last;
	Point2d g_centerxy;//Ĭ��Ϊͼ����
	bool g_goal_change;
	vector<Point2d> GoalXY;
	bool aim_change;
	int pathcount;
	double radius;

	PathPlan();
	Point2d GetOriginbaiduPoint(Point3d IniLatLonHeight, string destination, eRouteMode meRouteMode);
	void OverallPathPlanning(bool mbRoute, eRouteMode meRouteMode, Point2d d1_origin, double angle, Point3f abcBlind);
	double LocalPathPlanning(int FlagBarrier[ROOM][ROOM], Point3f abcBlind);
	void DrawingPathPlanWin(bool mbRoute, eRouteMode meRouteMode, Mat R_cg, Mat T_cg, double voltage, double scope,vector<Point3f> g_blindwaypar);
	Point3f PointTransform(Point3f m);
	Point3f BlindLineTransform(vector<Point3f> m);
	int ParallelLineJudge(vector<vector<Point3f>> linepoints, bool breakpointFlag);
	void GetCameraHeadingFromRot_C(cv::Mat &CurrentCameraR, double &headangle);

private:

	IMAGE imgNESW;

	bool mGlobalPath;//�Ƿ���ȫ��·��
	bool mTestPathOrigin;//�Ƿ���ȫ��·��
	bool initdraw;
	bool flag_blindline;//�Ƿ������ä����ֱ��
	
	//դ�񡪡���ͼ
	int startP[2];//���
	int stopP[2];//�յ�
	int vs;
	int vp;
	int flagBarrier[ROOM][ROOM];//�ϰ���
	int dynamic_flagBarrier[ROOM][ROOM];//�ϰ���
	int barriertemp[ROOM][ROOM];//��ʱ�ϰ���
	int flagBarrier_preview[ROOM][ROOM];//��̬�ϰ���λ��Ԥ��
	double alltemp[ROOM][ROOM];
	int flagBlind[ROOM][ROOM];
	int flagBlind1[ROOM][ROOM];//ä��������
	int flagBlind2[ROOM][ROOM];//ä����չ��
	int vectorP[ROOM*ROOM][2];//դ�������Ӧ����ţ�0Ϊ���½ǵ㣩
	int prev[ROOM*ROOM];//�洢·����
	Point3f abcBlind;//ä��������3��ϵ��-��ת����դ������ϵ
	double trainingP[ROOM*ROOM];

	//ȫ�֡�����ͼ
	double nowCoor[2];//��ǰ��λ��
	double goalCoor[2];//Ŀ���λ��
	Point2d P1;//Ԥ���1����
	Point2d P2;//Ԥ���2����

	//�ֶ�·��ֱ�߷�����⼰�ж�
	vector<Point2d> Current_XY;
	Point2d BreakPoint;//��ͼ�л�ʱ�ĵ�ǰ������
	vector<Point2d> RelativeGoal_XY;
	double path_k;
	double path_b;
	Point2d previewP;
	Point2d para1;
	Point2d para2;

	bool changegoal;
	bool neargoal;
	bool over_road;
	vector<Point2d> linekb;

	//Ԥ�����
	double PreviewR;//����Ԥ��뾶Ϊ3m����������Σ�
	double R;
	//�����������
	double direction;
	float direct_goal2now;//��ͼ�ж���
	float grid_goal2now;//����դ��ͼ�е����������ߣ�,��������Ȥ����ʱֱ������ýǶ���Ϊerr
	//�Ƿ�ʹ��·���滮�㷨�ı�־λ
	bool group_flag;
	//·�����Ǽ���
	int group;
	double group_angle[500];//�洢����ʱ�̵�·�����ǣ�ʱ����group������
	double path_angle1;
	double path_angle;//��ǰʱ��·������---��ģ������������

	int flag;

	//�洢����
	ofstream pathplan;
	ofstream gridresult;
	ofstream gridresult_err;
	ofstream ferr;
	ofstream back_pathXY;
	ofstream goal_xy;
	ofstream previewP_xy;
	ofstream parallelP_xy;
	ofstream current_xy;
	ofstream breakpoint;

	vector<Point2d> lineCalculate(vector<Point2d> Goal);
	bool OverRoadJudge(Point2d d1, int count, vector<Point2d>line);
	Point2d LinePreviewFollower(Point2d d1, Point2d d2, double path_k, double path_b, bool aim_change);
	Point2d LinePreviewParallel(Point2d d1, Point2d d2, double path_k, bool aim_change);
	Point2d CirclePreviewFollower(Point2d d1, Point2d d2, double circle_R, Point centerxy, Point d1_last, int count);
	void LocalTargetCalculate(double angle);
	void Singleline_Preview(Point3f abcline);//Ԥ��ä����

	void ClearGrid(void);
	void InitGrid(int FlagBarrier[ROOM][ROOM], Point3f abcBlind);
	bool WhetherUsePlanning(void);
	void ArtificialPotentialField(void);
	double WAstarAlgorithm(void);
	void SavingTestData(double err);

	void BackProjection(Point3f conv_m, Mat R_cg, Mat T_cg, Mat pathimage);

	//vector<Point2d> goalCalculate(Point2d d1, vector<Point2d> Goal, vector<Point2d> line);
	//bool goal_Judge(Point2d d1, int count, vector<Point2d>line, vector<Point2d> Goal);
	//Point3f parallel_line_Judge(Point2d d1, vector<Point3f> lineXYZ, double path_k);
	//int parallel_line_Judge(Point2d d1, Point2d d2, vector<vector<Point3f>> linepoints, double path_k);
	//void Singleline_Preview(Point3f abcline, Point3f bm1, Point3f bm2);
};
#endif