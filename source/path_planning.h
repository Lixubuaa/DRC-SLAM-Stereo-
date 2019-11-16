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

//定义栅格尺寸、像素单位
#define ROOM 27//栅格个数--公用
#define LENGTH 200//250//150//250  //栅格边长，单位为mm   27*20mm=5.4m
#define SCREEN_X 621//左图边长像素
#define SCREEN_Y 621
#define SCREEN_GAP (SCREEN_X/ROOM)//左图一栅格边长的像素个数
#define SCREEN_XX 621//右图边长像素
#define SCREEN_YY 621
#define SCREEN_GAP2 (3)//右图小方框内一栅格边长的像素个数
#define SCREEN_SIDE (ROOM*SCREEN_GAP2)//右图小方框的边长像素数
#define BLAND_SIDE  floor(400/LENGTH)//盲道中线周围裕度
#define INF 1000
#define PI 3.14159265
#define hu2du(x) (x*180/PI)
#define du2hu(x) (x*PI/180)
enum class eRouteMode{ Input, School, Beihang, Circle, Straight, Blindway };
extern Mat pathimage;

class PathPlan
{
public:
	int pGrid;// 621/2/5/12/2--240约2.5m切换一次  //13约47.7m切换一次//30约20.7切换一次//16约18*2米切换一次
	int PlanningMode;
	bool transfer_map;
	bool drawpath;

	//测试部分——路径轨迹参数设定
	Point2d g_d1;//当前点相对坐标
	double g_circle_R;//半径10m	
	double g_line_R;
	Point2d g_d2;
	Point2d d_south;//南
	Point2d d_north;//北
	Point2d d_west;//西
	Point2d d_east;
	Point2d d_line_west;//西
	Point2d d_line_east;
	//Point2d{ circle_R, 0 };//东
	Point2d g_d1_last;
	Point2d g_centerxy;//默认为图中心
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

	bool mGlobalPath;//是否有全局路径
	bool mTestPathOrigin;//是否有全局路径
	bool initdraw;
	bool flag_blindline;//是否仅沿着盲道走直线
	
	//栅格——左图
	int startP[2];//起点
	int stopP[2];//终点
	int vs;
	int vp;
	int flagBarrier[ROOM][ROOM];//障碍物
	int dynamic_flagBarrier[ROOM][ROOM];//障碍物
	int barriertemp[ROOM][ROOM];//临时障碍物
	int flagBarrier_preview[ROOM][ROOM];//动态障碍物位置预测
	double alltemp[ROOM][ROOM];
	int flagBlind[ROOM][ROOM];
	int flagBlind1[ROOM][ROOM];//盲道中心线
	int flagBlind2[ROOM][ROOM];//盲道扩展线
	int vectorP[ROOM*ROOM][2];//栅格坐标对应的序号（0为左下角点）
	int prev[ROOM*ROOM];//存储路径点
	Point3f abcBlind;//盲道中心线3个系数-已转换至栅格坐标系
	double trainingP[ROOM*ROOM];

	//全局——右图
	double nowCoor[2];//当前点位置
	double goalCoor[2];//目标点位置
	Point2d P1;//预瞄点1坐标
	Point2d P2;//预瞄点2坐标

	//分段路径直线方程求解及判断
	vector<Point2d> Current_XY;
	Point2d BreakPoint;//右图切换时的当前点坐标
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

	//预瞄计算
	double PreviewR;//设置预瞄半径为3m？比例尺如何？
	double R;
	//导航方向计算
	double direction;
	float direct_goal2now;//画图判断用
	float grid_goal2now;//绘制栅格图中导航方向（蓝线）,并在无兴趣对象时直接输出该角度作为err
	//是否使用路径规划算法的标志位
	bool group_flag;
	//路径误差角计算
	int group;
	double group_angle[500];//存储各个时刻的路径误差角（时刻由group计数）
	double path_angle1;
	double path_angle;//当前时刻路径误差角---本模块最终输出结果

	int flag;

	//存储数据
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
	void Singleline_Preview(Point3f abcline);//预瞄盲道线

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