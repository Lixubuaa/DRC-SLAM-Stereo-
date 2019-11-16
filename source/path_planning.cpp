#include "stdafx.h"
#include "path_planning.h"
#include "baidumapGPSpath.h"

Mat pathimage;

PathPlan::PathPlan() :
transfer_map(false), drawpath(false), g_d1({ 0, 0 }), g_d2({ 0, 0 }), g_d1_last({ 0, 0 }), g_centerxy({ 0, 0 }), g_goal_change(0),
	mGlobalPath(0), mTestPathOrigin(0), P1({ 0, 0 }), P2({ 0, 0 }), pathcount(0), BreakPoint({ 0, 0 }), path_k(0), path_b(0), previewP({ 0, 0 }), para1({ 0, 0 }),para2({ 0, 0 }),
	aim_change(false), changegoal(false), neargoal(false), over_road(false), R(0), group(0), path_angle1(0), path_angle(0),
	pGrid(16), g_circle_R(5), g_line_R(20), PreviewR(4), group_flag(1), GoalXY(0), initdraw(false), radius(3.0), flag_blindline(false), PlanningMode(0), flag(0)
{
	d_south = Point2d{ 0, -1 * g_circle_R };//��
	d_north = Point2d{ 0, g_circle_R };//��
	d_west = Point2d{ -1 *g_circle_R, 0 };//��
	d_east = Point2d{  g_circle_R, 0 };
	d_line_west = Point2d{ -1 * g_line_R, 0 };//��
	d_line_east = Point2d{ g_line_R, 0 };
	startP[0] = floor(ROOM / 2);//���
	startP[1] = 0;//���
	stopP[0] = floor(ROOM / 2);//�յ�
	stopP[1] = ROOM - 1;//�յ�
	for (int i = 0; i < ROOM*ROOM; i++)
		prev[i] = 0;
}

Point2d PathPlan::GetOriginbaiduPoint(Point3d IniLatLonHeight, string destination, eRouteMode meRouteMode)
{
	//��㾭γ��
	CString originLatGPS;
	originLatGPS.Format("%f", IniLatLonHeight.x * RADIANS_TO_DEGREES);
	CString originLonGPS;
	originLonGPS.Format("%f", IniLatLonHeight.y * RADIANS_TO_DEGREES);

	vector<Point2d> GoalLatLon;

	//�õ�����·��
	if (meRouteMode == eRouteMode(0)){
		GoalLatLon = getBaiduBreaks(originLatGPS, originLonGPS, destination.c_str());	//�ٶȹ滮·��
	}
	else
	{
		vector<Point2d> GoalLatLon_origin;
		if (meRouteMode == eRouteMode(2)){
			GoalLatLon_origin = getBeihangFixStartEnd();		//���������̶�·��(google�ֶ�ȡ)����baidumapGPSpath��(0227)
		}
		if (meRouteMode == eRouteMode(1)){
			GoalLatLon_origin = getSchoolFixStartEnd();		//äУ�����̶�·��(google�ֶ�ȡ)����baidumapGPSpath��(0227)
		}
		vector<Point2d> line = getStartEndline(GoalLatLon_origin);//0227ԭʼ�趨·��ֱ������k,bϵ��
		Point2d LatLonHeight = getBaiduLatLon(IniLatLonHeight);
		GoalLatLon = getCurrentGoalLatLon(Point3d(LatLonHeight.x, LatLonHeight.y, 0), line, GoalLatLon_origin);//0227�жϵ�ǰλ������ֱ��,���¼���滮·���㼯
	}

	GoalXY.clear();
	for (size_t i = 0; i < GoalLatLon.size(); i++)
	{
		Point2d adjustXY = ConvertLatLon2XY(GoalLatLon[0].x*DEGREES_TO_RADIANS, GoalLatLon[0].y*DEGREES_TO_RADIANS,
			GoalLatLon[i].x*DEGREES_TO_RADIANS, GoalLatLon[i].y*DEGREES_TO_RADIANS);
		GoalXY.push_back(Point2d(adjustXY.x, adjustXY.y + 0));
	}

	goal_xy.open(data + "/RelativeGoal_XY.txt", ios::app);   //����һ���ļ�
	goal_xy << "GoalXY=" << GoalXY.size() << endl;
	for (size_t i = 1; i <GoalXY.size(); i++)
	{
		goal_xy << "i-1 = " << GoalXY[i - 1] << endl;
	}
	goal_xy.close();

	cout << "Path chosen completed!!!" << endl;

	return GoalLatLon[0];
}


//************1.ȫ��·���滮����*************
void PathPlan::OverallPathPlanning(bool mbRoute, eRouteMode meRouteMode, Point2d d1_origin, double angle, Point3f abcBlind)//�������ݻ�δ���뺯��0508  angle��yaw?
{
	if (mbRoute == 0){
		PlanningMode = 2;
		return;//ɢ��ģʽ+����ϰ��ä��ģʽ: Ĭ��;������ä����ֱ�ߣ�ͬһģʽ
	}

	//��ȡ·���۵�
	if (meRouteMode == eRouteMode(0) || meRouteMode == eRouteMode(1) || meRouteMode == eRouteMode(2)){// &&mbDestination == 1

		//���뵱ǰ�㡢��ǰ�ֶ�·��Ŀ���
		if (transfer_map){
			g_d1.x = d1_origin.x - BreakPoint.x;//��ǰ������;BreakPoint��ʼΪ0�����ʵ��
			g_d1.y = d1_origin.y - BreakPoint.y;
			Current_XY.push_back(g_d1);
			if (!drawpath)///drawpathʹ��ֻ�ڵ�һ֡�棬���ÿ֡ͼ�����»�һ��
			{
				Current_XY.clear();
				RelativeGoal_XY.clear();
				for (size_t i = 0; i < GoalXY.size(); i++)
				{
					RelativeGoal_XY.push_back(Point2d(GoalXY[i].x - BreakPoint.x, GoalXY[i].y - BreakPoint.y));
				}
				drawpath = true;
			}
			g_d2 = RelativeGoal_XY[pathcount + 1];//���㵼�������Ŀ������꣺�������Ϊ����ԭ����ƣ�Ȼ��γת��ΪXY��
			
		}
		else{
			g_d1 = d1_origin;
			Current_XY.push_back(g_d1);//�ѹ�ȥ�ĵ�ǰ�㱣����Current_XY��,�ٴ��͵�initwin�л�
			if (!drawpath && GoalXY.size() != 0)///drawpathʹ��ֻ�ڵ�һ֡�棬���ÿ֡ͼ�����»�һ��
			{
				RelativeGoal_XY.clear();
				for (size_t i = 0; i < GoalXY.size(); i++)
				{
					RelativeGoal_XY.push_back(Point2d(GoalXY[i].x, GoalXY[i].y));
				}
				drawpath = true;
			}
			g_d2 = RelativeGoal_XY[pathcount + 1];
		}

		//�洢ֱ�߷���ϵ��
		linekb = lineCalculate(RelativeGoal_XY);
		path_k = linekb[pathcount].x;
		path_b = linekb[pathcount].y;


		//����Ԥ���
		if (!neargoal){
			over_road = OverRoadJudge(g_d1, pathcount, linekb);
			//if (pathcount == linekb.size() - 1 || over_road){//�������һ����||����·width||��width�ڵ���δ�ﵽwidthmin
			if (over_road){
				previewP = LinePreviewFollower(g_d1, g_d2, path_k, path_b, aim_change);//�������·���ϵ�Ԥ���
			}
			else {
				previewP = LinePreviewParallel(g_d1, g_d2, path_k, aim_change);//ƽ�з�������Ԥ���
			}
			aim_change = false;
		}
		else{//�ھ�������Ŀ�ĵغܽ�ʱ����������Ŀ�����Ϊ������stop
			previewP = g_d2;
		}

		//�ж��Ƿ��л��ֶ�����Ŀ���
		bool arriveflage = ISArriveDestination(g_d2, g_d1, radius);//�ɳ����ϰ���
		if (arriveflage)
		{
			if (pathcount == linekb.size() - 1){
				neargoal = 1;
				if (ISArriveDestination(g_d2, g_d1, 1.0)){
					aim_change = true;
					pathcount++;//�л�����һĿ���
				}
			}
			else{
				aim_change = true;
				pathcount++;//�л�����һĿ���
			}
		}
		else if (changegoal){
			aim_change = true;
			pathcount++;//�л�����һĿ���
		}


	}

	if (meRouteMode == eRouteMode(3) || meRouteMode == eRouteMode(4) || meRouteMode == eRouteMode(5)){
		//����ʵ��--����λ�õ�Բ��ֱ�����߹켣����

		if (meRouteMode == eRouteMode(3)){//Բ����Ϊ���
			if (!mTestPathOrigin){
				g_d2 = d_south;
				mTestPathOrigin = 1;
			}
			
			g_d1 = d1_origin+d_north;
			Current_XY.push_back(g_d1);
			drawpath = true;

			if (!g_goal_change){//��־��ʼֵΪ0��ֻ�任һ��
				if (ISArriveDestination(g_d2, g_d1, 1.0)){
					g_d2 = d_north;
					g_goal_change = 1;
					pathcount++;
				}
			}
			//����Ԥ���
			if (g_goal_change && ISArriveDestination(g_d2, g_d1, 1.0)){//�ھ�������Ŀ�ĵغܽ�ʱ����������Ŀ�����Ϊ������stop
				previewP = g_d2;
			}
			else{
				previewP = CirclePreviewFollower(g_d1, g_d2, g_circle_R, g_centerxy, g_d1_last, pathcount);
			}
			g_d1_last = previewP;

		}
		if (meRouteMode == eRouteMode(4)){//ֱ��:�Զ�����

			g_d1 = d1_origin + d_line_east;
			Current_XY.push_back(g_d1);
			drawpath = true;

			//����Ԥ���
			if (ISArriveDestination(d_line_west, g_d1, 3.0)){//�ھ�������Ŀ�ĵغܽ�ʱ����������Ŀ�����Ϊ������stop
				previewP = d_line_west;
			}
			else{
				g_d2 = d_line_west;
				previewP = LinePreviewFollower(g_d1, g_d2, 0, 0, 0);
			}
		}
		if ( meRouteMode == eRouteMode(5)){//��ä����ֱ��
			//flag_blindline = true;
			PlanningMode = 2;
			//Singleline_Preview(abcBlind);
	
		}
	}

	//////���볯��ȷ���ֲ�Ŀ���/////
	if (PlanningMode == 0){
		LocalTargetCalculate(angle);
	}

}//end

//************2.�ֲ�·���滮����*************
double PathPlan::LocalPathPlanning(int FlagBarrier[ROOM][ROOM], Point3f abcBlind)
{
	double err;
	ClearGrid();
	InitGrid(FlagBarrier, abcBlind);//��ʼ��

	
	//if (!flag_blindline){//flag_blindline=0����ԭ����ʽ����Ԥ���·���͹滮��flag_blindline=1����Ԥ��ä��
	//	PlanningMode = 0;
	//}

	//else{//����ä���߽�����ͼ������
	//	PlanningMode = 1;
	//}

	bool err_flag = 0;
	switch (PlanningMode){
	   case 0://�д�·��
		   err_flag = WhetherUsePlanning();
		   if (!err_flag){
			   err = du2hu(grid_goal2now);
		   }
		   else{
			   err = WAstarAlgorithm();//��������ֵΪpath_angle����err
		   }
		   break;

	   case 1:
		   err = -(atan2((stopP[0] - startP[0]), stopP[1]));
		   if (abs(hu2du(err)) < 7)
			   err = 0;
		   break;

	   case 2://ɢ��ģʽ����ä������//����ä����ֱ��
		   int count_b = 0;
		   //********����ä��ͬʱ����ʱ�����ϰ���ǰ��ʱ����ä���������ϰ���+ä�� ҡ�ڣ�
		   for (int i = 7; i < ROOM - 7; i++){//��
			   for (int j = 2; j < 9; j++){//��
				   if (flagBarrier[i][j]){
					   count_b++;//�ж��ϰ������һ����Χ�ڣ�������ǰ����
				   }
			   }
		   }

		   if (count_b > 0){//����һ���������
		//if (count_b > 0 && group_angle[group - 1] != 0){
			   if (flag == 0){
				  int temp= WAstarAlgorithm();
			   }
			   flag++;

			   group_angle[group] = group_angle[group - 1];//������һ֡
			   if (abs(hu2du(group_angle[group])) < 10){
				   if (group_angle[group]>0)
					   group_angle[group] = du2hu(12);//�����ϰ���ʱ��������ϽǶ�С��10�ȣ���=10��
				   else
					   group_angle[group] = -du2hu(12);
			   }
			   group++;
			   err=group_angle[group - 1];
		   }
		   else{
			   flag = 0;
			   Singleline_Preview(abcBlind);//�����µľֲ�Ŀ���
			   int temp = WAstarAlgorithm();//Ϊ�˴洢group_angle
			   err = -(atan2((stopP[0] - startP[0]), stopP[1]));
			   if (abs(hu2du(err)) < 11)//ԭ��Ϊ5������������ä����ҡ��
				   err = 0;
		   }
		   path_angle = err;
		   break;

	}

	SavingTestData(err);
	return err;
}//end

//************3.��ͼ����*************
void PathPlan::DrawingPathPlanWin(bool mbRoute, eRouteMode meRouteMode, Mat R_cg, Mat T_cg, double voltage, double scope, vector<Point3f> g_blindwaypar)
{
	double t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10;

	if (!initdraw){
		// ��ͼ������ʼ��
		initgraph(SCREEN_X + SCREEN_XX, SCREEN_Y, SHOWCONSOLE);
		HWND hwnd_pathplan = GetHWnd();
		SetWindowPos(hwnd_pathplan, HWND_TOP, 640, 0, 1242, 621, SWP_ASYNCWINDOWPOS | SWP_DEFERERASE | SWP_NOSIZE);

		initdraw = true;
	}
	//setbkcolor(RGB(200, 200, 200));
	//// ���ñ���ɫΪ��ɫ
	setbkcolor(RGB(235, 235, 235));
	// ���ñ���ɫΪ��ɫ
	cleardevice();
	// �ñ���ɫ�����Ļ
	loadimage(&imgNESW, _T("../DATA/img/NESW.jpg"), SCREEN_GAP*2.5, SCREEN_GAP*2.5, TRUE);
	// ��ȡͼƬ����ͼ����
	
	if (PlanningMode==0){

	t0 = (double)clock() / CLOCKS_PER_SEC;
	//**************draw right!!!!!****************
	setorigin(SCREEN_X, 0);
	//Part1ȫ��·��
	setlinecolor(RED);
	setlinestyle(PS_SOLID, 3);
	if (mbRoute == 1){
		if (meRouteMode == eRouteMode(0) || meRouteMode == eRouteMode(1) || meRouteMode == eRouteMode(2))
		{
			goal_xy.open(data + "/RelativeGoal_XY.txt", ios::app);   //����һ���ļ�
			goal_xy << "size=" << RelativeGoal_XY.size() << endl;
			for (size_t i = 1; i <RelativeGoal_XY.size(); i++)
			{
				line(RelativeGoal_XY[i].x*pGrid + SCREEN_X / 2, SCREEN_Y / 2 - RelativeGoal_XY[i].y*pGrid,
					RelativeGoal_XY[i - 1].x*pGrid + SCREEN_X / 2, SCREEN_Y / 2 - RelativeGoal_XY[i - 1].y *pGrid);
				solidcircle(RelativeGoal_XY[i].x*pGrid + SCREEN_X / 2, SCREEN_Y / 2 - RelativeGoal_XY[i].y*pGrid, 5);
				goal_xy << "i-1 = " << RelativeGoal_XY[i - 1] << endl;
			}
			goal_xy.close();
		}
		if (meRouteMode == eRouteMode(3))
			circle(g_centerxy.x + SCREEN_X / 2, g_centerxy.y + SCREEN_X / 2, g_circle_R*pGrid);//����Բ��·��
		if (meRouteMode == eRouteMode(4) || meRouteMode == eRouteMode(5)){
			line(d_line_east.x*pGrid + SCREEN_X / 2, SCREEN_Y / 2 - d_line_east.y*pGrid,
				d_line_west.x*pGrid + SCREEN_X / 2, SCREEN_Y / 2 - d_line_west.y*pGrid);
			setfillcolor(RED);
			solidcircle(d_line_west.x*pGrid + SCREEN_X / 2, SCREEN_Y / 2 - d_line_west.y*pGrid, 4);//����ֱ��·��
		}
	}

	//Part2�ֲ�����
	int pts1[] = {
		g_d1.x*pGrid + SCREEN_X / 2 + SCREEN_SIDE / 2 * cos((direction + 90)*PI / 180), -g_d1.y*pGrid + SCREEN_Y / 2 - SCREEN_SIDE / 2 * sin((direction + 90)*PI / 180),
		g_d1.x*pGrid + SCREEN_X / 2 + SCREEN_SIDE / 2 * cos((direction + 90)*PI / 180) + SCREEN_SIDE*cos(direction*PI / 180), -g_d1.y*pGrid + SCREEN_Y / 2 - SCREEN_SIDE / 2 * sin((direction + 90)*PI / 180) - SCREEN_SIDE*sin(direction*PI / 180),
		g_d1.x*pGrid + SCREEN_X / 2 + SCREEN_SIDE / 2 * cos((direction - 90)*PI / 180) + SCREEN_SIDE*cos(direction*PI / 180), -g_d1.y*pGrid + SCREEN_Y / 2 - SCREEN_SIDE / 2 * sin((direction - 90)*PI / 180) - SCREEN_SIDE*sin(direction*PI / 180),
		g_d1.x*pGrid + SCREEN_X / 2 + SCREEN_SIDE / 2 * cos((direction - 90)*PI / 180), -g_d1.y*pGrid + SCREEN_Y / 2 - SCREEN_SIDE / 2 * sin((direction - 90)*PI / 180) };

	setlinecolor(WHITE);//���α߿�����Ϊ��ɫ
	setlinestyle(PS_ENDCAP_FLAT, 2);
	setfillcolor(RGB(200, 200, 200));
	fillpolygon((POINT*)pts1, 4);//��ת

	//Part3Ԫ��
	setfillcolor(LIGHTGREEN);
	setlinestyle(PS_ENDCAP_FLAT, 1);
	int pts3[8] = { 0 };
	for (int i = 0; i<ROOM; i++)
	{
		for (int j = 0; j<ROOM; j++)
		{
			if (flagBlind[i][j] == 1)//ä��
			{
				pts3[0] = pts1[0] + SCREEN_GAP2*(double(i)*sin(direction*PI / 180) + double(j)*cos(direction*PI / 180));
				pts3[1] = pts1[1] - SCREEN_GAP2*(double(i)*-cos(direction*PI / 180) + double(j)*sin(direction*PI / 180));

				pts3[2] = pts1[0] + SCREEN_GAP2*(double(i + 1)*sin(direction*PI / 180) + double(j)*cos(direction*PI / 180));
				pts3[3] = pts1[1] - SCREEN_GAP2*(double(i + 1)*-cos(direction*PI / 180) + double(j)*sin(direction*PI / 180));

				pts3[4] = pts1[0] + SCREEN_GAP2*(double(i + 1)*sin(direction*PI / 180) + double(j + 1)*cos(direction*PI / 180));
				pts3[5] = pts1[1] - SCREEN_GAP2*(double(i + 1)*-cos(direction*PI / 180) + double(j + 1)*sin(direction*PI / 180));

				pts3[6] = pts1[0] + SCREEN_GAP2*(double(i)*sin(direction*PI / 180) + double(j + 1)*cos(direction*PI / 180));
				pts3[7] = pts1[1] - SCREEN_GAP2*(double(i)*-cos(direction*PI / 180) + double(j + 1)*sin(direction*PI / 180));

				fillpolygon((POINT*)pts3, 4);
			}
		}
	}

	setfillcolor(LIGHTRED);
	int pts4[8] = { 0 };
	for (int i = 0; i<ROOM; i++)
	{
		for (int j = 0; j<ROOM; j++)
		{
			if (flagBarrier[i][j] == 1)//�ϰ�
			{
				pts4[0] = pts1[0] + SCREEN_GAP2*(double(i)*sin(direction*PI / 180) + double(j)*cos(direction*PI / 180));
				pts4[1] = pts1[1] - SCREEN_GAP2*(double(i)*-cos(direction*PI / 180) + double(j)*sin(direction*PI / 180));

				pts4[2] = pts1[0] + SCREEN_GAP2*(double(i + 1)*sin(direction*PI / 180) + double(j)*cos(direction*PI / 180));
				pts4[3] = pts1[1] - SCREEN_GAP2*(double(i + 1)*-cos(direction*PI / 180) + double(j)*sin(direction*PI / 180));

				pts4[4] = pts1[0] + SCREEN_GAP2*(double(i + 1)*sin(direction*PI / 180) + double(j + 1)*cos(direction*PI / 180));
				pts4[5] = pts1[1] - SCREEN_GAP2*(double(i + 1)*-cos(direction*PI / 180) + double(j + 1)*sin(direction*PI / 180));

				pts4[6] = pts1[0] + SCREEN_GAP2*(double(i)*sin(direction*PI / 180) + double(j + 1)*cos(direction*PI / 180));
				pts4[7] = pts1[1] - SCREEN_GAP2*(double(i)*-cos(direction*PI / 180) + double(j + 1)*sin(direction*PI / 180));

				fillpolygon((POINT*)pts4, 4);
			}
		}
	}

	//Part4ä�����߹켣
	setlinecolor(BLUE);
	setlinestyle(PS_SOLID, 3);
	current_xy.open(data + "/Current_XY1.txt", ios::app);   //����һ���ļ�
	current_xy << "size=" << Current_XY.size() << endl;
	double precision = 0.0;
	for (size_t i = 0; i <Current_XY.size(); i++)
	{
		if (i>0)
			line(Current_XY[i].x*pGrid + SCREEN_X / 2, SCREEN_Y / 2 - Current_XY[i].y*pGrid,//���Ƶ�ǰ��·��
			Current_XY[i - 1].x*pGrid + SCREEN_X / 2, SCREEN_Y / 2 - Current_XY[i - 1].y*pGrid);
		current_xy << "Current_XY[" << i << "] = " << Current_XY[i] << endl;

		if ((Current_XY[i].x*pGrid + SCREEN_X / 2) >(SCREEN_X - 10) || (Current_XY[i].x*pGrid + SCREEN_X / 2) < 10 ||
			(SCREEN_Y / 2 - Current_XY[i].y*pGrid) > (SCREEN_Y - 10) || (SCREEN_Y / 2 - Current_XY[i].y*pGrid) <10)//�Ա�Ե���ص����жϹ���
		{
			transfer_map = true;
			drawpath = false;
			BreakPoint = BreakPoint + Current_XY[i];
			breakpoint.open(data + "/BreakPoint.txt", ios::app);   //����һ���ļ�
			breakpoint << BreakPoint.x << "," << BreakPoint.y << endl;
			breakpoint.close();
		}
		precision = precision + Current_XY[i].y;
	}
	precision = precision / Current_XY.size();
	current_xy << "precision = " << precision;
	current_xy.close();

	//Part5��ǰλ�ã�����+Բ)
	int pts2[] = { g_d1.x*pGrid + SCREEN_X / 2 + 4 * SCREEN_GAP2 / 2 * cos((direction - 45)*PI / 180), -g_d1.y*pGrid + SCREEN_Y / 2 - 4 * SCREEN_GAP2 / 2 * sin((direction - 45)*PI / 180),
		g_d1.x*pGrid + SCREEN_X / 2 + 4 * SCREEN_GAP2*cos(direction*PI / 180), -g_d1.y*pGrid + SCREEN_Y / 2 - 4 * SCREEN_GAP2*sin(direction*PI / 180),
		g_d1.x*pGrid + SCREEN_X / 2 + 4 * SCREEN_GAP2 / 2 * cos((direction + 45)*PI / 180), -g_d1.y*pGrid + SCREEN_Y / 2 - 4 * SCREEN_GAP2 / 2 * sin((direction + 45)*PI / 180) };
	setlinecolor(WHITE);
	setlinestyle(PS_ENDCAP_FLAT, 2);
	setfillcolor(LIGHTBLUE);
	fillpolygon((POINT*)pts2, 3);
	fillcircle(g_d1.x*pGrid + SCREEN_X / 2, -g_d1.y*pGrid + SCREEN_Y / 2, 4 * SCREEN_GAP2 / 2);//���

	//Part6Ԥ����
	if (!neargoal &&!over_road && (meRouteMode == eRouteMode(0) || meRouteMode == eRouteMode(1) || meRouteMode == eRouteMode(2))){//����·��ƽ����
		setlinecolor(GREEN);
		setlinestyle(PS_DASH, 1);
		line(para1.x*pGrid + SCREEN_X / 2, SCREEN_Y / 2 - para1.y*pGrid,
			para2.x*pGrid + SCREEN_X / 2, SCREEN_Y / 2 - para2.y *pGrid);
	}
	//����Ԥ����Ԥ����
	setlinecolor(GREEN);
	setlinestyle(PS_SOLID, 2);
	line(g_d1.x*pGrid + SCREEN_X / 2, SCREEN_Y / 2 - g_d1.y*pGrid,
		previewP.x*pGrid + SCREEN_X / 2, SCREEN_Y / 2 - previewP.y *pGrid);
	solidcircle(previewP.x*pGrid + SCREEN_X / 2, SCREEN_Y / 2 - previewP.y*pGrid, 3);


	//Part7�ֲ�·���滮���
	if (prev[vp] != vs && prev[vp] != 0){
		//setlinestyle(PS_SOLID, 1);
		//setfillcolor(BROWN);
		//for (int i = vp; i != vs; i = prev[i])
		//{
		//	settextcolor(BROWN);
		//	setbkmode(TRANSPARENT);
		//	settextstyle(3, 0, _T("����"));//����·��
		//	TCHAR s[8];
		//	_stprintf(s, _T("%d"), int(0));
		//	outtextxy(int(SCREEN_GAP2*(vectorP[i][0]) + 0.5), int(SCREEN_GAP2*(ROOM - vectorP[i][1] - 1) + 0.5), s);

		//	//solidcircle(pts1[0] + SCREEN_GAP2*((double(vectorP[i][0]) + 0.5)*sin(direction*PI / 180) + (double(vectorP[i][1]) + 0.5)*cos(direction*PI / 180)),
		//	//	pts1[1] - SCREEN_GAP2*((double(vectorP[i][0]) + 0.5)*-cos(direction*PI / 180) + (double(vectorP[i][1]) + 0.5)*sin(direction*PI / 180)),
		//	//	SCREEN_GAP2 / 3);//����·��--ԭ��
		//}
		setlinecolor(RED);//������ͼ�еĺ�ɫ������
		setlinestyle(PS_SOLID, 2);
		moveto(g_d1.x*pGrid + SCREEN_X / 2, SCREEN_Y / 2 - g_d1.y*pGrid);
		linerel(int(50 * cos(path_angle + du2hu(direction))), int(-50 * sin(path_angle + du2hu(direction))));
	}

	}//��������ä���߲�����ͼ

	t1 = (double)clock() / CLOCKS_PER_SEC;
	//**************draw left!!!!!****************
	setorigin(0, 0);//��ʼ������֤��ͼ���������Ͻ�
	int pts10[] = { 0, 0, SCREEN_X, 0, SCREEN_X, SCREEN_Y, 0, SCREEN_Y };
	setlinecolor(WHITE);//���α߿�����Ϊ��ɫ
	setlinestyle(PS_ENDCAP_FLAT, 1);
	setfillcolor(RGB(235, 235, 235));
	setbkcolor(RGB(235, 235, 235));
	fillpolygon((POINT*)pts10, 4);//��ת

	//Part1��դ����
	setlinestyle(PS_SOLID, 1);
	setlinecolor(BLACK);
	for (int i = 0; i<ROOM; i++)
	{
		line(0, SCREEN_Y / ROOM*i, SCREEN_X, SCREEN_Y / ROOM*i);//����
		line(SCREEN_X / ROOM*i, 0, SCREEN_X / ROOM*i, SCREEN_Y);//����
	}
	line(SCREEN_X, 0, SCREEN_X, SCREEN_Y);
	putimage(SCREEN_X, 0, &imgNESW);//ָ����

	t2 = (double)clock() / CLOCKS_PER_SEC;
	//Part2Ԫ��
	for (int i = 0; i<ROOM; i++)//����ä��
	{
		for (int j = 0; j<ROOM; j++)
		{
			if (flagBlind1[i][j] == 1)
			{
				setfillcolor(GREEN);
				fillrectangle(SCREEN_X / ROOM*(i), SCREEN_Y / ROOM*(ROOM - j), SCREEN_X / ROOM*(i + 1), SCREEN_Y / ROOM*(ROOM - j - 1));
			}
			else if (flagBlind2[i][j] == 1)
			{
				setfillcolor(LIGHTGREEN);
				fillrectangle(SCREEN_X / ROOM*(i), SCREEN_Y / ROOM*(ROOM - j), SCREEN_X / ROOM*(i + 1), SCREEN_Y / ROOM*(ROOM - j - 1));
			}
		}
	}

	setfillcolor(LIGHTBLUE);//�������
	fillrectangle(SCREEN_GAP*(startP[0]), SCREEN_GAP*(ROOM - startP[1] - 1), SCREEN_GAP*(startP[0] + 1), SCREEN_GAP*(ROOM - startP[1]));

	t3 = (double)clock() / CLOCKS_PER_SEC;
	for (int i = 0; i<ROOM; i++)//�����ϰ����
	{
		for (int j = 0; j<ROOM; j++)
		{
			if (flagBarrier[i][j])
			{
				if (flagBarrier[i][j] == 2)
				{
					setfillcolor(GREEN);
					fillrectangle(SCREEN_GAP*(i), SCREEN_GAP*(ROOM - j), SCREEN_GAP*(i + 1), SCREEN_GAP*(ROOM - j - 1));
				}
				else{
					setfillcolor(RED);
					fillrectangle(SCREEN_GAP*(i), SCREEN_GAP*(ROOM - j), SCREEN_GAP*(i + 1), SCREEN_GAP*(ROOM - j - 1));
				}
				
			}
			else if (barriertemp[i][j] == 1){
				/*setfillcolor(LIGHTRED);
				fillrectangle(SCREEN_GAP*(i), SCREEN_GAP*(ROOM - j), SCREEN_GAP*(i + 1), SCREEN_GAP*(ROOM - j - 1));*/

				settextcolor(GREEN);
				setbkmode(TRANSPARENT);
				settextstyle(9, 0, _T("����"));//������ʱ�ϰ���
				TCHAR s[8];
				_stprintf(s, _T("%d"), int(4));
				outtextxy(int(SCREEN_GAP* i+ 10), int(SCREEN_GAP*(ROOM - j - 1) + 10), s);
			}
		}
	}
	t4 = (double)clock() / CLOCKS_PER_SEC;

	if (stopP[0] != -1 && stopP[1] != -1){//�����յ�
		setfillcolor(BLUE);
		fillrectangle(SCREEN_GAP*(stopP[0]), SCREEN_GAP*(ROOM - stopP[1] - 1), SCREEN_GAP*(stopP[0] + 1), SCREEN_GAP*(ROOM - stopP[1]));
	}

	if (PlanningMode!=0){
		//����ä����Ԥ����Ԥ����
		setorigin(0, 0);//��ͼ
		setlinecolor(YELLOW);
		setlinestyle(PS_SOLID, 3);
		Point3f m1 = PointTransform(g_blindwaypar[0]);
		Point3f m2 = PointTransform(g_blindwaypar[1]);
		line(m1.x * SCREEN_X / ROOM + SCREEN_X / ROOM / 2, (ROOM - m1.y) * SCREEN_Y / ROOM,
			m2.x * SCREEN_X / ROOM + SCREEN_X / ROOM / 2, (ROOM - m2.y) * SCREEN_Y / ROOM);

		setlinecolor(BLUE);
		setlinestyle(PS_SOLID, 3);
		line(startP[0] * SCREEN_X / ROOM + SCREEN_X / ROOM / 2, (ROOM - startP[1]) * SCREEN_Y / ROOM,
			stopP[0] * SCREEN_X / ROOM + SCREEN_X / ROOM / 2, (ROOM - stopP[1]) * SCREEN_Y / ROOM);
		solidcircle(stopP[0] * SCREEN_X / ROOM + SCREEN_X / ROOM / 2, (ROOM - stopP[1]) * SCREEN_Y / ROOM, 3);
	}

	//Part3������Ϣ
	if (direct_goal2now >= 90 && direct_goal2now <= 270)
	{
		settextcolor(LIGHTRED);
		settextstyle(26, 0, _T("����"));
		if (direct_goal2now <= 180)
		{
			RECT r = { 0, 0, SCREEN_X / 2, SCREEN_Y };
			drawtext(_T("Turn Left"), &r, DT_CENTER | DT_VCENTER | DT_SINGLELINE);//��ʾ����ת:��+
		}
		else
		{
			RECT r = { SCREEN_X / 2, 0, SCREEN_X, SCREEN_Y };
			drawtext(_T("Turn Right"), &r, DT_CENTER | DT_VCENTER | DT_SINGLELINE);//��ʾ����ת:��-
		}
	}
	setlinecolor(BLUE);//��ʾ��������
	setlinestyle(PS_SOLID, 3);
	moveto(startP[0] * SCREEN_X / ROOM + SCREEN_X / ROOM / 2, (ROOM - startP[1]) * SCREEN_Y / ROOM);
	linerel(int(400 * sin(-1 * du2hu(grid_goal2now))), int(-400 * cos(du2hu(grid_goal2now))));
	//��������Ϊ���Ͻǣ�������ƽ��Ϊ-������ƽ��ҲΪ- 

	t5 = (double)clock() / CLOCKS_PER_SEC;
	//Part4�ֲ�·���滮���
	if (prev[vp] != vs && prev[vp] != 0){
		setfillcolor(BROWN);
		Point3f conv_m = {};
		for (int i = vp; i != vs; i = prev[i])
		{
			settextcolor(BROWN);
			setbkmode(TRANSPARENT);
			settextstyle(9, 0, _T("����"));
			TCHAR s[8];
			_stprintf(s, _T("%d"), int(0));
			outtextxy(int(SCREEN_GAP*(vectorP[i][0]) + 5), int(SCREEN_GAP*(ROOM - vectorP[i][1] - 1) + 5), s);
			//solidcircle(SCREEN_X / ROOM*(vectorP[i][0]) + SCREEN_X / (2 * ROOM), SCREEN_X / ROOM*(ROOM - (vectorP[i][1])) - SCREEN_X / (2 * ROOM), SCREEN_X / (4 * ROOM));//����·��

			/*conv_m.x = vectorP[i][0];
			conv_m.y = vectorP[i][1];
			BackProjection(conv_m, R_cg, T_cg, pathimage);*///��ͶӰ
		}
		//waitKey(33);//opencv����ʱ����
		//string str = data_dir[3] + "/Pathimage";
		//string end = to_string(filenum) + ".jpg";
		//imwrite(str + end, pathimage);//�滮·����ͶӰ

		setlinecolor(RED);
		setlinestyle(PS_DASH, 3);//��ɫ������
		moveto(startP[0] * SCREEN_X / ROOM + SCREEN_X / ROOM / 2, (ROOM - startP[1]) * SCREEN_Y / ROOM);
		linerel(int(200 * sin(-1 * path_angle1)), int(-200 * cos(-1 * path_angle1)));

		setlinecolor(RED);
		setlinestyle(PS_SOLID, 4);//��ɫ��ʵ��
		moveto(startP[0] * SCREEN_X / ROOM + SCREEN_X / ROOM / 2, (ROOM - startP[1]) * SCREEN_Y / ROOM);
		linerel(int(300 * sin(-1 * path_angle)), int(-300 * cos(-1 * path_angle)));
	}

	if (PlanningMode == 2){//
		setlinecolor(RED);
		setlinestyle(PS_SOLID, 4);//��ɫ��ʵ��
		moveto(startP[0] * SCREEN_X / ROOM + SCREEN_X / ROOM / 2, (ROOM - startP[1]) * SCREEN_Y / ROOM);
		linerel(int(300 * sin(-1 * path_angle)), int(-300 * cos(-1 * path_angle)));
	}
	
	//Part5 PID���ƽ��:voltage
	setlinecolor(YELLOW);
	setlinestyle(PS_SOLID, 5);	//����ͼ�л���voltage
	line(SCREEN_GAP*(ROOM / 2) + SCREEN_GAP / 2, SCREEN_GAP*(ROOM - 1) + SCREEN_GAP / 2,
		SCREEN_GAP*(ROOM / 2) + SCREEN_GAP / 2 - (SCREEN_GAP*(ROOM / 2) + SCREEN_GAP / 2)* voltage/ scope, SCREEN_GAP*(ROOM - 1) + SCREEN_GAP / 2);


	for (int i = 0; i<ROOM; i++)//��ʾ�ϰ���Ԥ��λ��
	{
		for (int j = 0; j<ROOM; j++)
		{
			if (flagBarrier_preview[i][j] == 1)
			{
				settextcolor(BLACK);
				setbkmode(TRANSPARENT);
				settextstyle(9, 0, _T("����"));
				TCHAR s[8];
				_stprintf(s, _T("%d"), int(1));
				outtextxy(int(SCREEN_GAP*(i)+5), int(SCREEN_GAP*(ROOM - j - 1) + 5), s);
			}
		}
	}
	t6 = (double)clock() / CLOCKS_PER_SEC;
	fstream pathtime;
	pathtime.open(data + "/DrawPathTime.txt", ios::app);   //����һ���ļ�
	pathtime << "������ͼ��ʱ�� " << t1 - t0 << " s" << endl;
	pathtime << "������ͼ��ʱ�� " << t6 - t1 << " s" << endl;
	pathtime << "����դ���ߺ�ʱ�� " << t2 - t1 << " s" << endl;
	pathtime << "����ä����ʱ�� " << t3 - t2 << " s" << endl;
	pathtime << "�����ϰ����ʱ�� " << t4 - t3 << " s" << endl;
	pathtime << "�����߻��ƺ�ʱ�� " << t5 - t4 << " s" << endl;
	pathtime << "����·��������ߺ�ʱ�� " << t6 - t5 << " s" << endl;
	pathtime << "·���滮��ͼģ��ȫ����ʱ�� " << t6 - t0 << " s" << endl;
	pathtime << endl;

}


//************�Ӻ���*************��������Ϊȫ�ֱ�����������Ҫ����0505Ҫ��
 //1.ȫ��ʹ�ã�
vector<Point2d> PathPlan::lineCalculate(vector<Point2d> Goal){
	//����ͼ�ֶ�·��ֱ�߷���
	vector<Point2d> line(Goal.size() - 1);
	for (size_t i = 0; i < Goal.size() - 1; i++){
		line[i].x = (Goal[i].y - Goal[i + 1].y) / (Goal[i].x - Goal[i + 1].x + 0.01);//k(i)
		line[i].y = (Goal[i].x*Goal[i + 1].y - Goal[i + 1].x*Goal[i].y) / (Goal[i].x - Goal[i + 1].x + 0.01);//b(i)
	}
	return line;
}

bool PathPlan::OverRoadJudge(Point2d d1, int count, vector<Point2d>line){
	//�жϵ�ǰ·���Ƿ���ȫ��·����Ч������
	double distance = 0;
	double width = 2;//����ƽ������
	//double widthmin = 0.5;
	distance = abs(line[count].x*d1.x - d1.y + line[count].y) / sqrt(1 + pow(line[count].x, 2));//�Ƚϵ�ǰ�㵽��ǰ·���ľ���

	if (distance <= width){
		//if (distance <= widthmin){
		//	return 1;//Ԥ��
		//}
		return 0;//ƽ��
	}
	else{
		return 1;
	}

}

Point2d PathPlan::LinePreviewFollower(Point2d d1, Point2d d2, double path_k, double path_b, bool aim_change)//����Ԥ��뾶��Ԥ���
{//ע��Goalpoint_XY�������趨·��ֱ�߷��̣�     transfer_line��Ϊpath_plan.transfer_map�������ò�֪�����Բ���,ƽ������
	//Current_XY���ڼ���Ŀ��������߷���ǰ���Ǻ󣿣��Լ�����ƽ�ƴ�С
	previewP_xy.open(data + "/PreviewP_xy.txt", ios::app);   //����һ���ļ�
	Point2d PreviewP_XY;
	double b = 0;
	double deta = 0;
	b = 2 * path_k * path_b - 2 * path_k*d1.y - 2 * d1.x;
	deta = pow(b, 2) - 4 * (1 + pow(path_k, 2))*(pow(d1.x, 2) + pow((path_b - d1.y), 2) - pow(PreviewR, 2));//bƽ����4ac>=0�жϷ����Ƿ�ɽ�
	R = PreviewR;
	previewP_xy << "d1��" << d1.x << "," << d1.y << endl;
	previewP_xy << "R1��" << R << endl;
	previewP_xy << "deta1��" << deta << endl;
	while (deta <= 0){//���޽������Ӱ뾶,һֱ����;  ֻ��һ�������Ȳ�����
		R = R + 1;
		deta = pow(b, 2) - 4 * (1 + pow(path_k, 2))*(pow(d1.x, 2) + pow((path_b - d1.y), 2) - pow(R, 2));
	}

	P1.x = (-1 * b + sqrt(deta)) / (2 * (1 + pow(path_k, 2)));
	P2.x = (-1 * b - sqrt(deta)) / (2 * (1 + pow(path_k, 2)));
	P1.y = path_k*P1.x + path_b;
	P2.y = path_k*P2.x + path_b;//�ж�ǰ���ϵ�����ĸ�������Ŀ��㣩

	if (!aim_change){
		if (path_k>-1 && path_k<1){//��ֹԤ�������ä����Ŀ�귽�򷴷�������
			while ((abs(P1.x - d2.x) > abs(d1.x - d2.x)) && (abs(P2.x - d2.x) > abs(d1.x - d2.x)))
			{
				R = R + 1;
				deta = pow(b, 2) - 4 * (1 + pow(path_k, 2))*(pow(d1.x, 2) + pow((path_b - d1.y), 2) - pow(R, 2));
				P1.x = (-1 * b + sqrt(deta)) / (2 * (1 + pow(path_k, 2)));
				P2.x = (-1 * b - sqrt(deta)) / (2 * (1 + pow(path_k, 2)));
				P1.y = path_k*P1.x + path_b;
				P2.y = path_k*P2.x + path_b;
			}
		}
		else{
			while ((abs(P1.y - d2.y) > abs(d1.y - d2.y)) && (abs(P2.y - d2.y) > abs(d1.y - d2.y)))
			{
				R = R + 1;
				deta = pow(b, 2) - 4 * (1 + pow(path_k, 2))*(pow(d1.x, 2) + pow((path_b - d1.y), 2) - pow(R, 2));
				P1.x = (-1 * b + sqrt(deta)) / (2 * (1 + pow(path_k, 2)));
				P2.x = (-1 * b - sqrt(deta)) / (2 * (1 + pow(path_k, 2)));
				P1.y = path_k*P1.x + path_b;
				P2.y = path_k*P2.x + path_b;
			}
		}
	}
	previewP_xy << "path_k��" << path_k << endl;
	previewP_xy << "path_b��" << path_b << endl;
	previewP_xy << "R��" << R << endl;
	previewP_xy << "deta2��" << deta << endl;

	double dp1 = sqrt(pow((P1.x - d2.x), 2) + pow((P1.y - d2.y), 2));
	double dp2 = sqrt(pow((P2.x - d2.x), 2) + pow((P2.y - d2.y), 2));
	previewP_xy << "���룺" << P1.x << "��" << dp1 << "��" << P2.x << "��" << dp2 << endl;
	if (dp1 <= dp2){//ȡ������
		PreviewP_XY.x = P1.x;
		PreviewP_XY.y = P1.y;
	}
	else{
		PreviewP_XY.x = P2.x;
		PreviewP_XY.y = P2.y;
	}

	return PreviewP_XY;//����Ԥ������꣨ÿ֡���������������滻�����d2
}

Point2d PathPlan::LinePreviewParallel(Point2d d1, Point2d d2, double path_k, bool aim_change)//����Ԥ��뾶��Ԥ���
{//ע��
	parallelP_xy.open(data + "./parallel.txt", ios::app);
	Point2d PreviewP_XY;
	double b = 0;//��⹫ʽ�е�b
	double deta = 0;
	double path_b = -1 * path_k*d1.x + d1.y;//�����ǰ���ƽ��ֱ�߷���
	b = 2 * path_k * path_b - 2 * path_k*d1.y - 2 * d1.x;
	deta = pow(b, 2) - 4 * (1 + pow(path_k, 2))*(pow(d1.x, 2) + pow((path_b - d1.y), 2) - pow(PreviewR, 2));//bƽ����4ac>=0�жϷ����Ƿ�ɽ�
	R = PreviewR;
	parallelP_xy << "d1��" << d1.x << "," << d1.y << endl;
	parallelP_xy << "R1��" << R << endl;
	parallelP_xy << "deta1��" << deta << endl;
	while (deta <= 0){//���޽������Ӱ뾶,һֱ����;  ֻ��һ����������
		R = R + 1;
		deta = pow(b, 2) - 4 * (1 + pow(path_k, 2))*(pow(d1.x, 2) + pow((path_b - d1.y), 2) - pow(R, 2));
	}
	parallelP_xy << "path_k��" << path_k << endl;
	parallelP_xy << "path_b��" << path_b << endl;
	parallelP_xy << "R��" << R << endl;
	parallelP_xy << "deta2��" << deta << endl;

	P1.x = (-1 * b + sqrt(deta)) / (2 * (1 + pow(path_k, 2)));
	P2.x = (-1 * b - sqrt(deta)) / (2 * (1 + pow(path_k, 2)));
	P1.y = path_k*P1.x + path_b;//Ԥ���1
	P2.y = path_k*P2.x + path_b;//Ԥ���2

	double dp1 = sqrt(pow((P1.x - d2.x), 2) + pow((P1.y - d2.y), 2));//Ԥ���1����ǰĿ������
	double dp2 = sqrt(pow((P2.x - d2.x), 2) + pow((P2.y - d2.y), 2));//Ԥ���2����ǰĿ������
	parallelP_xy << "���룺" << P1.x << "��" << dp1 << "��" << P2.x << "��" << dp2 << endl;
	//�ж�ǰ���ϵ�����ĸ�������Ŀ��㣩
	if (dp1 <= dp2){//ȡ������
		PreviewP_XY.x = P1.x;
		PreviewP_XY.y = P1.y;
	}
	else{
		PreviewP_XY.x = P2.x;
		PreviewP_XY.y = P2.y;
	}

	int length = 10;
	double deta_para = 0;
	deta_para = pow(b, 2) - 4 * (1 + pow(path_k, 2))*(pow(d1.x, 2) + pow((path_b - d1.y), 2) - pow(length, 2));
	while (deta_para <= 0){//���޽������Ӱ뾶,һֱ����;  ֻ��һ����������
		length = length + 1;
		deta_para = pow(b, 2) - 4 * (1 + pow(path_k, 2))*(pow(d1.x, 2) + pow((path_b - d1.y), 2) - pow(length, 2));
	}
	para1.x = (-1 * b + sqrt(deta_para)) / (2 * (1 + pow(path_k, 2)));
	para2.x = (-1 * b - sqrt(deta_para)) / (2 * (1 + pow(path_k, 2)));
	para1.y = path_k*para1.x + path_b;//����--Ԥ���1
	para2.y = path_k*para2.x + path_b;//����--Ԥ���2

	return PreviewP_XY;//����Ԥ������꣨ÿ֡���������������滻�����d2
}

Point2d PathPlan::CirclePreviewFollower(Point2d d1, Point2d d2, double circle_R, Point centerxy, Point d1_last, int count)//����Ԥ��뾶��Ԥ���
{//ע��Goalpoint_XY�������趨·��ֱ�߷��̣�     transfer_line��Ϊpath_plan.transfer_map�������ò�֪�����Բ���,ƽ������
	//Current_XY���ڼ���Ŀ��������߷���ǰ���Ǻ󣿣��Լ�����ƽ�ƴ�С

	Point2d PreviewP_XY;
	double temp = 1;//��ԲʱԤ��뾶��СΪ3-temp�����������ڻ�
	//Point d1, d2;
	//d1.x = start[0];//���أ�--��ǰ��
	//d1.y = start[1];
	//d2.x = stop[0];//����--Ŀ���1��2
	//d2.y = stop[1];
	double v1 = pow(circle_R, 2) - pow((PreviewR - temp), 2) - pow(centerxy.x, 2) - pow(centerxy.y, 2) + pow(d1.x, 2) + pow(d1.y, 2);//�м����
	double A = 4 * pow((d1.y - centerxy.y), 2) + 4 * pow((d1.x - centerxy.x), 2);
	double B = -1 * 8 * centerxy.x*pow((d1.y - centerxy.y), 2) - 4 * v1*(d1.x - centerxy.x) + 8 * centerxy.y*(d1.x - centerxy.x)*(d1.y - centerxy.y);
	double C = 4 * pow(centerxy.x, 2)*pow((d1.y - centerxy.y), 2) + pow(v1, 2) - 4 * centerxy.y*v1*(d1.y - centerxy.y) + 4 * pow((d1.y - centerxy.y), 2)*(pow(centerxy.y, 2) - pow(circle_R, 2));
	double deta = pow(B, 2) - 4 * A*C;//bƽ����4ac>=0�жϷ����Ƿ�ɽ�

	R = PreviewR - temp;
	while (deta <= 0){//���޽������Ӱ뾶,һֱ����;  ֻ��һ����������
		R = R + 1;
		v1 = pow(circle_R, 2) - pow(R, 2) - pow(centerxy.x, 2) - pow(centerxy.y, 2) + pow(d1.x, 2) + pow(d1.y, 2);
		B = -1 * 8 * centerxy.x*pow((d1.y - centerxy.y), 2) - 4 * v1*(d1.x - centerxy.x) + 8 * centerxy.y*(d1.x - centerxy.x)*(d1.y - centerxy.y);
		C = 4 * pow(centerxy.x, 2)*pow((d1.y - centerxy.y), 2) + pow(v1, 2) - 4 * centerxy.y*v1*(d1.y - centerxy.y) + 4 * pow((d1.y - centerxy.y), 2)*(pow(centerxy.y, 2) - pow(circle_R, 2));
		deta = pow(B, 2) - 4 * A*C;
	}

	P1.x = (-1 * B + sqrt(deta)) / (2 * A);
	P2.x = (-1 * B - sqrt(deta)) / (2 * A);
	P1.y = (v1 - 2 * (d1.x - centerxy.x)*P1.x) / (2 * (d1.y - centerxy.y));//Ԥ���1
	P2.y = (v1 - 2 * (d1.x - centerxy.x)*P2.x) / (2 * (d1.y - centerxy.y));//Ԥ���2

	if (!count){
		double dp1 = sqrt(pow((P1.x - d2.x), 2) + pow((P1.y - d2.y), 2));//Ԥ���1����ǰĿ������
		double dp2 = sqrt(pow((P2.x - d2.x), 2) + pow((P2.y - d2.y), 2));//Ԥ���2����ǰĿ������
		//��һ֡���ĸ�������Ŀ���
		if (dp1 <= dp2){//ȡ������
			PreviewP_XY.x = P1.x;
			PreviewP_XY.y = P1.y;
		}
		else{
			PreviewP_XY.x = P2.x;
			PreviewP_XY.y = P2.y;
		}
	}
	else{
		double dp1 = sqrt(pow((P1.x - d1_last.x), 2) + pow((P1.y - d1_last.y), 2));//Ԥ���1��ǰһԤ������
		double dp2 = sqrt(pow((P2.x - d1_last.x), 2) + pow((P2.y - d1_last.y), 2));//Ԥ���2����ǰĿ������
		//�ж�ǰ���ϵ�����ĸ���Զ����һλ�õ㣩
		if (dp1 < dp2){//ȡԶ����
			PreviewP_XY.x = P1.x;
			PreviewP_XY.y = P1.y;
		}
		else{//==�������
			PreviewP_XY.x = P2.x;
			PreviewP_XY.y = P2.y;
		}
	}

	return PreviewP_XY;//����Ԥ������꣨ÿ֡���������������滻�����d2
}

void PathPlan::LocalTargetCalculate(double angle)//���ݵ���������Сͼ�ֲ�Ŀ���
{
	direction = 360 - (angle - 90);//angle�����

	nowCoor[0] = g_d1.x;
	nowCoor[1] = g_d1.y;
	goalCoor[0] = previewP.x;
	goalCoor[1] = previewP.y;
	float direct_now = direction;//�˳���
	float direct_goal = atan2(goalCoor[1] - nowCoor[1], goalCoor[0] - nowCoor[0]) * 180 / PI;//Ŀ�귽�򣨶�����	
	direct_goal2now = int((direct_goal - direct_now) + 720) % 360;//good//����

	if (direct_goal2now >= 90 && direct_goal2now <= 270)
	{//������-90��90����Χ
		stopP[0] = -1;
		stopP[1] = -1;
		if (direct_goal2now <= 180)
		{
			grid_goal2now = 100;
		}
		else
		{
			grid_goal2now = -100;
		}
	}
	else
	{
		if (direct_goal2now < (atan(0.5) * 180 / PI))//zeta_goaltoGIS �ڣ�0��90��֮�䣺atanֵ��Ƕ�ֵͬ��
		{
			stopP[0] = int(float(ROOM / 2) - float((ROOM - 1)*tan(du2hu(direct_goal2now))));
			stopP[1] = ROOM - 1;
			grid_goal2now = direct_goal2now;
		}
		else if (direct_goal2now < 90)
		{
			stopP[0] = 0;
			stopP[1] = int(float(ROOM / 2) / tan(du2hu(direct_goal2now)));
			grid_goal2now = direct_goal2now;
		}
		else if ((360 - direct_goal2now) < (atan(0.5) * 180 / PI))
		{
			stopP[0] = int(float(ROOM / 2) - float((ROOM - 1)*tan(du2hu(direct_goal2now))));
			stopP[1] = ROOM - 1;
			grid_goal2now = -(360 - direct_goal2now);
		}
		else
		{
			stopP[0] = ROOM - 1;
			stopP[1] = int(-float(ROOM / 2) / tan(du2hu(direct_goal2now)));
			grid_goal2now = -(360 - direct_goal2now);
		}
	}

	if (abs(grid_goal2now) <= 4 && group_flag)//4����Լ�������40cm�����񣩵��˲�ԣ�ȣ�  group_flag==0����pathplan���˲�
	{
		grid_goal2now = 0;
	}

}


//2.����ת����
Point3f PathPlan::PointTransform(Point3f m)
{
	Point3f conv_m;//�涯����ת��Ϊդ������
	conv_m.x = floor(float((LENGTH*float(ROOM / 2) + m.x ) / LENGTH));//  /m.zΪ��1����???
	conv_m.y = floor(float((m.y) / LENGTH));
	return conv_m;
}

Point3f PathPlan::BlindLineTransform(vector<Point3f> m)
{
	Point3f m1, m2, m3, m4, m5;
	m1 = PointTransform(m[0]);
	m2 = PointTransform(m[1]);
	m3.x = m[2].x;//k(A)
	m3.y = m[2].y;//-1(B)
	m3.z = m2.y - m2.x*m[2].x;//b(C)
	return m3;
}


//3.�ֲ�ʹ�ã�
void PathPlan::ClearGrid(void){//���ȫ�ֱ���

	for (int i = 0; i < ROOM; i++)
	{
		for (int j = 0; j < ROOM; j++)
		{
			flagBarrier[i][j] = 0;//����ϰ���
			barriertemp[i][j] = 0;
			flagBlind[i][j] = 0;//���ä��
			flagBlind1[i][j] = 0;
			flagBlind2[i][j] = 0;

			flagBarrier_preview[i][j] = 0;//��̬�ϰ���Ԥ��
			alltemp[i][j] = 0;
		}
	}
	for (int i = 0; i < ROOM*ROOM; i++)
	{
		prev[i] = 0;//���·����
		trainingP[i] = 0;
	}
}

void PathPlan::InitGrid(int FlagBarrier[ROOM][ROOM], Point3f abcBlind){
	for (int i = 0; i < ROOM; i++)
	{
		for (int j = 0; j < ROOM; j++)
		{
			flagBarrier[i][j] = FlagBarrier[i][j];
		}
	}

	//��ʼ��դ�����
	for (int i = 0; i<ROOM; i++) {
		for (int j = 0; j<ROOM; j++) {
			vectorP[j*ROOM + i][0] = i;
			vectorP[j*ROOM + i][1] = j;
		}
	}
	
	float disTemp = 0;//��ȡä��
	float sqrtA2B2 = sqrt(abcBlind.x*abcBlind.x + abcBlind.y*abcBlind.y);
	for (int i = 0; i<ROOM; i++)
	{
		for (int j = 0; j<ROOM; j++)
		{
			disTemp = abs(abcBlind.x*i + abcBlind.y*j + abcBlind.z) / sqrtA2B2;
			if (disTemp<0.5)
			{
				flagBlind1[i][j] = 1;
				flagBlind[i][j] = 1;
			}
			else if (disTemp<1.5&&disTemp>0.5)
			{
				flagBlind2[i][j] = 1;
				flagBlind[i][j] = 1;
			}
		}
	}

	for (int i = 0; i < ROOM; i++){
		for (int j = 0; j < ROOM; j++){
			if (flagBarrier[i][j]){
				float distance_start = sqrt(pow((startP[0] - i), 2) + pow((startP[1] - j), 2));
				if (distance_start <= sqrt(2.1)) //2.1;8.1;18.1
				{
					flagBarrier[i][j] = 0;//�ų��ϰ���������
				}
				float distance_stop = sqrt(pow((stopP[0] - i), 2) + pow((stopP[1] - j), 2));
				if (distance_stop <= sqrt(3.1)) //2.1;8.1;18.1
				{
					flagBarrier[i][j] = 0;//�ų��ϰ����յ����
				}
			}
		}
	}

	for (int i = 0; i<ROOM*ROOM; i++)
	{
		if (flagBarrier[vectorP[i][0]][vectorP[i][1]])
		{
			if (vectorP[i][1] <= 10){//0930ʵ�鲹�䣺�ϰ�����ǰ��10*20mm=200mm=2m��ȡ��ä������
				for (int m = 0; m < ROOM; m++)
				{
					for (int n = 0; n < ROOM; n++)
					{
						flagBlind[m][n] = 0;
					}
				}
			}
			for (int j = 0; j<ROOM*ROOM; j++)
			{
				double distance = sqrt(pow((vectorP[i][0] - vectorP[j][0]), 2) + pow((vectorP[i][1] - vectorP[j][1]), 2));
				if (distance <= sqrt(2.1)) //2.1;8.1;18.1
				{
					barriertemp[vectorP[j][0]][vectorP[j][1]] = 1;//�ж����ϰ���(��ȫ������趨��
				}
			}
		}
	}


}

bool PathPlan::WhetherUsePlanning(void){
	for (int i = 4; i < ROOM - 4; i++){//��
		for (int j = 4; j < ROOM; j++){//��
			if (flagBarrier[i][j] || flagBlind[i][j]){
				return 1;//����ä�����ϰ�
			}
			else{
				continue;
			}
		}
	}
	group_flag = 1;
	return 0;//��ʹ��·���滮����GIS_PreviewFollower��err
}

void PathPlan::ArtificialPotentialField(void)
{
	//�˹��Ƴ�����⶯̬��ȨA*����������Ȩϵ��
	float slope[ROOM][ROOM] = { 0 };//Ŀ�귽���б�����ܣ�����Զ��
	float k_att2 = 2000;//б�����ܵı���ϵ��
	float A = 0;//�����ߴ��ߵ�ϵ��:���崹ֱ��Ŀ�귽���ҹ����½Ƕ��㣨��Ϊ��0��0����ͣ�SCREEN_X��0���㣩��ֱ�߷���ΪAx+By+C=0��
	float B = 1;
	float C = 0;
	float d;
	float u_att2;

	if (stopP[0]>13){//��
		A = 1;
		//B = tan(du2hu(90) - direct_goal2now);
		B = -tan(direct_goal2now);
		C = 0;
	}
	else if (stopP[0]<13){//��
		A = 1;
		//B = -tan(du2hu(90) - direct_goal2now);
		B = tan(direct_goal2now);
		C = -SCREEN_X;
	}
	else if (stopP[0]==13){//��
		A = 0;
		B = 1;
		C = 0;
	}

	for (int i = 0; i < ROOM; i++){
		for (int j = 0; j < ROOM; j++){
			d = abs((A*i + B*j + C) / sqrt(pow(A, 2) + pow(B, 2)));
			u_att2 = k_att2*(1 / (d + 5));
			slope[i][j] = u_att2;
		}
	}

	float obt_all[ROOM][ROOM] = { 0 };
	float k_rep = 200;//�ϰ������ϵ��������̬
	float u_rep = 0;
	float dm = 6;//�ϰ����������ΧΪ�ĸ�դ��
	for (int m = 0; m< ROOM; m++)
	{
		for (int n = 0; n< ROOM; n++)
		{
			if (flagBarrier[m][n] == 1)
			{
				float obt[ROOM][ROOM] = { 0 };
				for (int i = 0; i< ROOM; i++){
					for (int j = 0; j < ROOM; j++){
						if (m == i&&n == j)
						{
							obt[i][j] = 200;//���ܾ����ܴ�
						}
						else
						{
							float d1 = pow(float(m - i), 2) + pow(float(n - j), 2);
							u_rep = 0.5*k_rep*pow(max(float((1 / sqrt(d1) - 1 / dm)), float(0)), 2);
							obt[i][j] = u_rep;
						}
						obt_all[i][j] = obt_all[i][j] + obt[i][j];
					}
				}
			}
		}
	}
	//************��̬Ԥ��************
	for (int m = 0; m < ROOM; m++)
	{
		for (int n = 0; n < ROOM; n++)
		{
			if (flagBarrier[m][n] == 1){
				flagBarrier_preview[m][n - 1] = 1;//��ƽ��
				flagBarrier_preview[m][n - 2] = 1;
				flagBarrier_preview[m][n - 3] = 1;
				flagBarrier_preview[m][n - 4] = 1;
			}
		}
	}

	float obt_dynamic[ROOM][ROOM] = { 0 };
	float k_rep_d = 400;//�ϰ������ϵ��������̬
	float u_rep_d = 0;
	float dm_d = 6;//�ϰ����������ΧΪ�ĸ�դ��
	for (int m = 0; m< ROOM; m++)
	{
		for (int n = 0; n< ROOM; n++)
		{
			if (flagBarrier_preview[m][n] == 1)
			{
				float obt_d[ROOM][ROOM] = { 0 };
				for (int i = 0; i< ROOM; i++){
					for (int j = 0; j < ROOM; j++){
						if (m == i&&n == j)
						{
							obt_d[i][j] = 400;//���ܾ����ܴ�
						}
						else
						{
							float d1 = pow(float(m - i), 2) + pow(float(n - j), 2);
							u_rep_d = 0.5*k_rep_d*pow(max(float((1 / sqrt(d1) - 1 / dm_d)), float(0)), 2);
							obt_d[i][j] = u_rep_d;
						}
						obt_dynamic[i][j] = obt_dynamic[i][j] + obt_d[i][j];
					}
				}
			}
		}
	}
	
	int all[ROOM][ROOM] = { 0 };
	int maxall = 0;
	int minall = 0;
	for (int i = 0; i< ROOM; i++){
		for (int j = 0; j < ROOM; j++){
			all[i][j] = 1 * slope[i][j] + 1 * obt_all[i][j] + 1 * obt_dynamic[i][j];//�������ܺ�:ϵ���ɵ���

			if (maxall < all[i][j]){//�������С����
				maxall = all[i][j];
			}
			if (minall > all[i][j]){
				minall = all[i][j];
			}
			/*settextcolor(BLACK);
			setbkmode(TRANSPARENT);
			settextstyle(9, 0, _T("����"));
			TCHAR s[8];
			_stprintf(s, _T("%d"), int(all[i][j]));
			outtextxy(int(PIXEL / ROOM*(i)+5), int(PIXEL / ROOM*(ROOM - j - 1) + 5), s);*/
		}
	}
	//����ȫ������
	for (int i = 0; i < ROOM; i++){
		for (int j = 0; j < ROOM; j++){
			alltemp[i][j] = (double)(all[i][j] - minall) / (maxall - minall);//��һ������
			//cout << alltemp[i][j] << endl;
			//����ȫ������
			/*settextcolor(BLACK);
			setbkmode(TRANSPARENT);
			settextstyle(9, 4, _T("����"));
			TCHAR s[5];
			_stprintf(s, _T("%.2f"), float(alltemp[i][j]));
			outtextxy(int(PIXEL / ROOM*(i)+1), int(PIXEL / ROOM*(ROOM - j - 1) + 7), s);*/
		}
	}
}

double PathPlan::WAstarAlgorithm(void)
{
	if (group_flag){
		memset(group_angle, 0, sizeof(group_angle));//����һ֡Ϊ���滮���������һ֡����
		group = 0;
		group_flag = 0;//�ָ���ʼ��־λ
	}

	//********����ä��ͬʱ����ʱ�����ϰ���ǰ��ʱ����ä���������ϰ���+ä�� ҡ�ڣ�
	//int count_b = 0;
	//for (int i = 7; i < ROOM - 7; i++){//��
	//	for (int j = 2; j < 10; j++){//��
	//		if (flagBarrier[i][j]){
	//			count_b++;
	//		}
	//	}
	//}

	//if (count_b>2){
	//	for (int i = 0; i < ROOM; i++)
	//	{
	//		for (int j = 0; j < ROOM; j++)
	//		{
	//			flagBlind[i][j] = 0;//���ä��
	//			flagBlind1[i][j] = 0;
	//			flagBlind2[i][j] = 0;
	//		}
	//	}
	//}

	/*if (count_b > 1 && group_angle[group-1]!=0){
		group_angle[group] = group_angle[group - 1];
		group++;
		return group_angle[group-1];
	}
*/

	//**************����W***************
	//ArtificialPotentialField();

	//******************************Astar*******************************
	int i, j;
	vs = startP[1] * ROOM + startP[0];
	vp = stopP[1] * ROOM + stopP[0];

	vector<vector<int>>link(ROOM*ROOM, vector<int>(8, -1));//link���Ƿ��ϰ������Χ���ϰ�������ӹ�ϵ����
	vector<vector<float>>weight(ROOM*ROOM, vector<float>(8, INF));//weight���Ƿ��ϰ������Χ���ϰ����Ȩ��
	float gap = 1.5;//��dijkstra���������ò�ͬ���д�ʵ�ʲ���
	float or_weight = 0.4;
	for (int i = 0; i<ROOM*ROOM; i++)
	{
		int kk = 0;
		for (int j = 0; j<ROOM*ROOM; j++)
		{
			double distance = sqrt(pow((vectorP[i][0] - vectorP[j][0]), 2) + pow((vectorP[i][1] - vectorP[j][1]), 2));
			if (distance <= sqrt(2.1) && distance != 0)//��Χ8�㣬�������Լ�
			{
				if (!flagBarrier[vectorP[i][0]][vectorP[i][1]] && !flagBarrier[vectorP[j][0]][vectorP[j][1]])//i��j����Ϊ�ϰ����
				{
					if (distance == 1)//��������
					{
						if (barriertemp[vectorP[i][0]][vectorP[i][1]] && barriertemp[vectorP[j][0]][vectorP[j][1]])//i,j���ǰ�ȫ�����ڵĵ�
						{
							link[i][kk] = j;
							weight[i][kk] = 15 * gap;//��ȫ������
							kk++;
						}
						else if (barriertemp[vectorP[i][0]][vectorP[i][1]] || barriertemp[vectorP[j][0]][vectorP[j][1]])//i��j��һ����Ϊ��ȫ�����ڵ㣻
						{
							if (flagBlind1[vectorP[i][0]][vectorP[i][1]] || flagBlind1[vectorP[j][0]][vectorP[j][1]])//i,j��һ��Ϊä��2��,��һ��Ϊ��ȫ�����
							{
								link[i][kk] = j;
								weight[i][kk] = or_weight + 2 * gap;//��  mangdao shangxiazuoyou
								kk++;
							}
							else if (flagBlind2[vectorP[i][0]][vectorP[i][1]] || flagBlind2[vectorP[j][0]][vectorP[j][1]])//i,j��һ��Ϊä��1/3��,��һ��Ϊ��ȫ�����
							{
								link[i][kk] = j;
								weight[i][kk] = or_weight + 4 * gap;//��  mangdao shangxiazuoyou
								kk++;
							}
							else///iΪ��ȫ����㣬jΪ��ͨ��
							{
								link[i][kk] = j;
								weight[i][kk] = or_weight + 10.7 * gap;//ʮ normal shangxiazuoyou
								kk++;
							}
						}//��ȫ
						else{  //i,j�������Ϊ��ȫ�����ڵ�
							if (flagBlind[vectorP[i][0]][vectorP[i][1]] && flagBlind[vectorP[j][0]][vectorP[j][1]])//i,j��Ϊä����
							{
								if (flagBlind1[vectorP[i][0]][vectorP[i][1]] && flagBlind1[vectorP[j][0]][vectorP[j][1]])
								{
									link[i][kk] = j;
									weight[i][kk] = or_weight + 1 * gap;//�� mangdao�� shangxiazuoyou  i,j������2�ߣ�ֱ�ߣ�
									kk++;
								}
								else if (flagBlind2[vectorP[i][0]][vectorP[i][1]] && flagBlind2[vectorP[j][0]][vectorP[j][1]])
								{
									link[i][kk] = j;
									weight[i][kk] = or_weight + 4 * gap;//�� mangdao�� shangxiazuoyou i,j����1,3��
									kk++;
								}
								else
								{
									link[i][kk] = j;
									weight[i][kk] = or_weight + 2 * gap;//�� mangdao�� one in 2,one in1��3��
									kk++;
								}
							}
							else if (flagBlind2[vectorP[i][0]][vectorP[i][1]] || flagBlind2[vectorP[j][0]][vectorP[j][1]])//i,jһ��Ϊä1/3�㣬һ��Ϊ��ͨ��
							{
								link[i][kk] = j;
								weight[i][kk] = or_weight + 6 * gap;//�� normal shangxiazuoyou
								kk++;
							}
							else{//i��j��Ϊ��ͨ��(���ڲ�����ä�������ߵ㵽��ȫ���������)
								link[i][kk] = j;
								weight[i][kk] = or_weight + 9 * gap;//�� normal shangxiazuoyou
								kk++;
							}
						}
					}
					//****************�ֱ���9�����****************
					else//�Խǵ�
					{
						if (barriertemp[vectorP[i][0]][vectorP[i][1]] && barriertemp[vectorP[j][0]][vectorP[j][1]])//i,j���ǰ�ȫ�����ڵĵ�
						{
							link[i][kk] = j;
							weight[i][kk] = 15 * gap;//��ȫ������
							kk++;
						}
						else if (barriertemp[vectorP[i][0]][vectorP[i][1]] || barriertemp[vectorP[j][0]][vectorP[j][1]])//i��j��һ����Ϊ��ȫ�����ڵ㣻
						{
							if (flagBlind1[vectorP[i][0]][vectorP[i][1]] || flagBlind1[vectorP[j][0]][vectorP[j][1]])//i,j��һ��Ϊä��1��,��һ��Ϊ��ȫ�����
							{
								link[i][kk] = j;
								weight[i][kk] = or_weight + 3 * gap;//��  mangdao shangxiazuoyou
								kk++;
							}
							else if (flagBlind2[vectorP[i][0]][vectorP[i][1]] || flagBlind2[vectorP[j][0]][vectorP[j][1]])//i,j��һ��Ϊä��2��,��һ��Ϊ��ȫ�����
							{
								link[i][kk] = j;
								weight[i][kk] = or_weight + 5 * gap;//��  mangdao shangxiazuoyou
								kk++;
							}
							else///iΪ��ȫ����㣬jΪ��ͨ��
							{
								link[i][kk] = j;
								weight[i][kk] = or_weight + 10.7 * gap;//ʮ normal shangxiazuoyou
								kk++;
							}
						}//��ȫ
						else{  //i,j�������Ϊ��ȫ�����ڵ�
							if (flagBlind[vectorP[i][0]][vectorP[i][1]] && flagBlind[vectorP[j][0]][vectorP[j][1]])//i,j��Ϊä����
							{
								if (flagBlind1[vectorP[i][0]][vectorP[i][1]] && flagBlind1[vectorP[j][0]][vectorP[j][1]])
								{
									link[i][kk] = j;
									weight[i][kk] = or_weight;//�� mangdao�� shangxiazuoyou  i,j������2�ߣ��Խ��ߣ�
									kk++;
								}
								else if (flagBlind2[vectorP[i][0]][vectorP[i][1]] && flagBlind2[vectorP[j][0]][vectorP[j][1]])
								{
									link[i][kk] = j;
									weight[i][kk] = or_weight + 5 * gap;//�� mangdao�� shangxiazuoyou i,j����1,3��
									kk++;
								}
								else
								{
									link[i][kk] = j;
									weight[i][kk] = or_weight + 3 * gap;//�� mangdao�� one in 2,one in1/3
									kk++;
								}
							}
							else if (flagBlind2[vectorP[i][0]][vectorP[i][1]] || flagBlind2[vectorP[j][0]][vectorP[j][1]])//i,jһ��Ϊä1/3�㣬һ��Ϊ��ͨ��
							{
								link[i][kk] = j;
								weight[i][kk] = or_weight + 8 * gap;//�� normal shangxiazuoyou
								kk++;
							}
							else{//i��j��Ϊ��ͨ��(���ڲ�����ä�������ߵ㵽��ȫ���������)
								link[i][kk] = j;
								weight[i][kk] = or_weight + 10.7 * gap;//ʮ normal shangxiazuoyou
								kk++;
							}
						}
					}
				}
			}
		}
	}

	float minCost;
	int minCostId;
	bool flag[ROOM*ROOM];
	

	float f_cost[ROOM*ROOM];
	float g_cost[ROOM*ROOM];
	float h_diagonal;
	float h_straight;
	float h_manhadun;
	float h_oushi;
	float D = or_weight + 8 * gap;//����������Сֵ4.9
	float D2 = or_weight + 8.7 * gap;//�Խ�����Сֵ5.8
	double h_cost[ROOM*ROOM];
	double weight_h = 0;

	for (int i = 0; i < ROOM*ROOM; i++)
	{
		flag[i] = false;//��ʼ�����е���Ϊ��             
		f_cost[i] = INF;//��ʼ������fΪ����
		g_cost[i] = INF;
		prev[i] = vs; //��ʼ�����н������������ʼ��

		//weight_h = 0.25;
		weight_h = 0;
		//weight_h = alltemp[vectorP[i][0]][vectorP[i][1]];//�˴�Ϊ��̬��ȨA*�㷨�ӿ�

		//h_manhadun = D*( abs(vectorP[i][0] - stopP[0]) + abs(vectorP[i][1] - stopP[1]) );
		//h_cost[i] = (1 + weight_h) * h_manhadun;
		//h_oushi = D*( sqrt(pow((vectorP[i][0] - stopP[0]), 2) + pow((vectorP[i][1] - stopP[1]), 2)) );
		//h_cost[i] = (1 + weight_h) * h_oushi;
		h_diagonal = min(abs(vectorP[i][0] - stopP[0]), abs(vectorP[i][1] - stopP[1]));
		h_straight = (abs(vectorP[i][0] - stopP[0]) + abs(vectorP[i][1] - stopP[1]));
		h_cost[i] = (1 + weight_h) * (D2 * h_diagonal + D * (h_straight - 2 * h_diagonal));//��ʼ������hֵ
	}

	flag[vs] = true;//����ʼ��
	g_cost[vs] = 0;
	f_cost[vs] = g_cost[vs] + h_cost[vs];
	prev[vs] = -1;

	for (int i = 0; i<8 && link[vs][i] != -1; i++)//��ʼ�� �����Χ�㵽����f
	{
		g_cost[link[vs][i]] = weight[vs][i];//g
		f_cost[link[vs][i]] = weight[vs][i] + h_cost[link[vs][i]];//f
	}

	for (int i = 0; i < ROOM*ROOM; i++)
	{
		minCost = INF;
		minCostId = 0;
		for (int j = 0; j < ROOM*ROOM; j++)
		{
			if (!flag[j] && f_cost[j]<minCost)
			{
				minCost = f_cost[j];
				minCostId = j;
			}
		}

		////**��ʾÿ��������**�������㷨������
		//settextcolor(BLUE);
		//setbkmode(TRANSPARENT);
		//settextstyle(9, 0, _T("����"));
		//TCHAR s1[8];
		//_stprintf_s(s1, _T("%d"), int(i + 1));
		//outtextxy(int(SCREEN_GAP*(vectorP[minCostId][0])), int(SCREEN_GAP *(ROOM - vectorP[minCostId][1] - 1) + 1), s1);
		////m++;

		flag[minCostId] = true;

		for (int j = 0; j < 8 && link[minCostId][j] != -1; j++) //����ÿ����Χ�˵��е�g��f
		{
			double tmp = g_cost[minCostId] + weight[minCostId][j];//��ǰg = ԭ(f-h)-weight
			if (!flag[link[minCostId][j]] && (tmp < g_cost[link[minCostId][j]]))
			{
				g_cost[link[minCostId][j]] = tmp;//��������Ĵ���	
				f_cost[link[minCostId][j]] = tmp + h_cost[link[minCostId][j]];//��ǰf
				prev[link[minCostId][j]] = minCostId;
			}
		}

		if (flag[vp]){
			cout << "��չ�ڵ����� " << i + 1 << endl;
			break;
		}
	}


	//setfillcolor(BROWN);
	double weitemp = 0;
	double weight_angle = 0;
	double sum_angle = 0;
	double deflection = 0;
	j = 0;
	int count = 10;//ȡǰcount������ƽ�
	if (prev[vp] == vs){
		return path_angle = 0;
	}

	Point3f conv_m;
	for (i = vp; i != vs; i = prev[i])
	{
		deflection = atan(double(vectorP[i][0] - startP[0]) / double(vectorP[i][1] - startP[1]));
		++j;//���յ㵽������·�������
		if (i <= count*ROOM + (ROOM - 1))
		{
			weitemp = j + 2;
			sum_angle += weitemp*deflection;//��Ȩ�ͣ����ӣ�
			weight_angle += weitemp;//Ȩ�غͣ���ĸ��
		}
	}
	
	path_angle1 = -1 * sum_angle / weight_angle;
	if (abs(path_angle1) <= du2hu(10))
	{
		path_angle1 = 0;
	}
	cout << path_angle1 << endl;

	//************************�ۺ��˲�*********************
	if (group<1)
	{
		path_angle1 = path_angle1;
	}
	else if (group<2)
	{
		path_angle1 = 0.5*(path_angle1 + path_angle);
	}//����֡���ֵ��Ŀ��ƽ�
	else
	{
		path_angle1 = 0.6*path_angle1 + 0.3*group_angle[group - 1] + 0.1*group_angle[group - 2];
	}

	if (abs(path_angle1) <= du2hu(7))
	{
		path_angle1 = 0;
	}
	path_angle = path_angle1;
	////************************�������˲�*********************
	//int changedirection_flag = 0;
	//if (group < 1)
	//{
	//	path_angle = path_angle1;
	//}
	//else
	//{
	//	if ((path_angle1<0 && path_angle>0) || (path_angle1>0 && path_angle < 0))
	//	{
	//		if (abs(abs(path_angle1) - abs(path_angle)) > du2hu(5)){
	//			path_angle = path_angle1;
	//		}
	//		else{
	//			changedirection_flag = 1;
	//		}
	//	}
	//	else{
	//		path_angle = path_angle1;
	//	}
	//}//����֡�ȽϺ�Ŀ��ƽ�
	//cout << "\n·���ڵ����� " << j << endl;
	//cout << "\n���ƽ��Ƿ�����־��0���䣬1�ı䣩�� " << changedirection_flag << endl;

	group_angle[group] = path_angle;

	group++;
	return path_angle;

}

//����ä��������ֱ������
void PathPlan::Singleline_Preview(Point3f abcline)
{
	if (abcline.x == 0 && abcline.z == 0){//δ��⵽ֱ�������س�����
		stopP[0] = floor(ROOM / 2);
		stopP[1] = ROOM - 1;
	}
	else{
		Point2d PreviewP_XY;
		Point2d d1 = { floor(ROOM / 2), 0 };//��ǰ��
		Point2d d2 = { floor(ROOM / 2), ROOM - 1 };//����Ŀ���
		double path_k = abcline.x;
		double path_b = abcline.z;
		double b = 0;//��⹫ʽ�е�bs
		double deta = 0;
		double Preview_grid_R = 10;//ʮ��դ�񳤶�
		b = 2 * path_k * path_b - 2 * path_k*d1.y - 2 * d1.x;
		deta = pow(b, 2) - 4 * (1 + pow(path_k, 2))*(pow(d1.x, 2) + pow((path_b - d1.y), 2) - pow(Preview_grid_R, 2));//bƽ����4ac>=0�жϷ����Ƿ�ɽ�
		R = Preview_grid_R;

		while (deta <= 0){//���޽������Ӱ뾶,һֱ����;  ֻ��һ����������
			R = R + 1;
			deta = pow(b, 2) - 4 * (1 + pow(path_k, 2))*(pow(d1.x, 2) + pow((path_b - d1.y), 2) - pow(R, 2));
		}

		P1.x = (-1 * b + sqrt(deta)) / (2 * (1 + pow(path_k, 2)));
		P2.x = (-1 * b - sqrt(deta)) / (2 * (1 + pow(path_k, 2)));
		P1.y = path_k*P1.x + path_b;//Ԥ���1
		P2.y = path_k*P2.x + path_b;//Ԥ���2

		double dp1 = sqrt(pow((P1.x - d2.x), 2) + pow((P1.y - d2.y), 2));//Ԥ���1����ǰĿ������
		double dp2 = sqrt(pow((P2.x - d2.x), 2) + pow((P2.y - d2.y), 2));//Ԥ���2����ǰĿ������

		//�ж�ǰ���ϵ�����ĸ�������Ŀ��㣩
		if (dp1 <= dp2){//ȡ������
			PreviewP_XY.x = P1.x;
			PreviewP_XY.y = P1.y;
		}
		else{
			PreviewP_XY.x = P2.x;
			PreviewP_XY.y = P2.y;
		}

		if (PreviewP_XY.x>ROOM/2)
		    stopP[0] = PreviewP_XY.x + 2;//����Ԥ������꣨դ�񣩣�����������ֱ�ӽ���pathplan
		else
		    stopP[0] = PreviewP_XY.x - 2;

		stopP[1] = PreviewP_XY.y + 12;

	}

}


//4.����ʹ�ã�
void PathPlan::SavingTestData(double err){

	///////////////���դ��Ԫ������Ϊgridresult.txt�ļ�/////////////
	for (int i = 0; i < ROOM; i++){
		for (int j = 0; j < ROOM; j++){
			if (i == stopP[0] && j == stopP[1]){
				trainingP[j*ROOM + i] = 4;
			}
			else if (flagBarrier[i][j] == 1){
				trainingP[j*ROOM + i] = 3;
			}
			else if (flagBlind1[i][j] == 1){
				trainingP[j*ROOM + i] = 2;
			}
			else if (flagBlind2[i][j] == 1){
				trainingP[j*ROOM + i] = 1;
			}
			else{
				trainingP[j*ROOM + i] = 0;
				continue;
			}
		}
	}
	//gridresult_err.open(data_dir[7] + "/gridresult_err.txt", ios::app);
	//gridresult.open(data_dir[7] + "/gridresult.txt", ios::app);   //����һ���ļ�
	/*for (int i = 0; i < ROOM; i++){
		for (int j = 0; j < ROOM; j++){
			if (j*ROOM + i< ROOM*ROOM - 1)
			{
				gridresult << trainingP[j*ROOM + i] << ",";
				gridresult_err << trainingP[j*ROOM + i] << ",";
			}
			else{
				gridresult << trainingP[j*ROOM + i];
				gridresult_err << trainingP[j*ROOM + i] << endl;
			}
		}
	}
	gridresult_err.close();*/

	//ferr.open(data_dir[7] + "/err.txt", ios::app);   //����һ���ļ�
	//ferr << err << endl;
	//ferr.close();
	//gridresult << "," << err << endl;
	//gridresult.close();
}

void PathPlan::GetCameraHeadingFromRot_C(cv::Mat &CurrentCameraR, double &headangle){
	cv::Mat Zc = cv::Mat(3, 1, CV_32F);
	Zc.at<float>(0) = 0;
	Zc.at<float>(1) = 0;
	Zc.at<float>(2) = 1;
	cv::Mat Zo = CurrentCameraR.inv()*Zc;
	cv::Mat Zo_ = cv::Mat(3, 1, CV_32F);
	Zo_.at<float>(0) = Zo.at<float>(0);
	Zo_.at<float>(1) = 0;
	Zo_.at<float>(2) = Zo.at<float>(2);
	if (Zo_.at<float>(0) > 0)
	{
		headangle = acos(Zo_.dot(Zc) / norm(Zo_)) / PI * 180;
	}
	else
	{
		headangle = -acos(Zo_.dot(Zc) / norm(Zo_)) / PI * 180;
	}

}

//5.��ͼʹ�ã�
void PathPlan::BackProjection(Point3f conv_m, Mat R_cg, Mat T_cg, Mat pathimage)
{
	Point3f result;
	Point3f m;//դ������ת��Ϊ�涯����
	m.x = float(conv_m.x - startP[0])*LENGTH + float(LENGTH / 2);
	m.y = float(conv_m.y * LENGTH) + float(LENGTH / 2);
	m.z = 1;//�������

	Mat R_gc = R_cg.inv();
	Mat m1 = Mat::zeros(3, 1, CV_32FC1);
	m1.at<float>(0) = m.x;
	m1.at<float>(1) = m.y;
	m1.at<float>(2) = m.z;
	Mat m2 = R_gc*(m1 - T_cg);//�涯ת������ǰ�������ϵ��

	//��ǰ�������ϵת����ͼ������ϵ�£�*�������R
	result.x = _cameraMatrix1[0][0] * 2 * m2.at<float>(0) + _cameraMatrix1[0][2] * 2 * m2.at<float>(2);
	result.y = _cameraMatrix1[1][1] * 2 * m2.at<float>(1) + _cameraMatrix1[1][2] * 2 * m2.at<float>(2);
	result.z = m2.at<float>(2);
	result.x = result.x / result.z;
	result.y = result.y / result.z;
	back_pathXY.open(data + "./back_pathXY.txt", ios::app);
	back_pathXY << result.x << "," << result.y << endl;
	back_pathXY.close();

	circle(pathimage, Point(result.x, result.y), 5, Scalar(255, 0, 0), 2);
}

//6.����
int PathPlan::ParallelLineJudge(vector<vector<Point3f>> linepoints, bool breakpointFlag){
	
	Point preview_P = LinePreviewParallel(g_d1, g_d2, path_k, 1);
	int stop[2] = { 0, 0 };
	float now = direction;//�˳���
	float goal = atan2(preview_P.y - g_d1.y, preview_P.x - g_d1.x + 0.1) * 180 / PI;//Ŀ�귽�򣨶�����	
	float goal2now = int((goal - now) + 720) % 360;//good//����

	if (goal2now >= 90 && goal2now <= 270)
	{//������-90��90����Χ
		stop[0] = -1;
		stop[1] = -1;
	}
	else
	{
		if (goal2now < (atan(0.5) * 180 / PI))//zeta_goaltoGIS �ڣ�0��90��֮�䣺atanֵ��Ƕ�ֵͬ��
		{
			stop[0] = int(float(ROOM / 2) - float((ROOM - 1)*tan(du2hu(goal2now))));
			stop[1] = ROOM - 1;
		
		}
		else if (goal2now < 90)
		{
			stop[0] = 0;
			stop[1] = int(float(ROOM / 2) / tan(du2hu(goal2now)));
		}
		else if ((360 - goal2now) < (atan(0.5) * 180 / PI))
		{
			stop[0] = int(float(ROOM / 2) - float((ROOM - 1)*tan(du2hu(goal2now))));
			stop[1] = ROOM - 1;
		}
		else
		{
			stop[0] = ROOM - 1;
			stop[1] = int(-float(ROOM / 2) / tan(du2hu(goal2now)));
		}
	}

	double parallel_k = (stop[1] - 0) / (stop[0] - floor(ROOM / 2) + 0.1);//���դ������ϵ�µ�·��ƽ���ߵ�б��
	
	int n = linepoints.size();
	vector<Point3f> lineXYZ(0);
	vector<Point3f> lineABC(0);
	for (int i = 0; i < n; i++)
	{
		float k = (linepoints[i][1].y - linepoints[i][0].y) / (linepoints[i][1].x - linepoints[i][0].x + 0.1);
		float b = linepoints[i][1].y - k*linepoints[i][1].x;

		lineXYZ.push_back(Point3f(k, -1, b));//�涯����ϵ�µ�����ֱ��ϵ��A,B,C
		vector<Point3f> m;
		m.push_back(linepoints[i][0]);
		m.push_back(linepoints[i][1]);
		m.push_back(Point3f(k, -1, b));
		lineABC.push_back(BlindLineTransform(m));
	}

	//����ÿ��ֱ�ߵ�б�������
	vector<double> k(n);
	vector<double> distance(n);
	for (int i = 0; i < n; i++){
		k[i] = -1 * lineABC[i].x / lineABC[i].y;//ת��k
		distance[i] = abs(lineABC[i].x*floor(ROOM / 2) + lineABC[i].y * 0 + lineABC[i].z) / sqrt(pow(lineABC[i].x, 2) + pow(lineABC[i].y, 2));//�Ƚϵ�ǰ�㵽��ǰ·���ľ���
	}

	//��դ������ϵ�±Ƚ�ֱ��б��
	int count = 0;
	double min = 1000;
	vector<double> count2(0);
	for (int j = 1; j < n; j++){

		double dvalue = abs(atan(k[j]) - atan(parallel_k)) * 180 / M_PI;
		if (dvalue >= 90)
			dvalue = 180 - dvalue;

		if (dvalue < 30)
			count2.push_back(j);
	}

	//����ѡȡ���������ֱ��
	double gridaway;
	if (!breakpointFlag) //���ڹյ㣬ѡȡ����Ϊ0
		gridaway = 0;
	else //�ڹյ㣬ѡȡ����Ϊradius
		gridaway = 1000*radius / LENGTH;

	//ѡ����������ֱ��
	for (int i = 0; i < count2.size(); i++)
	{
		if (distance[count2[i]] < min && distance[count2[i]] > gridaway)
		{
			min = distance[count2[i]];
			count = count2[i];
		}
	}

	if (count2.size() == 0)
		return -1;
	else
		return count;
}




/*//////���º�����ʱ����///////

//bool path_plan::goal_Judge(Point2d d1, int count, vector<Point2d>line, vector<Point2d> Goal){
//	//������ڴ��ͼ����·����λ�ã��жϵ�ǰ·���Ƿ��л�
//	bool result;
//	double distance = 0;
//	double distance1 = 0;
//	distance = abs(line[count].x*d1.x - d1.y + line[count].y) / sqrt(1 + pow(line[count].x, 2));//��ǰ�㵽��ǰ·���ľ���
//	distance1 = abs(line[count + 1].x*d1.x - d1.y + line[count + 1].y) / sqrt(1 + pow(line[count + 1].x, 2));//��ǰ�㵽��һ·���ľ���
//	if (distance1 <= distance
//		//&& (abs(abs(d1.x) - abs(Goal[count + 1].x)) + abs(abs(d1.x) - abs(Goal[count + 2].x)) == abs(abs(Goal[count + 1].x) - abs(Goal[count + 2].x)))
//		//|| (abs(abs(d1.y) - abs(Goal[count + 1].y)) + abs(abs(d1.y) - abs(Goal[count + 2].y)) == abs(abs(Goal[count + 1].y) - abs(Goal[count + 2].y)))
//		){//��ǿԼ��
//		return result = 1;
//	}
//	else{
//		return result = 0;
//	}
//
//}


//vector<Point2d> path_plan::goalCalculate(Point2d d1, vector<Point2d> Goal, vector<Point2d>line){
//	//������ڴ��ͼ����·����λ�ã��ó���ǰ��Ҫ�ߵ�·������
//	vector<Point2d> pathGoalXY(0);
//	double distance = 0;
//	double distance1 = 0;
//	int count = 0;
//	distance = abs(line[0].x*d1.x - d1.y + line[0].y) / sqrt(1 + pow(line[0].x, 2));//�Ƚϵ�ǰ�㵽����·���ľ���
//	for (size_t i = 1; i < line.size(); i++){
//		distance1 = abs(line[i].x*d1.x - d1.y + line[i].y) / sqrt(1 + pow(line[i].x, 2));
//		if (distance1< distance){
//			distance = distance1;
//			count = i;
//		}
//	}
//	for (size_t j = count; j < Goal.size(); j++){
//		pathGoalXY.push_back(Point2d(Goal[j].x, Goal[j].y));
//	}
//	return pathGoalXY;
//}

//int PathPlan::parallel_line_Judge(vector<Point3f> lineXYZ){
//	//��������line�ж�����һ���뵱ǰ·���ӽ�ƽ��
//	int n = lineXYZ.size();
//	vector<double> k(n);
//	vector<double> distance(n);
//	int count = 0;
//	vector<double> count2(0);
//	for (int i = 0; i < n; i++){
//		k[i] = -1 * lineXYZ[i].x / lineXYZ[i].y;
//		distance[i] = abs(lineXYZ[i].x*g_d1.x + lineXYZ[i].y*g_d1.y + lineXYZ[i].z) / sqrt(pow(lineXYZ[i].x, 2) + pow(lineXYZ[i].y, 2));//�Ƚϵ�ǰ�㵽��ǰ·���ľ���
//	}
//
//	for (int j = 1; j< n; j++){
//		if (abs(k[j] - path_k)<abs(k[j-1] - path_k)){
//			count =j;
//		}
//		else if (abs(k[j] - path_k) == abs(k[j - 1] - path_k)){
//			count = j;
//			count2.push_back(j - 1);
//		}
//	}
//	count2.push_back(count);
//
//	if (count2.size()>1){
//		for (int i = 1; i < count2.size(); i++){
//			if (distance[i] <= distance[i - 1]){
//				count = i;
//			}
//		}
//	}
//
//	return count;
//}


*/