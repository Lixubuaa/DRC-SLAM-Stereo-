#include"header.hpp"
#include <time.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\nonfree\features2d.hpp>

//Int to string
string itos(int i)
{
    stringstream s;
    s << i;
    return s.str();
}

//Judge if point is in ROI
bool MOD::ROI_mod(int x1, int y1)
{
    if (x1 >= width * w_div && x1 <= width - width * w_div && y1 >= height * h_div && y1 <= height - height * h_div)
        return true;
    return false;
}

//Image pre-processing
void MOD::pre_process()
{
	
    F_prepoint.clear();
    F_nextpoint.clear();
   
   
    gray = frame.clone();
  
    return;
}


void MOD::optical_flow_check()
{
    double limit_of_check = 2120;
    int limit_edge_corner = 5;
    for (int i = 0; i < state.size(); i++)
       ///* if (state[i] != 0)
        {

       //    int dx[10] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
       //    int dy[10] = { -1, -1, -1, 0, 0, 0, 1, 1, 1 };
           int x1 = prepoint[i].x, y1 = prepoint[i].y;
           int x2 = nextpoint[i].x, y2 = nextpoint[i].y;
           if ((x1 < limit_edge_corner || x1 >= gray.cols - limit_edge_corner || x2 < limit_edge_corner || x2 >= gray.cols - limit_edge_corner
			   || y1 < 2 * limit_edge_corner || y1 >= gray.rows - 2 * limit_edge_corner || y2 < 2 * limit_edge_corner || y2 >= gray.rows - 2 * limit_edge_corner))
           {
               state[i] = 0;
               continue;
           }
       // double sum_check = 0;
       // for (int j = 0; j < 9; j++)
       //     sum_check += abs(prevgray.at<uchar>(y1 + dy[j], x1 + dx[j]) - gray.at<uchar>(y2 + dy[j], x2 + dx[j]));
       // if (sum_check>limit_of_check) state[i] = 0;*/

        if (state[i])
         {
            F_prepoint.push_back(prepoint[i]);
            F_nextpoint.push_back(nextpoint[i]);
         }
        }
    return;
}


bool MOD::stable_judge()
{
    int stable_num = 0;
    double limit_stalbe = 0.5;
    for (int i = 0; i < state.size(); i++)
        if (state[i])
        {
        if (sqrt((prepoint[i].x - nextpoint[i].x)*(prepoint[i].x - nextpoint[i].x) + (prepoint[i].y - nextpoint[i].y)*(prepoint[i].y - nextpoint[i].y)) < limit_stalbe) stable_num++;
        }
    if (stable_num*1.0 / Harris_num > 0.2) return 1;
    return 0;
}

//Draw the  detection results
void MOD::draw_detection()
{
    int tt = 10;
    double flag_meiju[100][100];
    memset(flag_meiju, 0, sizeof(flag_meiju));
	for (int i = 0; i < gray.rows / tt; i++){
        for (int j = 0; j < gray.cols / tt; j++)
        {
            double x1 = i*tt + tt / 2;
            double y1 = j*tt + tt / 2;
            for (int k = 0; k < T.size(); k++)
                if (ROI_mod(T[k].x, T[k].y) && sqrt((T[k].x - y1)*(T[k].x - y1) + (T[k].y - x1)*(T[k].y - x1)) < tt*sqrt(2)) flag_meiju[i][j]++;
        }
	}//类比栅格图
    double mm = 0;
    int mark_i = 0, mark_j = 0;
    for (int i = 0; i < gray.rows / tt; i++)
        for (int j = 0; j < gray.cols / tt; j++)
            if (ROI_mod(j*tt, i*tt) && flag_meiju[i][j] > mm)
            {
                mark_i = i;
                mark_j = j;
                mm = flag_meiju[i][j];
                if (mm < 1) continue;
                rectangle(frame, Point(mark_j*tt / scale - rec_width, mark_i*tt / scale + rec_width), Point(mark_j*tt / scale + rec_width, mark_i*tt / scale - rec_width), Scalar(0, 255, 255), 2);
            }
   // if (mm > 1111) rectangle(frame, Point(mark_j*tt / scale - rec_width, mark_i*tt / scale + rec_width), Point(mark_j*tt / scale + rec_width, mark_i*tt / scale - rec_width), Scalar(0, 255, 255), 2);
}
bool MOD::checkROI(Point2f Point){
	double roi_scale = 0.05;
	if (Point.x>width*roi_scale&&Point.x<width*(1 - roi_scale) && Point.y>height*roi_scale&&Point.y<height*(1 - roi_scale))
		return true;
	else
		return false;
}

vector<Point2f> MOD::process(Mat preimage,Mat image,Mat disp)
{
	double start_time, end_time;
	double t0, t1, t2, t3, t4,t5;
	start_time = (double)clock() / CLOCKS_PER_SEC;
    frame = image;

	
        double t = (double)cvGetTickCount();
  

        pre_process();
		
		prevgray = preimage;
		//cvtColor(prevgray, prevgray, CV_BGR2GRAY);
		
            //calcOpticalFlowPyrLK
		t0 = (double)clock() / CLOCKS_PER_SEC;
		//初始化
		//首先创建两个关键点数组，用于存放两张图像的关键点，数组元素是KeyPoint类型
		//std::vector<KeyPoint> keypoints_1, keypoints_2;
		vector<KeyPoint>keypoint1, keypoint2;
		//SURF
		int minHessian = 300;      //定义Hessian矩阵阈值
		SurfFeatureDetector detector(minHessian);       //定义Surf检测器
		detector.detect(prevgray, keypoint1);
	

		/*cout << "关键点数目为： " << keypoint1.size() << endl;*/
		//创建两张图像的描述子，类型是Mat类型
		
		t1 = (double)clock() / CLOCKS_PER_SEC;
		prepoint.clear();
		int keypoint_size = keypoint1.size();
		for (int i = 0; i < keypoint_size; i++){
			prepoint.push_back(keypoint1[i].pt);
				
		}
           // cornerSubPix(prevgray, prepoint, Size(10, 10), Size(-1, -1), TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
		
			calcOpticalFlowPyrLK(prevgray, gray, prepoint, nextpoint, state, err, Size(22, 22), 5, TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.01));//state
		
			optical_flow_check();
			t2 = (double)clock() / CLOCKS_PER_SEC;	//耗时30ms左右
			/*for (int i = 0; i < nextpoint.size();i++){
				circle(gray, nextpoint[i],2,Scalar(0,255,0),1);
			}
			imshow("gray", gray);
			waitKey(1);*/
            //Find corners
     //       for (int i = 0; i < state.size(); i++)
     //       {

     //           double x1 = prepoint[i].x, y1 = prepoint[i].y;
     //           double x2 = nextpoint[i].x, y2 = nextpoint[i].y;
     ////           if (state[i] != 0)
     ////           {

     ////               //Draw all corners
					////circle(img_scale, prepoint[i], 3, Scalar(255, 0, 255));
     ////               //circle(img_scale, nextpoint[i], 3, Scalar(255, 0, 255));
     ////              // circle(pre_frame, prepoint[i], 2, Scalar(255, 0, 255));
     ////           }
     //       }
     //       cout << Harris_num << endl;
			t3 = (double)clock() / CLOCKS_PER_SEC;
            //F-Matrix
           /* vector<Point2f> F2_prepoint, F2_nextpoint;
            F2_prepoint.clear();
            F2_nextpoint.clear();
            double errs = 0;*/
            Mat F = findFundamentalMat(F_prepoint, F_nextpoint, mask, FM_RANSAC, 0.7, 0.99);//F矩阵
			t4 = (double)clock() / CLOCKS_PER_SEC;
   //         for (int i = 0; i < mask.rows; i++)//mask是什么
   //         {
   //             if (mask.at<uchar>(i, 0) == 0);
   //             else
   //             {
   //                 ///circle(pre_frame, F_prepoint[i], 6, Scalar(255, 255, 0), 3);
   //                 double A = F.at<double>(0, 0)*F_prepoint[i].x + F.at<double>(0, 1)*F_prepoint[i].y + F.at<double>(0, 2);
   //                 double B = F.at<double>(1, 0)*F_prepoint[i].x + F.at<double>(1, 1)*F_prepoint[i].y + F.at<double>(1, 2);
   //                 double C = F.at<double>(2, 0)*F_prepoint[i].x + F.at<double>(2, 1)*F_prepoint[i].y + F.at<double>(2, 2);//极线
   //                 double dd = fabs(A*F_nextpoint[i].x + B*F_nextpoint[i].y + C) / sqrt(A*A + B*B);
   //                 errs += dd;
			//		if (dd > 0.1);
   //                   //  circle(pre_frame, F_prepoint[i], 6, Scalar(255, 0, 0), 3);
   //                 else
   //                 {
   //                     F2_prepoint.push_back(F_prepoint[i]);
   //                     F2_nextpoint.push_back(F_nextpoint[i]);
   //                 }
   //             }
   //         }
			////剔除粗大误差
   //         F_prepoint = F2_prepoint;
   //         F_nextpoint = F2_nextpoint;
   //         cout << "Errors in total is " << errs << "pixels" << endl;

			double total;
			total = 0;
            T.clear();
			int prepoint_size = prepoint.size();
			//vector<double> _dd;
			//for (int i = 0; i < prepoint_size-1; ++i)
   //         {
			//	
			//		if (state[i] != 0)
			//		{
			//			double A = F.at<double>(0, 0)*prepoint[i].x + F.at<double>(0, 1)*prepoint[i].y + F.at<double>(0, 2);
			//			double B = F.at<double>(1, 0)*prepoint[i].x + F.at<double>(1, 1)*prepoint[i].y + F.at<double>(1, 2);
			//			double C = F.at<double>(2, 0)*prepoint[i].x + F.at<double>(2, 1)*prepoint[i].y + F.at<double>(2, 2);
			//			double dd = fabs(A*nextpoint[i].x + B*nextpoint[i].y + C) / sqrt(A*A + B*B);
			//			// line(img_scale, Point((int)prepoint[i].x, (int)prepoint[i].y), Point((int)nextpoint[i].x, (int)nextpoint[i].y), Scalar{ 255, 255, 0 }, 2);
			//			// line(pre_frame, Point((int)prepoint[i].x, (int)prepoint[i].y), Point((int)nextpoint[i].x, (int)nextpoint[i].y), Scalar{ 0, 255, 0 }, 1);
			//			total = total + dd;
			//			//Judge outlier
			//			//Draw outliers
			//			// circle(pre_frame, prepoint[i], 3, Scalar(0, 0, 255), 2);
			//			//line(pre_frame, Point((int)prepoint[i].x, (int)prepoint[i].y), Point((int)nextpoint[i].x, (int)nextpoint[i].y), Scalar{ 0, 255, 0 }, 2);

			//		}
   //             
   //         }

			for (int i = 0; i < prepoint_size - 1; ++i)
			{

				if (state[i] != 0)
				{
					double A = F.at<double>(0, 0)*prepoint[i].x + F.at<double>(0, 1)*prepoint[i].y + F.at<double>(0, 2);
					double B = F.at<double>(1, 0)*prepoint[i].x + F.at<double>(1, 1)*prepoint[i].y + F.at<double>(1, 2);
					double C = F.at<double>(2, 0)*prepoint[i].x + F.at<double>(2, 1)*prepoint[i].y + F.at<double>(2, 2);
					double dd = fabs(A*nextpoint[i].x + B*nextpoint[i].y + C) / sqrt(A*A + B*B);
					// line(img_scale, Point((int)prepoint[i].x, (int)prepoint[i].y), Point((int)nextpoint[i].x, (int)nextpoint[i].y), Scalar{ 255, 255, 0 }, 2);
					// line(pre_frame, Point((int)prepoint[i].x, (int)prepoint[i].y), Point((int)nextpoint[i].x, (int)nextpoint[i].y), Scalar{ 0, 255, 0 }, 1);
					//cout << "average = " << 5.0 * (total / prepoint_size) << endl;
					//if (dd <= 5.0*(total / prepoint_size)) continue;
					if (dd < thre_dist2epipolar) continue;
					//if (!checkROI(nextpoint[i])) continue;
					//                    cout << "dis: " << dd << endl;
					//dis[T.size()] = dd;
					T.push_back(nextpoint[i]);//局外点集合

					//Draw outliers
					// circle(pre_frame, prepoint[i], 3, Scalar(0, 0, 255), 2);
					//line(pre_frame, Point((int)prepoint[i].x, (int)prepoint[i].y), Point((int)nextpoint[i].x, (int)nextpoint[i].y), Scalar{ 0, 255, 0 }, 2);

				}

			}
			

			t5 = (double)clock() / CLOCKS_PER_SEC;
			
			end_time = (double)clock() / CLOCKS_PER_SEC;
		
            
			/*std::cout << "MOD第一步耗时：" << t0 - start_time << "s" << std::endl;
			std::cout << "MOD第二步耗时：" << t1 - t0 << "s" << std::endl;
			std::cout << "MOD第三步耗时：" << t2 - t1 << "s" << std::endl;
			std::cout << "MOD第四步耗时：" << t3 - t2 << "s" << std::endl;
			std::cout << "MOD第五步耗时：" << t4 - t3 << "s" << std::endl;
			std::cout << "MOD第六步耗时：" << t5 - t4 << "s" << std::endl;*/
			/*std::cout << "MOD总耗时：" << end_time - start_time << "s" << std::endl;*/
       // t = (double)cvGetTickCount() - t;
       // cout << "cost time: " << t / ((double)cvGetTickFrequency()*1000.) << "ms" << endl;
		return T;
   // }
}
