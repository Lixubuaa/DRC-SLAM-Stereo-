/*
*盲道的定位计算，首先获取鸟瞰图
*修改日期：2017.8.15
*/
#include "stdafx.h"
#include "blindway_location.h"
#include "blindway_new_recognition.h"
#include <fstream>
#include <time.h>

static int serial = 1;

//构造函数
bw_location::bw_location(){}

//计算鸟瞰图
void bw_location::GetBirdviewImage(Mat im, Point3f CPlane)
{
	//鸟瞰图计算
	//计算相机坐标系到随动坐标系转换矩阵
	Mat R_cg = Mat::zeros(3, 3, CV_64F);
	Mat T_cg = Mat::zeros(3, 1, CV_64F);
	RandT(CPlane.x, CPlane.y, CPlane.z, R_cg, T_cg);

	//以x轴顺时针旋转90度，使相机坐标系的x轴向右，y轴向下，与图像坐标系方向一致
	R_cg.convertTo(R_cg, CV_64F);
	Mat R_rev = (Mat_<double>(3, 3) <<
		1, 0, 0,
		0, -1, 0,
		0, 0, -1);
	R_cg = R_rev*R_cg;

	//原图像标定矩阵
	Mat cameraL_K = (Mat_<double>(3, 3) << _cameraMatrix1[0][0] * 2, _cameraMatrix1[0][1], _cameraMatrix1[0][2] * 2,
		_cameraMatrix1[1][0], _cameraMatrix1[1][1] * 2, _cameraMatrix1[1][2] * 2,
		_cameraMatrix1[2][0], _cameraMatrix1[2][1], _cameraMatrix1[2][2]);

	//计算鸟瞰相机坐标系z轴与图像的交点，作为鸟瞰标定矩阵的主点
	Mat bird_pinX = (Mat_<double>(3, 1) << CPlane.x, CPlane.y, CPlane.z);
	Mat bird_pinlx = cameraL_K*bird_pinX;
	//鸟瞰标定矩阵，焦距位置不变，扩大了成像范围，使能够看到原先的目标
	Mat bird_cameraL_K = cameraL_K.clone();
	bird_cameraL_K.at<double>(0, 2) = bird_pinlx.at<double>(0) / bird_pinlx.at<double>(2);
	bird_cameraL_K.at<double>(1, 2) = bird_pinlx.at<double>(1) / bird_pinlx.at<double>(2);

	//计算原图像到鸟瞰图的转换矩阵
	H_ob = bird_cameraL_K*R_cg*(cameraL_K.inv());

	//转换为鸟瞰图			
	warpPerspective(im, bird_imL, H_ob, Size(im.cols, im.rows), INTER_CUBIC);
}

//盲道中心线定位
vector<Point3f> bw_location::blindwaytest(Mat image1, Mat image2, Mat homography, Mat showimage, Mat CameraR, Mat CameraT, Point3f CPlane)
{
	//计算时间的变量
	double t0, t1, t2, t3, t4, t5, t6;
	t0 = (double)clock() / CLOCKS_PER_SEC;

	//存储文件的定义
	char fm1[50], fm2[50], fm3[50], fm4[50], fm5[50];
	vector<Point3f> linepar(0);
	fstream myfile;
	myfile.open(data + "/LocationData.txt", ios::app);   //创建一个文件
	if (!myfile)                        //检查文件是否创建成功
	{
		cout << "error open" << endl;
		exit(0);
	}
	myfile << "filenum = " << filenum << endl;
	t1 = (double)clock() / CLOCKS_PER_SEC;

	//计算鸟瞰图
	GetBirdviewImage(image1, CPlane);

	//调用盲道识别类，传入鸟瞰图
	Mat bird_show;
	bird_imL.copyTo(bird_show);
	bw_recognition bw_recognition(bird_imL);

	//分类器检测结果初值为0
	int vBlindwayNum = 0, hBlindwayNum = 0;
	/*北航盲道的分类器*/
	vBlindwayNum += bw_recognition.cascadeDetect("hv_south_blindway_type2_0.4_29.xml", Scalar(0, 255, 0));
	vBlindwayNum += bw_recognition.cascadeDetect("northblindway4birdview.xml", Scalar(0, 0, 255));
	hBlindwayNum += bw_recognition.cascadeDetect("BhLeftblindwayBirdview.xml", Scalar(255, 0, 0));

	///*盲校盲道的分类器*/
	////垂直盲道分类器
	//vBlindwayNum += bw_recognition.cascadeDetect("blindwayrecognise62.xml", Scalar(0, 0, 255));
	//vBlindwayNum += bw_recognition.cascadeDetect("blindwayrecognise63.xml", Scalar(0, 0, 255));
	////水平盲道分类器
	//hBlindwayNum += bw_recognition.cascadeDetect("blindwayrecognise67.xml", Scalar(0, 255, 0));
	//hBlindwayNum += bw_recognition.cascadeDetect("blindwayrecognise68.xml", Scalar(255, 0, 0));
	t2 = (double)clock() / CLOCKS_PER_SEC;

	//如果水平分类器检测数目足够，则向左拐弯
	if (hBlindwayNum > 2)
	{
		//向左
		linepar.push_back(Point3f(-1500, 0, 0));
		linepar.push_back(Point3f(-1500, 0, 1000));

		return linepar;
	}

	//如果垂直分类器检测数目足够，则直行
	if (vBlindwayNum > 1)
	{
		t3 = (double)clock() / CLOCKS_PER_SEC;

		//盲道识别类对分类器检测结果进行筛选剔除
		bw_recognition.cascadeDisplay(vBlindwayNum, hBlindwayNum);
		//盲道识别类分水岭算法分割和提取盲道直线
		vector<Point2f> bird_leftimage = bw_recognition.watershedBlindway(filenum);

		//若检测结果为空，则返回的盲道中心线相机系坐标为0
		if (bird_leftimage.size() != 2)
		{
			linepar.push_back(Point3f(0, 0, 0));
			myfile << endl;
			myfile.close();
			return linepar;
		}
		//画出鸟瞰图的检测结果
		/*line(bird_imL, bird_leftimage[0], bird_leftimage[1], cv::Scalar(255, 0, 0), 2);
		imshow("bird_recogniton", bird_imL);*/

		//将鸟瞰图坐标转为原图坐标
		Mat bird_leftpoint1 = (Mat_<double>(3, 1) << bird_leftimage[0].x, bird_leftimage[0].y, 1);
		Mat bird_leftpoint2 = (Mat_<double>(3, 1) << bird_leftimage[1].x, bird_leftimage[1].y, 1);
		Mat leftpoint1 = H_ob.inv()*bird_leftpoint1;
		Mat leftpoint2 = H_ob.inv()*bird_leftpoint2;
		vector<Point2f> leftimage = { Point2f(leftpoint1.at<double>(0) / leftpoint1.at<double>(2), leftpoint1.at<double>(1) / leftpoint1.at<double>(2)),
			Point2f(leftpoint2.at<double>(0) / leftpoint2.at<double>(2), leftpoint2.at<double>(1) / leftpoint2.at<double>(2)) };

		t4 = (double)clock() / CLOCKS_PER_SEC;

		//将检测的直线延长至1/3——1图像高度
		float k = (leftimage[1].y - leftimage[0].y + 0.1) / (leftimage[1].x - leftimage[0].x + 0.1);
		float b = leftimage[1].y - k*leftimage[1].x;
		//在图像高度方向裁剪直线
		Point ip1, ip2;
		ip1.y = fmax(fmin(leftimage[0].y, leftimage[1].y), showimage.rows / 3); //判断最高点与1/3图像高度的关系，取大者
		ip2.y = showimage.rows - 1; //1个图像高度
		ip1.x = (ip1.y - b) / k;
		ip2.x = (ip2.y - b) / k;
		line(showimage, ip1, ip2, cv::Scalar(255, 0, 0), 2);

		//计算检测直线的相机系坐标
		vector<Point2f> linepoint1, linepoint2;
		//存储左图像盲道直线点的图像坐标，这里选取直线的1/3点和2/3点的坐标，除2是因为标定参数缩小了两倍
		Mat p = Mat::zeros(3, 2, CV_64F);
		p.at<double>(0, 0) = (2 * leftimage[0].x + leftimage[1].x) / 3 / 2;
		p.at<double>(1, 0) = (2 * leftimage[0].y + leftimage[1].y) / 3 / 2;
		p.at<double>(2, 0) = 1;
		p.at<double>(0, 1) = (leftimage[0].x + 2 * leftimage[1].x) / 3 / 2;
		p.at<double>(1, 1) = (leftimage[0].y + 2 * leftimage[1].y) / 3 / 2;
		p.at<double>(2, 1) = 1;
		linepoint1.push_back(Point2f(p.at<double>(0, 0), p.at<double>(1, 0)));
		linepoint1.push_back(Point2f(p.at<double>(0, 1), p.at<double>(1, 1)));

		//单应性矩阵计算右图对应的盲道直线点的图像坐标
	    //左相机标定矩阵
		Mat cameraMatrixL = (Mat_<double>(3, 3) << _cameraMatrix1[0][0], _cameraMatrix1[0][1], _cameraMatrix1[0][2],
			_cameraMatrix1[1][0], _cameraMatrix1[1][1], _cameraMatrix1[1][2],
			_cameraMatrix1[2][0], _cameraMatrix1[2][1], _cameraMatrix1[2][2]);
		//右相机标定矩阵
		Mat cameraMatrixR = (Mat_<double>(3, 3) << _cameraMatrix2[0][0], _cameraMatrix2[0][1], _cameraMatrix2[0][2],
			_cameraMatrix2[1][0], _cameraMatrix2[1][1], _cameraMatrix2[1][2],
			_cameraMatrix2[2][0], _cameraMatrix2[2][1], _cameraMatrix2[2][2]);
		//R,T
		Mat R = (Mat_<double>(3, 3) << _R[0][0], _R[0][1], _R[0][2],
			_R[1][0], _R[1][1], _R[1][2],
			_R[2][0], _R[2][1], _R[2][2]);
		Mat T = (Mat_<double>(3, 1) << _T[0], _T[1], _T[2]);
		Mat norm = (Mat_<double>(3, 1) << CPlane.x, CPlane.y, CPlane.z);
		Mat H = cameraMatrixR*(R - T*norm.t()/(-1))*cameraMatrixL.inv();
		
		Mat r = H*p;
		linepoint2.push_back(Point2f(r.at<double>(0, 0), r.at<double>(1, 0)));
		linepoint2.push_back(Point2f(r.at<double>(0, 1), p.at<double>(1, 1)));

		//计算盲道直线点的相机系坐标
		Point3f P1 = measureZ(linepoint1[0], linepoint2[0]);
		Point3f P2 = measureZ(linepoint1[1], linepoint2[1]);
		linepar.push_back(P1);
		linepar.push_back(P2);

		//画出和存储右图结果，乘2恢复原图大小的坐标
		circle(image2, Point(2 * r.at<double>(0, 0), 2 * r.at<double>(1, 0)), 3, cv::Scalar(255, 0, 0), 2);
		circle(image2, Point(2 * r.at<double>(0, 1), 2 * r.at<double>(1, 1)), 3, cv::Scalar(255, 0, 0), 2);
		line(image2, Point(2 * r.at<double>(0, 0), 2 * r.at<double>(1, 0)), Point(2 * r.at<double>(0, 1), 2 * r.at<double>(1, 1)), cv::Scalar(255, 0, 0), 2);
		/*sprintf_s(fm4, (data_dir[5] + "/Rightpoint%d").c_str(), filenum);
		strcat_s(fm4, ".jpg");
		imwrite(fm4, image2);*/

		t5 = (double)clock() / CLOCKS_PER_SEC;

		myfile << "左中心线上的点1图像坐标为：( " << p.at<double>(0, 0) << "," << p.at<double>(1, 0) << " )" << endl;
		myfile << "左中心线上的点2图像坐标为：( " << p.at<double>(0, 1) << "," << p.at<double>(1, 1) << " )" << endl;
		myfile << "右中心线上的点1图像坐标为：( " << r.at<double>(0, 0) << "," << r.at<double>(1, 0) << " )" << endl;
		myfile << "右中心线上的点2图像坐标为：( " << r.at<double>(0, 1) << "," << r.at<double>(1, 1) << " )" << endl;
		myfile << "中心线上的点1三维坐标为：( " << P1.x << "," << P1.y << "," << P1.z << " )" << endl;
		myfile << "中心线上的点2三维坐标为：( " << P2.x << "," << P2.y << "," << P2.z << " )" << endl;
		myfile << "盲道中心线识别耗时： " << t4 - t3 << " s" << endl;
		myfile << "盲道中心线定位耗时： " << t5 - t4 << " s" << endl;

		//在鸟瞰图中绘制
		Point3f n(P1.x - P2.x, P1.y - P2.y, P1.z - P2.z);
		double len = 600 / sqrt(n.x*n.x + n.y*n.y + n.z*n.z); //预设距离1.2m，取上下各0.6m
		//原图像标定矩阵
		Mat cameraL_K = (Mat_<double>(3, 3) << _cameraMatrix1[0][0] * 2, _cameraMatrix1[0][1], _cameraMatrix1[0][2] * 2,
			_cameraMatrix1[1][0], _cameraMatrix1[1][1] * 2, _cameraMatrix1[1][2] * 2,
			_cameraMatrix1[2][0], _cameraMatrix1[2][1], _cameraMatrix1[2][2]);
		//选取下0.6m点
		Mat c_leftpoint1 = (Mat_<double>(3, 1) << P1.x - n.x*len, P1.y - n.y*len, P1.z - n.z*len);
		Mat i_leftpoint1 = cameraL_K*c_leftpoint1;
		i_leftpoint1.at<double>(0) = i_leftpoint1.at<double>(0) / i_leftpoint1.at<double>(2);
		i_leftpoint1.at<double>(1) = i_leftpoint1.at<double>(1) / i_leftpoint1.at<double>(2);
		i_leftpoint1.at<double>(2) = 1;
		Mat b_leftpoint1 = H_ob*i_leftpoint1;
		b_leftpoint1.at<double>(0) = b_leftpoint1.at<double>(0) / b_leftpoint1.at<double>(2);
		b_leftpoint1.at<double>(1) = b_leftpoint1.at<double>(1) / b_leftpoint1.at<double>(2);
		//选取上0.6m点
		Mat c_leftpoint2 = (Mat_<double>(3, 1) << P1.x + n.x*len, P1.y + n.y*len, P1.z + n.z*len);
		Mat i_leftpoint2 = cameraL_K*c_leftpoint2;
		i_leftpoint2.at<double>(0) = i_leftpoint2.at<double>(0) / i_leftpoint2.at<double>(2);
		i_leftpoint2.at<double>(1) = i_leftpoint2.at<double>(1) / i_leftpoint2.at<double>(2);
		i_leftpoint2.at<double>(2) = 1;
		Mat b_leftpoint2 = H_ob*i_leftpoint2;
		b_leftpoint2.at<double>(0) = b_leftpoint2.at<double>(0) / b_leftpoint2.at<double>(2);
		b_leftpoint2.at<double>(1) = b_leftpoint2.at<double>(1) / b_leftpoint2.at<double>(2);

		circle(bird_show, Point(b_leftpoint1.at<double>(0), b_leftpoint1.at<double>(1)), 3, cv::Scalar(0, 255, 0), 2);
		circle(bird_show, Point(b_leftpoint2.at<double>(0), b_leftpoint2.at<double>(1)), 3, cv::Scalar(0, 0, 255), 2);
		line(bird_show, Point(b_leftpoint1.at<double>(0), b_leftpoint1.at<double>(1)), Point(b_leftpoint2.at<double>(0), b_leftpoint2.at<double>(1)), cv::Scalar(255, 0, 0), 2);

		imshow("bird", bird_show);
		char fml[50];
		sprintf_s(fml, (data_dir[6] + "/bird_imL%d").c_str(), serial++);
		strcat_s(fml, ".jpg");
		imwrite(fml, bird_show);
	}
	else
		linepar.push_back(Point3f(0, 0, 0));

	t6 = (double)clock() / CLOCKS_PER_SEC;

	myfile << "分类器检测耗时： " << t2 - t1 << " s" << endl;
	myfile << "盲道模块总耗时： " << t6 - t0 << " s" << endl;
	myfile << endl;
	myfile.close();

	return linepar;
}

