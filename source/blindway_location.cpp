/*
*ä���Ķ�λ���㣬���Ȼ�ȡ���ͼ
*�޸����ڣ�2017.8.15
*/
#include "stdafx.h"
#include "blindway_location.h"
#include "blindway_new_recognition.h"
#include <fstream>
#include <time.h>

static int serial = 1;

//���캯��
bw_location::bw_location(){}

//�������ͼ
void bw_location::GetBirdviewImage(Mat im, Point3f CPlane)
{
	//���ͼ����
	//�����������ϵ���涯����ϵת������
	Mat R_cg = Mat::zeros(3, 3, CV_64F);
	Mat T_cg = Mat::zeros(3, 1, CV_64F);
	RandT(CPlane.x, CPlane.y, CPlane.z, R_cg, T_cg);

	//��x��˳ʱ����ת90�ȣ�ʹ�������ϵ��x�����ң�y�����£���ͼ������ϵ����һ��
	R_cg.convertTo(R_cg, CV_64F);
	Mat R_rev = (Mat_<double>(3, 3) <<
		1, 0, 0,
		0, -1, 0,
		0, 0, -1);
	R_cg = R_rev*R_cg;

	//ԭͼ��궨����
	Mat cameraL_K = (Mat_<double>(3, 3) << _cameraMatrix1[0][0] * 2, _cameraMatrix1[0][1], _cameraMatrix1[0][2] * 2,
		_cameraMatrix1[1][0], _cameraMatrix1[1][1] * 2, _cameraMatrix1[1][2] * 2,
		_cameraMatrix1[2][0], _cameraMatrix1[2][1], _cameraMatrix1[2][2]);

	//��������������ϵz����ͼ��Ľ��㣬��Ϊ��궨���������
	Mat bird_pinX = (Mat_<double>(3, 1) << CPlane.x, CPlane.y, CPlane.z);
	Mat bird_pinlx = cameraL_K*bird_pinX;
	//��궨���󣬽���λ�ò��䣬�����˳���Χ��ʹ�ܹ�����ԭ�ȵ�Ŀ��
	Mat bird_cameraL_K = cameraL_K.clone();
	bird_cameraL_K.at<double>(0, 2) = bird_pinlx.at<double>(0) / bird_pinlx.at<double>(2);
	bird_cameraL_K.at<double>(1, 2) = bird_pinlx.at<double>(1) / bird_pinlx.at<double>(2);

	//����ԭͼ�����ͼ��ת������
	H_ob = bird_cameraL_K*R_cg*(cameraL_K.inv());

	//ת��Ϊ���ͼ			
	warpPerspective(im, bird_imL, H_ob, Size(im.cols, im.rows), INTER_CUBIC);
}

//ä�������߶�λ
vector<Point3f> bw_location::blindwaytest(Mat image1, Mat image2, Mat homography, Mat showimage, Mat CameraR, Mat CameraT, Point3f CPlane)
{
	//����ʱ��ı���
	double t0, t1, t2, t3, t4, t5, t6;
	t0 = (double)clock() / CLOCKS_PER_SEC;

	//�洢�ļ��Ķ���
	char fm1[50], fm2[50], fm3[50], fm4[50], fm5[50];
	vector<Point3f> linepar(0);
	fstream myfile;
	myfile.open(data + "/LocationData.txt", ios::app);   //����һ���ļ�
	if (!myfile)                        //����ļ��Ƿ񴴽��ɹ�
	{
		cout << "error open" << endl;
		exit(0);
	}
	myfile << "filenum = " << filenum << endl;
	t1 = (double)clock() / CLOCKS_PER_SEC;

	//�������ͼ
	GetBirdviewImage(image1, CPlane);

	//����ä��ʶ���࣬�������ͼ
	Mat bird_show;
	bird_imL.copyTo(bird_show);
	bw_recognition bw_recognition(bird_imL);

	//�������������ֵΪ0
	int vBlindwayNum = 0, hBlindwayNum = 0;
	/*����ä���ķ�����*/
	vBlindwayNum += bw_recognition.cascadeDetect("hv_south_blindway_type2_0.4_29.xml", Scalar(0, 255, 0));
	vBlindwayNum += bw_recognition.cascadeDetect("northblindway4birdview.xml", Scalar(0, 0, 255));
	hBlindwayNum += bw_recognition.cascadeDetect("BhLeftblindwayBirdview.xml", Scalar(255, 0, 0));

	///*äУä���ķ�����*/
	////��ֱä��������
	//vBlindwayNum += bw_recognition.cascadeDetect("blindwayrecognise62.xml", Scalar(0, 0, 255));
	//vBlindwayNum += bw_recognition.cascadeDetect("blindwayrecognise63.xml", Scalar(0, 0, 255));
	////ˮƽä��������
	//hBlindwayNum += bw_recognition.cascadeDetect("blindwayrecognise67.xml", Scalar(0, 255, 0));
	//hBlindwayNum += bw_recognition.cascadeDetect("blindwayrecognise68.xml", Scalar(255, 0, 0));
	t2 = (double)clock() / CLOCKS_PER_SEC;

	//���ˮƽ�����������Ŀ�㹻�����������
	if (hBlindwayNum > 2)
	{
		//����
		linepar.push_back(Point3f(-1500, 0, 0));
		linepar.push_back(Point3f(-1500, 0, 1000));

		return linepar;
	}

	//�����ֱ�����������Ŀ�㹻����ֱ��
	if (vBlindwayNum > 1)
	{
		t3 = (double)clock() / CLOCKS_PER_SEC;

		//ä��ʶ����Է��������������ɸѡ�޳�
		bw_recognition.cascadeDisplay(vBlindwayNum, hBlindwayNum);
		//ä��ʶ�����ˮ���㷨�ָ����ȡä��ֱ��
		vector<Point2f> bird_leftimage = bw_recognition.watershedBlindway(filenum);

		//�������Ϊ�գ��򷵻ص�ä�����������ϵ����Ϊ0
		if (bird_leftimage.size() != 2)
		{
			linepar.push_back(Point3f(0, 0, 0));
			myfile << endl;
			myfile.close();
			return linepar;
		}
		//�������ͼ�ļ����
		/*line(bird_imL, bird_leftimage[0], bird_leftimage[1], cv::Scalar(255, 0, 0), 2);
		imshow("bird_recogniton", bird_imL);*/

		//�����ͼ����תΪԭͼ����
		Mat bird_leftpoint1 = (Mat_<double>(3, 1) << bird_leftimage[0].x, bird_leftimage[0].y, 1);
		Mat bird_leftpoint2 = (Mat_<double>(3, 1) << bird_leftimage[1].x, bird_leftimage[1].y, 1);
		Mat leftpoint1 = H_ob.inv()*bird_leftpoint1;
		Mat leftpoint2 = H_ob.inv()*bird_leftpoint2;
		vector<Point2f> leftimage = { Point2f(leftpoint1.at<double>(0) / leftpoint1.at<double>(2), leftpoint1.at<double>(1) / leftpoint1.at<double>(2)),
			Point2f(leftpoint2.at<double>(0) / leftpoint2.at<double>(2), leftpoint2.at<double>(1) / leftpoint2.at<double>(2)) };

		t4 = (double)clock() / CLOCKS_PER_SEC;

		//������ֱ���ӳ���1/3����1ͼ��߶�
		float k = (leftimage[1].y - leftimage[0].y + 0.1) / (leftimage[1].x - leftimage[0].x + 0.1);
		float b = leftimage[1].y - k*leftimage[1].x;
		//��ͼ��߶ȷ���ü�ֱ��
		Point ip1, ip2;
		ip1.y = fmax(fmin(leftimage[0].y, leftimage[1].y), showimage.rows / 3); //�ж���ߵ���1/3ͼ��߶ȵĹ�ϵ��ȡ����
		ip2.y = showimage.rows - 1; //1��ͼ��߶�
		ip1.x = (ip1.y - b) / k;
		ip2.x = (ip2.y - b) / k;
		line(showimage, ip1, ip2, cv::Scalar(255, 0, 0), 2);

		//������ֱ�ߵ����ϵ����
		vector<Point2f> linepoint1, linepoint2;
		//�洢��ͼ��ä��ֱ�ߵ��ͼ�����꣬����ѡȡֱ�ߵ�1/3���2/3������꣬��2����Ϊ�궨������С������
		Mat p = Mat::zeros(3, 2, CV_64F);
		p.at<double>(0, 0) = (2 * leftimage[0].x + leftimage[1].x) / 3 / 2;
		p.at<double>(1, 0) = (2 * leftimage[0].y + leftimage[1].y) / 3 / 2;
		p.at<double>(2, 0) = 1;
		p.at<double>(0, 1) = (leftimage[0].x + 2 * leftimage[1].x) / 3 / 2;
		p.at<double>(1, 1) = (leftimage[0].y + 2 * leftimage[1].y) / 3 / 2;
		p.at<double>(2, 1) = 1;
		linepoint1.push_back(Point2f(p.at<double>(0, 0), p.at<double>(1, 0)));
		linepoint1.push_back(Point2f(p.at<double>(0, 1), p.at<double>(1, 1)));

		//��Ӧ�Ծ��������ͼ��Ӧ��ä��ֱ�ߵ��ͼ������
	    //������궨����
		Mat cameraMatrixL = (Mat_<double>(3, 3) << _cameraMatrix1[0][0], _cameraMatrix1[0][1], _cameraMatrix1[0][2],
			_cameraMatrix1[1][0], _cameraMatrix1[1][1], _cameraMatrix1[1][2],
			_cameraMatrix1[2][0], _cameraMatrix1[2][1], _cameraMatrix1[2][2]);
		//������궨����
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

		//����ä��ֱ�ߵ�����ϵ����
		Point3f P1 = measureZ(linepoint1[0], linepoint2[0]);
		Point3f P2 = measureZ(linepoint1[1], linepoint2[1]);
		linepar.push_back(P1);
		linepar.push_back(P2);

		//�����ʹ洢��ͼ�������2�ָ�ԭͼ��С������
		circle(image2, Point(2 * r.at<double>(0, 0), 2 * r.at<double>(1, 0)), 3, cv::Scalar(255, 0, 0), 2);
		circle(image2, Point(2 * r.at<double>(0, 1), 2 * r.at<double>(1, 1)), 3, cv::Scalar(255, 0, 0), 2);
		line(image2, Point(2 * r.at<double>(0, 0), 2 * r.at<double>(1, 0)), Point(2 * r.at<double>(0, 1), 2 * r.at<double>(1, 1)), cv::Scalar(255, 0, 0), 2);
		/*sprintf_s(fm4, (data_dir[5] + "/Rightpoint%d").c_str(), filenum);
		strcat_s(fm4, ".jpg");
		imwrite(fm4, image2);*/

		t5 = (double)clock() / CLOCKS_PER_SEC;

		myfile << "���������ϵĵ�1ͼ������Ϊ��( " << p.at<double>(0, 0) << "," << p.at<double>(1, 0) << " )" << endl;
		myfile << "���������ϵĵ�2ͼ������Ϊ��( " << p.at<double>(0, 1) << "," << p.at<double>(1, 1) << " )" << endl;
		myfile << "���������ϵĵ�1ͼ������Ϊ��( " << r.at<double>(0, 0) << "," << r.at<double>(1, 0) << " )" << endl;
		myfile << "���������ϵĵ�2ͼ������Ϊ��( " << r.at<double>(0, 1) << "," << r.at<double>(1, 1) << " )" << endl;
		myfile << "�������ϵĵ�1��ά����Ϊ��( " << P1.x << "," << P1.y << "," << P1.z << " )" << endl;
		myfile << "�������ϵĵ�2��ά����Ϊ��( " << P2.x << "," << P2.y << "," << P2.z << " )" << endl;
		myfile << "ä��������ʶ���ʱ�� " << t4 - t3 << " s" << endl;
		myfile << "ä�������߶�λ��ʱ�� " << t5 - t4 << " s" << endl;

		//�����ͼ�л���
		Point3f n(P1.x - P2.x, P1.y - P2.y, P1.z - P2.z);
		double len = 600 / sqrt(n.x*n.x + n.y*n.y + n.z*n.z); //Ԥ�����1.2m��ȡ���¸�0.6m
		//ԭͼ��궨����
		Mat cameraL_K = (Mat_<double>(3, 3) << _cameraMatrix1[0][0] * 2, _cameraMatrix1[0][1], _cameraMatrix1[0][2] * 2,
			_cameraMatrix1[1][0], _cameraMatrix1[1][1] * 2, _cameraMatrix1[1][2] * 2,
			_cameraMatrix1[2][0], _cameraMatrix1[2][1], _cameraMatrix1[2][2]);
		//ѡȡ��0.6m��
		Mat c_leftpoint1 = (Mat_<double>(3, 1) << P1.x - n.x*len, P1.y - n.y*len, P1.z - n.z*len);
		Mat i_leftpoint1 = cameraL_K*c_leftpoint1;
		i_leftpoint1.at<double>(0) = i_leftpoint1.at<double>(0) / i_leftpoint1.at<double>(2);
		i_leftpoint1.at<double>(1) = i_leftpoint1.at<double>(1) / i_leftpoint1.at<double>(2);
		i_leftpoint1.at<double>(2) = 1;
		Mat b_leftpoint1 = H_ob*i_leftpoint1;
		b_leftpoint1.at<double>(0) = b_leftpoint1.at<double>(0) / b_leftpoint1.at<double>(2);
		b_leftpoint1.at<double>(1) = b_leftpoint1.at<double>(1) / b_leftpoint1.at<double>(2);
		//ѡȡ��0.6m��
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

	myfile << "����������ʱ�� " << t2 - t1 << " s" << endl;
	myfile << "ä��ģ���ܺ�ʱ�� " << t6 - t0 << " s" << endl;
	myfile << endl;
	myfile.close();

	return linepar;
}

