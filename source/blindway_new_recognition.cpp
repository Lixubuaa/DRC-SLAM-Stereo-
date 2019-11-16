/*
*��������ä������ɫä����ʶ�𣬼���б��ʹ�ֱä����ʶ��
*���������ϡ����ķ�ɸѡֱ��
*�޸����ڣ�2017.7.21
*/
#include "stdafx.h"
#include "blindway_new_recognition.h"
#include <fstream>
#include <time.h>

//���캯��������ԭͼ��Ĭ�ϼ��y����ֱ�߼���ֱ����ֱ��
bw_recognition::bw_recognition(Mat src) :allblindways(0), frame(src), directH(0), directV(1){}

//canny��ⴹֱ�����Ե��Ϊopencv�ڲ�canny��������ȥ��ˮƽ����sobel�ݶȵļ��㣬ʹ�ú���ֻ���㴹ֱ�����Ե
void bw_recognition::dxCanny(InputArray _src, OutputArray _dst, double low_thresh, double high_thresh)
{
	int aperture_size = 3;
	bool L2gradient = false;

	Mat src = _src.getMat();
	CV_Assert(src.depth() == CV_8U);

	_dst.create(src.size(), CV_8U);
	Mat dst = _dst.getMat();

	if (!L2gradient && (aperture_size & CV_CANNY_L2_GRADIENT) == CV_CANNY_L2_GRADIENT)
	{
		//backward compatibility
		aperture_size &= ~CV_CANNY_L2_GRADIENT;
		L2gradient = true;
	}

	if ((aperture_size & 1) == 0 || (aperture_size != -1 && (aperture_size < 3 || aperture_size > 7)))
		CV_Error(CV_StsBadFlag, "");

	if (low_thresh > high_thresh)
		std::swap(low_thresh, high_thresh);

	const int cn = src.channels();
	Mat dx(src.rows, src.cols, CV_16SC(cn));
	Mat dy(src.rows, src.cols, CV_16SC(cn));

	Sobel(src, dx, CV_16S, 1, 0, aperture_size, 1, 0, cv::BORDER_REPLICATE);
	//Sobel(src, dy, CV_16S, 0, 1, aperture_size, 1, 0, cv::BORDER_REPLICATE);

	if (L2gradient)
	{
		low_thresh = std::min(32767.0, low_thresh);
		high_thresh = std::min(32767.0, high_thresh);

		if (low_thresh > 0) low_thresh *= low_thresh;
		if (high_thresh > 0) high_thresh *= high_thresh;
	}
	int low = cvFloor(low_thresh);
	int high = cvFloor(high_thresh);

	ptrdiff_t mapstep = src.cols + 2;
	AutoBuffer<uchar> buffer((src.cols + 2)*(src.rows + 2) + cn * mapstep * 3 * sizeof(int));

	int* mag_buf[3];
	mag_buf[0] = (int*)(uchar*)buffer;
	mag_buf[1] = mag_buf[0] + mapstep*cn;
	mag_buf[2] = mag_buf[1] + mapstep*cn;
	memset(mag_buf[0], 0, /* cn* */mapstep*sizeof(int));

	uchar* map = (uchar*)(mag_buf[2] + mapstep*cn);
	memset(map, 1, mapstep);
	memset(map + mapstep*(src.rows + 1), 1, mapstep);

	int maxsize = std::max(1 << 10, src.cols * src.rows / 10);
	std::vector<uchar*> stack(maxsize);
	uchar **stack_top = &stack[0];
	uchar **stack_bottom = &stack[0];

	/* sector numbers
	(Top-Left Origin)

	1   2   3
	*  *  *
	* * *
	0*******0
	* * *
	*  *  *
	3   2   1
	*/

#define CANNY_PUSH(d)    *(d) = uchar(2), *stack_top++ = (d)
#define CANNY_POP(d)     (d) = *--stack_top

	// calculate magnitude and angle of gradient, perform non-maxima suppression.
	// fill the map with one of the following values:
	//   0 - the pixel might belong to an edge
	//   1 - the pixel can not belong to an edge
	//   2 - the pixel does belong to an edge
	for (int i = 0; i <= src.rows; i++)
	{
		int* _norm = mag_buf[(i > 0) + 1] + 1;
		if (i < src.rows)
		{
			short* _dx = dx.ptr<short>(i);
			short* _dy = dy.ptr<short>(i);

			if (!L2gradient)
			{
				for (int j = 0; j < src.cols*cn; j++)
					_norm[j] = std::abs(int(_dx[j])) + std::abs(int(_dy[j]));
			}
			else
			{
				for (int j = 0; j < src.cols*cn; j++)
					_norm[j] = int(_dx[j])*_dx[j] + int(_dy[j])*_dy[j];
			}

			if (cn > 1)
			{
				for (int j = 0, jn = 0; j < src.cols; ++j, jn += cn)
				{
					int maxIdx = jn;
					for (int k = 1; k < cn; ++k)
					if (_norm[jn + k] > _norm[maxIdx]) maxIdx = jn + k;
					_norm[j] = _norm[maxIdx];
					_dx[j] = _dx[maxIdx];
					_dy[j] = _dy[maxIdx];
				}
			}
			_norm[-1] = _norm[src.cols] = 0;
		}
		else
			memset(_norm - 1, 0, /* cn* */mapstep*sizeof(int));

		// at the very beginning we do not have a complete ring
		// buffer of 3 magnitude rows for non-maxima suppression
		if (i == 0)
			continue;

		uchar* _map = map + mapstep*i + 1;
		_map[-1] = _map[src.cols] = 1;

		int* _mag = mag_buf[1] + 1; // take the central row
		ptrdiff_t magstep1 = mag_buf[2] - mag_buf[1];
		ptrdiff_t magstep2 = mag_buf[0] - mag_buf[1];

		const short* _x = dx.ptr<short>(i - 1);
		const short* _y = dy.ptr<short>(i - 1);

		if ((stack_top - stack_bottom) + src.cols > maxsize)
		{
			int sz = (int)(stack_top - stack_bottom);
			maxsize = maxsize * 3 / 2;
			stack.resize(maxsize);
			stack_bottom = &stack[0];
			stack_top = stack_bottom + sz;
		}

		int prev_flag = 0;
		for (int j = 0; j < src.cols; j++)
		{
#define CANNY_SHIFT 15
			const int TG22 = (int)(0.4142135623730950488016887242097*(1 << CANNY_SHIFT) + 0.5);

			int m = _mag[j];

			if (m > low)
			{
				int xs = _x[j];
				int ys = _y[j];
				int x = std::abs(xs);
				int y = std::abs(ys) << CANNY_SHIFT;

				int tg22x = x * TG22;

				if (y < tg22x)
				{
					if (m > _mag[j - 1] && m >= _mag[j + 1]) goto __ocv_canny_push;
				}
				else
				{
					int tg67x = tg22x + (x << (CANNY_SHIFT + 1));
					if (y > tg67x)
					{
						if (m > _mag[j + magstep2] && m >= _mag[j + magstep1]) goto __ocv_canny_push;
					}
					else
					{
						int s = (xs ^ ys) < 0 ? -1 : 1;
						if (m > _mag[j + magstep2 - s] && m > _mag[j + magstep1 + s]) goto __ocv_canny_push;
					}
				}
			}
			prev_flag = 0;
			_map[j] = uchar(1);
			continue;
		__ocv_canny_push:
			if (!prev_flag && m > high && _map[j - mapstep] != 2)
			{
				CANNY_PUSH(_map + j);
				prev_flag = 1;
			}
			else
				_map[j] = 0;
		}

		// scroll the ring buffer
		_mag = mag_buf[0];
		mag_buf[0] = mag_buf[1];
		mag_buf[1] = mag_buf[2];
		mag_buf[2] = _mag;
	}

	// now track the edges (hysteresis thresholding)
	while (stack_top > stack_bottom)
	{
		uchar* m;
		if ((stack_top - stack_bottom) + 8 > maxsize)
		{
			int sz = (int)(stack_top - stack_bottom);
			maxsize = maxsize * 3 / 2;
			stack.resize(maxsize);
			stack_bottom = &stack[0];
			stack_top = stack_bottom + sz;
		}

		CANNY_POP(m);

		if (!m[-1])         CANNY_PUSH(m - 1);
		if (!m[1])          CANNY_PUSH(m + 1);
		if (!m[-mapstep - 1]) CANNY_PUSH(m - mapstep - 1);
		if (!m[-mapstep])   CANNY_PUSH(m - mapstep);
		if (!m[-mapstep + 1]) CANNY_PUSH(m - mapstep + 1);
		if (!m[mapstep - 1])  CANNY_PUSH(m + mapstep - 1);
		if (!m[mapstep])    CANNY_PUSH(m + mapstep);
		if (!m[mapstep + 1])  CANNY_PUSH(m + mapstep + 1);
	}

	// the final pass, form the final image
	const uchar* pmap = map + mapstep + 1;
	uchar* pdst = dst.ptr();
	for (int i = 0; i < src.rows; i++, pmap += mapstep, pdst += dst.step)
	{
		for (int j = 0; j < src.cols; j++)
			pdst[j] = (uchar)-(pmap[j] >> 1);
	}
}

//���ķ�ɸѡ��ѱ߽�ֱ��
void bw_recognition::drawDetectLines(Mat& image, const vector<Vec4i> lines, vector<Point2f>& endpoints, vector<Point> centroid)
{
	//��⵽��ֱ����Ŀ
	int num = lines.size();
	float sum = 0, diff = 100;
	vector<vector<Point>> best_pt(0);

	//������ֱ��������ϼ��㣬���������ĵ���Թ�ϵɸѡ������ʵ�һ��ֱ��
	for (int a = 0; a < num; a++)
	{
		//��ȡֱ��1����
		Vec4i line1 = lines[a];
		Point pt11(line1[0], line1[1]);
		Point pt12(line1[2], line1[3]);
		float k1 = (float)(pt12.y - pt11.y) / (pt12.x - pt11.x + 0.01);
		float b1 = pt12.y - k1*pt12.x;

		//ֱ��1��y������ǲ�����60�Ȼ�x������ǲ�С��30��
		if (directV*(abs(atan(1 / k1)) > CV_PI / 3.) || directH*(-atan(1 / k1) < CV_PI / 6.))
			continue;

		for (int b = a + 1; b < num; b++)
		{
			//��ȡֱ��2����
			Vec4i line2 = lines[b];
			Point pt21(line2[0], line2[1]);
			Point pt22(line2[2], line2[3]);
			float k2 = (float)(pt22.y - pt21.y) / (pt22.x - pt21.x + 0.01);
			float b2 = pt22.y - k2*pt22.x;

			//ֱ��2��y������ǲ�����60�Ȼ�x������ǲ�С��30��
			if (directV*(abs(atan(1 / k2)) > CV_PI / 3.) || directH*(-atan(1 / k2) < CV_PI / 6.))
				continue;

			//����ÿ������
			for (int i = 0; i < centroid.size(); i++)
			{
				//���ĵ�ֱ��1��2���з��ž�����޷��ž���
				Point center = centroid[i];
				float dis1S = (k1*center.x - center.y + b1) / sqrt(k1*k1 + b1*b1);
				float dis2S = (k2*center.x - center.y + b2) / sqrt(k2*k2 + b2*b2);
				float dis1U = abs(dis1S), dis2U = abs(dis2S);
				//��ֱ��1��ֱ��2���������ࣨ����1�����Ҿ����ֵ��С������2����������������3������Ϊ���ֱ����ϣ�������ʵ����ұ߽�ֱ��
				if (k1*dis1S*k2*dis2S < 0 && abs(dis1U - dis2U) < diff && dis1U + dis2U > sum)
				{
					sum = dis1U + dis2U;
					diff = abs(dis1U - dis2U);
					vector<Point> maybe_pt1 = { Point(-b1 / k1, 0), Point((480 - b1) / k1, 480) };
					vector<Point> maybe_pt2 = { Point(-b2 / k2, 0), Point((480 - b2) / k2, 480) };
					best_pt.clear();
					best_pt.push_back(maybe_pt1);
					best_pt.push_back(maybe_pt2);
				}
			}
		}
	}

	//����ä���������߻�߽�����βͼ������
	if (best_pt.size() == 2)
	{
		//��������������
		line(image, best_pt[0][0], best_pt[0][1], Scalar(0, 0, 255), 2);
		line(image, best_pt[1][0], best_pt[1][1], Scalar(0, 0, 255), 2);
		endpoints.push_back(0.5*(best_pt[0][0] + best_pt[1][0]));
		endpoints.push_back(0.5*(best_pt[0][1] + best_pt[1][1]));

		////���ر߽�ֱ��������
		//int bottomx1 = (best_pt[0][0].y > best_pt[0][1].y) ? best_pt[0][0].x : best_pt[0][1].x;
		//int bottomx2 = (best_pt[1][0].y > best_pt[1][1].y) ? best_pt[1][0].x : best_pt[1][1].x;
		////<��߽�,>�ұ߽�
		//if (bottomx1 < bottomx2)
		//{
		//	endpoints.push_back(best_pt[0][0]);
		//	endpoints.push_back(best_pt[0][1]);
		//}
		//else
		//{
		//	endpoints.push_back(best_pt[1][0]);
		//	endpoints.push_back(best_pt[1][1]);
		//}

		line(image, endpoints[0], endpoints[1], Scalar(255, 0, 0), 2);
	}
}

//RANSAC���޳����Եľ���ä������������������Ϊ���������������ĵ�x��y���꣬���ؾ��ڽ�������
vector<int> bw_recognition::ransacFindInliners(vector<int> centers, float thresh)
{
	//���ݼ�
	vector<int> data = centers;
	//���ݼ���С
	int size = data.size();
	//���ڵ����
	int d2 = 0;     
	//���ֵ���ڵ㼯��
	vector<int> best_consensus_set;
	//���ֵ���ڵ���ż���
	vector<int> best_num;
	//��ѭ������
	int iterat = size*(size - 1) / 2; 
	//������
	int iterations = 0;

	while (iterations < iterat){
		vector<int>consensus_set;
		vector<int>maybe_num;
		float maybe_x;
		//���ѡȡ����x
		int a, b;
		do{
			a = rand() % data.size();
			b = rand() % data.size();
		} while (a == b);

		//������������ƽ��ֵ
		maybe_x = float(data[a] + data[b]) / 2;

		//�������е�
		for (int i = 0; i < data.size(); i++){
			if (abs(data[i] - maybe_x) < thresh) {
				consensus_set.push_back(data[i]);
				maybe_num.push_back(i);
			}
		}
		if ((consensus_set.size()>d2)){
			best_consensus_set = consensus_set;
			best_num = maybe_num;
			d2 = consensus_set.size();
		}
		iterations++;
	}

	/*float u = best_consensus_set.size();
	float v = data.size();
	float persent = u / v;
	cout << u << "  " << v << endl;
	cout << "ģ�ͷ����ʣ�" << persent << endl;*/
	return best_num;
}

//��������⣬���������·�����ƺʹ˷������������ʾ����ɫ
int bw_recognition::cascadeDetect(string name, Scalar color)
{
	//���η�����·������
	blindway_cascade_name = name;

	//�жϷ������Ƿ���ڣ��ܷ񱻼���
	if (!blindway_cascade.load(blindway_cascade_name)){
		cout << "[error] �޷����ؼ����������ļ���" << endl;
		return 0;
	}

	//��߶ȼ��ä�����򣬽����еļ����������ʱ������
	std::vector<Rect> blindways;
	blindway_cascade.detectMultiScale(frame, blindways, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(10, 10), Size(150, 150));

	//������ʾ�ĳ�Աͼ��detect_resultΪԭͼ��Ŀ���
	if (detect_result.empty())
		frame.copyTo(detect_result);

	//�þ��ο򻭳����η����������
	for (int i = 0; i < blindways.size(); i++){
		rectangle(detect_result, Rect(blindways[i].x, blindways[i].y, blindways[i].width, blindways[i].height), color, 2);
	}
	imshow("detect_result", detect_result);

	//�����η������ļ��������뵽��Ա����allblindways�У�ͨ�����ַ���ʹ��allblindways���԰�������������ļ����
	allblindways.insert(allblindways.end(), blindways.begin(), blindways.end());

	//���ر��η�����������Ŀ
	return blindways.size();
}

//�Լ����ɸѡ�޳�����ʾ���
void bw_recognition::cascadeDisplay(int vBlindwayNum, int hBlindwayNum)
{
	//��ԭRGBͼ��תΪ�Ҷ�ͼ
	Mat frame_gray;
	cvtColor(frame, frame_gray, CV_BGR2GRAY);

	//canny�㷨��ⴹֱ����ı�Ե����������ä������ɫä�����õĸߵ���ֵ��ͬ
	Mat canny;
	//dxCanny(frame_gray, canny, 300, 500); //����:300,500
	dxCanny(frame_gray, canny, 1, 250); //��ɫ:1,250

	//����Աͼ��cascade����ֵ
	if (cascade.empty())
	{
		Mat frame2(frame.size(), CV_8UC1, Scalar(0));
		frame2.copyTo(cascade);
	}

	//����RANSAC�޳������ä�������
	vector<int> centerXs(0), centerYs(0);
	for (int i = 0; i < allblindways.size(); i++){
		Point center(allblindways[i].x + allblindways[i].width*0.5, allblindways[i].y + allblindways[i].height*0.5);
		centerXs.push_back(center.x);
		centerYs.push_back(center.y);
	}

	//ѡȡ���ֵ
	vector<int> num(0);
	//����ֱä�������Ŀ����ˮƽä�������Ŀ�����xֵ�����޳�����Ĭ�ϼ�ⴹֱֱ��
	if (vBlindwayNum >= hBlindwayNum)
		num = ransacFindInliners(centerXs, 60);
	//��ˮƽä�������Ŀ���ڴ�ֱä�������Ŀ�����yֵ�����޳��������ˮƽֱ��
	else
	{
		num = ransacFindInliners(centerYs, 60);
		directH = 1;
		directV = 0;
	}

	//�����޳������ȡ���ڵ�ä�������
	vector<Rect> inliner_blindways(0);
	for (int i = 0; i < num.size(); i++)
		inliner_blindways.push_back(allblindways[num[i]]);

	//�Ծ��ڼ���������ñ�Ե����һ���޼���Ŀ�����ڽ�����ä���Ĳ����г���������ȫλ��ä�������ڵĲ���
	for (int i = 0; i < inliner_blindways.size(); i++){
		//����ʱͼ��tmp�л������ڼ������ȡ���ε�����ڽ�Բ
		Point center(inliner_blindways[i].x + inliner_blindways[i].width*0.5, inliner_blindways[i].y + inliner_blindways[i].height*0.5);
		Mat tmp(frame.size(), CV_8UC1, Scalar(0));
		circle(tmp, center, min(inliner_blindways[i].width*0.5, inliner_blindways[i].height*0.5), Scalar(255, 255, 255), -1);

		//ʹ��ä���߽�Ĵ�ֱ��Ե����ЩԲ�и�����ɲ���
		for (int i = 0; i < canny.rows; i++)
		{
			for (int j = 0; j < canny.rows; j++)
			{
				if (canny.at<uchar>(i, j) == 255)
					tmp.at<uchar>(i, j) = 0;
			}
		}
		erode(tmp, tmp, Mat(), Point(-1, -1), 2);

		//ѡȡÿ��Բ�и���������򲿷֣���Ϊ��Щ������򲿷�Ӧ��λ��ä��������
		vector<vector<Point>> contours;
		findContours(tmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		double area = 0;
		int label = 0;
		for (int i = 0; i < contours.size(); i++)
		{
			if (contourArea(contours[i]) >= area)
			{
				area = contourArea(contours[i]);
				label = i;
			}
		}
		//�����и��ļ����
		drawContours(cascade, contours, label, Scalar(255), -1);
	}

}

//��ˮ�뾫ȷ�ָ�ä��
vector<Point2f> bw_recognition::watershedBlindway(int filenum)
{
	//��ɸѡ�޳���ļ������Ϊ��ˮ���ǰ��ͼ������ä������ֵ��ɫ����������ֵ��ɫ
	Mat fg;
	erode(cascade, fg, Mat(), Point(-1, -1), 0);

	//��ǰ��ͼ���Ͻ���������ͣ�����ɫ������ͼ����Ϊ��ˮ��ı���ͼ���������е�����ֵ��ɫ��δ֪����ֵ��ɫ
	Mat bg;
	Mat element(5, 5, CV_8U, Scalar(1));
	dilate(cascade, bg, element, Point(-1, -1), 25); //25
	threshold(bg, bg, 1, 128, THRESH_BINARY_INV);

	//��ǰ���ͱ���ͼ��ϲ����õ���ˮ��ı��ͼ������ä������ֵ��ɫ�����е�����ֵ��ɫ��δ֪����ֵ��ɫ
	Mat markers(cascade.size(), CV_8U, Scalar(0));
	markers = fg + bg;

	//������ˮ��ָ����
	WatershedSegmenter segmenter;
	//���ñ��ͼ�񣬲����÷�ˮ���㷨
	segmenter.setMarkers(markers);
	segmenter.process(frame);

	//����ˮ������ɫ����ʱ�õ��ķ�ˮ�������������²��պ�
	Mat roi = cvScalar(255) - segmenter.getWatersheds();

	//��roi���±߸���ɫ�����Ապ�����
	line(roi, Point(0, 0), Point(roi.cols,0), Scalar(255), 2);
	line(roi, Point(0, roi.rows-1), Point(roi.cols, roi.rows-1), Scalar(255), 2);

	//ÿ���������������ν����Լ�С��Ե�������ֱ�߼���Ӱ�죬ͬʱ����һ��ÿ����������ģ�Ϊֱ��ɸѡ��׼��
	Mat croi(roi.size(), CV_8U, Scalar(0));
	vector<vector<Point>> contours;
	findContours(roi, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	vector<Point> centroid;
	for (int i = 0; i < contours.size(); i++)
	{
		//���������������׾�M00��һ�׾�M10,M01
		Moments mom = moments(contours[i],true);
		if (mom.m00 != 0)
		{
			//�ɾؼ����������꣬x=M10/M00��y=M01/M00
			circle(croi, Point(mom.m10 / mom.m00, mom.m01 / mom.m00), 2, Scalar(255), 2);
			centroid.push_back(Point(mom.m10 / mom.m00, mom.m01 / mom.m00));
		}

		//�����Ķ���ν��ƣ�����Χ��״����С͹�����
		vector<Point> poly;
		approxPolyDP(Mat(contours[i]), poly, 5, true);
		//��������ν��
		vector<Point>::const_iterator itp = poly.begin();
		while (itp != (poly.end() - 1)){
			line(croi, *itp, *(itp + 1), Scalar(255), 1);
			++itp;
		}
		line(croi, *(poly.begin()), *(poly.end() - 1), Scalar(255), 1);
	}

	//ֱ�߼��
	Mat result;
	frame.copyTo(result);
	vector<Point2f> points(0);
	vector<Vec4i> lines;
	//���ֱ�ߣ���СͶƱΪ30������������10����϶��С��5
	HoughLinesP(croi, lines, 1, CV_PI / 180, 30, 10, 5);
	//������ĵ���Ŀ��Ϊ0�����������ķ�ɸѡֱ��
	if (centroid.size() != 0)
		drawDetectLines(result, lines, points, centroid);
	//imshow("src2", result);

	//�洢���
	/*char fml[50];
	sprintf_s(fml, (data_dir[6] + "/Line%d").c_str(), filenum++);
	strcat_s(fml, ".jpg");
	imwrite(fml, result);*/
	return points;
}
