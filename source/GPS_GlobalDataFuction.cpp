#include "stdafx.h"
#include "GPS_GlobalDataFuction.h"
#include <Eigen/Dense>

#define e2  0.00669437999

bool initflag = false;
Mat IniR = (Mat_<float>(3, 3) << -1, 0, 0,
	0, -0.5, 0.8660254,
	0, 0.8660254, 0.5);  //c2*c1 matrix = [-1,0,0;0,-sin,cos;0,cos,sin]

int an_packet_transmit(an_packet_t *an_packet)
{
	an_packet_encode(an_packet);
	return SendBuf(an_packet_pointer(an_packet), an_packet_size(an_packet));
}

void set_sensor_ranges()
{
	an_packet_t *an_packet;
	sensor_ranges_packet_t sensor_ranges_packet;

	sensor_ranges_packet.permanent = TRUE;
	sensor_ranges_packet.accelerometers_range = accelerometer_range_4g;
	sensor_ranges_packet.gyroscopes_range = gyroscope_range_500dps;
	sensor_ranges_packet.magnetometers_range = magnetometer_range_2g;

	an_packet = encode_sensor_ranges_packet(&sensor_ranges_packet);

	an_packet_transmit(an_packet);

	an_packet_free(&an_packet);
}

void Euler2Rotmateix(float& yaw, float& roll, float& pitch, cv::Mat& rot){
	Eigen::Matrix3f cp, cr, cy, cpb, cnb, c1, c2;
	if (rot.rows == 3 && rot.cols == 3)
	{
		//东北天->北东地
		c1 << 0, 1, 0,
			1, 0, 0,
			0, 0, -1;
		//惯导->相机坐标系
		//matrix = [0,-1,0;-sin,0,-cos;cos,0,-sin]
		c2 << 0, -1, 0, 
			-0.5, 0, -0.8660254,
			0.8660254, 0, -0.5;
		cr << 1, 0, 0,
			0, cos(roll), sin(roll),
			0, -sin(roll), cos(roll);
		cp << cos(pitch), 0, -sin(pitch),
			0, 1, 0,
			sin(pitch), 0, cos(pitch);
		cy << cos(yaw), sin(yaw), 0,
			-sin(yaw), cos(yaw), 0,
			0, 0, 1;
		cnb = c2*cr*cp*cy*c1; //东北天->相机坐标系
		for (size_t i = 0; i < 3; i++)
		{
			for (size_t j = 0; j < 3; j++)
			{
				rot.at<float>(i, j) = cnb(i, j);
			}
		}
	}
	else
	{
		std::cout << "输入参数rot错误" << endl;
	}
}

double RotMatrix2Euler(const Eigen::Matrix3d& Rot)
{
	double Euler;
	Eigen::Matrix3d Rotation, c1, c2;
	//东北天->北东地
	c1 << 0, 1, 0,
		1, 0, 0,
		0, 0, -1;
	//惯导->相机坐标系
	c2 << 0, -1, 0,
		-0.5, 0, -0.8660254,
		0.8660254, 0, -0.5;

	Rotation = c2.inverse()*Rot*c1.inverse();
	Euler = atan2(Rotation(0, 1), Rotation(0, 0));//Ny/Nx
	//Euler[1] = atan2(-1 * Rotation[2], (Rotation[0] * cos(Euler[0]) + Rotation[1] * sin(Euler[0])));//Nz/(Nx+Ny)
	//Euler[2] = atan2((Rotation[6] * sin(Euler[0]) - Rotation[7] * cos(Euler[0])), (-1 * Rotation[3] * sin(Euler[0]) + Rotation[4] * cos(Euler[0])));//Ax-Ay/-Ox+Oy
	return Euler;
}
double RotMatrix2EulerSLAM(const Eigen::Matrix3d& Rot)
{
	double Euler;
	Eigen::Matrix3d Rotation, c1, c2;
	//东北天->北东地
	c1 << 0, 1, 0,
		1, 0, 0,
		0, 0, -1;
	//惯导->相机坐标系
	c2 << 0, -1, 0,
		-0.5, 0, -0.8660254,
		0.8660254, 0, -0.5;

	Rotation = Rot;
	Euler = atan2(Rotation(0, 1), Rotation(0, 0));//Ny/Nx
	//Euler[1] = atan2(-1 * Rotation[2], (Rotation[0] * cos(Euler[0]) + Rotation[1] * sin(Euler[0])));//Nz/(Nx+Ny)
	//Euler[2] = atan2((Rotation[6] * sin(Euler[0]) - Rotation[7] * cos(Euler[0])), (-1 * Rotation[3] * sin(Euler[0]) + Rotation[4] * cos(Euler[0])));//Ax-Ay/-Ox+Oy
	return Euler;
}

cv::Mat ConvertLatLon2RT(cv::Point3d Orign, cv::Point3d current, cv::Mat Rotation)
{
	cv::Mat tg = cv::Mat(3, 1, CV_32F);
	cv::Mat t = cv::Mat(3, 1, CV_32F);
	double Re;
	Re = Ra / sqrt(1 - e2*sin(Orign.x));
	tg.at<float>(1) = (Orign.x - current.x)*Re;
	tg.at<float>(0) = (Orign.y - current.y)*Re*cos(current.x);
	tg.at<float>(2) = 0;
	t = Rotation*tg;
	//cout << "t: " << tg << endl;
	//初始坐标系下
	return tg;
}

Point2d ConvertLatLon2XY(double originLat, double originLon, double goalLat, double goalLon){
	double Re;
	Point2d a;
	Re = Ra / sqrt(1 - e2*sin(originLat));
	a.y = (goalLat - originLat)* Re;
	a.x = (goalLon - originLon)* Re*cos(originLat);
	//cout << "a = " << a << endl;
	return a;
}

bool ISArriveDestination(cv::Point2d goal, cv::Point2d orig, double radius){
	double distance = 0.0;
	distance = sqrt((goal.x - orig.x)*(goal.x - orig.x) + (goal.y - orig.y)*(goal.y - orig.y));
	if (distance < radius)
	{
		return true;
	}
	else return false;
}
