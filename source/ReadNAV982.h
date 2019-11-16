#ifndef READNAV982_H
#define READNAV982_H

#include <iostream>

typedef struct
{
	unsigned int fixtype;
	double latitude;
	double longitude;
	double height;
	float roll;
	float pitch;
	float yaw;
	float yaw_raw;
	float position_standard_deviation[3];
	float euler_orientation_standard_deviation[3];
} GPSIMU;

namespace ORB_SLAM2
{
	class ReadNAV982
	{
	public:
		ReadNAV982();
		void Run();
		GPSIMU GetGPSIMU();
		bool GetGNSSFixFlag();
		unsigned int GetGNSSFixState();
	private:
		GPSIMU CurrentData;
		bool gnss_fix_enable;
		unsigned int gnss_fix_state;
	};
}
#endif