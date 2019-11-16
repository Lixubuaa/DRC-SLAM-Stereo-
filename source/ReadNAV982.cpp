#include "stdafx.h"
#include "ReadNAV982.h"
#include "GPS_GlobalDataFuction.h"
#include "generalDataFuction.h"
#include <fstream> 
#include <iostream>


using namespace std;

namespace ORB_SLAM2{
	ReadNAV982::ReadNAV982(){}
	void ReadNAV982::Run()
	{
		gnss_fix_enable = false;
		gnss_fix_state = 0;
		string strFileLatlon = data + "/FileLatlon.txt";
		string strFileRotation = data + "/FileRotation.txt";
		ofstream FileLatlon(strFileLatlon.c_str(), ios::app);
		ofstream FileRotation(strFileRotation.c_str(), ios::app);

		an_decoder_t an_decoder;
		an_packet_t *an_packet;
		system_state_packet_t system_state_packet;
		raw_sensors_packet_t raw_sensors_packet;
		position_standard_deviation_packet_t position_standard_deviation_packet;
		euler_orientation_standard_deviation_packet_t  euler_orientation_standard_deviation_packet;
		int bytes_received = 0;
		an_decoder_initialise(&an_decoder);

		while (1)
		{
			if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
			{
				/* increment the decode buffer length by the number of bytes received */
				an_decoder_increment(&an_decoder, bytes_received);

				/* decode all the packets in the buffer */
				while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
				{
					if (an_packet->id == packet_id_system_state) /* system state packet */
					{
						/* copy all the binary data into the typedef struct for the packet */
						/* this allows easy access to all the different values             */
						if (decode_system_state_packet(&system_state_packet, an_packet) == 0)
						{
							gnss_fix_state = system_state_packet.filter_status.b.gnss_fix_type;

							if (system_state_packet.filter_status.b.gnss_fix_type < 2)
								gnss_fix_enable = false;
							else
								gnss_fix_enable = true;
							CurrentData.fixtype = system_state_packet.filter_status.b.gnss_fix_type;
							CurrentData.latitude = system_state_packet.latitude;
							CurrentData.longitude = system_state_packet.longitude;
							CurrentData.height = system_state_packet.height;
							CurrentData.roll = system_state_packet.orientation[0];
							CurrentData.pitch = system_state_packet.orientation[1];
							CurrentData.yaw = system_state_packet.orientation[2];
							FileLatlon << setprecision(15) << system_state_packet.latitude*RADIANS_TO_DEGREES << "," << system_state_packet.longitude*RADIANS_TO_DEGREES << endl; //纬度、经度(角度)
							FileRotation << setprecision(6) << system_state_packet.orientation[0] * RADIANS_TO_DEGREES << "," << system_state_packet.orientation[1] * RADIANS_TO_DEGREES << system_state_packet.orientation[2] * RADIANS_TO_DEGREES << endl;
						}
					}
					else if (an_packet->id == packet_id_position_standard_deviation)
					{
						if (decode_position_standard_deviation_packet(&position_standard_deviation_packet, an_packet) == 0)
						{
							CurrentData.position_standard_deviation[0] = position_standard_deviation_packet.standard_deviation[0];
							CurrentData.position_standard_deviation[1] = position_standard_deviation_packet.standard_deviation[1];
							CurrentData.position_standard_deviation[2] = position_standard_deviation_packet.standard_deviation[2];

						}
						
					}
					else if (an_packet->id == packet_id_euler_orientation_standard_deviation)
					{
						if (decode_euler_orientation_standard_deviation_packet(&euler_orientation_standard_deviation_packet, an_packet) == 0)
						{
							CurrentData.euler_orientation_standard_deviation[0] = euler_orientation_standard_deviation_packet.standard_deviation[0];
							CurrentData.euler_orientation_standard_deviation[1] = euler_orientation_standard_deviation_packet.standard_deviation[1];
							CurrentData.euler_orientation_standard_deviation[2] = euler_orientation_standard_deviation_packet.standard_deviation[2];
						}
					}
					/* Ensure that you free the an_packet when your done with it or you will leak memory */
					an_packet_free(&an_packet);
				}
			}
			Sleep(10);
		}
	}

	GPSIMU ReadNAV982::GetGPSIMU()
	{
		return CurrentData;
	}
	bool ReadNAV982::GetGNSSFixFlag()
	{
		return gnss_fix_enable;
	}
	unsigned int ReadNAV982::GetGNSSFixState()
	{
		return gnss_fix_state;
	}
}