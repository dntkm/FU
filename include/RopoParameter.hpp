#ifndef ROPO_PARAMETER_HPP
#define ROPO_PARAMETER_HPP

#include "pros/motors.hpp"
#include "RopoMath/Misc.hpp"

namespace RopoParameter {
	// Chassis Motor Parameter
	static constexpr pros::motor_gearset_e CHASSIS_MOTOR_GEARSET = pros::E_MOTOR_GEAR_BLUE;
	static constexpr int LEFT_FRONT_MOTOR_PORT[] 	= {4 , 3};
	static constexpr int LEFT_BACK_MOTOR_PORT[] 	= {14 , 1};
	static constexpr int RIGHT_BACK_MOTOR_PORT[] 	= {9, 10};
	static constexpr int RIGHT_FRONT_MOTOR_PORT[] 	= {6 , 7};

	// Chassis Shape Parameter
	static constexpr float CHASSIS_WHEEL_ANGLE 		= (float)RopoMath::Pi / 3.0; // (rad)
	static constexpr float CHASSIS_PARA_L 			= 181.89f / 1000.0f; // (m)
	static constexpr float CHASSIS_WHEEL_R 			= 3.25f * 25.4f / 2.0f / 1000.0f; // (m)
	static constexpr float CHASSIS_WHEEL_GEAR_RATIO = 3.0f / 2.0f;
	
	// Imu Port Parameter
	static constexpr int IMU_PORT = 5;

	// Chassis Control Parameter
	static constexpr float CHASSIS_X_SCALE 			= 2.3f;
	static constexpr float CHASSIS_Y_SCALE 			= 1.4f;
	static constexpr float CHASSIS_W_SCALE 			= 8.0f;

	// Adapter Parameter
	static constexpr int ADAPTER_PARAMETER_PORT 	= 12;

	// EncodingDisk Parameter
	static constexpr int EncodingDisk_Receive_ID = 20;
	static constexpr int EncodingDisk_Receive_Baudrate = 115200;
	static constexpr int EncodingDisk_Send_ID = 13;
	static constexpr int EncodingDisk_Send_Baudrate    = 115200;
	int EncodingDisk_SamplingDelay = 10;

	//gps
	static constexpr int GPS_PORT = 8;
	static constexpr double GPSX_INITIAL = 0;
	static constexpr double GPSY_INITIAL = 0;
	static constexpr double GPS_HEADING_INITIAL = 0;
	static constexpr double GPSX_OFFSET = 0;
	static constexpr double GPSY_OFFSET = 0;
};

#endif // ROPO_PARAMETER_HPP