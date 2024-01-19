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
	//1.73
	static constexpr float CHASSIS_X_SCALE 			= 1.6f;//1.49
	static constexpr float CHASSIS_Y_SCALE 			= 1.0f;//0.86
	static constexpr float CHASSIS_W_SCALE 			= 7.0f;//9.56

	// Adapter Parameter
	static constexpr int ADAPTER_PARAMETER_PORT 	= 12;

	// EncodingDisk Parameter
	static constexpr int ENCODINGDISK_RECEIVE_ID = 20;
	static constexpr int ENCODINGDISK_RECEIVE_BAUDRATE = 115200;
	static constexpr int ENCODINGDISK_SEND_ID = 13;
	static constexpr int ENCODINGDISK_SEND_BAUDRATE    = 115200;
	static constexpr int ENCODINGDISK_SAMPLINGDELAY = 10;


	

	//gps 
	//    对于GPS，则将gps对准方向作为gps的前方，第一个参数为gps端口号
	//    第2、3个参数是车体中心在整个场地坐标系中的x、y位置，y轴正方向为0°，x轴正方向是90°，角度遵循顺时针
	//    第 4 个参数是gps传感器初始化时朝向场地的角度方向，y轴正方向为0°，角度遵循顺时针
	//    第 5、6个参数gps位于车体的坐标，若gps所视方向为车体0°，车体y轴正方向为车体0°，则车体x轴正方向为车体90°，角度遵循顺时针
	static constexpr double FIELD_HEADING_INITIAL = 90.0;//比赛场地相对于战队场地，逆时针为+
	static constexpr double ROPO_HEADING_INITIAL = 90.0 - FIELD_HEADING_INITIAL;//对于战队场地X轴
	
	static constexpr int GPS_PORT = 8;
	static constexpr double GPSX_INITIAL_0 = -0.6;
	static constexpr double GPSY_INITIAL_0 = 0.6;
	static double GPSX_INITIAL =  GPSX_INITIAL_0 * RopoMath::Cos(FIELD_HEADING_INITIAL) + GPSY_INITIAL_0 * RopoMath::Sin(FIELD_HEADING_INITIAL);
	static double GPSY_INITIAL = -GPSX_INITIAL_0 * RopoMath::Sin(FIELD_HEADING_INITIAL) + GPSY_INITIAL_0 * RopoMath::Cos(FIELD_HEADING_INITIAL);
	
	static constexpr double GPS_HEADING_INITIAL_0 = 270;
	static constexpr double GPS_HEADING_INITIAL = (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL) >= 360 ? (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL -  360) : (GPS_HEADING_INITIAL_0 + FIELD_HEADING_INITIAL);
	
	static constexpr double GPSX_OFFSET = 0.000;
	static constexpr double GPSY_OFFSET = 0.080;


};

#endif // ROPO_PARAMETER_HPP