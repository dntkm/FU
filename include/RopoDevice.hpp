#ifndef ROPO_DEVICE_HPP
#define ROPO_DEVICE_HPP

#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include "pros/gps.hpp"
#include "RopoParameter.hpp"
#include "RopoWheelModule.hpp"
#include "RopoChassis.hpp"
#include "RopoController.hpp"
#include "RopoSensor/RaspberryPiAdapter.hpp"
#include "RopoSensor/Debugger.hpp"
#include "RopoSensor/EncodingDisk.hpp"
#include "RopoXDrivePosition.hpp"
#include "RopoGpsAddPosition.hpp"
#include "RopoApi.hpp"

namespace RopoDevice {
	// Controller
	static pros::Controller masterController(pros::E_CONTROLLER_MASTER);

	// Chassis Motors
	static pros::Motor leftFrontMotor0(	RopoParameter::LEFT_FRONT_MOTOR_PORT[0] ,
										RopoParameter::CHASSIS_MOTOR_GEARSET, true);
	static pros::Motor leftFrontMotor1(	RopoParameter::LEFT_FRONT_MOTOR_PORT[1] ,
										RopoParameter::CHASSIS_MOTOR_GEARSET, false);

	static pros::Motor leftBackMotor0(	RopoParameter::LEFT_BACK_MOTOR_PORT[0] ,
										RopoParameter::CHASSIS_MOTOR_GEARSET, true);
	static pros::Motor leftBackMotor1(	RopoParameter::LEFT_BACK_MOTOR_PORT[1] ,
										RopoParameter::CHASSIS_MOTOR_GEARSET, false);

	static pros::Motor rightBackMotor0(	RopoParameter::RIGHT_BACK_MOTOR_PORT[0] ,
										RopoParameter::CHASSIS_MOTOR_GEARSET, true);
	static pros::Motor rightBackMotor1(	RopoParameter::RIGHT_BACK_MOTOR_PORT[1] ,
										RopoParameter::CHASSIS_MOTOR_GEARSET, false);

	static pros::Motor rightFrontMotor0(RopoParameter::RIGHT_FRONT_MOTOR_PORT[0],
										RopoParameter::CHASSIS_MOTOR_GEARSET, true);
	static pros::Motor rightFrontMotor1(RopoParameter::RIGHT_FRONT_MOTOR_PORT[1],
										RopoParameter::CHASSIS_MOTOR_GEARSET, false);

	static RopoWheelModule::WheelModule leftFrontMotorModule(leftFrontMotor0, leftFrontMotor1);
	static RopoWheelModule::WheelModule leftBackMotorModule(leftBackMotor0, leftBackMotor1);
	static RopoWheelModule::WheelModule rightBackMotorModule(rightBackMotor0, rightBackMotor1);
	static RopoWheelModule::WheelModule rightFrontMotorModule(rightFrontMotor0, rightFrontMotor1);
//三周惯性传感器
	static pros::Imu inertial(RopoParameter::IMU_PORT);
//GPS定位S
	static pros::Gps vexGps(RopoParameter::GPS_PORT           , RopoParameter::GPSX_INITIAL, RopoParameter::GPSY_INITIAL,
						     RopoParameter::GPS_HEADING_INITIAL, RopoParameter::GPSX_OFFSET , RopoParameter::GPSY_OFFSET);
//码盘定位 
	static RopoSensor::EncodingDisk xEncodingDisk(RopoParameter::ENCODINGDISK_RECEIVE_ID,
													RopoParameter::ENCODINGDISK_RECEIVE_BAUDRATE,
													RopoParameter::ENCODINGDISK_SEND_ID,
													RopoParameter::ENCODINGDISK_SEND_BAUDRATE,
													RopoParameter::ENCODINGDISK_SAMPLINGDELAY);
//编码器定位
	static RopoXDrivePosition::XPositionModule xDrivePositionModule( leftFrontMotor1, leftBackMotor1, rightBackMotor1, rightFrontMotor1, inertial );
//获取角度
	FloatType GetHeading(){
		return -RopoDevice::inertial.get_yaw();
	}

// 坐标获取函数
	Vector GetPosition(){
		Vector PositionVector(RopoMath::ColumnVector,2);
		PositionVector[1] =  RopoDevice::xDrivePositionModule.GetPosX();
		PositionVector[2] =  RopoDevice::xDrivePositionModule.GetPosY();
		PositionVector[3] =  GetHeading();

		return PositionVector;
	}
    
	RopoGpsAddPosition::GpsAddPositionModule gpsAddPosition(GetPosition,vexGps,2);

	Vector GetTransformedPosition(){
		return gpsAddPosition.GetTransformedPosition();
	}
	static RopoChassis::ChassisModule chassisModule(leftFrontMotorModule,
													leftBackMotorModule,
													rightBackMotorModule,
													rightFrontMotorModule);

	// Adapter
	static RopoSensor::RaspberryPiAdapter adapter(RopoParameter::ADAPTER_PARAMETER_PORT, 115200, 5);
	// static RopoSensor::Debugger adapter(RopoParameter::ADAPTER_PARAMETER_PORT, 115200, 5);

	void DeviceInit(){
		inertial.reset(true);
		while(inertial.is_calibrating())pros::delay(20);
		RopoDevice::masterController.clear();
		pros::delay(100);
		RopoDevice::masterController.print(0,0,"IMU Ready!!");
		pros::delay(100);
		xEncodingDisk.SetZero();
		pros::delay(100);
		xDrivePositionModule.ResetPosition();
		leftFrontMotor0.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		leftFrontMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		leftBackMotor0.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		leftBackMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		rightBackMotor0.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		rightBackMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		rightFrontMotor0.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		rightFrontMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		
	}

}

#endif // ROPO_DEVICE_HPP