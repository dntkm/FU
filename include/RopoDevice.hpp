#ifndef ROPO_DEVICE_HPP
#define ROPO_DEVICE_HPP

#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include "RopoParameter.hpp"
#include "RopoWheelModule.hpp"
#include "RopoChassis.hpp"
#include "RopoController.hpp"
#include "RopoSensor/RaspberryPiAdapter.hpp"
#include "RopoSensor/Debugger.hpp"
#include "RopoSensor/EncodingDisk.hpp"

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

	static pros::Imu Inertial(RopoParameter::IMU_PORT);

	static RopoSensor::EncodingDisk XEncodingDisk(RopoParameter::EncodingDisk_Receive_ID,
													RopoParameter::EncodingDisk_Receive_Baudrate,
													RopoParameter::EncodingDisk_Send_ID,
													RopoParameter::EncodingDisk_Send_Baudrate,
													RopoParameter::EncodingDisk_SamplingDelay);


	static RopoChassis::ChassisModule chassisModule(leftFrontMotorModule,
													leftBackMotorModule,
													rightBackMotorModule,
													rightFrontMotorModule);

	// Adapter
	static RopoSensor::RaspberryPiAdapter adapter(RopoParameter::ADAPTER_PARAMETER_PORT, 115200, 5);
	// static RopoSensor::Debugger adapter(RopoParameter::ADAPTER_PARAMETER_PORT, 115200, 5);

	void DeviceInit(){
		Inertial.reset(true);
		while(Inertial.is_calibrating())pros::delay(20);
		RopoDevice::masterController.clear();
		pros::delay(100);
		RopoDevice::masterController.print(0,0,"IMU Ready!!");
		pros::delay(100);
		XEncodingDisk.SetZero();
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