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

	static RopoChassis::ChassisModule chassisModule(leftFrontMotorModule,
													leftBackMotorModule,
													rightBackMotorModule,
													rightFrontMotorModule);

	// Adapter
	static RopoSensor::RaspberryPiAdapter adapter(RopoParameter::ADAPTER_PARAMETER_PORT, 115200, 5);
	// static RopoSensor::Debugger adapter(RopoParameter::ADAPTER_PARAMETER_PORT, 115200, 5);
}

#endif // ROPO_DEVICE_HPP