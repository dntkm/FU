#ifndef ROPO_PARAMETER_HPP
#define ROPO_PARAMETER_HPP

#include "pros/motors.hpp"
#include "RopoMath/Misc.hpp"

namespace RopoParameter {
	// Chassis Motor Parameter
	static constexpr pros::motor_gearset_e CHASSIS_MOTOR_GEARSET = pros::E_MOTOR_GEAR_BLUE;
	static constexpr int LEFT_FRONT_MOTOR_PORT[] 	= {5 , 4};
	static constexpr int LEFT_BACK_MOTOR_PORT[] 	= {1 , 2};
	static constexpr int RIGHT_BACK_MOTOR_PORT[] 	= {10, 9};
	static constexpr int RIGHT_FRONT_MOTOR_PORT[] 	= {7 , 6};

	// Chassis Shape Parameter
	static constexpr float CHASSIS_WHEEL_ANGLE 		= (float)RopoMath::Pi / 3.0; // (rad)
	static constexpr float CHASSIS_PARA_L 			= 282.29f / 1000.0f; // (m)
	static constexpr float CHASSIS_WHEEL_R 			= 3.25f * 25.4f / 2.0f / 1000.0f; // (m)
	static constexpr float CHASSIS_WHEEL_GEAR_RATIO = 3.0f / 2.0f;

	// Chassis Control Parameter
	static constexpr float CHASSIS_X_SCALE 			= 2.3f;
	static constexpr float CHASSIS_Y_SCALE 			= 1.4f;
	static constexpr float CHASSIS_W_SCALE 			= 8.0f;

	// Adapter Parameter
	static constexpr int ADAPTER_PARAMETER_PORT 	= 12;
};

#endif // ROPO_PARAMETER_HPP