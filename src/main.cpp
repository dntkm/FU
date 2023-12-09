#include "main.h"
#include "RopoDevice.hpp"
#include "RopoController.hpp"

void initialize() {
	pros::lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	RopoController::AxisValueCast xVelocityInput(RopoDevice::masterController, pros::E_CONTROLLER_ANALOG_LEFT_Y , RopoController::Exp);
	RopoController::AxisValueCast yVelocityInput(RopoDevice::masterController, pros::E_CONTROLLER_ANALOG_LEFT_X , RopoController::Exp);
	RopoController::AxisValueCast wVelocityInput(RopoDevice::masterController, pros::E_CONTROLLER_ANALOG_RIGHT_X, RopoController::Exp);

	float xInput, yInput, wInput;
	RopoMath::Vector<float> inputVelocity(RopoMath::ColumnVector, 3);

	while (true) {
		xInput =   xVelocityInput.GetAxisValue();
		yInput = - yVelocityInput.GetAxisValue();
		wInput = - wVelocityInput.GetAxisValue();
		inputVelocity[1] = (fabs(xInput) < 0.08) ? (0) : (xInput  * RopoParameter::CHASSIS_X_SCALE);
		inputVelocity[2] = (fabs(yInput) < 0.08) ? (0) : (yInput  * RopoParameter::CHASSIS_Y_SCALE);
		inputVelocity[3] = (fabs(wInput) < 0.08) ? (0) : (wInput  * RopoParameter::CHASSIS_W_SCALE);
		RopoDevice::chassisModule.MoveVelocity(inputVelocity);
		pros::delay(10);
	}
}
