#include "main.h"


namespace ControllerModule{
	void Display(){
		while(true){
			// RopoDevice::masterController.print(0,1,"degree: %.1lf",-RopoDevice::inertial.get_yaw());
			// pros::delay(100); 
			RopoDevice::masterController.print(2,0,"eX:%.3lf Y:%.3lf   ",RopoDevice::xEncodingDisk.GetPosX(),RopoDevice::xEncodingDisk.GetPosY());
			pros::delay(100); 
			RopoDevice::masterController.print(1,0,"xX: %.3lf mY:%.3lf  ",RopoDevice::xDrivePositionModule.GetPosX(),RopoDevice::xDrivePositionModule.GetPosY() );
			pros::delay(100); 
			RopoDevice::masterController.print(0,0,"gx:%.3lf y:%.3lf", (RopoDevice::GetTransformedPosition())[1],(RopoDevice::GetTransformedPosition())[2]);
			pros::delay(100); 
			// RopoDevice::masterController.print(2,0,"fx:%.3lf y:%.3lf", RopoDevice::gpsAddPosition.GetGpsTransformRelativePositionX(), RopoDevice::gpsAddPosition.GetGpsTransformRelativePositionY());
			// pros::delay(100); 
		}
	}

	void GpsUpdate(){
		RopoDevice::gpsAddPosition.GpsUpdate();
	}
}

void initialize() {
	pros::lcd::initialize();
	RopoDevice::DeviceInit();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	pros::Task *DisplayTask = new pros::Task(ControllerModule::Display);
	RopoController::AxisValueCast xVelocityInput(RopoDevice::masterController, pros::E_CONTROLLER_ANALOG_LEFT_Y , RopoController::Linear);
	RopoController::AxisValueCast yVelocityInput(RopoDevice::masterController, pros::E_CONTROLLER_ANALOG_LEFT_X , RopoController::Linear);
	RopoController::AxisValueCast wVelocityInput(RopoDevice::masterController, pros::E_CONTROLLER_ANALOG_RIGHT_X, RopoController::Exp);
	float xInput, yInput, wInput;
	RopoMath::Vector<float> inputVelocity(RopoMath::ColumnVector, 3);

	RopoController::ButtonTaskLine ButtonDetectLine(RopoDevice::masterController);
	ButtonDetectLine.AddButtonDetect(pros::E_CONTROLLER_DIGITAL_X, RopoController::Rising, ControllerModule::GpsUpdate);
	ButtonDetectLine.Enable();

	int cur = 0;
	/*
	while (true) {
		// FC FD 0A 00 00 00 01 00 00 00 6E 86 1B F0 F9 21 09 40 99 00 00 00 00 00 00 00 // 10 3.14 ture
		// FC FD 02 00 00 00 00 00 00 00 8a 76 15 52 7e 52 f2 3f 58 00 00 00 00 00 00 00 // 
		// FC FD ff ff ff 7f 01 00 00 00 8a 76 15 52 7e 52 f2 3f db 00 00 00 00 00 00 00 // 2147483647
		// FC FD 0A 00 00 00 01 01 00 00 6E 86 1B F0 F9 21 09 40 99 00 00 00 00 00 00 00 // Error
		pros::lcd::print(1, "Message: %d, %f, %d", RopoDevice::adapter.message.a, RopoDevice::adapter.message.b, RopoDevice::adapter.message.c);
		pros::lcd::print(2, "Reading: %d", RopoDevice::adapter.IsReading());
		pros::delay(10);
	}	
	*/

	while (true) {
		xInput =   xVelocityInput.GetAxisValue();
		yInput = - yVelocityInput.GetAxisValue();
		wInput = - wVelocityInput.GetAxisValue();
		wInput = wInput * (1 - 4.0 / 7.0 * fabs(sqrt(xInput * xInput + yInput * yInput)));
		inputVelocity[1] = (fabs(xInput) < 0.08) ? (0) : (xInput  * RopoParameter::CHASSIS_X_SCALE);
		inputVelocity[2] = (fabs(yInput) < 0.08) ? (0) : (yInput  * RopoParameter::CHASSIS_Y_SCALE);
		inputVelocity[3] = (fabs(wInput) < 0.08) ? (0) : (wInput  * RopoParameter::CHASSIS_W_SCALE);
		RopoDevice::chassisModule.MoveVelocity(inputVelocity);
		pros::delay(10);
	}
}
