/*
 * DriveTrain.h
 *
 *  Created on: Feb 4, 2017
 *      Author: Wyatt Marks
 */

#ifndef SRC_DRIVETRAIN_H_
#define SRC_DRIVETRAIN_H_
#include <RobotBase.h>
#include <Encoder.h>
#include <Spark.h>
#include <DoubleSolenoid.h>
#include <PowerDistributionPanel.h>
#include <SmartDashboard/SmartDashboard.h>
#include <XboxController.h>
#include <ADXRS450_Gyro.h>


class VPBSDrive {
private:
	//Driving related variables

	double lowMaxSpeed = 100.00; //Low gear max-speed. Currently a guess.
	double highMaxSpeed = 340.00; //High gear max-speed. Also a guess.
	double curMaxSpeed = lowMaxSpeed; //Max speed of the current gear.
	double lowSpeedGov = 1; //literally no governing
	double sensitivity = 0.5; //default
	frc::Spark* rSide [3];
	frc::Spark* lSide [3];


	//PID variables - AKA black magic
	double rCurSpeed = 0;
	double rSetSpeed = rCurSpeed;
	double rPrevSpeed = rCurSpeed;
	double rTErr = 0;
	double rPrevErr = 0;
	double lCurSpeed = 0;
	double lSetSpeed = lCurSpeed;
	double lPrevSpeed = lCurSpeed;
	double lTErr = 0;
	double lPrevErr = 0;
	double pK = 0.5;
	double iK = 0;
	double dK = 0;

	//Pneumatic related objects and stuff

	frc::DoubleSolenoid* gearShifter;
	frc::DoubleSolenoid::Value highGear = frc::DoubleSolenoid::Value::kReverse;
	frc::DoubleSolenoid::Value lowGear = frc::DoubleSolenoid::Value::kForward;

	//Encoder related stuff

	//double dPerPulse = (4 /*<- Wheel Diameter*/ * 3.1415 * (1/3) /*<- Ratio*/)/48 /*<- Pulse Per Revolution*/;
	double dPerPulse = (4 * 3.1415)/(144);
	double minRate = 1; //inches per second? to be considered moving

	//Power Stuff

	double curVolt;
	double lowVolt;
	frc::PowerDistributionPanel* pdp;
	double minVolt = 9.0;
public:
	//Public Encoder & Gyro stuffs
	frc::Encoder* rDriveEncoder;
	frc::Encoder* lDriveEncoder;
	frc::ADXRS450_Gyro* gyro;

	VPBSDrive (int frm, int crm, int brm, int flm, int clm, int blm, int reA, int reB, int leA, int leB, int solA, int solB);

	virtual ~VPBSDrive() = default;

	void DownShift();

	void UpShift();

	void PureTankDrive(double rVal, double lVal, bool nullZone = true);
	void PIDDrive(double rVal, double lVal);

	//Tank Driving! Now with PID control and Shifting all automagically inside!
	void TankDrive (frc::XboxController* controller);

	//Autonomous driving
	void Drive(double mag, double curve);
	void DriveDis(double desiredDis, double epsilon);
	void Turn(double desiredTurn, double epsilon);

	//Basically switch gears lol
	void ToggleGearShifter();

	//Stuff we have to implement to make program happy
	double GetCurVoltage();
	double GetLowVoltage();
	double GetMinVoltage();
	void SetExpiration(double timeout);
	double GetExpiration();
	bool IsSafetyEnabled();
	void SetSafetyEnabled(bool enabled);
	void StopMotor();
};

#endif /* SRC_DRIVETRAIN_H_ */
