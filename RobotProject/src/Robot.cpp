#include <iostream>
#include <memory>
#include <string>

#include <Joystick.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <RobotDrive.h>
#include <Timer.h>
#include <Spark.h>
#include <Encoder.h>
#include <PIDController.h>
#include <Compressor.h>
#include <Solenoid.h>
#include <DoubleSolenoid.h>

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will
 * automatically call your Autonomous and OperatorControl methods at the right
 * time as controlled by the switches on the driver station or the field
 * controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
class Robot: public frc::SampleRobot {
	frc::RobotDrive myRobot { 0, 1 }; // robot drive system
	frc::Joystick stick { 0 }; // only joystick

	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";

	frc::Spark myMotor {2};
	//encoder name {port A, port B, reversed?, Accuracy/Precision}
	frc::Encoder leftDriveEncoder {1,2, false, Encoder::EncodingType::k1X};
	frc::Encoder rightDriveEncoder {3,4, false, Encoder::EncodingType::k1X};
	//frc::Spark *motorPtr = &myMotor;
	//frc::Encoder *encoderPtr = &myEncoder;
	//frc::PIDController myPID { 1, .1, .01, encoderPtr, motorPtr};
	//frc::PIDController myPID { 1,.1,.01, &myEncoder, &myMotor};
	double highMaxSpeed = 16.00;
	double lowMaxSpeed = 9.00;
	double curMaxSpeed = lowMaxSpeed;

	frc::Compressor myCompressor {0};
	//Solenoids for shifting on single valve attached at PCM slot 1 (and 2)
	frc::DoubleSolenoid gearShifter { 1,2 };
	//frc::Solenoid mySingleSolenoid { 1 };

public:
	Robot() {
		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
		myRobot.SetExpiration(0.1);
	}

	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	void Autonomous() {
		auto autoSelected = chooser.GetSelected();
		// std::string autoSelected = frc::SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
			std::cout << "Running custom Autonomous" << std::endl;
			myRobot.SetSafetyEnabled(false);
			myRobot.Drive(-0.5, 1.0); // spin at half speed
			frc::Wait(2.0);                // for 2 seconds
			myRobot.Drive(0.0, 0.0);  // stop robot
		} else {
			// Default Auto goes here
			std::cout << "Running default Autonomous" << std::endl;
			myRobot.SetSafetyEnabled(false);
			myRobot.Drive(-0.5, 0.0); // drive forwards half speed
			frc::Wait(2.0);                // for 2 seconds
			myRobot.Drive(0.0, 0.0);  // stop robot
		}
	}

	/*
	 * Runs the motors with arcade steering.
	 */
	void OperatorControl() override {
		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled()) {

			/**
			 This is for manual Shifting, we want Automatic to prevent driver from taking off in high gear
			 //low gear
			if(stick.GetPOV() == 0){
				myDoubleSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
				speedLimit = 1;
				curMaxSpeed = lowMaxSpeed;
			}
			//high gear
			if(stick.GetPOV() == 180){
				myDoubleSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
				speedLimit = .9;
				curMaxSpeed = highMaxSpeed;
			}
			**/
			if(leftDriveEncoder.GetRate() >= lowMaxSpeed && rightDriveEncoder.GetRate() >= lowMaxSpeed){
				//if both sides are moving at low max speed or higher then shift to high gear
				gearShifter.Set(frc::DoubleSolenoid::Value::kReverse);
			}
			if(leftDriveEncoder.GetRate() <= lowMaxSpeed && rightDriveEncoder.GetRate() <= lowMaxSpeed){
				//if both sides are moving slower than max speed then shift to low gear
				gearShifter.Set(frc::DoubleSolenoid::Value::kForward);
			}

			// drive with tank style (use both sticks)
			myRobot.TankDrive(stick, 1, stick, 5);
			//myMotor.Set(1);

			/**
			  int speedLimit = 1;
			  //use PID controller to smoothly change distance (convert to speed)
			if(abs(stick.GetRawAxis(2))>speedLimit){
				myPID.SetSetpoint( curMaxSpeed*(stick.GetRawAxis(2)/abs(stick.GetRawAxis(2)))*speedLimit);
			}
			else{
				myPID.SetSetpoint(stick.GetRawAxis(2));
			}
			**/
			
			frc::SmartDashboard::PutBoolean("Compressor Running", myCompressor.Enabled());
			frc::SmartDashboard::PutNumber("Current Pressure psi", myCompressor.GetCompressorCurrent());
			frc::SmartDashboard::PutNumber("Left Encoder Value", leftDriveEncoder.GetRaw());
			frc::SmartDashboard::PutNumber("Right Encoder Value", rightDriveEncoder.GetRaw());

			// wait for a motor update time
			frc::Wait(0.005);
			//possibility to prevent waiting too long?
			//if((.005-(curTime-initTime))>0){frc::Wait(0.005 - (curTime-initTime));}
		}
	}

	/*
	 * Runs during test mode
	 */
	void Test() override {

	}
};

START_ROBOT_CLASS(Robot)
