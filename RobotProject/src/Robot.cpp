#include <iostream>
#include <memory>
#include <string>
#include <math.h>

#include <XboxController.h>
#include <SampleRobot.h>
#include <RobotDrive.h>
#include <Timer.h>
#include <PIDController.h>
#include <Compressor.h>
#include <Solenoid.h>
#include <CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cscore_oo.h>



#include <DriveTrain.h>
/* Welcome! This is Team 4187's 2017 FIRST Robotics Competition code. Go Roborams!
 * All of our code is in one file right now. Lol, what the hell.
 * We had to create our own Robot class (Screw inheriting!)
 * 		We had a reason, okay. We wanted a 6 CIM drive base and didn't see a default one.
 */


class Robot: public frc::SampleRobot {
	VPBSDrive* myRobot = new VPBSDrive(1,3,5,0,2,4,0,1,2,3,1,2); // robot drive system
	frc::XboxController* controller = new frc::XboxController(0);

private:

	//This is a seperate thread that handles the camera screen on the Dashboard.
	static void CameraThread() {
		cs::UsbCamera camera0 = cs::UsbCamera("USB Camera 0", 0);
		cs::UsbCamera camera1 = cs::UsbCamera("USB Camera 1", 1);

		camera0.SetResolution(320, 240);
		camera1.SetResolution(320, 240);

		cs::CvSink cvSink0 = CameraServer::GetInstance()->GetVideo(camera0);
		cs::CvSink cvSink1 = CameraServer::GetInstance()->GetVideo(camera1);

		cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("2CamFeed", 320, 240);

		cv::Mat source; //Raw Image Mat
		cv::Mat output; //Edited image after we make it black and white

		frc::XboxController* controller = new frc::XboxController(0);
		bool wasAPressed = false;
		bool camera = true; //true for camera 0; false for camera 1

		while (true) {
			if (camera) {
				cvSink0.GrabFrame(source);
			} else {
				cvSink1.GrabFrame(source);
			}

			//cvtColor(source, output, cv::COLOR_BGR2GRAY);
			output = source;
			outputStreamStd.PutFrame(output);

			bool isADown = controller->GetAButton();

			if (!wasAPressed and isADown) {
				camera = !camera;
			}

			wasAPressed = isADown;
		}
	}

public:
	Robot() {
		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
		myRobot->SetExpiration(0.1);
	}

	void RobotInit() {
		std::cout<<"this is std::out"<<std::endl;
		std::thread cameraThread(CameraThread);
		cameraThread.detach();
		myRobot->DownShift();
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
		myRobot->Drive(0,.25);
		frc::Wait(1);
		myRobot->Drive(0,0);
	}
	void OperatorControl() override {
		myRobot->SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled()) {
			//if(winch is running){myRobot->NoDrive; *in class* void NoDrive(){this->speedGov = .001} else{ if(myRobot->NotDriving()){myRobot->GrantDrive();}}
			myRobot->TankDrive(controller);
			myRobot->rDriveEncoder->GetRate();
			if (controller->GetPOV() == 0){
				myRobot->UpShift();
			}
			if (controller->GetPOV() == 180) {
				myRobot->DownShift();
			}
			if(myRobot->GetCurVoltage() < myRobot->GetMinVoltage()){
				//stop intake && other junk like compressor
			}
			//frc::SmartDashboard::PutBoolean("Compressor Running", myCompressor.Enabled());
			// wait for a motor update time
			frc::Wait(0.005);
		}
	}
	void Test() override {
		while(IsEnabled()){
			frc::SmartDashboard::PutNumber("encoderRaw", myRobot->rDriveEncoder->GetRaw());
			frc::SmartDashboard::PutNumber("encoderDis", myRobot->rDriveEncoder->GetDistance());
			frc::SmartDashboard::PutNumber("encoderRate", myRobot->rDriveEncoder->GetRate());
			std::cout<<myRobot->rDriveEncoder->GetRate()<<std::endl;
			frc::Wait(.5);
		}
	}
};

START_ROBOT_CLASS(Robot)
