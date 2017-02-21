#include <iostream>
#include <memory>
#include <string>
#include <math.h>

#include <XboxController.h>
#include <SampleRobot.h>
#include <RobotDrive.h>
#include <Timer.h>
#include <Compressor.h>
#include <CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cscore_oo.h>
#include <Servo.h>


#include <DriveTrain.h>
/* Welcome! This is Team 4187's 2017 FIRST Robotics Competition code. Go Roborams!
 * All of our code is in one file right now. Lol, what the hell.
 * We had to create our own Robot class (Screw inheriting!)
 * 		We had a reason, okay. We wanted a 6 CIM drive base and didn't see a default one.
 */


class Robot: public frc::SampleRobot {
	VPBSDrive* myRobot = new VPBSDrive(3,4,5,0,1,2,2,3,0,1,4,5); // robot drive system
	frc::XboxController* controller = new frc::XboxController(0);
	frc::Compressor* compressor = new frc::Compressor();
	frc::DoubleSolenoid* gearDoor = new frc::DoubleSolenoid(2,6); //2 and 6 on the PCM
	frc::DoubleSolenoid* clawSol = new frc::DoubleSolenoid(7, 3);
	//frc::Spark* clawServo = new frc::Spark(11);
	frc::Servo* clawServo = new frc::Servo(10);
	frc::Servo* ballGate = new frc::Servo(11);
	frc::Spark* winch0 = new frc::Spark(6);
	frc::Spark* winch1 = new frc::Spark(7);
	frc::Spark* intake = new frc::Spark(8);
	frc::Spark* conveyor = new frc::Spark(9);
	double rightTriggerValue = 0;
	bool intakeRunning = false;
	bool conveyorRunning = false;
	bool compressorRunning = true;
	bool compressorButtons = false;
	bool leftBumperButton = false;
	bool rightBumperButton = false;
	bool leftTriggerDown = false;
	frc::DoubleSolenoid::Value gearOpen = frc::DoubleSolenoid::Value::kForward;
	frc::DoubleSolenoid::Value gearClose = frc::DoubleSolenoid::Value::kReverse;
	frc::DoubleSolenoid::Value clawIn = frc::DoubleSolenoid::Value::kReverse;
	frc::DoubleSolenoid::Value clawOut = frc::DoubleSolenoid::Value::kForward;
	double ballGateUp = 1;
	double ballGateDown = 0;
	double clawOpen = 1;
	double clawShut = 0;
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
		intake->Set(0);
		conveyor->Set(0);
		winch0->Set(0);
		winch1->Set(0);
		clawServo->Set(clawOpen);
		ballGate->Set(ballGateUp);
		compressor->Stop();
		clawSol->Set(clawIn);
		gearDoor->Set(gearClose);
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
		myRobot->SetSafetyEnabled(false);
		myRobot->DriveDis(10,2);
		frc::Wait(3);
		myRobot->DriveDis(-10,2);
		frc::Wait(3);
		myRobot->DriveDis(24,2);
		frc::Wait(3);
		myRobot->DriveDis(-24,2);
		frc::Wait(3);
		myRobot->Turn(45,10);
		frc::Wait(3);
		myRobot->Turn(-45,10);
	}
	void OperatorControl() override {
		myRobot->SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled()) {
			//driving
			myRobot->TankDrive(controller);
			//manual shifting
			/*if (controller->GetPOV() == 0){
				myRobot->UpShift();
			}
			if (controller->GetPOV() == 180) {
				myRobot->DownShift();
			} the other thing uses POV so just going to toggle with B or stick buttons until better solution arises
			*//*
			if(controller->GetAButton()){
				myRobot->DownShift();
			}
			if(controller->GetBButton()){
				myRobot->UpShift();
			}*/
			if(!leftTriggerDown and (controller->GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand) > .3)){
				myRobot->ToggleGearShifter();
			}
			leftTriggerDown = (controller->GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand));
			//power management
			if(myRobot->GetCurVoltage() < myRobot->GetMinVoltage()){
				intake->Set(0);
				conveyor->Set(0);
				compressor->Stop();
			}
			//gear door
			if(controller->GetXButton()){
				gearDoor->Set(gearClose);
			}
			if(controller->GetYButton()){
				gearDoor->Set(gearOpen);
			}
			//winch
			rightTriggerValue = controller->GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand);
			winch0->Set(rightTriggerValue);
			winch1->Set(rightTriggerValue);
			//claw
			//moves claw in and out
			if(controller->GetPOV() == 0){
					clawSol->Set(clawOut);
			}
			if(controller->GetPOV() == 180){
					clawSol->Set(clawIn);
			}

			//opens and closes claw
			if(controller->GetPOV() == 90){
					clawServo->Set(clawOpen);

			}
			if(controller->GetPOV() == 270){
					clawServo->Set(clawShut);
			}

			//intake
			if (!leftBumperButton && controller->GetBumper(frc::GenericHID::JoystickHand::kLeftHand)) {
				if (intakeRunning) {
						intake->Set(0);
						intakeRunning = false;
						conveyor->Set(0);
				}
				else {
						intake->Set(-0.6);
						intakeRunning = true;
						conveyor->Set(-1.0);
				}
			}
			leftBumperButton = controller->GetBumper(frc::GenericHID::JoystickHand::kLeftHand);
			//conveyor "low goal scoring"
			if (!rightBumperButton && controller->GetBumper(frc::GenericHID::JoystickHand::kRightHand)) {
				if (conveyorRunning) {
						conveyorRunning = false;
						conveyor->Set(0);
						ballGate->Set(ballGateUp); //straight up
				}
				else {
						conveyorRunning = true;
						conveyor->Set(-1.0);
						ballGate->Set(ballGateDown); //flat
				}
				intake->Set(0); //turn off intake if
			}
			rightBumperButton = controller->GetBumper(frc::GenericHID::JoystickHand::kRightHand);
			//compressor toggle
			if (!compressorButtons && (controller->GetStartButton() or controller->GetBackButton())) {
				if (compressorRunning) {
						compressorRunning = false;
						compressor->Stop();
				}
				else {
						compressorRunning = true;
						compressor->Start();
				}
				intake->Set(0); //turn off intake if
			}

			compressorButtons = (controller->GetStartButton() or controller->GetBackButton());
			//frc::SmartDashboard::PutBoolean("Compressor Running", myCompressor.Enabled());
			// wait for a motor update time
			frc::Wait(0.005);
		}
	}
	void Test() override {
		while(IsEnabled()){
			frc::SmartDashboard::PutNumber("rDis",myRobot->rDriveEncoder->GetDistance());
			frc::SmartDashboard::PutNumber("lDis",myRobot->lDriveEncoder->GetDistance());
			clawServo->Set(controller->GetY(frc::GenericHID::JoystickHand::kLeftHand));
			std::cout<<myRobot->gyro->GetRate()<<std::endl;
			frc::Wait(.005);
		}
	}
};

START_ROBOT_CLASS(Robot)
