#include <iostream>
#include <memory>
#include <string>
#include <math.h>

#include <XboxController.h>
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
#include <PowerDistributionPanel.h>

/* Welcome! This is Team 4187's 2017 FIRST Robotics Competition code. Go Roborams!
 * All of our code is in one file right now. Lol, what the hell.
 * We had to create our own Robot class (Screw inheriting!)
 * 		We had a reason, okay. We wanted a 6 CIM drive base and didn't see a default one.
 */

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
		double iK = 0.025;
		double dK = 0;



		//Pneumatic related objects and stuff

		frc::DoubleSolenoid* gearShifter;
		frc::DoubleSolenoid::Value highGear = frc::DoubleSolenoid::Value::kReverse;
		frc::DoubleSolenoid::Value lowGear = frc::DoubleSolenoid::Value::kForward;


		//Encoder related stuff

		double dPerPulse = (4 /*<- Wheel Diameter*/ * 3.1415 * (1/3) /*<- Ratio*/)/48 /*<- Pulse Per Revolution*/;
		double minRate = 1; //inches per second? to be considered moving



		//Power Stuff

		double curVolt;
		double lowVolt;
		frc::PowerDistributionPanel* pdp;
		double minVolt = 9.0;



	public:
		//Public Encoder stuffs
		frc::Encoder* rDriveEncoder;
		frc::Encoder* lDriveEncoder;

		VPBSDrive (int frm, int crm, int brm, int flm, int clm, int blm, int reA, int reB, int leA, int leB, int solA, int solB) { //TODO: Make these vars less shitty


			//Initialize all the Sparks (in conveniently named arrays!)

			rSide[0] = new frc::Spark(frm);
			rSide[1] = new frc::Spark(crm);
			rSide[2] = new frc::Spark(brm);

			lSide[0] = new frc::Spark(flm);
			lSide[1] = new frc::Spark(clm);
			lSide[2] = new frc::Spark(blm);
			for(int i = 0; i<3; i++){
				lSide[i]->SetInverted(true);
			}


			//Initialize the encoders and set up their stuff

			rDriveEncoder = new frc::Encoder(reA, reB, false, Encoder::EncodingType::k1X);
			lDriveEncoder = new frc::Encoder(leA, leB, true, Encoder::EncodingType::k1X);
			rDriveEncoder->SetDistancePerPulse(dPerPulse);
			lDriveEncoder->SetDistancePerPulse(dPerPulse);
			rDriveEncoder->SetMinRate(minRate);
			lDriveEncoder->SetMinRate(minRate);



			//Solenoid for shifting on single valve attached at PCM slot 1 (and 2)
			gearShifter = new frc::DoubleSolenoid(solA, solB );
			gearShifter->Set(this->lowGear);



			//Initialize the PDP :D
			
			pdp = new PowerDistributionPanel();
			curVolt = pdp->GetVoltage();
			lowVolt = curVolt;
		}


		virtual ~VPBSDrive() = default;


		void DownShift(){
			this->gearShifter->Set(this->lowGear);
			this->curMaxSpeed = this->lowMaxSpeed;
		}


		void UpShift(){
			this->gearShifter->Set(this->highGear);
			this->curMaxSpeed = this->highMaxSpeed;
		}


		void PIDDrive(double rVal, double lVal){
			//PID driving using speed control with rVal and lVal being the goal percentage of max speed

			//PID loop variables
			double rPVal, rIVal, rDVal, rCor, rErr;
			double lPVal, lIVal, lDVal, lCor, lErr;
			this->rCurSpeed = this->rDriveEncoder->GetRate();
			this->lCurSpeed = this->lDriveEncoder->GetRate();


			//negated so positive is forward
			rErr = -rVal - (this->rCurSpeed/this->curMaxSpeed);
			lErr = -lVal - (this->lCurSpeed/this->curMaxSpeed);


			//Proportionate
			rPVal = this->pK * rErr;
			lPVal = this->pK * lErr;


			//Integral
			this->rTErr += rErr;
			this->lTErr += lErr;
			rIVal = this->iK * this->rTErr;
			lIVal = this->iK * this->lTErr;


			//Derivative
			rDVal = this->dK * (rErr - this->rPrevErr);
			lDVal = this->dK * (lErr - this->lPrevErr);
			this->rPrevErr = rErr;
			this->lPrevErr = lErr;


			//total corrections
			rCor = rPVal + rIVal + rDVal + (this->rCurSpeed/this->curMaxSpeed);
			lCor = lPVal + lIVal + lDVal + (this->lCurSpeed/this->curMaxSpeed);
			if(std::abs(rCor) > 1){
				rCor = this->lowSpeedGov*std::abs(rCor)/rCor;
			}
			if(std::abs(lCor)>1){
				lCor = this->lowSpeedGov*std::abs(lCor)/lCor;
			}
			if(curMaxSpeed == lowMaxSpeed){
				int rSign = (std::abs(rCor)/rCor);
				int lSign = (std::abs(lCor)/lCor);
				rVal = rSign * std::min(std::abs(rCor), lowSpeedGov);
				lVal = lSign * std::min(std::abs(lCor), lowSpeedGov);
			}


			//set motors to Correction Value, negative to set back since _Val was negated earlier
			for(int i = 0; i < 3; i++){
				this->rSide[i]->Set(-rCor);
			}
			for(int i = 0; i < 3; i++){
				this->lSide[i]->Set(-lCor);
			}


			//report a bunch of Driving Values
			frc::SmartDashboard::PutNumber("rDis", this->rDriveEncoder->GetDistance());
			frc::SmartDashboard::PutNumber("lDis", this->lDriveEncoder->GetDistance());
			frc::SmartDashboard::PutNumber("rCurSpeed", this->rCurSpeed);
			frc::SmartDashboard::PutNumber("lCurSpeed", this->lCurSpeed);
			frc::SmartDashboard::PutNumber("rCor", rCor);
			frc::SmartDashboard::PutNumber("lCor", lCor);
			frc::SmartDashboard::PutNumber("curMaxSpeed", this->curMaxSpeed);
		}


		//Tank Driving! Now with PID control and Shifting all automagically inside!
		void TankDrive (frc::XboxController* controller){
			
			double rVal = controller->GetY(kRightHand);
			double lVal = controller->GetY(kLeftHand);

			//Null Zone 
			if (std::abs(rVal) < .05 ) {
				rVal = 0.0;
			}
			if (std::abs(lVal) < .05 ) {
				lVal = 0.0;
			}

			//Update the voltage variables
			this->curVolt = this->pdp->GetVoltage();
			if(this->curVolt < this->lowVolt){
				this->lowVolt = this->curVolt;
			}
			frc::SmartDashboard::PutNumber("lowest Voltage", this->lowVolt);

			//"Brownout" to avoid the actual browning out lol
			if(this->curVolt < this->minVolt){
				rVal *= .5;
				lVal *= .5;
			}

		
			/**
			//auto shifting based on speed
			if(this->lDriveEncoder->GetRate() > lowMaxSpeed && this->rDriveEncoder->GetRate() > lowMaxSpeed){
				//if both sides are moving at low max speed or higher then shift to high gear
				this->UpShift();
			}
			if (this->lDriveEncoder->GetRate() < lowMaxSpeed && this->rDriveEncoder->GetRate() < lowMaxSpeed){
				//if both sides are moving slower than max speed then shift to low gear
				this->DownShift();
			}
			**/

			this->PIDDrive(rVal, lVal);
		}



		//Autonomous driving
		void Drive (double mag, double curve){
			double ratio;
			double rVal;
			double lVal;
			if(curve<0){
				ratio = (std::log10(-curve) - this->sensitivity)/(std::log10(-curve) + this->sensitivity);
				lVal = mag;
				rVal = (mag/ratio);
			}
			else if(curve>0){
				ratio = (std::log10(curve) - this->sensitivity)/(std::log10(curve) + this->sensitivity);
				rVal = (mag/ratio);
				lVal = (mag);
			}
			else {
				rVal = (mag);
				lVal = (mag);
			}
			//Call PID Drive w/ calculated rVal and lVal values as arguments for left and ride side set points
			this->PIDDrive(rVal, lVal);
		}


		//drive so many units +- epsilon forward (at the moment inches due to dPerPulse settings), negative should be backwards
		void DriveDis(double desiredDis, double epsilon){
			double rStart = this->rDriveEncoder->GetDistance();
			double curDis = rStart;
			double goalDis = desiredDis + curDis;
			double lowGoal = goalDis - epsilon;
			double highGoal = goalDis + epsilon;
			//uses just right side since they should work move in sync
			while(curDis < lowGoal or curDis > highGoal){
				//slows down as it gets closer, since this fraction will approach 0. Won't work well with small distances
				double err = (curDis - goalDis)/std::abs(desiredDis);
				this->PIDDrive(err, err);
				curDis = this->rDriveEncoder->GetDistance();
			}
			//once at desiredDis, stop robot
			this->Drive(0,0);
		}



		//Basically switch gears lol
		void ToggleGearShifter(){
			if(this->gearShifter->Get() == this->lowGear) {
				this->UpShift();
			} else {
				this->DownShift();
			}
		}


		//Stuff we have to implement to make program happy
		double GetCurVoltage(){
			return this->curVolt;
		}
		double GetLowVoltage(){
			return this->lowVolt;
		}
		double GetMinVoltage(){
			return this->minVolt;
		}
		void SetExpiration(double timeout){
			for(int i = 0; i < 3; i++){
				this->rSide[i]->SetExpiration(timeout);
			}
			for(int i = 0; i < 3; i++){
				this->lSide[i]->SetExpiration(timeout);
			}
		}
		double GetExpiration(){
			return this->rSide[0]->GetExpiration();
		}
		bool IsSafetyEnabled(){
			return this->rSide[0]->IsSafetyEnabled();
		}
		void SetSafetyEnabled(bool enabled){
			for(int i = 0; i < 3; i++){
				this->rSide[i]->SetSafetyEnabled(enabled);
			}
			for(int i = 0; i < 3; i++){
				this->lSide[i]->SetSafetyEnabled(enabled);
			}
		}
		void StopMotor(){
			for(int i = 0; i < 3; i++){
				this->rSide[i]->StopMotor();
			}
			for(int i = 0; i < 3; i++){
				this->lSide[i]->StopMotor();
			}
		}
};

class Robot: public frc::SampleRobot {
	VPBSDrive* myRobot = new VPBSDrive(1,3,5,0,2,4,0,1,2,3,1,2); // robot drive system
	frc::XboxController* controller = new frc::XboxController(0);

private:

	//This is a seperate thread that handles the camera screen on the Dashboard.
	static void CameraThread() {
		cs::UsbCamera camera0 = CameraServer::GetInstance()->StartAutomaticCapture(); //Automatic capture starts at device 0 and increments every call
		cs::UsbCamera camera1 = CameraServer::GetInstance()->StartAutomaticCapture();

		camera0.SetResolution(640, 480);
		camera1.SetResolution(640, 480);

		cs::CvSink cvSink0 = CameraServer::GetInstance()->GetVideo(camera0);
		cs::CvSink cvSink1 = CameraServer::GetInstance()->GetVideo(camera1);

		cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Gray", 640, 480);

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

			cvtColor(source, output, cv::COLOR_BGR2GRAY);
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
