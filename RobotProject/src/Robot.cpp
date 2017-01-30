#include <iostream>
#include <memory>
#include <string>
#include <math.h>

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
#include <PowerDistributionPanel.h>


class VPBSDrive {
	private:
		//driving
		double lowMaxSpeed = 100.00; //guess
		double highMaxSpeed = 340.00; //another guess
		double curMaxSpeed = lowMaxSpeed; //a good default
		double lowSpeedGov = 1; //literally minimal governing
		double sensitivity = 0.5; //default
		frc::Spark* rSide [3];
		frc::Spark* lSide [3];
		//PID Driving
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
		//pneumatics
		frc::DoubleSolenoid* gearShifter;
		frc::DoubleSolenoid::Value hGear = frc::DoubleSolenoid::Value::kReverse;
		frc::DoubleSolenoid::Value lGear = frc::DoubleSolenoid::Value::kForward;
		//encoder
		double wheelDiameter = 4; //in inches
		double eWRatio = 1.00/3.00; //wheel revolutions / encoder revolutions
		double pPR = 48.00; //encoder settings of pulses per revolution
		double dPerPulse = (wheelDiameter * 3.1415 * eWRatio)/pPR; //in inches
		double minRate = 1; //inches per second? to be considered moving

		//other stuff
		double curVolt;
		double lowVolt;
		frc::PowerDistributionPanel* pdp;
		double minVolt = 9.0;

	public:
		//class scope variables
		frc::Encoder* rDriveEncoder;
		frc::Encoder* lDriveEncoder;

		//init function
		VPBSDrive (int frm, int crm, int brm, int flm, int clm, int blm, int reA, int reB, int leA, int leB, int solA, int solB) {

			rSide[0] = new frc::Spark(frm);
			rSide[1] = new frc::Spark(crm);
			rSide[2] = new frc::Spark(brm);

			lSide[0] = new frc::Spark(flm);
			lSide[1] = new frc::Spark(clm);
			lSide[2] = new frc::Spark(blm);
			for(int i = 0; i<3; i++){
				lSide[i]->SetInverted(true);
			}

			//encoder name {port A, port B, reversed?, Accuracy/Precision}
			rDriveEncoder = new frc::Encoder(reA, reB, false, Encoder::EncodingType::k1X);
			lDriveEncoder = new frc::Encoder(leA, leB, true, Encoder::EncodingType::k1X);
			rDriveEncoder->SetDistancePerPulse(dPerPulse);
			lDriveEncoder->SetDistancePerPulse(dPerPulse);
			rDriveEncoder->SetMinRate(minRate);
			lDriveEncoder->SetMinRate(minRate);
			//Solenoids for shifting on single valve attached at PCM slot 1 (and 2)

			gearShifter = new frc::DoubleSolenoid(solA, solB );
			gearShifter->Set(this->lGear);
			//frc::Solenoid mySingleSolenoid { 1 };
			pdp = new PowerDistributionPanel();
			curVolt = pdp->GetVoltage();
			lowVolt = curVolt;
		}
		virtual ~VPBSDrive() = default;
		void DownShift(){
			this->gearShifter->Set(this->lGear);
			this->curMaxSpeed = this->lowMaxSpeed;
		}
		void UpShift(){
			this->gearShifter->Set(this->hGear);
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
			//p
			rPVal = this->pK * rErr;
			lPVal = this->pK * lErr;
			//i
			this->rTErr += rErr;
			this->lTErr += lErr;
			rIVal = this->iK * this->rTErr;
			lIVal = this->iK * this->lTErr;
			//d
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
		void TankDrive (frc::Joystick* stick, int rAxis, int lAxis){
			//mimics normal tank drive but adds automatic shifting and PID control, may comment out auto shift and just manual
			double rVal = stick->GetRawAxis(rAxis);
			double lVal = stick->GetRawAxis(lAxis);
			//Null Zone for Joysticks
			if (std::abs(rVal) < .05 ) {
				rVal = 0.0;
			}
			if (std::abs(lVal) < .05 ) {
				lVal = 0.0;
			}
			this->curVolt = this->pdp->GetVoltage();
			if(this->curVolt < this->lowVolt){
				this->lowVolt = this->curVolt;
			}
			//low voltage handling
			if(this->curVolt < this->minVolt){
				rVal *= .5;
				lVal *= .5;
			}
			frc::SmartDashboard::PutNumber("lowest Voltage", this->lowVolt);
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
			//calls PID Drive with joystick values as arguments
			this->PIDDrive(rVal, lVal);
		}
		void Drive (double mag, double curve){
			//for auto
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
		void DriveDis(double desiredDis, double epsilon){
			//drive so many units +- epsilon forward (at the moment inches due to dPerPulse settings), negative should be backwards
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
		void ToggleGearShifter(){
			if(this->curMaxSpeed == this->lowMaxSpeed){
				this->UpShift();
			}
			if(this->curMaxSpeed == this->highMaxSpeed){
				this->DownShift();
			}
		}
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
	VPBSDrive* myRobot = new VPBSDrive(1,3,5,0,2,4,0,1,2,3,1,2); // robot drive system,
	frc::Joystick* stick = new frc::Joystick(0); // only joystick

	//frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "myAuto";

	//frc::Spark *motorPtr = &myMotor;
	//frc::Encoder *encoderPtr = &myEncoder;
	//frc::Compressor myCompressor {0};

public:
	Robot() {
		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
		myRobot->SetExpiration(0.1);
	}

	void RobotInit() {
		//chooser.AddDefault(autoNameDefault, autoNameDefault);
		//chooser.AddObject(autoNameCustom, autoNameCustom);
		//frc::SmartDashboard::PutData("Auto Modes", &chooser);
		std::cout<<"this is std::out"<<std::endl;
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
		//auto autoSelected = chooser.GetSelected();
		std::string autoSelected = frc::SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			//drive forward 15 units (inches), +- 1 inches
			myRobot->DriveDis(15,1);
			//Look at camera to see if lined up
			//CheckCamera()
			//if close enough, drop off gear. else go closer.
		} else {
			// Default Auto goes here
			myRobot->Drive(0,.25);
			frc::Wait(1);
			myRobot->Drive(0,0);
		}
	}
	void OperatorControl() override {
		myRobot->SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled()) {
			//if(winch is running){myRobot->NoDrive; *in class* void NoDrive(){this->speedGov = .001} else{ if(myRobot->NotDriving()){myRobot->GrantDrive();}}
			// drive with tank style (use both sticks)
			myRobot->TankDrive(stick, 5, 1);
			myRobot->rDriveEncoder->GetRate();
			if (stick->GetRawButton(1)){
				myRobot->UpShift();
			}
			if (stick->GetRawButton(2)) {
				myRobot->DownShift();
			}
			if(myRobot->GetCurVoltage() < myRobot->GetMinVoltage()){
				//stop intake && other junk like compressor
			}
			//frc::SmartDashboard::PutBoolean("Compressor Running", myCompressor.Enabled());
			// wait for a motor update time
			frc::Wait(0.005);
			//possibility to prevent waiting too long?
			/**while(not IsNewDataAvailable()){
				frc::Wait(0.001);
			}
			**/
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
