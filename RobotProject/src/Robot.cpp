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
		//double highMaxSpeed = 16.00; //guess
		double lowMaxSpeed = 9.00; //guess
		double highMaxSpeed = 18.00; //another guess
		double curMaxSpeed = lowMaxSpeed; //a good default
		double lowSpeedGov = 0.9; //literally minimal governing
		double dPerPulse = 2.0; //no clue on
		double minRate = 2.0; //no clue on
		double sensitivity = 0.5; //default
		frc::Spark* rSide [3];
		frc::Spark* lSide [3];
		frc::DoubleSolenoid* gearShifter;
		double curVolt;
		double lowVolt;
		frc::PowerDistributionPanel* pdp;
		double minVolt = 0.8;
		frc::DoubleSolenoid::Value lGear = frc::DoubleSolenoid::Value::kReverse;
		frc::DoubleSolenoid::Value hGear = frc::DoubleSolenoid::Value::kForward;

	public:
		//class scope variables
		//frc::Encoder* rightDriveEncoder;
		//frc::Encoder* leftDriveEncoder;
		//frc::DoubleSolenoid* gearShifter;

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
			/**
			//encoder name {port A, port B, reversed?, Accuracy/Precision}
			rightDriveEncoder = new frc::Encoder(reA, reB, false, Encoder::EncodingType::k1X);
			leftDriveEncoder = new frc::Encoder(leA, leB, false, Encoder::EncodingType::k1X);
			rightDriveEncoder->SetDistancePerPulse(dPerPulse);
			leftDriveEncoder->SetDistancePerPulse(dPerPulse);
			rightDriveEncoder->SetMinRate(minRate);
			leftDriveEncoder->SetMinRate(minRate);
			//Solenoids for shifting on single valve attached at PCM slot 1 (and 2)
			**/
			gearShifter = new frc::DoubleSolenoid(solA, solB );
			gearShifter->Set(this->lGear);
			//frc::Solenoid mySingleSolenoid { 1 };
			pdp = new PowerDistributionPanel();
			curVolt = pdp->GetVoltage();
			lowVolt = curVolt;
		}
		virtual ~VPBSDrive() = default;
		//mimics normal tank drive but adds automatic shifting
		void TankDrive (frc::Joystick* stick, int rAxis, int lAxis){
			double rVal = stick->GetRawAxis(rAxis);
			double lVal = stick->GetRawAxis(lAxis);
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
			if(this->leftDriveEncoder->GetRate() > lowMaxSpeed && this->rightDriveEncoder->GetRate() > lowMaxSpeed){
				//if both sides are moving at low max speed or higher then shift to high gear
				this->gearShifter->Set(this->hGear);
				this->curMaxSpeed = this->highMaxSpeed;
			}
			if(this->leftDriveEncoder->GetRate() <= lowMaxSpeed && this->rightDriveEncoder->GetRate() <= lowMaxSpeed){
				//if both sides are moving slower than max speed then shift to low gear
				this->gearShifter->Set(this->lGear);
				this->curMaxSpeed = this->lowMaxSpeed;
			}
			**/

			if(curMaxSpeed == lowMaxSpeed){
				int rSign = (std::abs(rVal)/rVal);
				int lSign = (std::abs(lVal)/lVal);
				rVal = rSign * std::min(std::abs(rVal), lowSpeedGov);
				lVal = lSign * std::min(std::abs(lVal), lowSpeedGov);
			}

			for(int i = 0; i < 3; i++){
				this->rSide[i]->Set(rVal);
			}
			for(int i = 0; i < 3; i++){
				this->lSide[i]->Set(lVal);
			}

		}
		void Drive (double mag, double curve){
			/**
			if(this->leftDriveEncoder->GetRate() > lowMaxSpeed && this->rightDriveEncoder->GetRate() > lowMaxSpeed){
				//if both sides are moving at low max speed or higher then shift to high gear
				this->gearShifter->Set(frc::DoubleSolenoid::Value::kReverse);
				this->curMaxSpeed = this->highMaxSpeed;
			}
			if(this->leftDriveEncoder->GetRate() <= lowMaxSpeed && this->rightDriveEncoder->GetRate() <= lowMaxSpeed){
				//if both sides are moving slower than max speed then shift to low gear
				this->gearShifter->Set(frc::DoubleSolenoid::Value::kForward);
				this->curMaxSpeed = this->lowMaxSpeed;
			}
			**/
			double ratio;
			if(curve<0){
				ratio = (std::log10(-curve) - this->sensitivity)/(std::log10(-curve) + this->sensitivity);
				for(int i = 0; i < 3; i++){
					this->rSide[i]->Set(mag);
				}
				for(int i = 0; i < 3; i++){
					this->lSide[i]->Set(mag/ratio);
				}
			}
			else if(curve>0){
				ratio = (std::log10(curve) - this->sensitivity)/(std::log10(curve) + this->sensitivity);
				for(int i = 0; i < 3; i++){
					this->rSide[i]->Set(mag/ratio);
				}
				for(int i = 0; i < 3; i++){
					this->lSide[i]->Set(mag);
				}
			}
			else {
				for(int i = 0; i < 3; i++){
					this->rSide[i]->Set(mag);
				}
				for(int i = 0; i < 3; i++){
					this->lSide[i]->Set(mag);
				}
			}

		}
		void ToggleGearShifter(){
			if(this->curMaxSpeed == this->lowMaxSpeed){
				this->gearShifter->Set(this->hGear);
			}
			if(this->curMaxSpeed == this->highMaxSpeed){
				this->gearShifter->Set(this->lGear);
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
	VPBSDrive* myRobot = new VPBSDrive(1,3,5,0,2,4,0,1,2,3,1,2); // robot drive system,
	frc::Joystick* stick = new frc::Joystick(0); // only joystick

	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";

	//frc::Spark *motorPtr = &myMotor;
	//frc::Encoder *encoderPtr = &myEncoder;
	//frc::PIDController myPID { 1, .1, .01, encoderPtr, motorPtr};
	//frc::PIDController myPID { 1,.1,.01, &myEncoder, &myMotor};

	//frc::Compressor myCompressor {0};

public:
	Robot() {
		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
		myRobot->SetExpiration(0.1);
	}

	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		std::cout<<"this is std::out"<<std::endl;
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
			myRobot->SetSafetyEnabled(false);
			myRobot->Drive(-0.5, 1.0); // spin at half speed
			frc::Wait(2.0);                // for 2 seconds
			myRobot->Drive(0.0, 0.0);  // stop robot
		} else {
			// Default Auto goes here
			std::cout << "Running default Autonomous" << std::endl;
			myRobot->SetSafetyEnabled(false);
			myRobot->Drive(-0.5, 0.0); // drive forwards half speed
			frc::Wait(2.0);                // for 2 seconds
			myRobot->Drive(0.0, 0.0);  // stop robot
		}
	}

	/*
	 * Runs the motors with arcade steering.
	 */
	void OperatorControl() override {
		myRobot->SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled()) {
			// drive with tank style (use both sticks)
			/**
			if(doorsAreOpening){
				myRobot->StopMotor();
			}
			//winch code
			else if(stick->GetRawButton(0)){
				myWinchMotor->Set(1);
				if(pitch > 45){
					myRobot->StopMotor();
				}
				else myRobot->TankDrive(stick, 1, 5);
			}
			else {
				myRobot->TankDrive(stick, 1, 5);
			}
			**/
			myRobot->TankDrive(stick, 1, 5);
			if (stick->GetRawButton(1)){
				myRobot->ToggleGearShifter();
			}
			if (stick->GetRawButton(2)) {
				myRobot->ToggleGearShifter();
			}
			if(myRobot->GetCurVoltage() < myRobot->GetMinVoltage()){
				//stop intake && other junk like compressor
			}
			//myMotor.Set(1);
			//frc::SmartDashboard::PutBoolean("Compressor Running", myCompressor.Enabled());
			//frc::SmartDashboard::PutNumber("Current Pressure psi", myCompressor.GetCompressorCurrent());
			//frc::SmartDashboard::PutNumber("Left Encoder Value", myRobot.leftDriveEncoder->GetRaw());
			//frc::SmartDashboard::PutNumber("Right Encoder Value", myRobot.rightDriveEncoder->GetRaw());

			// wait for a motor update time
			frc::Wait(0.005);

			//possibility to prevent waiting too long?
			/**while(not IsNewDataAvailable()){
				frc::Wait(0.001);
			}
			**/
		}
	}

	/*
	 * Runs during test mode
	 */
	void Test() override {

		if (stick->GetRawButton(1)){
			myRobot->gearShifter->Set(frc::DoubleSolenoid::Value::kForward);
		}
		if (stick->GetRawButton(2)) {
			myRobot->gearShifter->Set(frc::DoubleSolenoid::Value::kReverse);
		}
	}
};

START_ROBOT_CLASS(Robot)
