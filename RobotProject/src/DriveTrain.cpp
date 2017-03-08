/*
 * DriveTrain.cpp
 *
 *  Created on: Feb 4, 2017
 *      Author: Wyatt Marks
 */


#include <iostream>
#include <string>



#include <DriveTrain.h>
#include <RobotBase.h>
#include <Encoder.h>
#include <Spark.h>
#include <DoubleSolenoid.h>
#include <PowerDistributionPanel.h>
#include <SmartDashboard/SmartDashboard.h>
#include <XboxController.h>
#include <ADXRS450_Gyro.h>

VPBSDrive::VPBSDrive (int frm, int crm, int brm, int flm, int clm, int blm, int reA, int reB, int leA, int leB, int solA, int solB, frc::XboxController* ctrl) { //TODO: Make these vars less shitty


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
	//rDriveEncoder->SetMinRate(minRate);
	//lDriveEncoder->SetMinRate(minRate);

	//Gyro init
	gyro = new frc::ADXRS450_Gyro();
	gyro->Reset();
	gyro->Calibrate();

	//Solenoid for shifting on single valve attached at PCM slot 1 (and 2)
	gearShifter = new frc::DoubleSolenoid(solA, solB );
	gearShifter->Set(this->lowGear);


	//Initialize the PDP :D

	pdp = new PowerDistributionPanel();
	curVolt = pdp->GetVoltage();
	lowVolt = curVolt;

	//yay to breaking some fundamentals of OOP!! :) (I'm naming it controller interface so it appears as if we are following the rules)
	controllerInterface = ctrl;
}


void VPBSDrive::DownShift(){
	this->gearShifter->Set(this->lowGear);
	this->curMaxSpeed = this->lowMaxSpeed;
}


void VPBSDrive::UpShift(){
	this->gearShifter->Set(this->highGear);
	this->curMaxSpeed = this->highMaxSpeed;
}

void VPBSDrive::PureTankDrive(double rVal, double lVal, bool nullZone){
	if(std::abs(rVal) < .1){
		rVal = 0;
	}
	if(std::abs(lVal) < .1){
		lVal = 0;
	}
	for(int i = 0; i < 3; i++){
		this->rSide[i]->Set(rVal);
		this->lSide[i]->Set(lVal);
	}
}
void VPBSDrive::PIDDrive(double rVal, double lVal){
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
	frc::SmartDashboard::PutNumber("gyro", this->gyro->GetAngle());
	frc::SmartDashboard::PutNumber("gyroRate", this->gyro->GetRate());
}


//Tank Driving! Now with PID control and Shifting all automagically inside!
void VPBSDrive::TankDrive (frc::XboxController* controller, bool invert, bool manual){

	double rVal = controller->GetY(GenericHID::kRightHand);
	double lVal = controller->GetY(GenericHID::kLeftHand);
	//if invert is true, swap directions of joysticks.
	if(invert){
		//rVal *= -1;
		//lVal *= -1;
		//might need to be
		 double negR = -rVal;
		 double negL = -lVal;
		 rVal = negL;
		 lVal = negR;

	}
	//if manual, use PureTankDrive and skip the PID control loop
	if(manual){this->PureTankDrive(rVal,lVal);}
		else{
		//Null Zone
		if (std::abs(rVal) < .1 ) {
			rVal = 0.0;
		}
		if (std::abs(lVal) < .1 ) {
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



		//auto shifting based on speed
		if (this->lDriveEncoder->GetRate() < 0.45*lowMaxSpeed && this->rDriveEncoder->GetRate() < 0.45*lowMaxSpeed){
			//if both sides are moving slower than max speed then shift to low gear
			this->DownShift();
		}
		if(this->lDriveEncoder->GetRate() > 0.65*lowMaxSpeed && this->rDriveEncoder->GetRate() > 0.65*lowMaxSpeed){
			//if both sides are moving at low max speed or higher then shift to high gear
			this->UpShift();
		}


		this->PIDDrive(rVal, lVal);
	}
}



//Autonomous driving
void VPBSDrive::Drive (double mag, double curve){
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
void VPBSDrive::DriveDis(double desiredDis, double epsilon){
	double rStart = this->rDriveEncoder->GetDistance();
	double curDis = rStart;
	double goalDis = desiredDis + curDis;
	double lowGoal = goalDis - epsilon;
	double highGoal = goalDis + epsilon;
	//uses just right side since they should work move in sync
	while(curDis < lowGoal or curDis > highGoal){
		double difference = ((curDis - goalDis)/abs(curDis - goalDis))*std::max(std::abs((curDis-goalDis)/desiredDis), .5);
		this->PIDDrive(difference, difference);
		curDis = this->rDriveEncoder->GetDistance();
		//don't let the name fool you, it's just a controller pointer. BUT good practice says it should be a special interface but I didn't make one so this is a reminder of good practice :)
		if(this->controllerInterface->GetXButton()){
			std::cout<<"we canceled via handy dandy X button"<<std::endl;
			break;
		}
	}
	//once at desiredDis, stop robot
	this->Drive(0,0);
}

//Turn so many units +- epsilon (units in inches? whatever gyro spits out), negative should be counter clockwise
void VPBSDrive::Turn(double desiredTurn, double epsilon){
	double start = this->gyro->GetAngle();
	double cur = start;
	double goal = desiredTurn + cur;
	double lowGoal = goal - epsilon;
	double highGoal = goal + epsilon;
	while(cur < lowGoal or cur > highGoal) {
		//slows down as it gets closer, since this fraction will approach 0. Won't work well with small distances.
		double difference = ((cur - goal)/std::abs(cur-goal))*std::max(std::abs((cur - goal)/desiredTurn), .75);
		std::cout<<"cur "<<this->gyro->GetAngle()<< " turning to "<< desiredTurn<< " epsilon "<< epsilon<< " difference "<< difference<<std::endl;
		this->PIDDrive(-difference, difference); //turn right motors backwards and left forwards to turn clockwise, maybe math it if too violent or too weak
		//this->Drive(.5 ,errorTurn); //try this if that ^ doesn't work. This won't turn in place though
		cur = this->gyro->GetAngle();
		//don't let the name fool you, it's just a controller pointer. BUT good practice says it should be a special interface but I didn't make one so this is a reminder of good practice :)
		if(this->controllerInterface->GetXButton()){
			std::cout<<"we canceled via handy dandy X button"<<std::endl;
			break;
		}
	}
	//once to desired angle stop motors
	this->Drive(0,0);
}


//Basically switch gears lol
void VPBSDrive::ToggleGearShifter(){
	if(this->gearShifter->Get() == this->lowGear) {
		this->UpShift();
	} else {
		this->DownShift();
	}
}


//Stuff we have to implement to make program happy
double VPBSDrive::GetCurVoltage(){
	return this->curVolt;
}
double VPBSDrive::GetLowVoltage(){
	return this->lowVolt;
}
double VPBSDrive::GetMinVoltage(){
	return this->minVolt;
}
void VPBSDrive::SetExpiration(double timeout){
	for(int i = 0; i < 3; i++){
		this->rSide[i]->SetExpiration(timeout);
	}
	for(int i = 0; i < 3; i++){
		this->lSide[i]->SetExpiration(timeout);
	}
}
double VPBSDrive::GetExpiration(){
	return this->rSide[0]->GetExpiration();
}
bool VPBSDrive::IsSafetyEnabled(){
	return this->rSide[0]->IsSafetyEnabled();
}
void VPBSDrive::SetSafetyEnabled(bool enabled){
	for(int i = 0; i < 3; i++){
		this->rSide[i]->SetSafetyEnabled(enabled);
	}
	for(int i = 0; i < 3; i++){
		this->lSide[i]->SetSafetyEnabled(enabled);
	}
}
void VPBSDrive::StopMotor(){
	for(int i = 0; i < 3; i++){
		this->rSide[i]->StopMotor();
	}
	for(int i = 0; i < 3; i++){
		this->lSide[i]->StopMotor();
	}
}
