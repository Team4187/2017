/*
 * EncoderMotor.cpp
 *
 *  Created on: Feb 21, 2017
 *      Author: connor
 */

#include <EncoderMotor.h>
#include <WPILib.h>
#include <Spark.h>
#include <Encoder.h>

//Not elegant, concise, or clever. Likely majority unnecessary, built to be quick to plugnplay and to test and calibrate. General enough to be recycled

EncoderMotor::EncoderMotor(int pwmPort, int encoderPortA, int encoderPortB) {
	this->motor = new frc::Spark(pwmPort);
	this->encoder = new frc::Encoder(encoderPortA, encoderPortB, true, frc::Encoder::EncodingType::k1X);
	this->encoder->Reset();
	this->encoderValue = this->encoder->GetDistance();
	this->encoderRate = this->encoder->GetRate();
	this->motorValue = 0;
	this->tolerance = 0.1;
	this->holdPower = 0;
	this->maxSet = 0.2;
};
void EncoderMotor::Reset(bool motorToZero){
	this->encoder->Reset();
	if(motorToZero){
		this->Set(0);
	}
}
void EncoderMotor::SetTolerance(double newTolerance){this->tolerance=newTolerance;}
void EncoderMotor::SetHoldPower(double newHoldPower){this->holdPower=newHoldPower;}
void EncoderMotor::SetValue(double newEncoderValue, double epsilon){
	this->encoderValue = newEncoderValue;

	double difference = this->GetRealValue() - newEncoderValue;

	if((this->GetRealValue() < newEncoderValue - epsilon) or (this->GetRealValue() > newEncoderValue + epsilon)){

		this->Set(difference);

	}
}
double EncoderMotor::GetSetValue(){return this->encoderValue;}
double EncoderMotor::GetRealValue(){return this->encoder->GetDistance();}
void EncoderMotor::SetRate(double newEncoderRate){
	this->encoderRate = newEncoderRate;
	if((this->GetRealRate() < this->GetSetRate()*(1 - this->tolerance)) or (this->GetRealRate() > this->GetSetRate()*(1 + this->tolerance)) ){
			this->Set(((this->GetRealRate()-this->GetSetRate())/this->GetSetRate()) + this->GetRawSet());
	}
	else{
		this->Set(this->holdPower);
	}
}
double EncoderMotor::GetSetRate(){return this->encoderRate;}
double EncoderMotor::GetRealRate(){return this->encoder->GetRate();}
void EncoderMotor::Set(double value){
	//this->motorValue = (value/std::abs(value))*std::min(std::abs(value),this->maxSet);
	this->motorValue = (value/std::abs(value))*.15;
	this->motor->Set(this->GetRawSet());
}
double EncoderMotor::GetRawSet(){return this->motorValue;}
//"saftey"
void EncoderMotor::SetExpiration(double timeout){this->motor->SetExpiration(timeout);}
double EncoderMotor::GetExpiration(){return this->motor->GetExpiration();}
bool EncoderMotor::IsSafetyEnabled(){return this->motor->IsSafetyEnabled();}
void EncoderMotor::SetSafetyEnabled(bool enabled){this->motor->SetSafetyEnabled(enabled);}
void EncoderMotor::StopMotor(){this->motor->StopMotor();}
