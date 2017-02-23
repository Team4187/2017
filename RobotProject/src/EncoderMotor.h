/*
 * EncoderMotor.h
 *
 *  Created on: Feb 21, 2017
 *      Author: connor
 */

#ifndef SRC_ENCODERMOTOR_H_
#define SRC_ENCODERMOTOR_H_

#include <WPILib.h>
#include <Spark.h>
#include <Encoder.h>

//Not elegant, concise, or clever. Likely majority unnecessary, built to be quick to plugnplay and to test and calibrate. General enough to be recycled

class EncoderMotor {
private:
	frc::Spark* motor;
	frc::Encoder* encoder;
	double encoderValue;
	double encoderRate;
	double tolerance;
	double holdPower;
	double motorValue;
	double maxSet;

public:
	EncoderMotor(int pwmPort, int encoderPortA, int encoderPortB); //do these variables work better?
	virtual ~EncoderMotor() = default;

	void Reset(bool motorToZero = false); //resets encoder and possibly sets to new Zero position
	void SetTolerance(double newTolerance);
	void SetHoldPower(double newHoldPower);
	void SetValue(double newEncoderValue, double epsilon); //sets and holds at encoder position
	double GetSetValue(); //returns value the motor is set to hold
	double GetRealValue();
	void SetRate(double newEncoderRate); //set a speed at which to turn
	double GetSetRate(); //returns the current set speed
	double GetRealRate();
	void Set(double value); //just sents motor normally
	double GetRawSet();//returns what motor is set at

	//"saftey"
	void SetExpiration(double timeout);
	double GetExpiration();
	bool IsSafetyEnabled();
	void SetSafetyEnabled(bool enabled);
	void StopMotor();
};

#endif /* SRC_ENCODERMOTOR_H_ */
