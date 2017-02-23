/*
 * Targeting.cpp
 *
 *  Created on: Feb 16, 2017
 *      Author: connor
 */
#include <networktables/NetworkTable.h>
#include <string>
#include <Target.h>

Target::Target(std::string tableName){
	this->table = NetworkTable::GetTable(tableName);
	this->center = 220;
	this->expectedArea = 50;
	this->range = 15;
}
double Target::GetCenterX(){
	return this->table->GetNumber("peg", -10000);
}
double Target::GetArea(){
	return this->table->GetNumber("area", -10000);
}
double Target::GetAngle(){
	double pegPixelValue = this->GetCenterX();
	if(pegPixelValue < 0){
		if(std::abs(pegPixelValue) > (this->center + this->range)){
			return (this->GetArea()/expectedArea) * 30;
		}
		if(std::abs(pegPixelValue) < (this->center - this->range)){
			return (this->GetArea()/expectedArea) * -30; //turn 30 ccw
		}
		else {
			return 0;
		}
	}
	else if(pegPixelValue > 0){
		if(std::abs(pegPixelValue) > (this->center + this->range)){
			return (this->GetArea()/expectedArea) * 15;
		}
		if(std::abs(pegPixelValue) < (this->center - this->range)){
			return (this->GetArea()/expectedArea) * -15; //turn 30 ccw
		}
		else {
			return 0;
		}
	}
	else{
		return 0;
	}
}
//double Target::GetCenterY(int whichOne){
//	return this->table->GetNumber("center"+std::to_string(whichOne)+"Y", -10000);
//}

