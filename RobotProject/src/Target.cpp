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
}
double Target::GetCenterX(int whichOne){
	return this->table->GetNumber("center" + std::to_string(whichOne) + "X", -10000);
}
double Target::GetCenterY(int whichOne){
	return this->table->GetNumber("center"+std::to_string(whichOne)+"Y", -10000);
}

