/*
 * Targeting.h
 *
 *  Created on: Feb 16, 2017
 *      Author: connor
 */

#ifndef SRC_TARGET_H_
#define SRC_TARGET_H_

#include <networktables/NetworkTable.h>
#include <string>

class Target {
	public:
		std::shared_ptr<NetworkTable> table;
		double center;
		double expectedArea;
		double range;
	//public:
		Target(std::string tableName);
		double GetCenterX();
		double GetArea();
		double GetAngle();
		//double GetCenterY(int whichOne);
};

#endif /* SRC_TARGET_H_ */
