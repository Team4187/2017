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
	private:
		std::shared_ptr<NetworkTable> table;
	public:
		Target(std::string tableName);
		double GetCenterX();
		//double GetCenterY(int whichOne);
};

#endif /* SRC_TARGET_H_ */
