/*
 * Double.h
 *
 *  Created on: Jan 25, 2015
 *      Author: Simon Tsaoussis
 */

#ifndef DOUBLE_H_
#define DOUBLE_H_

class Double {
public:
	Double(long int information, short radixloc);
	char equals(Double j);
	char equals(char j);
	char equals(short j);
	char equals(int j);
	char equals(long int j);
	void setRadixLocation(short j);
	void setInformation(long int j);
	short getRadixLocation();
	long int getInformation();
	virtual ~Double();
private:
	long int Information;
	short radixLocation;
};

#endif /* DOUBLE_H_ */
