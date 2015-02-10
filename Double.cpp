/*
 * Double.cpp
 *
 *  Created on: Jan 25, 2015
 *      Author: Simon Tsaoussis
 */

#include "Double.h"

Double::Double(long int information, short radixloc) {
	// TODO Auto-generated constructor stub
	Information = information;
	radixLocation = radixloc;
}
char Double::equals(Double j){
	if (j.getInformation() == Information && j.getRadixLocation() == radixLocation) {
		return 0x01;
	}
	return 0x00;
}
char Double::equals(char j){
	if (j == Information && 1 == radixLocation) {
		return 0x01;
	}
	return 0x00;
}
char Double::equals(short j){
	if (j == Information && 1 == radixLocation) {
		return 0x01;
	}
	return 0x00;
}
char Double::equals(int j){
	if (j == Information && 1 == radixLocation) {
		return 0x01;
	}
	return 0x00;
}
char Double::equals(long int j){
	if (j == Information && 1 == radixLocation) {
		return 0x01;
	}
	return 0x00;
}
long int Double::getInformation() {
	return Information;
}
void Double::setInformation(long int j) {
	Information = j;
}
short Double::getRadixLocation() {
	return radixLocation;
}
void Double::setRadixLocation(short j) {
	radixLocation = j;
}
Double::~Double() {
	// TODO Auto-generated destructor stub
	delete &Information;
	delete &radixLocation;
}

