/*
polyLeft.h - Library that defines the polynomial and limits
for the input forcef from the pushrim on the power-assist
wheelchair
Written by: Garrett Kryt
Date: May 27 2021
*/

#ifndef polyLeft_h
#define polyLeft_h

#include "polyLeft.h"

extern const int16_t lookupLeft[];
extern int lowerLimitLeft;         //need to set based on polynomial
extern int upperLimitLeft;         //need to set based on polynomial
extern int lowerThresholdLeft;
extern int upperThresholdLeft;

extern int motorCmdBitLeft;
extern int motorCmdLeft;
extern int tempLeft;



class polyLeft
{
	public:
		polyLeft();
		void sortBitLeft();
	private:
};
#endif
