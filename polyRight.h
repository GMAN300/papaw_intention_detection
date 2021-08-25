/*
polyRight.h - Library that defines the polynomial and limits
for the input forcef from the pushrim on the power-assist
wheelchair
Written by: Garrett Kryt
Date: May 27 2021
*/

#ifndef polyRight_h
#define polyRight_h

#include "polyRight.h"

extern const int16_t lookupRight[];
extern int lowerLimitRight;         //need to set based on polynomial
extern int upperLimitRight;         //need to set based on polynomial
extern int lowerThresholdRight;
extern int upperThresholdRight;

extern int motorCmdBitRight;
extern int motorCmdRight;
extern int tempRight;



class polyRight
{
	public:
		polyRight();
		void sortBitRight();
	private:
};
#endif
