/*
  This script receives data from a csv file. it calculates then converts the accelerometer data to 42  parameter based on statistics.
  These stats are then sent out and saved in a txt. file on the local pc

   Written by: Garrett Kryt
   June 17th 2021

*/

#include "Arduino.h"
#include "scaler.h"
#include "SF100000_Trees10_MD5_Garrett.h"   //.h file including the RandomForest Classifier

#include "intention_detection.h"
#include <math.h>

const int _scalar = 100000;
float _featureBuff[6] = {0.0};          //buffer to hold all the features
float _meanArrayLeft[11] = {0.0};
float _meanArrayRight[11] = {0.0};
float _meanLeft = 0.0;
float _meanRight = 0.0;
float _stdLeft = 0.0;
float _stdRight = 0.0;
float _stdSum = 0.0;
float _stdDiff = 0.0;
float _discDiffLeft = 0.0;
float _discDiffRight = 0.0;
float _prevLeft = 0.0;
float _prevRight = 0.0;
int32_t _predictedClass = 0.0;
uint8_t _startCnt = 0;






intention_detection::intention_detection() {
}



int32_t intention_detection::classify(float forceBitLeft, float forceBitRight, uint8_t buttonFlag, uint8_t toggleFlag) {

  if (buttonFlag == 1) {
    // _forceBitLeft = floorf(accelX * 1000.0) / 1000.0;
    // _forceBitRight = floorf(accelY * 1000.0) / 1000.0;



    if (_startCnt < 1) {                            //drop first two points
      _startCnt++;

    } else if (_startCnt <= 11) {                    //store first 11 points and calculate the mean after dropping one point
      _meanArrayLeft[_startCnt - 1] = forceBitLeft;
      _meanArrayRight[_startCnt - 1] = forceBitRight;
      //            Serial.print(_meanArrayLeft[_startCnt - 1]);

      if (_startCnt == 11) {                         //if on the last time through this loop
        _meanLeft = _calcMean(11, &_meanArrayLeft[0]);
        _meanRight = _calcMean(11, &_meanArrayRight[0]);

        //store elements for first order difference
        _prevLeft = forceBitLeft - _meanLeft;
        _prevRight = forceBitRight - _meanRight;

        //mirror _prevLeft about the mean
        _prevLeft = -1.0 * _prevLeft + 2.0 * _meanLeft;

        Serial.print("Left Mean=");
        Serial.print(_meanLeft);
        Serial.print(" Mean Right");
        Serial.println(_meanRight);
      }

      _startCnt++;

    } else {
      //mirror left side data about the mean
      forceBitLeft = -1.0 * forceBitLeft + 2.0 * _meanLeft;


      //perform all calculations
      _stdLeft = forceBitLeft - _meanLeft;
      _stdRight = forceBitRight - _meanRight;
      _stdSum = _stdLeft + _stdRight;
      _stdDiff = _stdRight - _stdLeft;
      _discDiffLeft = _stdLeft - _prevLeft;
      _discDiffRight = _stdRight - _prevRight;
      _prevLeft = _stdLeft;
      _prevRight = _stdRight;

      //set all values into array
      _featureBuff[0] = _stdLeft;
      _featureBuff[1] = _stdRight;
      _featureBuff[2] = _stdSum;
      _featureBuff[3] = _stdDiff;
      _featureBuff[4] = _discDiffLeft;
      _featureBuff[5] = _discDiffRight;

      //normalize all values
      for (int _featCnt = 0; _featCnt < 6; _featCnt++) {
        //first normalize

        //        Serial.print(_featureBuff[_featCnt]);
        //        Serial.print(",");

        _featureBuff[_featCnt] = (_featureBuff[_featCnt] - _preScaler[_featCnt][0]) / _preScaler[_featCnt][1];
        //
        //        Serial.print(_featureBuff[_featCnt],5);
        //        Serial.print(",");

        //then make fized point
        _featureBuff[_featCnt] = float(int(_featureBuff[_featCnt] * _scalar));
        Serial.print(_featureBuff[_featCnt]);
        Serial.print(",");
      }

//      if (toggleFlag == 0) {
////        Serial.print("Old Model");
        _predictedClass = SF100000_Trees10_MD5_Garrett_predict(_featureBuff, 6);

//      }
    }

  } else {                          //button flag has been turned off

    _predictedClass = 0;
    _startCnt = 0;
  }

  return _predictedClass;

}


/*
   Calculates mean for a single _column
*/
float intention_detection::_calcMean(uint16_t numRows, float * arr) {
  uint16_t cnt = 0;
  float sum = 0.0;
  float mean = 0.0;

  for (cnt = 0; cnt < numRows; cnt++) {
    sum = sum + arr[cnt];
  }

  mean = sum / (float)numRows;

  return mean;
};
