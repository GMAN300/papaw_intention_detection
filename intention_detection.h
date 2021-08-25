


#ifndef intention_detection_h
#define intention_detection_h

#include "intention_detection.h"




class intention_detection
{
  public:
    intention_detection();
    int32_t classify(float forceBitLeft, float forceBitRight, uint8_t buttonflag, uint8_t toggleFlag);

  private:
    float _calcMean(uint16_t numRows, float *arr);

    char _sendBackBuff[64];
    const int _scalar = 100000;
    float _featureBuff[6];          //buffer to hold all the features
    float _meanArrayLeft[11];
    float _meanArrayRight[11];
    float _meanLeft;
    float _meanRight;
    float _stdLeft;
    float _stdRight;
    float _stdSum;
    float _stdDiff;
    float _discDiffLeft;
    float _discDiffRight;
    float _prevLeft;
    float _prevRight;
    int32_t _predictedClass;
    uint8_t _startCnt;
    uint8_t _featureNumber;

};
#endif
