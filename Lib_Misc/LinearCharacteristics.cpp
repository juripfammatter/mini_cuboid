#include "LinearCharacteristics.h"

using namespace std;

LinearCharacteristics::LinearCharacteristics(float gain,float offset){    // standard lin characteristics
    this->gain = gain;
    this->offset = offset;
    this->ulim = 999999.0;          // a large number
    this->llim = -999999.0;         // a large neg. number
}

LinearCharacteristics::LinearCharacteristics(float xmin,float xmax, float ymin, float ymax){    // standard lin characteristics
    this->gain = (ymax-ymin)/(xmax-xmin);
    this->offset = -ymin/(this->gain)+xmin;
    this->ulim = 999999.0;         
    this->llim = -999999.0;
}



LinearCharacteristics::~LinearCharacteristics() {}


float LinearCharacteristics::evaluate(float x)
{   
    if(x>ulim){
        x = ulim;
    }else if (x<llim){
        x = llim;
    }
    // calculate result as y(x) = gain * (x-offset)
    float y = gain*(x-offset);

    return y;
}
