#include "IIR_filter.h"

// constructors
IIR_filter::IIR_filter(float tau,float Ts)
{
        this->a0 = -tau/(tau+Ts);
        this->b0 = Ts/(tau+Ts);

}
IIR_filter::IIR_filter(float tau,float Ts,float K)
{


}

// Methods:

float IIR_filter::eval(float u)
{
    float y_new = -a0*y_old+b0*u;
    y_old = y_new;
    return y_new;       
}


// Deconstructor
IIR_filter::~IIR_filter() {} 