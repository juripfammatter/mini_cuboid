#include "IIR_filter.h"

// constructors
IIR_filter::IIR_filter(float tau,float Ts)
{
        this->Ts = Ts;
        this->a0 = -tau/(tau+Ts);
        this->b0 = Ts/(tau+Ts);
        this->y_old=0;

}

IIR_filter::IIR_filter(float tau,float Ts)
{
        this->Ts = Ts;
        this->a0 = -tau/(tau+Ts);
        this->b0 = Ts/(tau+Ts);
        this->y_old=0;

}

IIR_filter::IIR_filter(float tau,float Ts,float K)
{
        this->Ts = Ts;
        this->a0 = -tau/(tau+Ts);
        this->b0 = K*Ts/(tau+Ts);
        this->y_old=0;

}

// Methods:

float IIR_filter::eval(float u)
{
    float y_new = b1*u+b0*u_old-a0*y_old;
    y_old = y_new;
    return y_new;       
}


// Deconstructor
IIR_filter::~IIR_filter() {} 