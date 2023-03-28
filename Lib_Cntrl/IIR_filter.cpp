#include "IIR_filter.h"

// constructors

IIR_filter::IIR_filter(float Ts)
{
        this->Ts = Ts;
        this->a0 = 0;
        this->b0 = -1/Ts;
        this->b1 = 1/Ts;
        this->u_old = 0;
        this->y_old = 0;
        this->ulim = 999999;
        this->llim = -999999;

}

IIR_filter::IIR_filter(float lim,float Ts)
{
        this->Ts = Ts;
        this->a0 = -1.0;
        this->b0 = 0;
        this->b1 = Ts;
        this->u_old = 0;
        this->y_old = 0;
        this->ulim = lim;
        this->llim = -lim;

}


IIR_filter::IIR_filter(float tau,float Ts,float K)
{
        this->Ts = Ts;
        this->a0 = -tau/(tau+Ts);
        this->b0 =0;
        this->b1 = K*Ts/(tau+Ts);
        this->u_old = 0;
        this->y_old = 0;
        this->ulim = 999999;
        this->llim = -999999;

}

// Methods:

float IIR_filter::eval(float u)
{
    float y_new = b1*u+b0*u_old-a0*y_old;
    if(y_new>ulim){
        y_new = ulim;
    } else if(y_new<llim){
        y_new=llim;
    }
    y_old = y_new;
    u_old = u;
    return y_new;       
}


// Deconstructor
IIR_filter::~IIR_filter() {} 