#include "sensors_actuators.h"

#define PI 3.1415927
// constructors

sensors_actuators::sensors_actuators(float Ts) : di(.05,Ts),counter(PA_8, PA_9),
                            i_enable(PB_1),i_des(PA_4),uw(4*2048,16),spi(PA_12, PA_11, PA_1),imu(spi, PB_0)
{
    i2u.setup(-15,15,0.0f,1.0f);
    ax2ax.setup(-32767,32768,-9.81*2,9.81*2);
    ay2ay.setup(-17420,15450,-9.81,9.81);
    gz2gz.setup(-32767,32768,-1000*PI/180,1000*PI/180);

    
    i_enable = 0;       // disable current first
    counter.reset();   // encoder reset
    imu.init_inav();
    imu.configuration();
    
}
// Deconstructor
sensors_actuators::~sensors_actuators() {} 

void sensors_actuators::read_sensors_calc_speed(void)
{
    phi = uw(counter);
    Vphi = di(phi);
    //-------------- read imu ------------
    accx = ax2ax(imu.readAcc_raw(1));
    accy = ay2ay(-imu.readAcc_raw(0));
    gyrz = gz2gz(imu.readGyro_raw(2));
}

void sensors_actuators::enable_escon(void)
{
    i_enable = 1;    
}
void sensors_actuators::disable_escon(void)
{
    i_enable = 0;    
}

void sensors_actuators::write_current(float i_des)
{
        i_des = i2u(i_des);   
}

float sensors_actuators::get_phi(void)
{
    return phi;
}
float sensors_actuators::get_vphi(void)
{
    return Vphi;
}
float sensors_actuators::get_ax(void)
{
    return accx;
}
float sensors_actuators::get_ay(void)
{
    return accy;
}
float sensors_actuators::get_gz(void)
{
    return gyrz;
}
