#include "sensors_actuators.h"

#define PI 3.1415927
// constructors

sensors_actuators::sensors_actuators(float Ts) : di(2*Ts,Ts),counter(PA_8, PA_9),
                            i_enable(PB_1),button(PA_10),i_des(PA_4),uw(4*2048,16),spi(PA_12, PA_11, PA_1),imu(spi, PB_0)
{
    i2u.setup(-15,15,0.0f,1.0f);
    //ax2ax.setup(0,1,0,1);     // use these for first time, adapt values according 
    //ay2ay.setup(0,1,0,1);     //              "

    ax2ax.setup(-16000,16800,-9.81,9.81);
    ay2ay.setup(-17450,15450,-9.81,9.81);
    gz2gz.setup(-32767,32768,-1000*PI/180,1000*PI/180);     // check offset (value at standstill)
// --------------------------------------------------
    button.fall(callback(this, &sensors_actuators::but_pressed));          // attach key pressed function
    button.rise(callback(this, &sensors_actuators::but_released));         // attach key pressed function
    key_was_pressed = false;
    i_enable = 0;
    counter.reset();   // encoder reset
    imu.init_inav();
    imu.configuration();
    
}
// Deconstructor
sensors_actuators::~sensors_actuators() {} 

void sensors_actuators::read_sensors_calc_speed(void)
{
    phi_fw = uw(counter);
    Vphi_fw = di(phi_fw);
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

void sensors_actuators::write_current(float _i_des)
{
        i_des = i2u(_i_des);   
}

float sensors_actuators::get_phi_fw(void)
{
    return phi_fw;
}
float sensors_actuators::get_vphi_fw(void)
{
    return Vphi_fw;
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
// start timer as soon as Button is pressed
void sensors_actuators::but_pressed()
{
    t_but.start();
    key_was_pressed = false;
}
 
// evaluating statemachine
void sensors_actuators::but_released()
{
     // readout, stop and reset timer
    float ButtonTime = t_but.read();
    t_but.stop();
    t_but.reset();
    if(ButtonTime > 0.05f && ButtonTime < 0.5) 
        key_was_pressed = true;
}
 