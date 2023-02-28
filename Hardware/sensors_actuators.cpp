#include "sensors_actuators.h"

#define PI 3.1415927
// constructors

sensors_actuators::sensors_actuators(float Ts) : counter(PA_8, PA_9),
                            i_enable(PB_1),button(PA_10),i_des(PA_4),uw(4*2048,16),spi(PA_12, PA_11, PA_1),imu(spi, PB_0),
                            ax2ax(3.9917e-4,0),ay2ay(3.9917e-4,0),gz2gz(7.1018e-4,0)
{
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
    Vphi_fw = 0;//
    //-------------- read imu ------------
    accx = imu.readAcc_raw(1);
    accy = -imu.readAcc_raw(0);
    gyrz = imu.readGyro_raw(2);
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

float sensors_actuators::get_phi_bd(void)
{
    return phi_bd;
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
    return ax2ax.evaluate(accx);//ax2ax(accx);
}
float sensors_actuators::get_ay(void)
{
    return ay2ay.evaluate(accy);
}
float sensors_actuators::get_gz(void)
{
    return gz2gz.evaluate(gyrz);
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
bool sensors_actuators::get_key_state(void)
{
    bool temp = key_was_pressed;
    key_was_pressed = false;
    return temp;
} 