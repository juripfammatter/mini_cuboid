#include "sensors_actuators.h"

#define PI 3.1415927
// constructors

sensors_actuators::sensors_actuators(float Ts) : counter(PA_8, PA_9),
                            i_enable(PB_1),button(PA_10),i_des(PA_4),uw(4*2048,16),spi(PA_12, PA_11, PA_1),imu(spi, PB_0),
                            ax2ax(-16360, 16520, -9.81, 9.81),ay2ay(-17100, 15750, -9.81, 9.81),gz2gz(-32768, 32767, -1000.0f/180.0f*PI, 1000.0f/180.0f*PI),
                            filter_ax(1.0f, Ts), filter_ay(1.0f, Ts), filter_gz(1.0f, Ts, 1.0f), i2u(-15,15,0,1.0f), diff_phi(Ts)
                            

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
    phi_fw = uw(counter);           //unwrap prevents overflow
    omega_fw = diff_phi(phi_fw);    //differentiation
    //-------------- read imu ------------
    accx = ax2ax(imu.readAcc_raw(1));
    accy = ay2ay(-imu.readAcc_raw(0));
    gyrz = gz2gz(imu.readGyro_raw(2));

    // Complementary filter
    phi_bd = atan2(filter_ax(accx), filter_ay(accy))+ filter_gz(gyrz) - PI/4;
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
float sensors_actuators::get_omega_fw(void)
{
    return omega_fw;
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
bool sensors_actuators::get_key_state(void)
{
    bool temp = key_was_pressed;
    key_was_pressed = false;
    return temp;
} 