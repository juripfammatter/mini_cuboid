#include "ControllerLoop.h"
using namespace std;


// contructor for controller loop
ControllerLoop::ControllerLoop(sensors_actuators *sa, float Ts) : thread(osPriorityHigh,4096), I4(2.8, Ts)
{
    this->Ts = Ts;
    this->m_sa = sa;
    m_sa->disable_escon();
    ti.reset();
    ti.start();
}

// decontructor for controller loop
ControllerLoop::~ControllerLoop() {}

// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void ControllerLoop::loop(void){

    float K2[2]{-1.3924,-0.0864}; // based on modelling cuboid with EV -10+-10i
    float K4[4]{-2.8684, -0.2764, -0.0076, 0.0065};
    float km = 36.9e-3;
    while(1)
    {
        ThisThread::flags_wait_any(threadFlag);
       // THE LOOP ------------------------------------------------------------
        m_sa->read_sensors_calc_speed();        // first read all sensors, calculate mtor speed
        //float M_soll = 0 - (K2[0] * m_sa->get_phi_bd() +K2[1] * m_sa->get_gz()); //Simple Controller 2d order
        float xi = I4(-m_sa->get_omega_fw());   //intergrator state
        float M_soll = 0 - (K4[0] * m_sa->get_phi_bd() +K4[1] * m_sa->get_gz()+ K4[2]*m_sa->get_omega_fw()+K4[3]*xi);
        float i_soll = M_soll / km;
        // -------------------------------------------------------------
        if(ti.read()>6)
            m_sa->enable_escon();
        m_sa->write_current(i_soll);                   // write to motor 0 
        // handle enable
    }// endof the main loop
}

void ControllerLoop::sendSignal() {
    thread.flags_set(threadFlag);
}
void ControllerLoop::start_loop(void)
{
    thread.start(callback(this, &ControllerLoop::loop));
    ticker.attach(callback(this, &ControllerLoop::sendSignal), Ts);
}

/* est_angle: estimate angle from acc and gyro data. This function would also fit to the "sensors_actuators"- class
but here it is better visible for students. 
*/
float ControllerLoop::est_angle(void)
{
    return 0;
}

void ControllerLoop::enable_vel_cntrl(void)
{
    vel_cntrl_enabled = true;
}
void ControllerLoop::enable_bal_cntrl(void)
{
    bal_cntrl_enabled = true;
}
void ControllerLoop::reset_cntrl(void)
{

}
void ControllerLoop::disable_all_cntrl()
{
    bal_cntrl_enabled = false;
    vel_cntrl_enabled = false;
}
