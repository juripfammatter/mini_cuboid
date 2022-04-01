#include "ControllerLoop.h"
using namespace std;

extern GPA myGPA;
extern DataLogger myDataLogger;

// contructor for controller loop
ControllerLoop::ControllerLoop(sensors_actuators *sa, float Ts) : thread(osPriorityHigh,4096*4)
{
    this->Ts = Ts;
    this->m_sa = sa;
    ti.reset();
    ti.start();
    }

// decontructor for controller loop
ControllerLoop::~ControllerLoop() {}

// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void ControllerLoop::loop(void){
    float i_des = 0;
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        m_sa->read_encoders_calc_speed();       // first read encoders and calculate speed
        // -------------------------------------------------------------
        m_sa->write_current(i_des);                   // write to motor 0 
        // handle enable
        m_sa->enable_motors(true);      // enable motors
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
