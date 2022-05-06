#include "state_machine.h"
using namespace std;
#define PI 3.1415927
// contructor for controller loop
state_machine::state_machine(sensors_actuators *sa, ControllerLoop *loop, float Ts) : thread(osPriorityNormal,4096)
{
    this->Ts = Ts;
    this->CS = INIT;
    this->m_sa = sa;
    this->m_loop = loop;
    this->phi_target = -PI/4;
    ti.reset();
    ti.start();
    }

// decontructor for controller loop
state_machine::~state_machine() {}

// ----------------------------------------------------------------------------
void state_machine::loop(void){
    float om = 2*PI/.8;
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        // this statemachine is for later use, here, just test sensors
        float tim = ti.read();
        switch(CS)
            {
            case INIT:
                if(m_sa->key_was_pressed && tim >.5)
                    {
                    printf("switch to FLAT, rotate\r\n");
                    m_loop->start_loop();
                    m_sa->key_was_pressed = false;
                    m_loop->enable_flat_vel_cntrl();
                    ti.reset();
                    CS = FLAT;
                    }
                break;
            case FLAT:
                if(tim>5)
                    {
                    printf("switch to BALANCE\r\n");
                    phi_target += PI/4;
                    m_loop->phi_bd_des += PI/4;
                    m_loop->reset_cntrl();
                    m_loop->enable_bal_cntrl();
                    CS = BALANCE;
                    ti.reset();
                    }
                break;
            case BALANCE:
                if(tim>5)
                    {
                    printf("switch to SWD\r\n");
                    CS = SWD_POS;
                    phi_target += PI/4;
                    ti.reset();
                    }
                break;
            case SWD_POS:
                m_loop->phi_bd_des = -PI/8*(cos(om*tim)-1.0f) +phi_target-PI/4.0f;
                //printf("%f ",m_loop->phi_bd_des);
                if(fabs(phi_target - m_sa->get_phi_bd())<.1 || tim>.4)
                    {
                    m_loop->disable_all_cntrl();
                    m_loop->enable_flat_vel_cntrl();
                    m_loop->phi_bd_des = phi_target;
                    CS = FLAT;
                    ti.reset();
                    }
                break;
            default:
                break;
            }   // end switch
        }// endof the main loop
}

void state_machine::sendSignal() {
    thread.flags_set(threadFlag);
}
void state_machine::start_loop(void)
{
    thread.start(callback(this, &state_machine::loop));
    ticker.attach(callback(this, &state_machine::sendSignal), Ts);
}