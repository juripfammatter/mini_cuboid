#include "state_machine.h"
using namespace std;

// contructor for controller loop
state_machine::state_machine(sensors_actuators *sa, ControllerLoop *loop, float Ts) : thread(osPriorityNormal,4096)
{
    this->Ts = Ts;
    this->CS = INIT;
    this->m_sa = sa;
    this->m_loop = loop;
    ti.reset();
    ti.start();
    }

// decontructor for controller loop
state_machine::~state_machine() {}

// ----------------------------------------------------------------------------
void state_machine::loop(void){
    
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        switch(CS)
            {
            case INIT:
                if(m_sa->key_was_pressed)
                    {
                    printf("switch to FLAT\r\n");
                    m_sa->key_was_pressed = false;
                    CS = FLAT;
                    }
                break;
            case FLAT:
                
                if(m_sa->key_was_pressed)
                    m_sa->enable_escon();
                    m_sa->write_current(1);
                    {
                    printf("switch to BALANCE\r\n");
                    m_sa->disable_escon();
                    m_sa->write_current(0);
                    m_sa->key_was_pressed = false;
                    CS = BALANCE;
                    }
                break;
            case BALANCE:
                if(m_sa->key_was_pressed)
                    {
                    printf("switch to INIT\r\n");
                    m_sa->key_was_pressed = false;
                    CS = INIT;
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
