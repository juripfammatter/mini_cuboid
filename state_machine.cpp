#include "state_machine.h"
using namespace std;

// contructor for state_machine
state_machine::state_machine(sensors_actuators *sa, ControllerLoop *loop, float Ts) : thread(osPriorityNormal,4096)
{
    this->Ts = Ts;
    this->CS = INIT;
    this->m_sa = sa;
    this->m_loop = loop;
    gti.reset();
    gti.start();
    float bar2sec = 2.1f;
}

state_machine::~state_machine() {}
// ----------------------------------------------------------------------------
void state_machine::loop(void){
    float down_speed = 1.1;
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        // this statemachine is for later use, here, just test sensors
        uint8_t old_state = CS;
        CS = bar2sec * gti.read(); 
        if(old_state != CS)
        {
            C_SS = 0;
            lti.reset();
        }
        switch(CS)
            {
            case INIT:
                if(m_sa->key_was_pressed && gti.read()>.5)
                    {
                    printf("switch to FLAT\r\n");
                    m_sa->enable_escon();
                    m_loop->enable_vel_cntrl();
                    m_sa->key_was_pressed = false;
                    gti.reset();
                    CS = FLAT;
                    }
                break;
            case FLAT: 
                if(m_sa->key_was_pressed && gti.read()>.5)
                    {
                    printf("switch to BALANCE\r\n");
                    m_sa->key_was_pressed = false;
                    m_loop->reset_cntrl();
                    m_loop->enable_bal_cntrl();
                    CS = BALANCE;
                    }
                break;
            case BALANCE:
                if(m_sa->key_was_pressed && gti.read()>.5)
                    {
                    printf("switch to INIT\r\n");
                    m_sa->key_was_pressed = false;
                    CS = DOWN_R;
                    lti.reset();
                    }
                break;
            case WALK_LEFT:
                switch(C_SS)
                    {
                    case 0:
                        if(detect_on_edge())
                            {
                            C_SS = BALANCE;
                             m_loop->enable_bal_cntrl();
                            }
                        else
                            {
                            C_SS = FLAT;
                            m_loop->enable_vel_cntrl();
                            }
                        break;
                    
                        if(lti.read()*bar2)
                        ;
                    }
                break;
            case DOWN_R:
                m_loop->phi_bd_des += down_speed*Ts;
                if(gti.read()>0.7)
                    {
                    m_loop->disable_all_cntrl();
                    m_loop->enable_vel_cntrl();
                    m_loop->phi_bd_des = 0;
                    CS = FLAT;
                    gti.reset();                    
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
bool state_machine::detect_on_edge(void)
{
    if(fabs((m_sa->get_phi_fw*2)%(PI))<.5)
        return true;
    else
        return false;
}