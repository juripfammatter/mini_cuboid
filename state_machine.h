#include "mbed.h"
#include "sensors_actuators.h"
#include "ControllerLoop.h"

#define PI 3.1415927

#define INIT 11
#define FLAT 21
#define BALANCE 2
#define WIGGLE_SLOW 3
#define WIGGLE_FAST 4
#define WALK_LEFT 5
#define WALK_RIGHT 6
#define DOWN_L 51
#define DOWN_R 61


// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class state_machine
{
public:
    state_machine(sensors_actuators *,ControllerLoop *,float Ts);
    virtual     ~state_machine();
    void start_loop(void);

private:
    void loop(void);
    uint8_t CS;             // the current state
    uint8_t C_SS;             // the current state
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    Timer lti;
    Timer gti;
    float Ts;
    void sendSignal();
    sensors_actuators *m_sa;
    ControllerLoop *m_loop;
    bool detect_on_edge();
};