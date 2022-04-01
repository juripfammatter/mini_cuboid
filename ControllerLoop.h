#include "mbed.h"
#include "EncoderCounter.h"
#include "EncoderCounterIndex.h"
#include "LinearCharacteristics.h"
#include "ThreadFlag.h"
#include "PID_Cntrl.h"
#include "DataLogger.h"
#include "sensors_actuators.h"


// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class ControllerLoop
{
public:
    ControllerLoop(sensors_actuators *,float Ts);
    virtual     ~ControllerLoop();
    void start_loop(void);
    void reset_pids(void);


private:
    void loop(void);
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    Timer ti;
    float Ts;
    void sendSignal();
    sensors_actuators *m_sa;
};
