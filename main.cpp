#include "mbed.h"
#include <stdint.h>
#include "math.h" 
#include "ControllerLoop.h"
#include "sensors_actuators.h"
#include "state_machine.h"
#include "GPA.h"


static BufferedSerial serial_port(USBTX, USBRX);

float Ts = 0.02f;    // sampling time, typically approx 1/500
GPA          myGPA( .7,  250,    30,4,4, Ts); // para for plant

//******************************************************************************
//---------- main loop -------------
//******************************************************************************

int main()
{

    // --------- mini cuboid,
    sensors_actuators hardware(Ts);         // in this class all the physical ios are handled
    ControllerLoop loop(&hardware,Ts);       // this is for the main controller loop
    state_machine sm(&hardware,&loop,0.02);
    ThisThread::sleep_for(200);
    uint32_t *uid = (uint32_t *)0x1FFF7590;
    printf("\r\nUnique ID: %08X %08X %08X \r\n", uid[0], uid[1], uid[2]);
// ----------------------------------
    ThisThread::sleep_for(20);
    loop.start_loop();
    ThisThread::sleep_for(20);
    sm.start_loop();
    while(1)
        ThisThread::sleep_for(200); 
        
}   // END OF main

