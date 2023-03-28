#include "mbed.h"
#include <stdint.h>
#include "math.h" 
#include "ControllerLoop.h"
#include "sensors_actuators.h"
#include "state_machine.h"
#include "IIR_filter.h"

#define WAIT_MS(x) ThisThread::sleep_for(chrono::milliseconds(x));

static BufferedSerial serial_port(USBTX, USBRX,115200);

/* 
This is the main function of embedded project "mini_cuboid" ZHAW FS23
Altenburger February 2023
Modified by Juri Pfammatter for RT2 lectures
Robot Model: Violett4
*/

//******************************************************************************
//---------- main loop -------------
//******************************************************************************

int main()
{

    // --------- mini cuboid,
    float Ts = 0.002f;                      // sampling time, typically approx 1/500
    //float tau = 0.001f;

    //IIR_filter fil_ax(tau, Ts);
    //IIR_filter fil_ay(tau, Ts);
    //IIR_filter fil_gz(tau, Ts);

    sensors_actuators hardware(Ts);         // in this class all the physical ios are handled
    ControllerLoop cntrl_loop(&hardware,Ts);       // this is for the main controller loop
    state_machine sm(&hardware,&cntrl_loop,0.02);
    WAIT_MS(200);
    printf("- - - - MiniCuboid Start! - - - \r\n");
// ----------------------------------
    cntrl_loop.start_loop();
    WAIT_MS(20);
    sm.start_loop();
    while(1)
        {
        WAIT_MS(500);
        printf("%f; %f; %f; %f;",hardware.get_ax(),hardware.get_ay(),hardware.get_gz(), hardware.get_phi_bd());
        printf("phi_bd: %f\r\n",hardware.get_phi_bd());
        }
}   // END OF main

