#include "mbed.h"
#include <stdint.h>
#include "math.h" 
#include "ControllerLoop.h"
#include "sensors_actuators.h"
#include "uart_comm_thread.h"
#include "state_machine.h"
#include "GPA.h"


static BufferedSerial serial_port(USBTX, USBRX);

float Ts = 0.002f;    // sampling time, typically approx 1/500
GPA          myGPA( 0.5,  250,    30,2,2, Ts); // para for plant

//******************************************************************************
//---------- main loop -------------
//******************************************************************************

int main()
{

    // --------- mini cuboid,
    uart_comm_thread uart_com(&serial_port,.05f);   // this is the communication thread
    sensors_actuators hardware(Ts);         // in this class all the physical ios are handled
    ControllerLoop loop(&hardware,Ts);       // this is for the main controller loop
    state_machine sm(&hardware,&loop,0.02);
    ThisThread::sleep_for(200);
// ----------------------------------
    serial_port.set_baud(115200);
    serial_port.set_format(8,BufferedSerial::None,1);
    serial_port.set_blocking(false); // force to send whenever possible and data is there
// ----------------------------------
    uart_com.start_uart();
    ThisThread::sleep_for(20);
    loop.start_loop();
    ThisThread::sleep_for(20);
    // sm.start_loop();
    uart_com.send_text((char *)"Start Mini-Cuboid 1.10 ");
    while(1)
        ThisThread::sleep_for(200); 
        
}   // END OF main

