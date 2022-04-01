#include "mbed.h"
#include <stdint.h>
#include "math.h" 
#include "DataLogger.h"
#include "ControllerLoop.h"
#include "sensors_actuators.h"
#include "uart_comm_thread.h"
 
static BufferedSerial serial_port(USBTX, USBRX);

float Ts = 0.002f;                    // sampling time

DataLogger myDataLogger(1);

//******************************************************************************
//---------- main loop -------------
//******************************************************************************

int main()
{

    // --------- Mirror kinematik, define values, trafos etc there
    uart_comm_thread uart_com(&serial_port,.05f);   // this is the communication thread
    sensors_actuators hardware(Ts);         // in this class all the physical ios are handled
    ControllerLoop loop(&hardware,Ts);       // this is forthe main controller loop
    ThisThread::sleep_for(200);
// ----------------------------------
    serial_port.set_baud(115200);
    serial_port.set_format(8,BufferedSerial::None,1);
    serial_port.set_blocking(false); // force to send whenever possible and data is there
    
    uart_com.start_uart();
    loop.start_loop();
    ThisThread::sleep_for(200);
    uart_com.send_text((char *)"Start Mini-Cuboid 1.0");
    while(1)
        ThisThread::sleep_for(200);
        
}   // END OF main


void reset_data(Data_Xchange *da)
{
    for(uint8_t k=0;k<2;k++)
        {
        da->sens_phi[k] = 0;
        da->est_xy[k] = 0;
        da->sens_Vphi[k] = 0;
        da->cntrl_phi_des[k] = 0;
        da->cntrl_Vphi_des[k] = 0;
        da->cntrl_xy_des[k] = 0;
        da->i_des[k] = 0;        
        da->wMot[k] = 0;         
        }
    da->laser_on = false;
    da->num_it = 0;
} 
