/**
 * test-1-EndDevice-4D.cpp
 *
 *  Created on: 16-Sep-2017
 *      Author: manohar
 * 
 * Raspberry Pi-3 UART settings: 
 * 		Enabled Serial in raspi-config. Or sudo nano /boot/config.txt -> make enable_uart=1 or add this line.
 * 		sudo nano /boot/cmdline.txt -> remove serial port part like ttyAMA0 or serial0 115200.
 *   
 */

#include "MeshNetwork.h"
#include <geniePi.h>  //the ViSi-Genie-RaspPi library

int main(int argc, char *argv[])	//Enter C, R1, R2, E1 // Nodes configurations
{
	// Genie display setup
	//	Using the Raspberry Pi's on-board serial port.
	//char serial_port[] = "/dev/ttyS0";
	//if (genieSetup (serial_port, 115200) < 0) {
	if (genieSetup ("/dev/ttyS0", 115200) < 0) {
		fprintf (stderr, "rgb: Can't initialise Genie Display: %s\n", strerror (errno)) ;
		return 1 ;
	}
	
	int status_led;
	char display_data[20];
	status_led = atoi(argv[1]);

	printf("statu Led : %d",status_led);
	if (status_led ==1)
	{
		genieWriteObj(GENIE_OBJ_USER_LED,0x00,1);
		strcpy(display_data, "Brake");
		genieWriteStr(0x01, display_data);
	}
	else
	{
		genieWriteObj(GENIE_OBJ_USER_LED,0x00,0);
		strcpy(display_data, " ");
		genieWriteStr(0x01, display_data);
	}
	
	return 0;
}
