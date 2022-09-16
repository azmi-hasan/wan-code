/**
 * test-1.cpp
 *
 *  Created on: 13-Sep-2017
 *      Author: manohar
 * 
 * Single  code for all configurations Coordinator, Repeater, EndDevice
 * 
 */

#include "MeshNetwork.h"

int main(int argc, char *argv[])	//Enter C, R1, R2, E1 // Nodes configurations
{	
	//Parameters: DeviceType _device_type, byte _node_id, int _total_static_nodes
	//COORDINATOR, REPEATER, ENDDEVICE
	
	if(strcmp(argv[1], "C") == 0) {
		MeshNetwork network(COORDINATOR, 0x01, 5);
		// Initiate Detection alert.
		network.Create_sample_event();
		network.Read_task();
	}
	else if(strcmp(argv[1], "R1") == 0) {
		MeshNetwork network(REPEATER, 0x02, 5);
		network.Read_task();
	}
	else if(strcmp(argv[1], "R2") == 0) {
		MeshNetwork network(REPEATER, 0x03, 5);
		network.Read_task();
	}
	else if(strcmp(argv[1], "R3") == 0) {
		MeshNetwork network(REPEATER, 0x04, 5);
		network.Read_task();
	}
	else if(strcmp(argv[1], "R4") == 0) {
		MeshNetwork network(REPEATER, 0x05, 5);
		network.Read_task();
	}
	else if(strcmp(argv[1], "E1") == 0) {
		MeshNetwork network(ENDDEVICE, 0x06, 5);
		network.Read_task();
	}
	else
		printf("Enter arguments\n");
	
	return 0;
}

