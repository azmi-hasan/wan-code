/**
 * test-1.cpp
 *
 *  Created on: 13-Sep-2017
 *      Author: manohar
 * 
 * Single  code for all configurations Coordinator, Repeater, EndDevice
 * 
 * Support up to 1 Coordinator , 10 Repeaters, 1 EndDevice
 * 
 */

#include "MeshNetwork.h"

int main(int argc, char *argv[])	// Enter C or R1 or R2.... or E1 // Nodes configurations. 
									// Enter total static nodes including Coordinator, Repeaters, not EndDevice (moving node).
{	
	//Parameters: DeviceType _device_type, byte _node_id, int _total_static_nodes
	//COORDINATOR, REPEATER, ENDDEVICE
	
	if(argc != 3) {
		printf("Enter arguments properly, %d", argc);
		return 1;
	}
	
	int _total_static_nodes = atoi(argv[2]);
	
	if(strcmp(argv[1], "C") == 0) {
		MeshNetwork network(COORDINATOR, 0x01, _total_static_nodes);
		// Initiate Detection alert.
		network.Create_sample_event();
		network.Read_task();
	}
	else if(strcmp(argv[1], "R1") == 0) {
		MeshNetwork network(REPEATER, 0x02, _total_static_nodes);
		network.Read_task();
	}
	else if(strcmp(argv[1], "R2") == 0) {
		MeshNetwork network(REPEATER, 0x03, _total_static_nodes);
		network.Read_task();
	}
	else if(strcmp(argv[1], "R3") == 0) {
		MeshNetwork network(REPEATER, 0x04, _total_static_nodes);
		network.Read_task();
	}
	else if(strcmp(argv[1], "R4") == 0) {
		MeshNetwork network(REPEATER, 0x05, _total_static_nodes);
		network.Read_task();
	}
	else if(strcmp(argv[1], "R5") == 0) {
		MeshNetwork network(REPEATER, 0x06, _total_static_nodes);
		network.Read_task();
	}
	else if(strcmp(argv[1], "R6") == 0) {
		MeshNetwork network(REPEATER, 0x07, _total_static_nodes);
		network.Read_task();
	}
	else if(strcmp(argv[1], "R7") == 0) {
		MeshNetwork network(REPEATER, 0x08, _total_static_nodes);
		network.Read_task();
	}
	else if(strcmp(argv[1], "R8") == 0) {
		MeshNetwork network(REPEATER, 0x09, _total_static_nodes);
		network.Read_task();
	}
	else if(strcmp(argv[1], "R9") == 0) {
		MeshNetwork network(REPEATER, 0x0A, _total_static_nodes);
		network.Read_task();
	}
	else if(strcmp(argv[1], "R10") == 0) {
		MeshNetwork network(REPEATER, 0x0B, _total_static_nodes);
		network.Read_task();
	}
	else if(strcmp(argv[1], "E1") == 0) {
		MeshNetwork network(ENDDEVICE, 0x0C, _total_static_nodes);
		network.Read_task();
	}
	else
		printf("Enter arguments\n");
	
	return 0;
}

