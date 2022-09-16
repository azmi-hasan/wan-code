/**
 * End Device related tasks
 */

#include "MeshNetwork.h"

void MeshNetwork::EndDevice_Vehicle_Signal()
{
	printf("Task EndDevice_Vehicle_Signal\n");
	
	Frame fr1, fr1_ack;
	
	while(1) {
		//Wait until broadcast frame comes.
		vehile_sig_q.wait_and_pop(fr1);	//block
		
		// Ack the reply. Broadcast.
		fr1_ack.thread_id = VEHICLE_SIG_THREAD_ID;
		fr1_ack.next_hop = BROADCAST_ID;
		fr1_ack.from_id = NODE_ID;
		fr1_ack.source_id = NODE_ID;
		fr1_ack.dest_id = fr1.from_id;
		fr1_ack.fr_type = VEHICLE_SIG_BROADCAST_ACK_FRAME;
		fr1_ack.fr_num = fr1.fr_num;
		fr1_ack.event_num = fr1.event_num;
		strcpy(fr1_ack.data, "12706");
		//write_ack_q.push(fr1_ack);	// Should not be much delay, other wise train will move far.
		write_q.push(fr1_ack);
		
		//If req, multiple transmissions of ack broadcast.
		
		// Alert the driver: Do here or pass to Display thread.
		printf("Alert. Event-x detected at %s\n", fr1.data);
		display_q.push(std::string(fr1.data));
		
		// Should not be blocked much time here. Need to reply next message soon.
	}
}

void MeshNetwork::Display()
{
	printf("Task Display\n");
	std::string str1;
	char display_data[30], blank[] = " ";
	bool start = false, restart = false, end = true;
	unsigned long timeout=0, duration = 7000; // 10 seconds
	long blink_time = 2000;	//2 seconds
	
	while(1)
	{
		// try_pop(). If any new entry in display_q,
			// Process data to display
			// If timeout expired, then start
			// else, then restart
		// 
		
		if(display_q.try_pop(str1)) {
			// Process data to display. Later
			strcpy(display_data, "Brake");
			
			// if start==false, start time
			// else, restart time
			//if(!start) { end = false;	timeout = millis() + duration;}
			//else { timeout = millis() + duration; }
			printf("New Display Message: %s\n", display_data);
			timeout = millis() + duration;
			end = false;
		}
		
		if(!end) {
			if(millis() < timeout) {
				//Continue displaying for an iteration
				genieWriteObj(GENIE_OBJ_USER_LED,0,1);
				genieWriteStr(1, display_data);
				sleep_msec(blink_time);
				genieWriteObj(GENIE_OBJ_USER_LED,0,0);
				genieWriteStr(1, blank);
				sleep_msec(1000);
			}
			else {
				genieWriteObj(GENIE_OBJ_USER_LED,0,0);
				genieWriteStr(1, blank);
				//start = false;
				end = true;
			}
		}
		
	}
}
