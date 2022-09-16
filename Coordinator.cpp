/**
 * Coordinator related tasks.
 * 
 */

#include "MeshNetwork.h"

void MeshNetwork::Receive_Sensor_Data()
{
	
}

void MeshNetwork::Create_sample_event()
{
	Frame fr1;
	fr1.fr_type = VEHICLE_TASK_A_FRAME;
	strcpy(fr1.data, "El67/8");
	vehile_sig_q.push(fr1);
}

/***********************************************************************
 * Vehicle_Signalling()
 * Coordinator Signalling to Moving Node/Enddevice.
 * Multiple requests handling through queue.
 * In coordinator, sensor data processing function will initiate this event/Frame, 
 * with only detection related data and fr_type in Frame object. 
 * Currently a dummy function used. Later integrating with RPI-server sensor data processing.
 * 
 * Two types of frames handling. One created by detection alert function. 
 * Two, ACK from Vehicle, either directly from vehicle or passed from repeaters. 
 * 
 * If req, broadcast multiple times here or initiate multiple requests in detection function.	
 **********************************************************************/
void MeshNetwork::Coordinator_to_Vehicle_Signalling()
{
	printf("Task Coordinator_to_Vehicle_Signalling\n");
	Frame fr1, fr1_ack;
	
	while(1) {
		// Wait until a detection alert comes, after processing sensor data. 
		vehile_sig_q.wait_and_pop(fr1);
		
		// First type frame ------------------------------------------------
		if(fr1.fr_type == VEHICLE_TASK_A_FRAME) {
			// First pass to next node. addressed. -----------------------------
			fr1.thread_id = VEHICLE_SIG_THREAD_ID;
			fr1.from_id = NODE_ID;
			fr1.source_id = NODE_ID;
			fr1.dest_id = NetworkMap[TOTAL_STATIC_NODES-1];	// Send until end node
			fr1.next_hop = getNextHop(NODE_ID, fr1.dest_id);	
			//fr1.fr_type same
			boost::mutex::scoped_lock lock(fr_num_mutex);
				fr1.fr_num = ++frame_number;	
				// frame number is local purpose only, valid for one-hop.
			lock.unlock();
			event_num_mutex.lock();
				fr1.event_num = ++event_number;	
				// event_num global purpose
			event_num_mutex.unlock();
			//fr1.Data -> already filled by requesting function
			// Write
			write_q.push(fr1);
			// Get ACK from next node 
			if((ACK_ENABLE) && (!ACK_SKIP)) {
				vehicle_sig_ack_q.wait_and_pop(fr1_ack);	//block. Something will come for sure from write_task.
				if((fr1_ack.fr_type == VEHICLE_TASK_A_ACK_FRAME) && (fr1_ack.tx_status == transmitted))
					printf("%s ACK %d\n", getThreadName(VEHICLE_SIG_THREAD_ID), fr1_ack.fr_num);
				else
					printf("ACK missed");
				//Error handling later.
			}
			
			// Broadcast to vehicle/Enddevice in range. No waiting for ACK from vehicle. -----------------
			fr1.thread_id = VEHICLE_SIG_THREAD_ID;
			fr1.next_hop = BROADCAST_ID;
			fr1.from_id = NODE_ID;
			fr1.source_id = NODE_ID;
			fr1.dest_id = BROADCAST_ID;
			fr1.fr_type = VEHICLE_SIG_BROADCAST_FRAME;
			// event_num is same as above. Unique per detection request.
			//fr1.data -> already filled by requesting function
			// Write
			for(int i=0; i<NO_OF_VEHICLE_SIG_BROADCAST; i++) {
				boost::mutex::scoped_lock lock(fr_num_mutex);
					fr1.fr_num = ++frame_number;
				lock.unlock();
				write_q.push(fr1);
			}
					
		}
		
		// Second type frame --------------------------------------------
		// Print here. Later add to database. 
		// No problem if repeating frames, as vehilce can reply/broadcast ack to a max 2-to 3 nodes near-by.
		
		else if(fr1.fr_type == VEHICLE_SIG_BROADCAST_ACK_FRAME) {
			printf("Vehicle direct ACK. Vehicle no: %s\n", fr1.data);
		}
		else if(fr1.fr_type == VEHICLE_TASK_B_FRAME) {
			/*if(ACK_ENABLE == true) {
				fr1_ack.thread_id = VEHICLE_SIG_THREAD_ID;
				fr1_ack.next_hop = fr1.from_id;
				fr1_ack.from_id = NODE_ID;
				fr1_ack.source_id = NODE_ID;
				fr1_ack.dest_id = fr1.from_id;
				fr1_ack.fr_type = VEHICLE_TASK_B_ACK_FRAME;
				fr1_ack.fr_num = fr1.fr_num;
				fr1_ack.event_num = fr1.event_num;
				fr1_ack.data[0] = '\0';
				write_ack_q.push(fr1_ack);
			}*/
			printf("Vehicle Indirect ACK. Vehicle no: %s. Source: %d\n", fr1.data, fr1.source_id);
		}
	}
}
