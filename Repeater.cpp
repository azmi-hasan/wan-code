/**
 * All Repeater related tasks
 */

#include "MeshNetwork.h"


/***********************************************************************
 * Repeater_task().
 * Hopping frames.
 * Frame types: Normal Data frames, and Vehicle task B frames
 **********************************************************************/
void MeshNetwork::Repeater_task()
{
	printf("repeater task\n");
	Frame fr1, fr2, fr2_ack;
	while(1) {
		repeater_q.wait_and_pop(fr1);	//block
		// First ack to sent node -----------------------
		/*if(ACK_ENABLE == true) {
			fr1_ack.thread_id = REPEATER_THREAD_ID;
			fr1_ack.next_hop = fr1.from_id;
			fr1_ack.from_id = NODE_ID;
			fr1_ack.source_id = NODE_ID;
			fr1_ack.dest_id = fr1.from_id;
			if(fr1.fr_type == DATA_FRAME)
				fr1_ack.fr_type = ACK_FRAME;
			else if(fr1.fr_type == VEHICLE_TASK_B_FRAME)
				fr1_ack.fr_type = VEHICLE_TASK_B_ACK_FRAME;
			fr1_ack.fr_num = fr1.fr_num;
			fr1_ack.event_num = fr1.event_num;
			fr1_ack.data[0] = '\0';
			write_ack_q.push(fr1_ack);
		}*/
		printf("Repeating frame from: %d \n", fr1.from_id);
		fr2.thread_id = REPEATER_THREAD_ID;
		fr2.next_hop = getNextHop(NODE_ID, fr1.dest_id); //NEXT_HOP_1;
		fr2.from_id = NODE_ID;
		fr2.source_id = fr1.source_id;	// same. event_num+source_id = unique
		fr2.dest_id = fr1.dest_id;	// same
		fr2.fr_type = fr1.fr_type;	// same
		boost::mutex::scoped_lock lock(fr_num_mutex);
			fr2.fr_num = ++frame_number;	
			// frame number is local purpose only, valid for one-hop. event_num global purpose
		lock.unlock();
		fr2.event_num = fr1.event_num;	// event_num same
		strcpy(fr2.data, fr1.data);	// same
		// Write
		write_q.push(fr2);
		// Get ACK
		if((ACK_ENABLE) && (!ACK_SKIP)) {
			repeater_ack_q.wait_and_pop(fr2_ack);	//block. Something will come for sure from write_task.
			if(fr2_ack.tx_status == transmitted) {
				if(fr2_ack.fr_type == ACK_FRAME)
					printf("%s ACK %d \n", getThreadName(REPEATER_THREAD_ID), fr2_ack.fr_num);
				else if(fr2_ack.fr_type == VEHICLE_TASK_B_ACK_FRAME)
					printf("%s ACK Task-B %d \n", getThreadName(REPEATER_THREAD_ID), fr2_ack.fr_num);
			}
			else
				printf("ACK missed\n");
			//Error handling later.
		}
	}
}

/***********************************************************************************************
 * Repeater_to_Vehicle_Signalling()
 * Repeater Signalling to Moving Node/Enddevice.
 * 
 * Two types of frames. One passed by previous hop, to initiate vehicle broadcast. 
 * Two, ACK from Vehicle, either directly from vehicle or passed from repeaters.
 * 
 * If req, broadcast multiple times here or initiate multiple requests in detection function.
 ***********************************************************************************************/
void MeshNetwork::Repeater_to_Vehicle_Signalling()
{
	printf("Task Repeater_to_Vehicle_Signalling\n");
	Frame fr1, fr2, fr2_ack;
	
	while(1) {
		//Wait until frame comes.
		vehile_sig_q.wait_and_pop(fr1);

		// 1.Acknowledge received frame, if ACK_ENABE
		// 2. Pass to next node. addressed
		// 3. Broadcast to vehicle
		
		// First type frame ------------------------------------------------
		if(fr1.fr_type == VEHICLE_TASK_A_FRAME) {
			// Acknowledge received frame, if ACK_ENABE --------------------------------------
			/*if(ACK_ENABLE == true) {
				fr1_ack.thread_id = VEHICLE_SIG_THREAD_ID;
				fr1_ack.next_hop = fr1.from_id;
				fr1_ack.from_id = NODE_ID;
				fr1_ack.source_id = NODE_ID;
				fr1_ack.dest_id = fr1.from_id;
				fr1_ack.fr_type = VEHICLE_TASK_A_ACK_FRAME;
				fr1_ack.fr_num = fr1.fr_num;
				fr1_ack.event_num = fr1.event_num;
				fr1_ack.data[0] = '\0';
				write_ack_q.push(fr1_ack);
			}*/
			// Pass to next node. addressed -------------------------------------------------
			
			//if(NODE_ID != NetworkMap[TOTAL_STATIC_NODES-1]) {	// Not End node
			if(fr1.dest_id != NODE_ID) {	// Not End node
				printf("Vehicle task. Repeating frame from: %d \n", fr1.from_id);
				fr2.thread_id = VEHICLE_SIG_THREAD_ID;
				fr2.next_hop = getNextHop(NODE_ID, fr1.dest_id);	//NEXT_HOP_1;
				fr2.from_id = NODE_ID;
				fr2.source_id = fr1.source_id;	// same. event_num+source_id = unique
				fr2.dest_id = fr1.dest_id;	// same
				fr2.fr_type = fr1.fr_type;	// same
				boost::mutex::scoped_lock lock(fr_num_mutex);
					fr2.fr_num = ++frame_number;	
					// frame number is local purpose only, valid for one-hop.
				lock.unlock();
				fr2.event_num = fr1.event_num;	// event_num same
				strcpy(fr2.data, fr1.data);	// same
				// Write
				write_q.push(fr2);
				// Get ACK from next node
				if((ACK_ENABLE) && (!ACK_SKIP)) {
					vehicle_sig_ack_q.wait_and_pop(fr2_ack);	//block. Something will come for sure from write_task.
					if((fr2_ack.fr_type == VEHICLE_TASK_A_ACK_FRAME) && (fr2_ack.tx_status == transmitted))
						printf("%s ACK %d\n", getThreadName(VEHICLE_SIG_THREAD_ID), fr2_ack.fr_num);
					else
						printf("ACK missed\n");
					//Error handling later.
				}
			}
			
			// Broadcast to vehicle/Enddevice in range. No waiting for ACK from vehicle.------------
			fr2.thread_id = VEHICLE_SIG_THREAD_ID;
			fr2.next_hop = BROADCAST_ID;
			fr2.from_id = NODE_ID;
			fr2.source_id = NODE_ID;
			fr2.dest_id = BROADCAST_ID;
			fr2.fr_type = VEHICLE_SIG_BROADCAST_FRAME;
			// event_num is same as above. Unique per detection request.
			//fr2.data same as above
			// Write
			for(int i=0; i<NO_OF_VEHICLE_SIG_BROADCAST; i++) {
				boost::mutex::scoped_lock lock(fr_num_mutex);
					fr2.fr_num = ++frame_number;
				lock.unlock();
				write_q.push(fr2);
			}
					
		}
		
		// Second type frame --------------------------------------------
		// Print here. Later add to database. 
		// No problem if repeating frames, as vehilce can reply/broadcast ack to a max 2-to 3 nodes near-by.
		
		else if(fr1.fr_type == VEHICLE_SIG_BROADCAST_ACK_FRAME) {
			printf("Vehicle direct ACK. Event no. %d, Vehicle no: %s\n", fr1.event_num, fr1.data);
			//Pass to coordinator
			printf("Passing to Coordinator...\n");
			fr2.thread_id = VEHICLE_SIG_THREAD_ID;
			fr2.next_hop =	getNextHop(NODE_ID, COORDINATOR_ID);
			fr2.from_id = NODE_ID;
			fr2.source_id = NODE_ID;	// Or Same. vehicle number is there to identify node.
			fr2.dest_id = COORDINATOR_ID;
			fr2.fr_type = VEHICLE_TASK_B_FRAME;
			boost::mutex::scoped_lock lock(fr_num_mutex);
				fr2.fr_num = ++frame_number;
			lock.unlock();
			fr2.event_num = fr1.event_num;	// event_num same
			strcpy(fr2.data, fr1.data);	// same
			write_q.push(fr2);
			// Get ACK from next node
			if((ACK_ENABLE) && (!ACK_SKIP)) {
				vehicle_sig_ack_q.wait_and_pop(fr2_ack);	//block. Something will come for sure from write_task.
				if((fr2_ack.fr_type == VEHICLE_TASK_B_ACK_FRAME) && (fr2_ack.tx_status == transmitted))
					printf("%s ACK %d\n", getThreadName(VEHICLE_SIG_THREAD_ID), fr2_ack.fr_num);
				else
					printf("ACK missed\n");
				//Error handling later.
			}
		}
	}
}
