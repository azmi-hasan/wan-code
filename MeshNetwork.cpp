/*
 * MeshNetwork.cpp
 *
 *  Created on: 12-Sep-2017
 *      Author: manohar
 */

#include "MeshNetwork.h"


MeshNetwork::MeshNetwork(DeviceType _device_type, byte _node_id, int _total_static_nodes)
{
	// LoRa(), SX1272() constructors automatically loaded. Inherited classes.
	
	// Set LoRa device --------------------------------
	//Initialize("10", "125");
	Initialize("11", "500");
	
	// Wait for Ready input key
	printf("LoRa ready. Enter any key: ");
	scanf("%s");
	printf("...Started...\n");
	
	TOTAL_STATIC_NODES = _total_static_nodes;
	NetworkMap = new unsigned char[TOTAL_STATIC_NODES];
	//Setting static map based on totol static nodes
	for(int i=0; i < TOTAL_STATIC_NODES; i++)
		NetworkMap[i] = i+1;
	 
	
	//NODE_ID = 0x01;
	COORDINATOR_ID = 0x01;
	DEVICE_TYPE = _device_type;	// COORDINATOR, REPEATER, ENDDEVICE
	if(DEVICE_TYPE == COORDINATOR)
		NODE_ID = COORDINATOR_ID;
	else
		NODE_ID = _node_id;
	BROADCAST_ID = 0xFF;
	
	
	
	// Initialize frame number, event number. After 255 it automatically becomes 0, 1 byte.
	frame_number = 0;
	event_number = 0;
		
	ACK_ENABLE = true;
	ACK_SKIP = false;
	
	// Frame types
	DATA_FRAME = 0x01;
	ACK_FRAME = 0x02;
	BROADCAST_DATA_FRAME = 0x03;
	BROADCAST_ACK_FRAME = 0x04;
	VEHICLE_TASK_A_FRAME = 0x05;
	VEHICLE_TASK_A_ACK_FRAME = 0x06;
	VEHICLE_SIG_BROADCAST_FRAME = 0x07;
	VEHICLE_SIG_BROADCAST_ACK_FRAME = 0x08;
	VEHICLE_TASK_B_FRAME = 0x09;
	VEHICLE_TASK_B_ACK_FRAME = 0x0A;
	
	NO_OF_VEHICLE_SIG_BROADCAST = 2;
		
	// CSMA
	macMinBE = 4;	//Test for different values of random func
	unitBackoffPeriod = 10;  //# msec. Integer only.  t_frame
	macMaxCSMABackoffs = 15;
	max_wait_for_frame_ACK = 5 * 780;  //# msec. 
	// Choose worst case max possibility, with ACK. But not very large.
	// E.g., Node-1-to-2 two threads TX data frames, Node-2 Rx, Node-3-to-1 Tx, Node-4-to-1. 40 iterations running in each Tx thread.
	// Node-1 takes time to serve 2 tx and 2 ack frames. So in Node-3,4 timout happening frequently.
	srand(time(NULL));		//Initialize random seed
	
	//Task Thread IDs
	VEHICLE_SIG_THREAD_ID = 1;	
	REPEATER_THREAD_ID = 2;
	PARSING_THREAD_ID = 3;
	//COORDINATOR_THREAD_ID = 3;
	//ENDDEVICE_THREAD_ID = 4;
			
	// Threads Launching
	// Basic threads
	write_thread = boost::thread(&MeshNetwork::Write_task, this);
	if(ACK_ENABLE == true)	write_ack_thread = boost::thread(&MeshNetwork::Write_ack_task, this);
	parsing_thread = boost::thread(&MeshNetwork::Parsing_task, this);
	// Task threads
	if(DEVICE_TYPE == COORDINATOR) {
		printf("Device Type: Coordinator. Node-ID: %d ...\n\n", NODE_ID);
		// Other network parameters...
		//repeater_thread = boost::thread(&MeshNetwork::Repeater_task, this, REPEATER_THREAD_ID);	// Not required for Coordinator
		vehicle_sig_thread = boost::thread(&MeshNetwork::Coordinator_to_Vehicle_Signalling, this);
		//sensor_data_thread...
	}
	else if(DEVICE_TYPE == REPEATER)	{
		printf("Device Type: Repeater. Node-ID: %d ...\n\n", NODE_ID);
		repeater_thread = boost::thread(&MeshNetwork::Repeater_task, this);
		vehicle_sig_thread = boost::thread(&MeshNetwork::Repeater_to_Vehicle_Signalling, this);
	}
	else if(DEVICE_TYPE == ENDDEVICE)	{
		//start_mutex.lock();
			//start = false;
		//start_mutex.unlock();
		//restart_mutex.lock();
		//	restart = true;
		//restart_mutex.unlock();
		char serial_port[] = "/dev/ttyS0";
		char blank[] = " ";
		if (genieSetup (serial_port, 115200) < 0) {
			fprintf (stderr, "rgb: Can't initialise Genie Display: %s\n", strerror (errno)) ;
			exit(1);
		}
		// Clear the display
		genieWriteObj(GENIE_OBJ_USER_LED,0,0);
		genieWriteStr(1, blank);
		printf("Device Type: EndDevice. Node-ID: %d ...\n\n", NODE_ID);
		vehicle_sig_thread = boost::thread(&MeshNetwork::EndDevice_Vehicle_Signal, this);
		display_thread = boost::thread(&MeshNetwork::Display, this);
	}
		
}

/************************************************************************
 * 
 ***********************************************************************/
MeshNetwork::~MeshNetwork() {
	write_thread.join();
	write_ack_thread.join();
	parsing_thread.join();
	if(DEVICE_TYPE == COORDINATOR) {
		vehicle_sig_thread.join();
	}
	else if(DEVICE_TYPE == REPEATER) {
		repeater_thread.join();
		vehicle_sig_thread.join();
	}
	else if(DEVICE_TYPE == ENDDEVICE) {
		vehicle_sig_thread.join();
		display_thread.join();
	}
	delete NetworkMap;
}

/************************************************************************
 * getNextHop()
 * Based on dest_id and Network Map. 
 * Later modify for two immediate neighbors.
 ***********************************************************************/
byte MeshNetwork::getNextHop(byte _node_id, byte _dest_id)
{
	int node_idx, dest_idx;
	for(int i=0; i < TOTAL_STATIC_NODES; i++) {
		if(NetworkMap[i] == _node_id)
			node_idx = i;
		if(NetworkMap[i] == _dest_id)
			dest_idx = i;
	}
	if(abs(node_idx-1 - dest_idx) < abs(node_idx+1 - dest_idx))
		return NetworkMap[node_idx-1];
	else
		return NetworkMap[node_idx+1];
}

/************************************************************************
 * 
 ***********************************************************************/
void MeshNetwork::get_frame_str(Frame & fr, char * fr_str) {
	fr_str[0] = fr.from_id;
	fr_str[1] = fr.next_hop;
	fr_str[2] = fr.source_id;
	fr_str[3] = fr.dest_id;
	fr_str[4] = fr.fr_type;
	fr_str[5] = fr.fr_num;
	fr_str[6] = fr.event_num;
	//fr_str[7] = '\0';
	//strcat(fr_str, fr.data);
	int i, j=7+strlen(fr.data);
	for(i=7; i < j ; i++)	fr_str[i] = fr.data[i-7];
	fr_str[i] = '\0';
	printf("Debug: %s, Next_hop: %d, fr_type: %d\n", fr_str, fr.next_hop, fr.fr_type);	//------------------Debug-----
}

/************************************************************************
 * 
 ***********************************************************************/
void MeshNetwork::parse_frame(const char * fr_str, Frame & fr) {
	// Parse as it is the received frame. 
	//next_hop is TO address
	fr.from_id = fr_str[0];
	fr.next_hop = fr_str[1];
	fr.source_id = fr_str[2];
	fr.dest_id = fr_str[3];
	fr.fr_type = fr_str[4];
	fr.fr_num = fr_str[5];
	fr.event_num = fr_str[6];
	int i, len = strlen(fr_str);
	for(i=7; i<len; i++) fr.data[i-7] = fr_str[i];
	fr.data[i-7] = '\0';
}

/************************************************************************
 * Using for Data frames and Broadcast frames, broadcast ack frames. 
 * Not for addressed ACK frames.
 ***********************************************************************/
Tx_Status MeshNetwork::CSMA_CA(Frame& write_fr, char * fr_str, Frame& fr_ack, int _BE){
	int NB=0;
	int BE= (_BE<0) ? macMinBE : _BE;
	Tx_Status status;
	long delay_msec;
	CADstatus ch_status;
	unsigned long t1, elapsed_time;
	bool t_check = true;
	
	get_frame_str(write_fr, fr_str);
	while(NB <= macMaxCSMABackoffs) {
		status = nottransmitted;
		delay_msec = (BE==0) ? 0 : (unitBackoffPeriod * (rand() % (int)(pow(2, BE))));	// random integer (0, (2^BE - 1))
		sleep_msec(delay_msec);		//random backoff delay
		//printf("Random backoff delay= %ld ms \n", delay_msec);	//--------------------------Debug-----
		//if(t_check == true) {t1 = millis(); t_check=false;}//only once t1	//--------------------------Debug-----
		//SPI_mutex.lock();	//-------------------
		boost::mutex::scoped_lock lock(SPI_mutex);
		ch_status = CAD(false);	// CAD
		if(ch_status == channel_free) 
		{
			//printf("Debug: %s\n", fr_str);
			sendPacket((uint8_t)strlen(fr_str), fr_str);	// Send
			//SPI_mutex.unlock();	//-------------------
			lock.unlock();
						
			if((ACK_ENABLE==true) && (write_fr.next_hop!=BROADCAST_ID)) // More fr_type checks required to merge with CSMA_CA_2()
			{
				if(!get_ack_q.timed_wait_and_pop(fr_ack, 
					boost::posix_time::milliseconds(max_wait_for_frame_ACK))) {	// Block until frame ACK received or timeout
					// If ACK not eceived. Queue empty
					printf("Frame ACK not received. Timeout. Retransmitting %d\n", write_fr.fr_num);
					NB += 1;
					BE = std::min(BE + 1, macMinBE);
				}
				else {
					// Check desired ACK or not
					if(fr_ack.fr_num == write_fr.fr_num) {
						elapsed_time = millis() - t1;
						//printf("Elapsed time: %lu ms\n", elapsed_time);	//--------------------------Debug-----
						status = transmitted;
						return status;
					}
					else {
						//printf("Undesired ACK %d. Retransmitting %d\n", fr_ack.fr_num, write_fr.fr_num);
						//NB += 1;
						//BE = std::min(BE + 1, macMinBE);
						//wait again ---------------------------
						printf("Undesired ACK %d. Waiting again for %d\n", fr_ack.fr_num, write_fr.fr_num);
						if(!get_ack_q.timed_wait_and_pop(fr_ack, 
							boost::posix_time::milliseconds(max_wait_for_frame_ACK))) {	// Block until frame ACK received or timeout
							// If ACK not eceived. Queue empty
							printf("Frame ACK not received. Timeout. Retransmitting %d\n", write_fr.fr_num);
							NB += 1;
							BE = std::min(BE + 1, macMinBE);
						}
						else {
							// Check desired ACK or not
							if(fr_ack.fr_num == write_fr.fr_num) {
								elapsed_time = millis() - t1;
								//printf("Elapsed time: %lu ms\n", elapsed_time);	//--------------------------Debug-----
								status = transmitted;
								return status;
							}
							else {
								printf("Undesired ACK %d. Retransmitting %d\n", fr_ack.fr_num, write_fr.fr_num);
								NB += 1;
								BE = std::min(BE + 1, macMinBE);
							}
						}
						//-------------------------
					}
				}
			}
			else {
				elapsed_time = millis() - t1;
				//printf("Elapsed time: %lu ms\n", elapsed_time);	//--------------------------Debug-----
				status = transmitted;
				return status;
			}
		}
		else if(ch_status == channel_busy) {
			//SPI_mutex.unlock();	//-------------------
			lock.unlock();
			printf("Channel busy\n");
			NB += 1;
			BE = std::min(BE + 1, macMinBE);
		}
		else if(ch_status == cad_unexecuted) {
			//SPI_mutex.unlock();	//-------------------
			lock.unlock();
			printf("Error in CAD execution\n");
			NB += 1;
		}
				
	}
	status = max_retries;
	return status;
}

/************************************************************************
 * Using this for writing 
 * ACK frames only. (Addressed)
 ***********************************************************************/
Tx_Status MeshNetwork::CSMA_CA_2(Frame& write_fr, char * fr_str, int _BE){
	int NB=0;
	int BE= (_BE<0) ? macMinBE : _BE;
	Tx_Status status;
	long delay_msec;
	CADstatus ch_status;
	unsigned long t1, elapsed_time;
	bool t_check = true;

	get_frame_str(write_fr, fr_str);
	while(NB <= macMaxCSMABackoffs) {
		status = nottransmitted;
		delay_msec = (BE==0) ? 0 : (unitBackoffPeriod * (rand() % (int)(pow(2, BE))));	// random integer (0, (2^BE - 1))
		sleep_msec(delay_msec);		//random backoff delay
		//if(BE!=0)	printf("Random backoff delay= %ld ms \n", delay_msec); //--------------------------Debug-----
		//if(t_check == true) {t1 = millis(); t_check=false;}//only once t1	//--------------------------Debug-----
		//SPI_mutex.lock();	//-------------------
		boost::mutex::scoped_lock lock(SPI_mutex);
			ch_status = CAD(false);	// CAD
			if(ch_status == channel_free) {
				sendPacket((uint8_t)strlen(fr_str), fr_str);	// Send
				//SPI_mutex.unlock();	//-------------------
				lock.unlock();
				elapsed_time = millis() - t1;
				//printf("Elapsed time: %lu ms\n", elapsed_time);		//--------------------------Debug-----
				status = transmitted;
				return status;
			}
			else if(ch_status == channel_busy) {
				//SPI_mutex.unlock();	//-------------------
				lock.unlock();
				printf("Channel busy\n");
				NB += 1;
				BE = std::min(BE + 1, macMinBE);
			}
			else if(ch_status == cad_unexecuted) {
				//SPI_mutex.unlock();	//-------------------
				lock.unlock();
				printf("Error in CAD execution\n");
				NB += 1;
			}
		
	}
	status = (ch_status!=cad_unexecuted) ? max_retries : error_tx;
	return status;
}

/************************************************************************
 * Write addressed ACK Frame only. Not Broadcast ACK frames.
 ***********************************************************************/
void MeshNetwork::Write_ack_task()
{
	printf("write_ack_task\n");
	Frame write_fr;
	Tx_Status ack_tx_status;
	char fr_str[PAYLOAD_MAX_SIZE];
	
	while(1) {
		//write_ack_q.wait_and_pop(write_fr);		//bolck
		if(write_ack_q.try_pop(write_fr)) {
		//# With CSMA/CA in first iteration.
		//ack_tx_status = CSMA_CA_2(write_fr, -1);
		//Without CSMA/CA in first iteration
		ack_tx_status = CSMA_CA_2(write_fr, fr_str, 0);
		//write_fr.next_hop = 0xFF;
		if (ack_tx_status == transmitted)
			printf("%s transmitted ACK %d\n", getThreadName(write_fr.thread_id), write_fr.fr_num);
		else if (ack_tx_status == max_retries)
			printf("ACK Tx failed. CSMA Max retries done. %d\n", write_fr.fr_num);
		else
			printf("ERROR in ACK Transmission. %d", write_fr.fr_num);
		//# No further ERROR handling for ACK frame, as source is checking ACK-recv.
		//sleep_msec(10);
		}	
	}
}

/************************************************************************
 * write_task()
 * write_task_2() in v1
 * Using for Data frames and Broadcast frames, broadcast ack frames. 
 * Not for addressed ACK frames.
 ***********************************************************************/
void MeshNetwork::Write_task()
{
	printf("write_task\n");
	Frame write_fr, fr_ack;
	Tx_Status fr_tx_status, ack_tx_status;
	char fr_str[PAYLOAD_MAX_SIZE];
	
	while(1) {
		//write_q.wait_and_pop(write_fr);	//Block
		if(write_q.try_pop(write_fr)) {	//if not empty
			fr_tx_status = CSMA_CA(write_fr, fr_str, fr_ack, -1);
			if (fr_tx_status == transmitted)
				printf("%s transmitted Frame %d\n", getThreadName(write_fr.thread_id), write_fr.fr_num);
			else if (fr_tx_status == max_retries)
				printf("Frame Tx failed. CSMA Max retries done. %d\n", write_fr.fr_num);
			else
				printf("ERROR in Frame Transmission. %d", write_fr.fr_num);
			//# Notify the success/failure to the task thread
			if(write_fr.next_hop != BROADCAST_ID) {	// For broadcast, no need to notify.
				fr_ack.tx_status = fr_tx_status;
				if(write_fr.thread_id == VEHICLE_SIG_THREAD_ID)		vehicle_sig_ack_q.push(fr_ack); 
				else if(write_fr.thread_id == REPEATER_THREAD_ID)	repeater_ack_q.push(fr_ack);
				//else if(write_fr.thread_id ==)
				//else if(write_fr.thread_id ==)
				//strcpy(fr_str, "0");
			}
		}
	}
}

/************************************************************************
 * Name: Read_task
 * Run in Main() thread/function
 ***********************************************************************/
void MeshNetwork::Read_task()
{
	state_r rs;
	char fr_str[PAYLOAD_MAX_SIZE];
	
	while(1) {
		//SPI_mutex.lock();
		boost::mutex::scoped_lock lock(SPI_mutex);
			rs = receivePacket(fr_str, true);
		//SPI_mutex.unlock();	
		lock.unlock();
		if (rs.e==success){
			//printf("received Packet. %s\n", frame_str_r);
			receive_q.push(std::string(fr_str));
		}
	}
}

/************************************************************************
 * Parse the received frames and pass to corresponding task threads.
 * 
 * Parse based on frame types in top level. First writing a simple long state machine for all types. Then simpification of common needs, etc.
 * Or Parse based on different possible states or sub-events...
 * Or Parse based on different functions/responses handled by each thread/node type - coordinator, repeater, enddevice.
 * If required, launch dynamic task threads.
 ***********************************************************************/
void MeshNetwork::Parsing_task()
{
	printf("parsing task\n");
	std::string fr_str;
	Frame fr1, fr1_ack;
	while(1) {
		receive_q.wait_and_pop(fr_str);	//block
		printf("%s\n", fr_str.c_str());
		parse_frame(fr_str.c_str(), fr1);
		if((fr1.next_hop==NODE_ID) || (fr1.next_hop==BROADCAST_ID)) {	// Check whether addressed to this node
			// First check if received ACK frames (addressed). More priority
			if( (ACK_ENABLE==true) && ( (fr1.fr_type==ACK_FRAME) || (fr1.fr_type==VEHICLE_TASK_A_ACK_FRAME)
				|| (fr1.fr_type==VEHICLE_TASK_B_ACK_FRAME) ) ) {
				if(ACK_SKIP) {
					printf("ACK frame %d", fr1.fr_num);
				}
				else {
					get_ack_q.push(fr1);	// Write thread
				}
			}
			else if(fr1.fr_type == VEHICLE_TASK_A_FRAME) {
				// Received by Repeater only. From coordinator or previous repeater.
				if(DEVICE_TYPE == REPEATER) {
					if(ACK_ENABLE == true) Acknowledge(fr1, fr1_ack);
					vehile_sig_q.push(fr1);
				}
			}
			else if(fr1.fr_type == VEHICLE_SIG_BROADCAST_FRAME) {
				// Received by all, but for Enddevice only.
				if(DEVICE_TYPE == ENDDEVICE)
					vehile_sig_q.push(fr1);					
			}
			else if(fr1.fr_type == VEHICLE_SIG_BROADCAST_ACK_FRAME) {
				// Received by all, but for Coordinator and Repeater.
				if(DEVICE_TYPE != ENDDEVICE)
					vehile_sig_q.push(fr1);
			}
			else if(fr1.fr_type == VEHICLE_TASK_B_FRAME) {
				// Received by ... But for Coordinator and Repeater.
				if(ACK_ENABLE == true) Acknowledge(fr1, fr1_ack);
				if(DEVICE_TYPE == COORDINATOR)
					vehile_sig_q.push(fr1);
				else if(DEVICE_TYPE == REPEATER)
					repeater_q.push(fr1);
			}
			else if(fr1.fr_type == DATA_FRAME) {
				if(ACK_ENABLE == true) Acknowledge(fr1, fr1_ack);
				// Repeat incoming frame.
				if(fr1.dest_id != NODE_ID)
					repeater_q.push(fr1);
				else
					printf("DATA_FRAME %s\n", fr1.data);
				// Receied Sensor Data at Coordinator. Later development.
			}
			else if(fr1.fr_type == BROADCAST_DATA_FRAME) {
				// Not yet any need
				printf("BROADCAST_DATA_FRAME %s\n", fr1.data);
			}
			else if(fr1.fr_type == BROADCAST_ACK_FRAME) {
				// Not yet any need
				printf("BROADCAST_ACK_FRAME %s\n", fr1.data);
			}
		}
	}
}


/************************************************************************
 * Acknowledgement generation
 ***********************************************************************/
 void MeshNetwork::Acknowledge(Frame fr1, Frame fr1_ack)
 { 
	 if(fr1.fr_type == VEHICLE_TASK_A_FRAME) {
		fr1_ack.thread_id = PARSING_THREAD_ID;
		fr1_ack.next_hop = fr1.from_id;
		fr1_ack.from_id = NODE_ID;
		fr1_ack.source_id = NODE_ID;
		fr1_ack.dest_id = fr1.from_id;
		fr1_ack.fr_type = VEHICLE_TASK_A_ACK_FRAME;
		fr1_ack.fr_num = fr1.fr_num;
		fr1_ack.event_num = fr1.event_num;
		fr1_ack.data[0] = '\0';
		write_ack_q.push(fr1_ack);
	 }
	 else if(fr1.fr_type == VEHICLE_TASK_B_FRAME) {
		fr1_ack.thread_id = PARSING_THREAD_ID;
		fr1_ack.next_hop = fr1.from_id;
		fr1_ack.from_id = NODE_ID;
		fr1_ack.source_id = NODE_ID;
		fr1_ack.dest_id = fr1.from_id;
		fr1_ack.fr_type = VEHICLE_TASK_B_ACK_FRAME;
		fr1_ack.fr_num = fr1.fr_num;
		fr1_ack.event_num = fr1.event_num;
		fr1_ack.data[0] = '\0';
		write_ack_q.push(fr1_ack);
	 }
	 else if(fr1.fr_type == DATA_FRAME) {
		fr1_ack.thread_id = PARSING_THREAD_ID;
		fr1_ack.next_hop = fr1.from_id;
		fr1_ack.from_id = NODE_ID;
		fr1_ack.source_id = NODE_ID;
		fr1_ack.dest_id = fr1.from_id;
		fr1_ack.fr_type = ACK_FRAME;
		fr1_ack.fr_num = fr1.fr_num;
		fr1_ack.event_num = fr1.event_num;
		fr1_ack.data[0] = '\0';
		write_ack_q.push(fr1_ack);
	 }
 }
 
/************************************************************************
 * 
 ***********************************************************************/
const char* MeshNetwork::getThreadName(uint8_t thread_id) {
	if (thread_id == VEHICLE_SIG_THREAD_ID)	return "Vehicle_Sig";
	if (thread_id == REPEATER_THREAD_ID)	return "Repeater";
	if (thread_id == PARSING_THREAD_ID)	return "Parsing thread";
	else return "--";
	//if (thread_id == COORDINATOR_THREAD_ID)	return "Coordinator";
	//if (thread_id == ENDDEVICE_THREAD_ID)	return "EndDevice";
	
}

//------------------------------------------------------

/************************************************************************
 * 
 ***********************************************************************/
int sleep_msec(long milliseconds) {
	struct timespec req;
	//struct timespec rem;
	if(milliseconds > 999) {   
	  req.tv_sec = (int)(milliseconds / 1000);                            /* Must be Non-Negative */
	  req.tv_nsec = (milliseconds - ((long)req.tv_sec * 1000)) * 1000000; /* Must be in range of 0 to 999999999 */
	}   
	else {   
	  req.tv_sec = 0;                         /* Must be Non-Negative */
	  req.tv_nsec = milliseconds * 1000000;    /* Must be in range of 0 to 999999999 */
	}   
	//rem = NULL;
	return nanosleep(&req , NULL);
}

//------------------------------------------------------

/************************************************************************
 * 
 ***********************************************************************/
int sleep_nsec(long nanoseconds) {
	struct timespec req;
	//struct timespec rem;
	if (nanoseconds > 999999999) {
	  req.tv_sec = (int)(nanoseconds / 1000000000);
	  req.tv_nsec = (nanoseconds - ((long)req.tv_sec * 1000000000));
	}
	else {
	  req.tv_sec = 0;
	  req.tv_nsec = nanoseconds;
	}
	//rem = NULL;
	return nanosleep(&req , NULL);
}

//--------------------------------------------

