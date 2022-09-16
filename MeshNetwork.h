/**
 * MeshNetwork.h
 *
 *  Created on: 12-Sep-2017
 *      Author: manohar
 * 
 * MeshNetwork-v2. 
 * 
 * MeshNetwork-v1 -> LoRa_NW-v1
 * 
 * Install boost library in Raspberry Pi
 * 		sudo apt-get install libboost-all-dev
 * 
 * Compilation error. Modified header file in boost library at root.
 * 		sudo nano /usr/include/boost/thread/pthread/thread_data.hpp
 * 		Changed getpagesize() to unistd::getpagesize()
 * 
 * Boost library doubts: Ref examples of C++11 also, as they are similar.
 * 
 * Shared variables, queues. Python didn't supported class variables as shared/modifiable (with mutexes) by all threads. But C++ does.
 * 
 * Atomic class: I think its a wrapper with mutex locks or some syncronization. Looks complex. Not required.
 * 
 * Compilation in RPI: In RPI, transfer only modified code files. make taking long time to compile/include boost library includes.
 * 
 * Queue: C++ thread safe blocking queue using boost. Doc in Queue.cpp
 * Acknowledgement: 
 * 		Anthony Williams! 
 * 		https://www.justsoftwaresolutions.co.uk/threading/implementing-a-thread-safe-queue-using-condition-variables.html
 * 		https://www.justsoftwaresolutions.co.uk/threading/condition-variable-spurious-wakes.html
 * 
 * Not using much iostream, -std=c++11, as later code should be used with Arduino or RTOS, etc for repeater. Doubt...
 * 
 * Note: Push and Pop operations here have mutexes internally. Avoid nested mutex locks, unless very necessary.
 * 
 * frame number fr_num: frame number is local purpose only, valid for one-hop. 
 * event_num: event_num global purpose. Source and event_num combinedly unique. Initiated at source only.
 * 
 * Frame types (fr_type): Defined as per the different tasks/sub-tasks created in the system, to parse the received frame and 
 * 	pass to corresponding task thread. Can be used for Dynamic sub-task threads also.
 * 	1: Data frame, normal
 * 	2: ACK
 * 	3: Broadcast data frame
 * 	4: Braodcast ack frame, 			(if any)
 * 	5: Vehicle sig task A data frame 	(For coord/rep to repeater, for initiating Veh sig)
 * 	6: Vehicle sig task A ack frame		(For coord/rep  to repeater, for initiating Veh sig)
 * 	7: Vehicle sig broadcast data frame (For Coord/Rep to Enddevice only, one hop)
 * 	8: Vehicle sig broadcast ack frame 	(For Enddevice to Coord/Rep only, one hop)
 * 	9: Vehicle sig task B data frame 	(Global ACK of vehicle) (For repeater to coordinator/repeater)
 * 10: Vehicle sig task B ack frame 	(Global ACK of vehicle) (For repeater to coordinator/repeater)
 * 
 * No waiting for ACK, in case of Broadcast frames. Handling separately, if any.
 * 
 * If any slow running or missing any event or unsyncronization, replace wait_and_pop() with time_wait_and_pop().
 * 
 * Note: strcat() is not working properly. Taking first 0x00 value as ending of string.
 * Avoid 0x00 values in strings...
 * 0x00 is NULL character itself.
 * 
 * Broadcast frames no Qos gaurantee, due to no ACK expectation. So best is multiple transmissions.
 * 
 * In EndDevice, seems like Vehicle ack broadcast needs multiple transmissions, as much collision happening. Later.
 * But lot of packets will be generated with same content.
 * 
 * Problem: If one node gets blocked for ACK retransmission, next node, next-next node... will also keep retransmitting. 
 * But ACK is served in another thread. This is the advantage of separate ACK thread (Telit Test_11)
 * 
 * Problem: Broadcast ACK from Enddevice, colliding drastically. But CAD() is done. Try Passing through write_task().
 * Also in lab it is 4 node simultaneous test, which causes high collision. Yes, improved, more vehicle ack received at Coordinator.
 * 
 * Raspberry Pi-3 UART settings: 
 * 		Enabled Serial in raspi-config. Or sudo nano /boot/config.txt -> make enable_uart=1 or add this line.
 * 		sudo nano /boot/cmdline.txt -> remove serial port part like ttyAMA0 or serial0 115200.
 */


#ifndef MESHNETWORK_H_
#define MESHNETWORK_H_

// Includes =============================================================================================================
#include "LoRa.h"
#include <boost/thread/thread.hpp>
#include <queue>
#include <geniePi.h>  //the ViSi-Genie-Raspberry Pi library

// MACROS and Global variables ==========================================================================================
#define PAYLOAD_MAX_SIZE 64		//Note: Total payload to RFM95W should not exceed FIFO size: SX1276-Lora:256 or 128, FSK:64
// Also set FIFO_rw_buffer size in LoRa.h
#define FRAME_DATA_MAX_SIZE 10	//...	
#define QUEUE_MAX_SIZE 100

//Data types ============================================================================================================

enum Tx_Status {transmitted, nottransmitted, error_tx, max_retries};

enum DeviceType {COORDINATOR, REPEATER, ENDDEVICE}; //If required, add more types.

/*****************************************
 * Requirements:
 *   - T must have a copy constructor
 *   - T must have a trivial assignment operator
 *   - T must have a trivial destructor
 * ***************************************/
class Frame {
public:
	uint8_t thread_id;
	byte next_hop;
	byte from_id;
	byte source_id;
	byte dest_id;
	byte fr_type;
	byte fr_num;
	byte event_num;
	char data[FRAME_DATA_MAX_SIZE];
	Tx_Status tx_status;
	
	// Plane constructor
	Frame(){tx_status = nottransmitted; data[0]='\0';}
	// Parameterized constructor
	Frame(uint8_t _thread_id, byte _next_hop,	byte _from_id, byte _source_id, 
		byte _dest_id, byte _fr_type, byte _fr_num, byte _event_num, 
		const char *_data, Tx_Status _tx_status) {	//const char, will accept both const string and pointer.
		thread_id = _thread_id;
		next_hop = _next_hop;
		from_id = _from_id;
		source_id = _source_id;
		dest_id = _dest_id;
		fr_type = _fr_type;
		fr_num = _fr_num;
		event_num = _event_num;		
		strcpy(data, _data);
		tx_status = _tx_status;
	}
	//Copy constructor
	Frame(const Frame &f)	{	
		thread_id = f.thread_id;
		next_hop = f.next_hop;
		from_id = f.from_id;
		source_id = f.source_id;
		dest_id = f.dest_id;
		fr_type = f.fr_type;
		fr_num = f.fr_num;
		event_num = f.event_num;
		strcpy(data, f.data);
		tx_status = f.tx_status;
	}
	//Destructor
	~Frame() { 
		//delete data;
	}	
	//Copy assignment operator
	Frame& operator = (const Frame &f) {
		// Check for self assignment
		if(this != &f) {
			thread_id = f.thread_id;
			next_hop = f.next_hop;
			from_id = f.from_id;
			source_id = f.source_id;
			dest_id = f.dest_id;
			fr_type = f.fr_type;
			fr_num = f.fr_num;
			event_num = f.event_num;
			strcpy(data, f.data);
			tx_status = f.tx_status;
	   }
	   return *this;
	}
};



// Classes ==============================================================================================================
// Note: Here variables are passed by reference. Between threads might not be safe. Doubt..?
template<typename Data>
class concurrent_queue
{
private:
    std::queue<Data> the_queue;
    mutable boost::mutex the_mutex;
    boost::condition_variable the_condition_variable;
public:
    void push(Data const& data)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        the_queue.push(data);
        lock.unlock();
        the_condition_variable.notify_one();
    }

    bool empty() const
    {
        boost::mutex::scoped_lock lock(the_mutex);
        return the_queue.empty();
    }
	
	// Just pop() without waiting/blocking.
	// Alternative of empty(). Efficient implementation of If not empty() pop().
	// Useful when no waiting/blocking required.
    bool try_pop(Data& popped_value)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        if(the_queue.empty())
        {
            return false;
        }
        
        popped_value=the_queue.front();
        the_queue.pop();
        return true;
    }

    /*void wait_and_pop(Data& popped_value)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        while(the_queue.empty())
        {
            the_condition_variable.wait(lock);
        }
        
        popped_value=the_queue.front();
        the_queue.pop();
    }*/
    
    //-----------
    
    struct queue_not_empty
    {
        std::queue<Data>& queue;

        queue_not_empty(std::queue<Data>& queue_):
            queue(queue_)
        {}
        bool operator()() const
        {
            return !queue.empty();
        }
    };
        
    void wait_and_pop(Data& popped_value)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        the_condition_variable.wait(lock,queue_not_empty(the_queue));
        popped_value=the_queue.front();
        the_queue.pop();
    }
    
    template<typename Duration>
    bool timed_wait_and_pop(Data& popped_value,
                            Duration const& wait_duration)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        if(!the_condition_variable.timed_wait(lock,wait_duration,
            queue_not_empty(the_queue)))
            return false;
        popped_value=the_queue.front();
        the_queue.pop();
        return true;
    }

};


/*****************************************************
 * MeshNetwork
 ****************************************************/
class MeshNetwork: public LoRa
{
private:
	//My static, single thread variables -----------------------------------
	byte COORDINATOR_ID;
	byte NODE_ID;	// 1 to 254. 0x00 NULL avoid. reserved for end node indication.
	byte BROADCAST_ID;	//255
	
	//byte DEVICE_TYPE;	// COORDINATOR, REPEATER, ENDDEVICE. If required, add more types.
	DeviceType DEVICE_TYPE;
	
	bool ACK_ENABLE;
	bool ACK_SKIP;
	
	// Frame Types fr_type.
	byte DATA_FRAME;
	byte ACK_FRAME;
	byte BROADCAST_DATA_FRAME;
	byte BROADCAST_ACK_FRAME;
	byte VEHICLE_TASK_A_FRAME;
	byte VEHICLE_TASK_A_ACK_FRAME;
	byte VEHICLE_SIG_BROADCAST_FRAME;
	byte VEHICLE_SIG_BROADCAST_ACK_FRAME;
	byte VEHICLE_TASK_B_FRAME;
	byte VEHICLE_TASK_B_ACK_FRAME;
	
	uint8_t NO_OF_VEHICLE_SIG_BROADCAST;	// Broadcast frames no Qos gaurantee. So best is multiple transmissions
	
	//CSMA related
	int macMinBE;	//Test for different values of random func
	int unitBackoffPeriod;  //# msec. Integer only.  self.telit_t_frame
	int macMaxCSMABackoffs;
	int max_wait_for_frame_ACK;  //# msec.
	//int NB, BE;
		
	//Task Thread Ids.
	uint8_t VEHICLE_SIG_THREAD_ID;
	uint8_t REPEATER_THREAD_ID;
	uint8_t PARSING_THREAD_ID;
	//uint8_t COORDINATOR_THREAD_ID;
	//uint8_t ENDDEVICE_THREAD_ID;
	
	
	//My Dynamic variables--------------------------------------------------
	unsigned char *NetworkMap;
	int TOTAL_STATIC_NODES;
	
	byte frame_number;
	mutable boost::mutex fr_num_mutex;
	byte event_number;
	boost::mutex event_num_mutex;
	//mutable boost::mutex event_num_mutex;
	
	mutable boost::mutex SPI_mutex;
	
	// All class variabes are shared and modifiable (with mutex) in all threads.
	concurrent_queue<Frame> write_q;
	concurrent_queue<Frame> write_ack_q;
	concurrent_queue<Frame> get_ack_q;
	
	concurrent_queue<std::string> receive_q;
	
	//Task queues
	concurrent_queue<Frame> repeater_q;
	//concurrent_queue<Frame> coordinator_q;
	//concurrent_queue<Frame> enddevice_q;
	concurrent_queue<Frame> vehile_sig_q;
	
	//ACK queues corresponding to a task
	concurrent_queue<Frame> repeater_ack_q;
	//concurrent_queue<Frame> coordinator_ack_q;
	//concurrent_queue<Frame> enddevice_ack_q;
	concurrent_queue<Frame> vehicle_sig_ack_q;
	
	concurrent_queue<std::string> display_q;
	//bool start;
	//bool restart;
	//boost::mutex start_mutex;
	//boost::mutex restart_mutex;
		
	boost::thread write_thread;
	boost::thread write_ack_thread;
	boost::thread parsing_thread;
	boost::thread repeater_thread;
	boost::thread coordinator_thread;
	boost::thread enddevice_thread;
	boost::thread vehicle_sig_thread;
	boost::thread display_thread;
	
public:
	MeshNetwork(DeviceType _device_type, byte _node_id, int _total_static_nodes);
	~MeshNetwork();
	
	const char* getThreadName(uint8_t thread_id);
	void get_frame_str(Frame & fr, char * frame_str);
	void parse_frame(const char * fr_str, Frame & fr);
	byte getNextHop(byte _node_id, byte _dest_id);
	
	// Basic tasks
	void Write_task();
	void Write_ack_task();
	Tx_Status CSMA_CA(Frame& write_fr, char * fr_str, Frame& fr_ack, int _BE);
	Tx_Status CSMA_CA_2(Frame& write_fr, char * fr_str, int _BE);
	void Read_task();
	void Parsing_task();
	void Acknowledge(Frame fr1, Frame fr1_ack);
	
	// Coordinator related tasks
	void Coordinator_to_Vehicle_Signalling();
	void Receive_Sensor_Data();
	void Create_sample_event();
	
	// Repeater related tasks
	void Repeater_task();
	void Repeater_to_Vehicle_Signalling();
	
	// EndDevice related tasks
	void EndDevice_Vehicle_Signal();
	void Display();
		
	//void Coordinator_task(uint8_t thread_id);	
	//void EndDevice_task(uint8_t thread_id);
		
};

/**********************************************************************************************
 * Other Functions * 
 **********************************************************************************************/
int sleep_msec(long milliseconds);	// Enter integer values
int sleep_nsec(long nanoseconds);


#endif /* MESHNETWORK_H_ */
