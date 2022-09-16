/* 
 *  Library for LoRa 868 / 915MHz SX1272, SX1276(RFM95W) LoRa module
 * 
 *  Extension/modification of SX1272 library of C.Pham/Libelium
 *  by Manohar
 *  17-Aug-2017
 */

#include "LoRa.h"

// Constructor
LoRa::LoRa() 	//==============================================
{
	// SX1272() constructor is automatically called
	//Allocate memory
	FIFO_rw_buffer = new char[BUFFER_SIZE];
}

// Destructor
LoRa::~LoRa() 	//==============================================
{
	// Deallocate memory
	delete FIFO_rw_buffer;
}

// Write nBYTES to FIFO in burst
void LoRa::writeFIFO(char address, uint8_t len, char * data)	//==================================================
{
	bitSet(address, 7);
    FIFO_rw_buffer[0]=address; FIFO_rw_buffer[1]='\0';
    strcat(FIFO_rw_buffer, data);
	digitalWrite(SX1272_SS,LOW);
    SPI.transfernb(FIFO_rw_buffer, FIFO_rw_buffer, len+1);
    digitalWrite(SX1272_SS,HIGH);
}

// Read n Bytes from FIFO in burst
void LoRa::readFIFO(char address, uint8_t len, char * data) 	//==================================================
{
	bitClear(address, 7);
    FIFO_rw_buffer[0]=address;
	digitalWrite(SX1272_SS,LOW);
    SPI.transfernb(FIFO_rw_buffer, FIFO_rw_buffer, len+1);
    digitalWrite(SX1272_SS,HIGH);
    //Ignore the first byte in rxBufer, as data comes from next byte of address
    for (int i=0; i<len; i++) data[i]=FIFO_rw_buffer[i+1];
    data[len]='\0';
}

//Interface to upper layer from LoRa PHY layer.
//Note: len is data size, not buffer size. len<=buffer_size
uint8_t LoRa::sendPacket(uint8_t len, char * payload) //=============================================
{
	uint8_t state = 2;
    byte value = 0x00;
    //unsigned long previous;
    unsigned long exitTime;

    if( _modem == LORA )
    {
		//To write to FIFO buffer for transmission, stand-by mode change required
		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Stdby LoRa mode to write in FIFO
		//clear Tx flag
		writeRegister(REG_IRQ_FLAGS, 0xFF);	// LoRa mode flags register. Status register. Clear Flags by writing 0xFF.
		//set Payload length
		writeRegister(REG_PAYLOAD_LENGTH_LORA, len);
		// Setting address pointer in FIFO data buffer, for SPI writing
		writeRegister(REG_FIFO_ADDR_PTR, 0x80);  
		// Write in FIFO, in burst
		writeFIFO(REG_FIFO, len, payload);
		// Set timeout
		exitTime = millis() + MAX_TIMEOUT;
		// LORA mode - Tx. LoRa will transmit data in FIFO
		writeRegister(REG_OP_MODE, LORA_TX_MODE);
		// Wait until the packet is sent (TX Done flag) or the timeout expires
		value = readRegister(REG_IRQ_FLAGS);
		while ((bitRead(value, 3) == 0) && (millis() < exitTime)) {
            value = readRegister(REG_IRQ_FLAGS);
        }
        state = 1;
	}
	if( bitRead(value, 3) == 1 )
    {
        state = 0;	// Packet successfully sent
#if (SX1272_debug_mode > 1)
        printf("## Packet successfully sent ##\n");
        printf("\n");
#endif
    }
    else
    {
        if( state == 1 )
        {
#if (SX1272_debug_mode > 1)
            printf("** Timeout has expired **\n");
            printf("\n");
#endif
        }
        else
        {
#if (SX1272_debug_mode > 1)
            printf("** There has been an error and packet has not been sent **\n");
            printf("\n");
#endif
        }
    }
    
    // Clear Flags
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    // Change to Receive mode back
    // After Tx mode, STANDBY mode is automatic
	// FIFO automatically cleared after mode change TX -> STANDBY or STANDBY -> RX. Ref P-62 Table-73
	// So no need to reset REG_FIFO_RX_CURRENT_ADDR pointer
	writeRegister(REG_OP_MODE, LORA_RX_MODE);  	  // LORA mode - RXCONTINUOUS
    //delay(10);
    return state;
}

// Run this in loop. Unlike sendPacket(), no timeout required here.
// For LoRa Mode only
state_r LoRa::receivePacket(char * read_data, bool getrssi) //=================================================================================
{
	state_r rs;
	rs.e = unexecuted;
	rs.rssi=0;
	byte flags=0x00, address=0x00;
	uint8_t len=0;
	//RSSI
	//int rssi=0;
	
	// Check RX flag
	flags = readRegister(REG_IRQ_FLAGS);
	// If raised, fetch, else return. 
	if(bitRead(flags, 6) == 1)		// RxDone Interrupt
	{
		if (getrssi){
			//get RSSI of received packet
			rs.rssi = readRegister(REG_PKT_RSSI_VALUE) - 137;
			//printf("Recv packet RSSI: %d dBm\n", rssi);
		}
		// Change mode to STANDBY. Required [P-71 Section 4.2.13.4]
		// Stdby LoRa mode to read from FIFO. Also to clear flags, once RxDone interrupt occurs, even if wrong packet.
		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	
		
		// Check other flags. CRC error interrupt, valid header interrupt,
		if((bitRead(flags, 5) == 0) && (bitRead(flags, 4) == 1)) {			
			// Get starting address of current recv packet
			address = readRegister(REG_FIFO_RX_CURRENT_ADDR);
			// Set FIFO pointer to current recv packet address
			writeRegister(REG_FIFO_ADDR_PTR, address);
			// read length of recv packet
			len = readRegister(REG_RX_NB_BYTES);	//char to int converted.
			// Burst read FIFO
			readFIFO(REG_FIFO, len, read_data);
			rs.e = success;
		}else {
			//printf("Error in received packet");
			rs.e = error;
		}
		// Clear Flags
		writeRegister(REG_IRQ_FLAGS, 0xFF);	// Status register. Clear Flags by writing 0xFF.
		// Change to Receive mode back
		// FIFO automatically cleared after mode change TX -> STANDBY or STANDBY -> RX. Ref P-62 Table-73
		// So no need to reset REG_FIFO_RX_CURRENT_ADDR pointer
		writeRegister(REG_OP_MODE, LORA_RX_MODE);  	  // LORA mode - RXCONTINUOUS		
	}
	return rs;
}


// Initialize receive mode. Only at beginning.
void LoRa::setDefaultReceiveMode()	//=================================================================================
{
	// Mode change to STAND-BY...?
	writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Stdby mode to write in registers
	// Set LowPnTxPllOff. modified by C. Pham from 0x09 to 0x08
    writeRegister(REG_PA_RAMP, 0x08);	
    // modified by C. Pham
    writeRegister(REG_LNA, LNA_MAX_GAIN);	// Important in reception. High frequency >860 MHz
    writeRegister(REG_FIFO_ADDR_PTR, 0x00);  // Setting address pointer in FIFO data buffer
    /*// modified by C. Pham
    if (_spreadingFactor == SF_10 || _spreadingFactor == SF_11 || _spreadingFactor == SF_12) {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x05); // Required for Single Rx mode, not for RXCONTINUOUS.
    } else {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x08);
    }
    //end*/
    writeRegister(REG_FIFO_RX_BYTE_ADDR, 0x00); // Setting current value of reception buffer pointer
    if( _modem == LORA )
    {
		writeRegister(REG_PAYLOAD_LENGTH_LORA, MAX_LENGTH);	//set Payload length
        writeRegister(REG_IRQ_FLAGS, 0xFF);	// LoRa mode flags register. Clear flags, by writing 0xFF
        writeRegister(REG_OP_MODE, LORA_RX_MODE);  	  // LORA mode - RXCONTINUOUS
#if (SX1272_debug_mode > 1)
        printf("## Receiving LoRa mode activated with success ##\n");
        printf("\n");
#endif
    }
    return;
}

//  Channel Activity Detection (CAD) or Listen before Talk (LBT)
// RSSI check is also done
// ch_status : channel_free, channel_busy, unexecuted (or error)
CADstatus LoRa::CAD(bool _print_rssi)
{
	//uint8_t channel_busy = 2;
	CADstatus ch_status = cad_unexecuted;
	byte value = 0x00;
	unsigned long wait=100;
	unsigned long exitTime;
	//RSSI
	int rssi=0, tmp=0;
	int rssi_threshold = RSSI_THRESHOLD;
	int rssi_samples = RSSI_SAMPLES-1;
	
	if( _modem == LORA ) { // LoRa mode
		for (int i=0; i<CAD_ITERATIONS; i++) {
			//mode change to standby
			writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
			//clear interrupt flags
			writeRegister(REG_IRQ_FLAGS, 0xFF);	// LoRa mode flags register. Clear flags, by writing 0xFF
			// Perform CAD. Change mode to CAD
			writeRegister(REG_OP_MODE, LORA_CAD_MODE);
			//Check CAD interrupt,
			exitTime = millis() + wait; 
			value = readRegister(REG_IRQ_FLAGS);
			while ((bitRead(value, 2) == 0) && (millis() < exitTime))
				value = readRegister(REG_IRQ_FLAGS);
			//check CadDetected interrupt.
			if((bitRead(value, 2) == 1) && (bitRead(value, 0) == 1)) {
				ch_status = channel_busy;// yes. LoRa signal detected
				break;
			}
		}
		// Clear Flags
		writeRegister(REG_IRQ_FLAGS, 0xFF);
		// Change to Receive mode back
		// After CAD mode, STANDBY mode is automatic
		writeRegister(REG_OP_MODE, LORA_RX_MODE);  	  // LORA mode - RXCONTINUOUS
		/*
		// get RSSI value
		rssi=0;
		rssi = readRegister(REG_RSSI_VALUE_LORA) - 137;
		for (int i=0; i<rssi_samples;i++) {
			tmp = readRegister(REG_RSSI_VALUE_LORA) - 137;
			if (rssi < tmp)
				rssi = tmp;
		}
		printf("---RSSI value: %d dBm\n", rssi);
		*/
		
		if ((bitRead(value, 2) == 1) && (bitRead(value, 0) == 0)) {
			// No LoRa signal Cad detected.
			// Check any non-LoRa signal using RSSI.
			// get RSSI value, only if LoRa signal not detected in CAD. No extra time waste.
			rssi=0;
			rssi = readRegister(REG_RSSI_VALUE_LORA) - 137;
			for (int i=0; i<rssi_samples;i++) {
				tmp = readRegister(REG_RSSI_VALUE_LORA) - 137;
				if (rssi < tmp)	//Max value
					rssi = tmp;
			}
			if(_print_rssi == true)
				printf("RSSI value: %d dBm\n", rssi);
			//
			if (rssi >= rssi_threshold)
				ch_status = channel_busy;
			else
				ch_status = channel_free; 
		}
	}
	return ch_status;
}

void LoRa::checkRSSI()
{
	int rssi=0;
	rssi = readRegister(REG_RSSI_VALUE_LORA) - 137;
	printf("RSSI value: %d dBm\n", rssi);
}

void LoRa::Initialize(const char * SF, const char * BW) 
{
	int e;
	
	// Set GPIO pins first
	int RFM95_RST_pin = 5;
	int RFM95_EN_pin = 4;
	pinMode(RFM95_EN_pin,OUTPUT);
    digitalWrite(RFM95_EN_pin,HIGH);
    pinMode(RFM95_RST_pin,OUTPUT);
    digitalWrite(RFM95_RST_pin,HIGH);
    delay(20);
    
    //RESET RFM95W manually
    digitalWrite(RFM95_RST_pin, LOW);
	delay(10);
	digitalWrite(RFM95_RST_pin, HIGH);
	delay(10);
	
	//Setup LoRa ====================================================================================
	// Print a start message
	printf("RFM95W/SX1276 module and Raspberry Pi\n");
	// Power ON the module
	e = ON();	
	printf("Setting power ON: state %d\n", e);
	// Set transmission mode
	//e |= lora.setMode(4);
	//printf("Setting Mode: state %d\n", e);
	
	setCR(CR_5);        // CR = 4/5
	
	if(strcmp(BW, "125")==0)		setBW(BW_125);      // BW = 125 KHz
	if(strcmp(BW, "250")==0)		setBW(BW_250);		// BW = 250 KHz
	else if(strcmp(BW, "500")==0)	setBW(BW_500);      // BW = 500 KHz
	else 							setBW(BW_125);      // BW = 125 KHz default
	
	//Set SF
	if (strcmp(SF, "12")==0)		setSF(SF_12);       // SF = 12
	else if (strcmp(SF, "11")==0)	setSF(SF_11);       // SF = 11
	else if (strcmp(SF, "10")==0)	setSF(SF_10);       // SF = 10
	else if (strcmp(SF, "9")==0)	setSF(SF_9);       // SF = 9
	else if (strcmp(SF, "8")==0)	setSF(SF_8);       // SF = 8
	else if (strcmp(SF, "7")==0)	setSF(SF_7);       // SF = 7
	else 							setSF(SF_12);       // SF = 12
		
	// Set header
	e |= setHeaderON();
	printf("Setting Header ON: state %d\n", e);
	// Select frequency channel
	e |= setChannel(CH_05_866);
	printf("Setting Channel: state %d\n", e);
	// Set CRC
	e |= setCRC_ON();
	printf("Setting CRC ON: state %d\n", e);
	// Select output power (Max, High or Low)
	e |= setPower('X');
	printf("Setting Power: state %d\n", e);
	//Set receive mode
	setDefaultReceiveMode();
	// Print a success message
	if (e == 0)
		printf("RFM95W/SX1276 successfully configured\n");
	else
		printf("RFM95W/SX1276 initialization failed\n");
	delay(1000);
}
