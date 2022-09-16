/*
 * LoRa.h
 *
 * Created on: 08-Aug-2017
 *     Author: manohar
 * 
 * Extension/modifications to following library:
 * 
 * Using Libelium's arduPi (for RPI-2-3) C++ library for basic interfaces SPI, GPIO, etc. 
 * In RPI, this library supports Pin numbers and function names compatible with Arduino. Compatible to RPI - Arduino bridge board v2
 * 
 * For LoRa library/driver
 * Using C.Pham's SX1272/SX1276/RFM95W C++ library, which is extended version of Libelium's arduPiLoRa.h for SX1272.
 * Changes done by Manohar in SX1272.cpp. Changed SPI clock divider to 32 (8MHz). Changed arduPi library include part.
 * 
 * 
 * Note: This extended code is supported for Raspberry Pi 2 to 3 only. Using arduPi.h of RPI-2-3.
 * Note: This code is mainly for LoRa mode only, very less for FSK mode. Also for Spreading Factor SF>6 only.
 */

#ifndef LORA_H_
#define LORA_H_

// Includes =============================================================================================================
#include "SX1272.h"

// MACROS and Global variables ==========================================================================================
#define BUFFER_SIZE 128	//32, 64, 128, 256. 
#define MAX_LENGTH_2 127
#define MAX_PAYLOAD_2 123
// CAD
#define CAD_ITERATIONS 1
// RSSI
#define RSSI_THRESHOLD -70	// -70dBm
#define RSSI_SAMPLES 10

//FREQUENCY CHANNELS:
//added by Manohar, for Indian channels 865 to 867 MHz, 10 channels, each 200KHz max BW
// (Frequency_in_MHz * 2^14). Convert decimal to Hex, leave floating part of hex value.
const uint32_t CH_00_866 = 0xD84666; // channel 00, central freq = 865.10MHz
const uint32_t CH_01_866 = 0xD85333; // channel 01, central freq = 865.30MHz
const uint32_t CH_02_866 = 0xD86000; // channel 02, central freq = 865.50MHz
const uint32_t CH_03_866 = 0xD86CCC; // channel 03, central freq = 865.70MHz
const uint32_t CH_04_866 = 0xD87999; // channel 04, central freq = 865.90MHz
const uint32_t CH_05_866 = 0xD88666; // channel 05, central freq = 866.10MHz
const uint32_t CH_06_866 = 0xD89333; // channel 06, central freq = 866.30MHz
const uint32_t CH_07_866 = 0xD8A000; // channel 07, central freq = 866.50MHz
const uint32_t CH_08_866 = 0xD8ACCC; // channel 08, central freq = 866.70MHz
const uint32_t CH_09_866 = 0xD8B999; // channel 09, central freq = 866.90MHz

//Data types ============================================================
enum state_e {success, error, unexecuted};
struct state_r{
	state_e e;
	int rssi;
};
enum CADstatus {channel_free, channel_busy, cad_unexecuted};

// Classes ================================================================================================================
class LoRa: public SX1272
{
private:
	char * FIFO_rw_buffer;
	
public:
	LoRa();
	~LoRa();
	void Initialize(const char * SF, const char * BW) ;
	void setBuffer();
	void setDefaultReceiveMode();
	void writeFIFO(char address, uint8_t len, char * data);
	void readFIFO(char address, uint8_t len, char * data);
	uint8_t sendPacket(uint8_t len, char * payload);
	state_r receivePacket(char * read_data, bool printrssi);
	CADstatus CAD(bool _print_rssi);
	void checkRSSI();
	
};


#endif /* LORA_H_ */
