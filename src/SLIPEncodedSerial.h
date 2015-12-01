#ifndef SLIPEncodedSerial_h
#define SLIPEncodedSerial_h

#include <stdint.h> 

#define MAX_MSG_SIZE 256 // the maximum un encoded size.  the max encoded size will be this * 2 for slip overhead
#define WAITING 1
#define RECEIVING 2

class SLIPEncodedSerial {

private:

public:

	SLIPEncodedSerial();

	uint8_t rstate;

	// encoded message
	uint8_t encodedBuf[MAX_MSG_SIZE * 2]; // the encoded can be up to 2 * longer
	uint32_t encodedBufIndex;
	uint32_t encodedLength;

	// decoded message
	uint8_t decodedBuf[MAX_MSG_SIZE];
	uint32_t decodedBufIndex;
	uint32_t decodedLength;

	uint8_t rxPacket[MAX_MSG_SIZE * 2];
	uint32_t rxPacketIndex;

	//SLIP specific method which begins a transmitted packet
	void beginPacket();

	//SLIP specific method which ends a transmittedpacket
	void endPacket();

	void encode(const uint8_t *buf, int size);
	void encode(uint8_t b);

	void decode(const uint8_t *buf, int size);

	int sendMessage(const uint8_t *buf, uint32_t len);

	int recvMessage(void);

};

#endif
