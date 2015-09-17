#ifndef SERIAL_H
#define SERIAL_H

#include <string>
#define RX_BUF_SIZE 256


/*

Serial IO

(c) 2015 Owen Osborn, Critter & Guitari

*/

class Serial
{
	public:
	/*	uint8_t rxBuf[RX_BUF_SIZE];  // circular buffer for incoming serial, should be 2 times serail read size
		uint32_t rxBufHead;
		uint32_t rxBufTail;*/


		// Default constructor takes serial port as argument
		Serial ();

		// destructor
		~Serial();

		// send stuff
		int writeBuffer(void *buffer, int len);

		// receive stuff
		int readBuffer(void *buffer, int bufferSize);

};


#endif
