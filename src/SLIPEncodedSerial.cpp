#include "SLIPEncodedSerial.h"

extern "C" {
#include "uart.h"
#include "BlinkLed.h"
}

extern uint8_t uart2_recv_buf[];
extern uint16_t uart2_recv_buf_head;
extern uint16_t uart2_recv_buf_tail;

/*
 CONSTRUCTOR
 */
SLIPEncodedSerial::SLIPEncodedSerial() {
	rstate = WAITING;
	rxPacketIndex = 0;
	encodedBufIndex = 0;
	decodedBufIndex = 0;
}

static const uint8_t eot = 0300;
static const uint8_t slipesc = 0333;
static const uint8_t slipescend = 0334;
static const uint8_t slipescesc = 0335;

int SLIPEncodedSerial::sendMessage(const uint8_t *buf, uint32_t len) {
	uint32_t i;
	encode(buf, len);
	for (i = 0; i < encodedLength; i++) {
		uart2_send(encodedBuf[i]);
	}
	return encodedLength;
}

int SLIPEncodedSerial::recvMessage(void) {
	// process rx buffer, this might return before the whole thing
	// is proccessed,  but we'll just get it next time

	while (uart2_recv_buf_tail != uart2_recv_buf_head) {
		uint8_t tmp8 = uart2_recv_buf[uart2_recv_buf_tail++];
		uart2_recv_buf_tail %= UART2_BUFFER_SIZE;

		if (rstate == WAITING) {
			if (tmp8 == eot)
				rstate = WAITING; // just keep waiting for something afer EOT
			else {
				rxPacketIndex = 0;
				rxPacket[rxPacketIndex++] = tmp8;
				rstate = RECEIVING;
			}
		} // waiting
		else if (rstate == RECEIVING) {
			if (rxPacketIndex >= MAX_MSG_SIZE) {
				rstate = WAITING;
				//AUX_LED_RED_ON;
			} else if (tmp8 == eot) {
				rstate = WAITING;
				decode(rxPacket, rxPacketIndex);
				return 1;
			} else {
				rxPacket[rxPacketIndex++] = tmp8;
				rstate = RECEIVING;
			}
		} //receiving

	} // gettin bytes
	return 0;
}

//encode SLIP, put it in the encoded buffer
void SLIPEncodedSerial::encode(uint8_t b) {
	if (b == eot) {
		encodedBuf[encodedBufIndex++] = slipesc;
		encodedBuf[encodedBufIndex++] = slipescend;
	} else if (b == slipesc) {
		encodedBuf[encodedBufIndex++] = slipesc;
		encodedBuf[encodedBufIndex++] = slipescesc;
	} else {
		encodedBuf[encodedBufIndex++] = b;
	}
}

void SLIPEncodedSerial::encode(const uint8_t *buf, int size) {
	beginPacket();
	while (size--)
		encode(*buf++);
	endPacket();
}

// decode SLIP, put it in the decoded buffer
void SLIPEncodedSerial::decode(const uint8_t *buf, int size) {
	int i;
	decodedBufIndex = 0;
	i = 0;

	while (i < size) {
		if (buf[i] == slipesc) { // TODO error out here if slipescend or slipescesc doesn't follow slipesc
			i++;
			if (buf[i] == slipescend)
				decodedBuf[decodedBufIndex++] = eot;
			if (buf[i] == slipescesc)
				decodedBuf[decodedBufIndex++] = slipesc;
			i++;
		} else {
			decodedBuf[decodedBufIndex++] = buf[i];
			i++;
		}
	}
	decodedLength = decodedBufIndex;
}

//SLIP specific method which begins a transmitted packet
void SLIPEncodedSerial::beginPacket() {
	encodedBufIndex = 0;
	encodedBuf[encodedBufIndex++] = eot;
}

//signify the end of the packet with an EOT
void SLIPEncodedSerial::endPacket() {
	encodedBuf[encodedBufIndex++] = eot;
	encodedLength = encodedBufIndex;
}

