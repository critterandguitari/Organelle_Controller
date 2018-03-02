//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"

extern "C" {
#include "uart.h"
#include "stm32f0xx.h"
#include "Timer.h"
#include "BlinkLed.h"
#include "ssd1306.h"
}

#include "OSC/OSCMessage.h"
#include "SLIPEncodedSerial.h"
#include "OSC/SimpleWriter.h"

// ADC DMA stuff
#define ADC1_DR_Address    0x40012440
__IO uint16_t RegularConvData_Tab[9];

// keys and knobs
uint8_t keyValuesRaw[26];
uint8_t keyValues[4][26];
uint8_t keyValuesLast[26];
uint32_t knobValues[6];

// key mux
#define MUX_SEL_A_1 GPIO_SetBits(GPIOC, GPIO_Pin_6)
#define MUX_SEL_A_0 GPIO_ResetBits(GPIOC, GPIO_Pin_6)
#define MUX_SEL_B_1 GPIO_SetBits(GPIOC, GPIO_Pin_7)
#define MUX_SEL_B_0 GPIO_ResetBits(GPIOC, GPIO_Pin_7)
#define MUX_SEL_C_1 GPIO_SetBits(GPIOC, GPIO_Pin_8)
#define MUX_SEL_C_0 GPIO_ResetBits(GPIOC, GPIO_Pin_8)

// key mux states
#define MUX_0 MUX_SEL_A_0;MUX_SEL_B_0;MUX_SEL_C_0;
#define MUX_1 MUX_SEL_A_1;MUX_SEL_B_0;MUX_SEL_C_0;
#define MUX_2 MUX_SEL_A_0;MUX_SEL_B_1;MUX_SEL_C_0;
#define MUX_3 MUX_SEL_A_1;MUX_SEL_B_1;MUX_SEL_C_0;
#define MUX_4 MUX_SEL_A_0;MUX_SEL_B_0;MUX_SEL_C_1;
#define MUX_5 MUX_SEL_A_1;MUX_SEL_B_0;MUX_SEL_C_1;
#define MUX_6 MUX_SEL_A_0;MUX_SEL_B_1;MUX_SEL_C_1;
#define MUX_7 MUX_SEL_A_1;MUX_SEL_B_1;MUX_SEL_C_1;

// OSC stuff
SLIPEncodedSerial slip;
SimpleWriter oscBuf;

// for outputting to screen
uint8_t spi_out_buf[130];
uint8_t spi_out_buf_remaining = 0;
uint8_t spi_out_buf_index = 0;

// OLED frame
extern uint8_t pix_buf[];

//// hardware init
static void ADC_Config(void);
static void DMA_Config(void);
void hardwareInit(void);


// OSC callbacks
void oledControl(OSCMessage &msg);
void ledControl(OSCMessage &msg);
void getKnobs(OSCMessage &msg);
void shutdown(OSCMessage &msg);
// end OSC callbacks

/// scan keys
void keyMuxSel(uint32_t sel);
uint32_t scanKeys();
void remapKeys();
void checkForKeyEvent();
void updateKnobs() ;
void checkEncoder(void) ;

//foot
void checkFootSwitch (void) ;

int main(int argc, char* argv[]) {

	OSCMessage msgIn;

	blink_led_init();
	blink_led_off();


	timer_start();

	// flash leds while power stabilizes
	// before initializing ADC
	stopwatchStart();
	while (stopwatchReport() < 500){ AUX_LED_RED_ON; }
	stopwatchStart();
	while (stopwatchReport() < 500){ AUX_LED_GREEN_ON; }
	stopwatchStart();
	while (stopwatchReport() < 500){ AUX_LED_BLUE_ON; }
	stopwatchStart();

	AUX_LED_RED_OFF;
	AUX_LED_GREEN_OFF;
	AUX_LED_BLUE_OFF;

	uart2_init();

	hardwareInit();

	// Infinite loop

	// oled init
	ssd1306_init(0);

	println_8_spacy("   ORGANELLE", 12, 6, 4);

	//println_8("for more patches visit", 22, 0, 22);
	println_8("www.organelle.io", 16, 8, 28);
//	println_8("for patches", 11, 8, 42);

	char progressStr[20];
	int len = 0;
	int progress = 0;
	len = sprintf(progressStr, "starting: %d %%", progress);
	println_8(progressStr, len, 8, 52);
	ssd1306_refresh();

	stopwatchStart();

	while (1) {
		if (slip.recvMessage()) {
			// fill the message and dispatch it

			msgIn.fill(slip.decodedBuf, slip.decodedLength);

			// dispatch it
			if (!msgIn.hasError()) {
				// wait for start message so we aren't sending stuff during boot
				if (msgIn.fullMatch("/ready", 0)) {
					msgIn.empty(); // free space occupied by message
					break;
				}
				msgIn.empty();
			} else {   // just empty it if there was an error
				msgIn.empty(); // free space occupied by message
			}
		}
		if (stopwatchReport() > 1500) {
			stopwatchStart();
			len = sprintf(progressStr, "starting: %d %%", progress);
			if (progress < 99)
				progress++;
			println_8(progressStr, len, 8, 52);
			ssd1306_refresh();
		}

	} // waiting for /ready command

	stopwatchStart();   // used to check encoder only 1 per 5 ms

	while (1) {
		if (slip.recvMessage()) {
			// fill the message and dispatch it

			msgIn.fill(slip.decodedBuf, slip.decodedLength);

			// dispatch it
			if (!msgIn.hasError()) {
				msgIn.dispatch("/led", ledControl, 0);
				msgIn.dispatch("/oled", oledControl, 0);
				msgIn.dispatch("/getknobs", getKnobs, 0);
				msgIn.dispatch("/shutdown", shutdown, 0);
				msgIn.empty(); // free space occupied by message
			} else {   // just empty it if there was an error
				msgIn.empty(); // free space occupied by message
			}
		}

		// every time mux gets back to 0 key scan is complete
		if (scanKeys() == 0) {
			checkForKeyEvent(); // and send em out if we got em
			// also check about the foot switch
			checkFootSwitch();
		}
		updateKnobs();

		// check encoder
		// only check every 2 ms cause the button needs a lot of debounce time
		if (stopwatchReport() > 5){
			stopwatchStart();
			checkEncoder();
		}

	} // Infinite loop, never return.
}

void hardwareInit(void){


	/* DMA configuration */
	DMA_Config();

	/* ADC1 configuration */
	ADC_Config();

	// MUX SEL Lines
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOC, GPIO_Pin_6); //
	GPIO_SetBits(GPIOC, GPIO_Pin_7); //
	GPIO_SetBits(GPIOC, GPIO_Pin_8); //
	// end mux select lines

	// Enc lines
	// config as input
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// end enc lines

	// key lines
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2
			| GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// foot switch
	/*RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);*/

}

//// ADC DMA stuff
static void ADC_Config(void) {
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	/* ADC1 DeInit */
	ADC_DeInit(ADC1);

	/* GPIOC Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	/* ADC1 Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* Configure  as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Initialize ADC structure */
	ADC_StructInit(&ADC_InitStructure);

	/* Configure the ADC1 in continuous mode withe a resolution equal to 12 bits  */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_10b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* Convert the ADC1 Channel11 and channel10 with 55.5 Cycles as sampling time */
	ADC_ChannelConfig(ADC1, ADC_Channel_14, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_15, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_4, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_8, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_9, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_1, ADC_SampleTime_55_5Cycles);

	/* ADC Calibration */
	ADC_GetCalibrationFactor(ADC1);

	/* ADC DMA request in circular mode */
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);

	/* Enable ADC_DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable the ADC peripheral */
	ADC_Cmd(ADC1, ENABLE);

	/* Wait the ADRDY flag */
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY))
		;

	/* ADC1 regular Software Start Conv */
	ADC_StartOfConversion(ADC1);
}

/**
 * @brief  DMA channel1 configuration
 * @param  None
 * @retval None
 */
static void DMA_Config(void) {
	DMA_InitTypeDef DMA_InitStructure;
	/* DMA1 clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* DMA1 Channel1 Config */
	DMA_DeInit(DMA1_Channel1);
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) RegularConvData_Tab;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 6;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* DMA1 Channel1 enable */
	DMA_Cmd(DMA1_Channel1, ENABLE);

}

//// end ADC DMA

// OSC callbacks
void oledControl(OSCMessage &msg) {

	uint8_t tmp[132];
	uint8_t i;
	uint8_t line = 0;

	if (msg.isInt(0)) {
		line = msg.getInt(0) & 0x7;
	}
	if (msg.isBlob(1)) {
		msg.getBlob(1, tmp, 132);
	}

	// shift array 4 spaces cause first 4 bytes are the length of blob
	for (i = 0; i < 128; i++)
		pix_buf[i + (line * 128)] = tmp[i + 4];

	ssd1306_refresh_line(line);
}

void ledControl(OSCMessage &msg) {

	blink_led_on();

	int stat;

	// digitalWrite(ledPin, LOW);
	if (msg.isInt(0)) {
		stat = msg.getInt(0);

		stat %= 8;

		if (stat == 0) {
			AUX_LED_RED_OFF;
			AUX_LED_GREEN_OFF;
			AUX_LED_BLUE_OFF;
		}
		if (stat == 1) {
			AUX_LED_RED_OFF;
			AUX_LED_GREEN_OFF;
			AUX_LED_BLUE_ON;
		}
		if (stat == 2) {
			AUX_LED_RED_OFF;
			AUX_LED_GREEN_ON;
			AUX_LED_BLUE_OFF;
		}
		if (stat == 3) {
			AUX_LED_RED_OFF;
			AUX_LED_GREEN_ON;
			AUX_LED_BLUE_ON;
		}
		if (stat == 4) {
			AUX_LED_RED_ON;
			AUX_LED_GREEN_OFF;
			AUX_LED_BLUE_OFF;
		}
		if (stat == 5) {
			AUX_LED_RED_ON;
			AUX_LED_GREEN_OFF;
			AUX_LED_BLUE_ON;
		}
		if (stat == 6) {
			AUX_LED_RED_ON;
			AUX_LED_GREEN_ON;
			AUX_LED_BLUE_OFF;
		}
		if (stat == 7) {
			AUX_LED_RED_ON;
			AUX_LED_GREEN_ON;
			AUX_LED_BLUE_ON;
		}
	}
}

void getKnobs(OSCMessage &msg) {

	OSCMessage msgKnobs("/knobs");

	uint32_t i;
	for (i = 0; i < 6; i++) {
		msgKnobs.add((int32_t) knobValues[i]);
	}

	msgKnobs.send(oscBuf);
	slip.sendMessage(oscBuf.buffer, oscBuf.length);
	msgKnobs.empty();
}

void shutdown(OSCMessage &msg) {

	int i;
	char progressStr[20];
	int len = 0;
	int progress = 0;

	// clear screen
	for (i = 0; i < 1024; i++) {
		pix_buf[i] = 0;
	}

	/*
	 len = sprintf(progressStr, "starting: %d %%", progress);
	 println_8(progressStr, len, 8, 52);
	 ssd1306_refresh();
	 */

	stopwatchStart();
	while (progress < 99) {
		if (stopwatchReport() > 500) {
			stopwatchStart();
			len = sprintf(progressStr, "Shutting down: %d %%", progress++);
			println_8(progressStr, len, 1, 21);
			ssd1306_refresh();
		}
	}
	// clear screen
	for (i = 0; i < 1024; i++) {
		pix_buf[i] = 0;
	}
	len = sprintf(progressStr, "Shutdown complete.");
	println_8(progressStr, len, 1, 21);
	len = sprintf(progressStr, "Safe to remove power.");
	println_8(progressStr, len, 1, 43);
	ssd1306_refresh();

	for (;;)
		;  // endless loop here
}

// end OSC callbacks

/// scan keys
void keyMuxSel(uint32_t sel) {
	if (sel == 0) {
		MUX_0
		;
	} else if (sel == 1) {
		MUX_1
		;
	} else if (sel == 2) {
		MUX_2
		;
	} else if (sel == 3) {
		MUX_3
		;
	} else if (sel == 4) {
		MUX_4
		;
	} else if (sel == 5) {
		MUX_5
		;
	} else if (sel == 6) {
		MUX_6
		;
	} else if (sel == 7) {
		MUX_7
		;
	} else {
		MUX_0
		;
	}
}

uint32_t scanKeys() {
	static uint32_t seqCount = 0;
	static uint32_t muxSelCount = 0;

	if (seqCount == 0) {
		keyMuxSel(muxSelCount);  // select the mux
		muxSelCount++;        	// and increment for next time through
		muxSelCount %= 8;
	}
	if (seqCount == 1) {
		// do nothing, wait for next conversion sequence since the muxes were just changed
	}
	if (seqCount == 2) {
		keyValuesRaw[0] = (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)) ? 0 : 100; // the aux key
		keyValuesRaw[1 + muxSelCount] =
				(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2)) ? 0 : 100; //RegularConvData_Tab[3];
		keyValuesRaw[9 + muxSelCount] =
				(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1)) ? 0 : 100; //RegularConvData_Tab[4];
		keyValuesRaw[17 + muxSelCount] =
				(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0)) ? 0 : 100; //RegularConvData_Tab[5];
		keyValuesRaw[25] = 0;//(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)) ? 0 : 100; // the foot switch
	}
	seqCount++;
	seqCount %= 3;
	return muxSelCount;
}

void remapKeys() {
	static uint32_t cycleCount = 0;

	keyValues[cycleCount][0] = keyValuesRaw[0];
	keyValues[cycleCount][25] = keyValuesRaw[25];

	keyValues[cycleCount][1] = keyValuesRaw[23];
	keyValues[cycleCount][2] = keyValuesRaw[21];
	keyValues[cycleCount][3] = keyValuesRaw[17];
	keyValues[cycleCount][4] = keyValuesRaw[18];
	keyValues[cycleCount][5] = keyValuesRaw[24];
	keyValues[cycleCount][6] = keyValuesRaw[22];
	keyValues[cycleCount][7] = keyValuesRaw[19];
	keyValues[cycleCount][8] = keyValuesRaw[20];

	keyValues[cycleCount][9] = keyValuesRaw[13];
	keyValues[cycleCount][10] = keyValuesRaw[15];
	keyValues[cycleCount][11] = keyValuesRaw[10];
	keyValues[cycleCount][12] = keyValuesRaw[9];
	keyValues[cycleCount][13] = keyValuesRaw[16];
	keyValues[cycleCount][14] = keyValuesRaw[11];
	keyValues[cycleCount][15] = keyValuesRaw[14];
	keyValues[cycleCount][16] = keyValuesRaw[12];

	keyValues[cycleCount][17] = keyValuesRaw[5];
	keyValues[cycleCount][18] = keyValuesRaw[7];
	keyValues[cycleCount][19] = keyValuesRaw[2];
	keyValues[cycleCount][20] = keyValuesRaw[1];
	keyValues[cycleCount][21] = keyValuesRaw[4];
	keyValues[cycleCount][22] = keyValuesRaw[8];
	keyValues[cycleCount][23] = keyValuesRaw[3];
	keyValues[cycleCount][24] = keyValuesRaw[6];

	cycleCount++;
	cycleCount &= 0x3;  // i between 0-3
}



void checkForKeyEvent() {
	remapKeys();
	uint32_t i;

	for (i = 0; i < 26; i++) {
		if ((keyValues[0][i]) && (keyValues[1][i]) && (keyValues[2][i])
				&& (keyValues[3][i])) {

			if (!keyValuesLast[i]) {
				OSCMessage msgKey("/key");

				msgKey.add((int32_t) i);
				msgKey.add((int32_t) 100);

				msgKey.send(oscBuf);
				slip.sendMessage(oscBuf.buffer, oscBuf.length);

				msgKey.empty(); // free space occupied by message
				keyValuesLast[i] = 100;
			}
		}
		if ((!keyValues[0][i]) && (!keyValues[1][i]) && (!keyValues[2][i])
				&& (!keyValues[3][i])) {
			if (keyValuesLast[i]) {
				OSCMessage msgKey("/key");

				msgKey.add((int32_t) i);
				msgKey.add((int32_t) 0);

				msgKey.send(oscBuf);
				slip.sendMessage(oscBuf.buffer, oscBuf.length);

				msgKey.empty();
				keyValuesLast[i] = 0;
			}
		}
	}
}

/// end keys

void updateKnobs() {

	// see if a new conversion is ready
	if ((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == SET) {
		DMA_ClearFlag(DMA1_FLAG_TC1);

		knobValues[4] = RegularConvData_Tab[0];
		knobValues[0] = RegularConvData_Tab[1];
		knobValues[1] = RegularConvData_Tab[2];
		knobValues[3] = RegularConvData_Tab[3];
		knobValues[2] = RegularConvData_Tab[4];
		knobValues[5] = RegularConvData_Tab[5];
	}
}

// check for foot switch change
void checkFootSwitch (void) {
	static uint8_t foot_last = 0;
	static uint8_t foot_debounce[2] = {0, 0};
	static uint8_t foot_debounce_count = 0;

	if (knobValues[5] < 100) foot_debounce[foot_debounce_count] = 0;
	if (knobValues[5] > 900) foot_debounce[foot_debounce_count] = 1;

	foot_debounce_count++;
	foot_debounce_count &= 1;
	//only proceed if debounced (same for 2 times)
	if (foot_debounce[0] == foot_debounce[1]){
		if ((knobValues[5] < 100) && foot_last){
			foot_last = 0;
			// send press
			OSCMessage msgEncoder("/fs");
			msgEncoder.add(1);
			msgEncoder.send(oscBuf);
			slip.sendMessage(oscBuf.buffer, oscBuf.length);
		}
		if ((knobValues[5] > 900) && !foot_last){
			foot_last = 1;
			// send press
			OSCMessage msgEncoder("/fs");
			msgEncoder.add(0);
			msgEncoder.send(oscBuf);
			slip.sendMessage(oscBuf.buffer, oscBuf.length);
		}
	}
}

void checkEncoder(void) {

	static uint8_t encoder_last = 0;
	uint8_t encoder = 0;


	// because the encoder has a crappy switch, we need  a
	// different debouce time for press and release
	// assume checkEncoder gets called every 5ms,
	// we will wait 10 counts for press and 50 counts for release
	#define PRESS 0
	#define RELEASE 1
	uint8_t button;
	static uint8_t button_last = RELEASE;
	static uint8_t press_count = 0;
	static uint8_t release_count = 0;

	button = (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13));

	if (button == PRESS) {
		press_count++;
		release_count = 0;
	}
	if ((press_count > 10) && (button_last == RELEASE)){	// press
			button_last = PRESS;
			release_count = 0;

			// send press
			OSCMessage msgEncoder("/encbut");
			msgEncoder.add(1);
			msgEncoder.send(oscBuf);
			slip.sendMessage(oscBuf.buffer, oscBuf.length);
	}

	if (button == RELEASE) {
		release_count++;
		press_count = 0;
	}
	if ((release_count > 50) && (button_last == PRESS)){	// release
			button_last = RELEASE;
			press_count = 0;

			// send release
			OSCMessage msgEncoder("/encbut");
			msgEncoder.add(0);
			msgEncoder.send(oscBuf);
			slip.sendMessage(oscBuf.buffer, oscBuf.length);
	}




	// turning
	encoder = 0;
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14))
		encoder |= 0x1;
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))
		encoder |= 0x2;

	if (encoder != encoder_last) {

		if (encoder_last == 0) {
			OSCMessage msgEncoder("/enc");
			if (encoder == 2)
				msgEncoder.add(0);
			if (encoder == 1)
				msgEncoder.add(1);
			msgEncoder.send(oscBuf);
			slip.sendMessage(oscBuf.buffer, oscBuf.length);
		}
		if (encoder_last == 3) {
			OSCMessage msgEncoder("/enc");
			if (encoder == 1)
				msgEncoder.add(0);
			if (encoder == 2)
				msgEncoder.add(1);
			msgEncoder.send(oscBuf);
			slip.sendMessage(oscBuf.buffer, oscBuf.length);
		}
		encoder_last = encoder;

		//msgEncoder.setAddress("/enc");
	}
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
