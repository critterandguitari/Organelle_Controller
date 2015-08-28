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
#include "OSC/SLIPEncodedSerial.h"

// ----------------------------------------------------------------------------
//
// STM32F0 led blink sample (trace via $(trace)).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// To demonstrate POSIX retargetting, reroute the STDOUT and STDERR to the
// trace device and display messages on both of them.
//
// Then demonstrates how to blink a led with 1Hz, using a
// continuous loop and SysTick delays.
//
// On DEBUG, the uptime in seconds is also displayed on the trace device.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f0xx.c
//

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 2 / 3)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

// ADC DMA stuff
#define ADC1_DR_Address    0x40012440
__IO uint16_t RegularConvData_Tab[9];
uint16_t keyValuesRaw[25];
uint16_t keyValues[25];
uint16_t keyValuesLast[25];
uint32_t knobValues[5];

// key mux
#define MUX_SEL_A_1 GPIO_SetBits(GPIOC, GPIO_Pin_6)
#define MUX_SEL_A_0 GPIO_ResetBits(GPIOC, GPIO_Pin_6)
#define MUX_SEL_B_1 GPIO_SetBits(GPIOC, GPIO_Pin_7)
#define MUX_SEL_B_0 GPIO_ResetBits(GPIOC, GPIO_Pin_7)
#define MUX_SEL_C_1 GPIO_SetBits(GPIOC, GPIO_Pin_8)
#define MUX_SEL_C_0 GPIO_ResetBits(GPIOC, GPIO_Pin_8)

#define MUX_0 MUX_SEL_A_0;MUX_SEL_B_0;MUX_SEL_C_0;
#define MUX_1 MUX_SEL_A_1;MUX_SEL_B_0;MUX_SEL_C_0;
#define MUX_2 MUX_SEL_A_0;MUX_SEL_B_1;MUX_SEL_C_0;
#define MUX_3 MUX_SEL_A_1;MUX_SEL_B_1;MUX_SEL_C_0;
#define MUX_4 MUX_SEL_A_0;MUX_SEL_B_0;MUX_SEL_C_1;
#define MUX_5 MUX_SEL_A_1;MUX_SEL_B_0;MUX_SEL_C_1;
#define MUX_6 MUX_SEL_A_0;MUX_SEL_B_1;MUX_SEL_C_1;
#define MUX_7 MUX_SEL_A_1;MUX_SEL_B_1;MUX_SEL_C_1;

SLIPEncodedSerial SLIPSerial;

// reset to default turn on state
void reset(OSCMessage &msg){
    blink_led_off();
}

void renumber(OSCMessage &msg){
  // send out a
  /*OSCMessage  msgOut("asdf");
  SLIPSerial.beginPacket();
  msgOut.send(SLIPSerial);
  SLIPSerial.endPacket();
  msgOut.empty();*/
  blink_led_on();
}


extern uint8_t pix_buf[];

void oledControl(OSCMessage &msg){

	uint8_t tmp[132];
	uint8_t i;
	uint8_t line = 0;

	if (msg.isInt(0)){
		line = msg.getInt(0) & 0x7;
	}
	if (msg.isBlob(1)) {
	  msg.getBlob(1, tmp, 132);
	//  SLIPSerial.beginPacket();
	//  msg.send(SLIPSerial);
	//  SLIPSerial.endPacket();
	}

	// shift array 4 spaces cause first 4 bytes are the length of blob
	for (i = 0; i<128; i++)
	  pix_buf[i + (line * 128)] = tmp[i + 4];

	ssd1306_refresh_line(line);
}

void ledControl(OSCMessage &msg) {

	  blink_led_on();


	  int stat;

	  // digitalWrite(ledPin, LOW);
	  if (msg.isInt(0)) {
	    stat = msg.getInt(0);

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


//// ADC DMA stuff
/**
  * @brief  ADC1 channel configuration
  * @param  None
  * @retval None
  */
static void ADC_Config(void)
{
  ADC_InitTypeDef     ADC_InitStructure;
  GPIO_InitTypeDef    GPIO_InitStructure;
  /* ADC1 DeInit */
  ADC_DeInit(ADC1);

  /* GPIOC Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

   /* ADC1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  /* Configure  as analog input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure as analog input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure as analog input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
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
  ADC_ChannelConfig(ADC1, ADC_Channel_10 , ADC_SampleTime_55_5Cycles);
  ADC_ChannelConfig(ADC1, ADC_Channel_11 , ADC_SampleTime_55_5Cycles);
  ADC_ChannelConfig(ADC1, ADC_Channel_12 , ADC_SampleTime_55_5Cycles);
  ADC_ChannelConfig(ADC1, ADC_Channel_13 , ADC_SampleTime_55_5Cycles);

  ADC_ChannelConfig(ADC1, ADC_Channel_14 , ADC_SampleTime_55_5Cycles );
  ADC_ChannelConfig(ADC1, ADC_Channel_15 , ADC_SampleTime_55_5Cycles );
  ADC_ChannelConfig(ADC1, ADC_Channel_4 , ADC_SampleTime_55_5Cycles );
  ADC_ChannelConfig(ADC1, ADC_Channel_8 , ADC_SampleTime_55_5Cycles );
  ADC_ChannelConfig(ADC1, ADC_Channel_9 , ADC_SampleTime_55_5Cycles );


  /* ADC Calibration */
  ADC_GetCalibrationFactor(ADC1);

  /* ADC DMA request in circular mode */
  ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);

  /* Enable ADC_DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* Enable the ADC peripheral */
  ADC_Cmd(ADC1, ENABLE);

  /* Wait the ADRDY flag */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));

  /* ADC1 regular Software Start Conv */
  ADC_StartOfConversion(ADC1);
}

/**
  * @brief  DMA channel1 configuration
  * @param  None
  * @retval None
  */
static void DMA_Config(void)
{
	DMA_InitTypeDef   DMA_InitStructure;
	/* DMA1 clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);

	/* DMA1 Channel1 Config */
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 9;
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

  //Enable transfer complete interrupt for DMA1 channel 1
  /*	DMA_ClearITPendingBit(DMA1_IT_TC1);
  	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

  	NVIC_InitTypeDef NVIC_InitStructure;
  	//Enable DMA1 channel IRQ Channel
  	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  //	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  //	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);*/

}

/*
void DMA1_Channel1_IRQHandler(void){
	if(DMA_GetITStatus(DMA1_IT_TC1)){
	     keyValues[0] = ((int32_t)RegularConvData_Tab[0]);  // only for the first one
	     keyValues[1] = ((int32_t)RegularConvData_Tab[1]);
	     keyValues[2] = ((int32_t)RegularConvData_Tab[2]);
	     keyValues[3] = ((int32_t)RegularConvData_Tab[3]);

	       DMA_ClearITPendingBit(DMA1_IT_TC1);
	     //Clear DMA1 Channel1 Half Transfer, Transfer Complete and Global interrupt pending bits
	     DMA_ClearITPendingBit(DMA1_IT_GL1);
	}
}
*/

//// end ADC DMA


/// ad hoc scan keys

void keyMuxSel(uint32_t sel){
	if (sel == 0) {MUX_0;}
	else if (sel == 1) {MUX_1;}
	else if (sel == 2) {MUX_2;}
	else if (sel == 3) {MUX_3;}
	else if (sel == 4) {MUX_4;}
	else if (sel == 5) {MUX_5;}
	else if (sel == 6) {MUX_6;}
	else if (sel == 7) {MUX_7;}
	else {MUX_0;}
}


// this could / should be DMA interrupt
uint32_t scanKeys(){
	static uint32_t seqCount = 0;
	static uint32_t muxSelCount = 0;

	// see if a new conversion is ready
	if ((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == SET ){
		DMA_ClearFlag(DMA1_FLAG_TC1);
		if (seqCount == 0){
			keyMuxSel(muxSelCount);  // select the mux
			muxSelCount++;        	// and increment for next time through
			muxSelCount %= 8;
		}
		if (seqCount == 1){
			// do nothing, wait for next conversion sequence since the muxes were just changed
		}
		if (seqCount == 2){
			keyValuesRaw[0] = RegularConvData_Tab[2]; // the aux key, cause we scan in reverse for some reason
			keyValuesRaw[1 + muxSelCount] = RegularConvData_Tab[3];
			keyValuesRaw[9 + muxSelCount] = RegularConvData_Tab[4];
			keyValuesRaw[17 + muxSelCount] = RegularConvData_Tab[5];

			knobValues[4] = RegularConvData_Tab[0];
			knobValues[0] = RegularConvData_Tab[1];
			knobValues[1] = RegularConvData_Tab[6];
			knobValues[3] = RegularConvData_Tab[7];
			knobValues[2] = RegularConvData_Tab[8];
		}
		seqCount++;
		seqCount %= 3;
	}
	return muxSelCount;
}

void getKnobs(OSCMessage &msg){

	OSCMessage msgKnobs("/knobs");

	stopwatchStart();

	uint32_t i;
	for (i = 0; i < 5; i++){
		msgKnobs.add((int32_t)knobValues[i]);
	}

	SLIPSerial.beginPacket();
	msgKnobs.send(SLIPSerial); // send the bytes to the SLIP stream
	SLIPSerial.endPacket(); // mark the end of the OSC Packet
	msgKnobs.empty(); // free space occupied by message

}

void remapKeys(void){
	keyValues[0] = keyValuesRaw[0];

	keyValues[1] = keyValuesRaw[23];
	keyValues[2] = keyValuesRaw[21];
	keyValues[3] = keyValuesRaw[17];
	keyValues[4] = keyValuesRaw[18];
	keyValues[5] = keyValuesRaw[24];
	keyValues[6] = keyValuesRaw[22];
	keyValues[7] = keyValuesRaw[19];
	keyValues[8] = keyValuesRaw[20];

	keyValues[9] = keyValuesRaw[13];
	keyValues[10] = keyValuesRaw[15];
	keyValues[11] = keyValuesRaw[10];
	keyValues[12] = keyValuesRaw[9];
	keyValues[13] = keyValuesRaw[16];
	keyValues[14] = keyValuesRaw[11];
	keyValues[15] = keyValuesRaw[14];
	keyValues[16] = keyValuesRaw[12];

	keyValues[17] = keyValuesRaw[5];
	keyValues[18] = keyValuesRaw[7];
	keyValues[19] = keyValuesRaw[2];
	keyValues[20] = keyValuesRaw[1];
	keyValues[21] = keyValuesRaw[4];
	keyValues[22] = keyValuesRaw[8];
	keyValues[23] = keyValuesRaw[3];
	keyValues[24] = keyValuesRaw[6];
}

void sendKeyEvent(uint32_t note, uint32_t vel){

	OSCMessage msgKey("/key");

    msgKey.add((int32_t)note);
    msgKey.add((int32_t)vel);

	 SLIPSerial.beginPacket();
	 msgKey.send(SLIPSerial); // send the bytes to the SLIP stream
	 SLIPSerial.endPacket(); // mark the end of the OSC Packet
	 msgKey.empty(); // free space occupied by message
}

void checkKeyEvent(void){
	uint32_t i;

	for (i=0; i<25; i++){
		if ((keyValues[i] > 10) && (keyValuesLast[i] < 10))
			sendKeyEvent(i, keyValues[i]);
		if ((keyValuesLast[i] > 10) && (keyValues[i] < 10))
			sendKeyEvent(i, 0);
		keyValuesLast[i] = keyValues[i];
	}

}

void getKeys(/*OSCMessage &msg*/){



	remapKeys();
	checkKeyEvent();

	/*
	OSCMessage msgKey("/key");
    uint32_t i;
   for (i = 0; i < 25; i++){
    	msgKey.add((int32_t)keyValues[i]);
    }

	 SLIPSerial.beginPacket();
	 msgKey.send(SLIPSerial); // send the bytes to the SLIP stream
	 SLIPSerial.endPacket(); // mark the end of the OSC Packet
	 msgKey.empty(); // free space occupied by message
	 */
}


int main(int argc, char* argv[]) {

	/* ADC1 configuration */
	ADC_Config();

	/* DMA configuration */
	DMA_Config();

	// MUX SEL Lines
	GPIO_InitTypeDef GPIO_InitStructure;
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


	// timer_start();

	// blink_led_init();

	OSCMessage msgIn;

	int size;

	uart2_init();

	blink_led_init();

	blink_led_on();
	blink_led_off();

	timer_start();
	// Infinite loop

	int msgInSize = 0;

	// oled init
	ssd1306_init(0);
	put_char_small('I', 0, 0);
	put_char_small('O', 48, 0);
	ssd1306_refresh();

	uint32_t numTimesScanned = 0;
	uint32_t muxNum = 0;
	uint32_t muxNumLast = 0;

	while (1)
	{
			  // use msgInSize hack cause endofPacket can return true at beginning and end of a packet
			while((!SLIPSerial.endofPacket()) || (msgInSize < 4) ) {

			  if( (size =SLIPSerial.available()) > 0) {

				while(size--) {
				  msgIn.fill(SLIPSerial.read());
				  msgInSize++;
				}
			  }
			  muxNum = scanKeys();   // scan keys while we are waiting

			  if (muxNum != muxNumLast){
				  if (muxNum == 0){
					  numTimesScanned++;
					  getKeys(); // and send em out if we got em
				  }
			  }
			  muxNumLast = muxNum;

			  if (numTimesScanned == 1000){
					numTimesScanned = 0;
					OSCMessage msgStat("/thousand");

					 SLIPSerial.beginPacket();
					 msgStat.send(SLIPSerial); // send the bytes to the SLIP stream
					 SLIPSerial.endPacket(); // mark the end of the OSC Packet
					 msgStat.empty(); // free space occupied by message
			  }

			}
			msgInSize = 0;

			if(!msgIn.hasError()) {


				// led
				msgIn.dispatch("/led", ledControl, 0);

				msgIn.dispatch("/oled", oledControl, 0);

				//msgIn.dispatch("/getkeys", getKeys, 0);

				msgIn.dispatch("/getknobs", getKnobs, 0);

				msgIn.empty(); // free space occupied by message

			}
		   else {   // just empty it if there was an error
				msgIn.empty(); // free space occupied by message
			}


	} // Infinite loop, never return.
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
