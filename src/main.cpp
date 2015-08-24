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

int
main(int argc, char* argv[])
 {

	// being ADC setup
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef  ADC_InitStructure;

	ADC_DeInit(ADC1);

	//(#) Enable the ADC interface clock using
	//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	//(#) ADC pins configuration
	//   (++) Enable the clock for the ADC GPIOs using the following function:
	//        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOx, ENABLE);
	//   (++) Configure these ADC pins in analog mode using GPIO_Init();
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//(#) Configure the ADC conversion resolution, data alignment, external
	//    trigger and edge, scan direction and Enable/Disable the continuous mode
	//    using the ADC_Init() function.
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
	ADC_Init(ADC1, &ADC_InitStructure);

	// Calibrate ADC before enabling
	ADC_GetCalibrationFactor(ADC1);
	//(#) Activate the ADC peripheral using ADC_Cmd() function.
	ADC_Cmd(ADC1, ENABLE);

	// Wait until ADC enabled
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN) == RESET);
	// end setup ADC

	// MUX SEL Lines
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
  
  uint32_t seconds = 0;


  //the message wants an OSC address as first argument
  OSCMessage msg("/key");
  msg.add(99);

  OSCMessage msgIn;

  int size;

  uart2_init();

  blink_led_init();

  blink_led_on();
  blink_led_off();

  timer_start();
  // Infinite loop

  int msgInSize = 0;

  int32_t adc = 0;



  // oled init
  ssd1306_init(0);
  put_char_small('I', 0, 0);
  put_char_small('O', 48, 0);
  ssd1306_refresh();

/*
  ssd1306_refresh_line(0);
  timer_sleep(50);
  ssd1306_refresh_line(1);
  timer_sleep(50);
  ssd1306_refresh_line(2);
  timer_sleep(50);
  ssd1306_refresh_line(3);
  timer_sleep(50);
  ssd1306_refresh_line(4);
  timer_sleep(50);
  ssd1306_refresh_line(5);
  timer_sleep(50);
  ssd1306_refresh_line(6);
  timer_sleep(50);
  ssd1306_refresh_line(7);
  timer_sleep(50);*/


  AUX_LED_RED_OFF;
  AUX_LED_GREEN_ON;
  AUX_LED_BLUE_ON;
  //for(;;);

  while (1)
  {
     // blink_led_on();
    //  timer_sleep(5);

     // blink_led_off();

     // uart_2_send(0xAA);



      ++seconds;

      // Count seconds on the trace device.
     // trace_printf("Second %u\n", seconds);

              // use msgInSize hack cause endofPacket can return true at beginning and end of a packet
            while((!SLIPSerial.endofPacket()) || (msgInSize < 4) ) {

              if( (size =SLIPSerial.available()) > 0) {

                while(size--) {
                  msgIn.fill(SLIPSerial.read());
                  msgInSize++;
                }
              }
            }
            msgInSize = 0;

            if(!msgIn.hasError()) {

                // pass it along
               // blink_led_on();
               // SLIPSerial.beginPacket();
               // msgIn.send(SLIPSerial); // send the bytes to the SLIP stream
               // SLIPSerial.endPacket(); // mark the end of the OSC Packet
               // blink_led_off();

                // renumber
              //  msgIn.dispatch("/sys/renumber", renumber, 0);

                // reset
               // msgIn.dispatch("/sys/reset", reset, 0);

                // led
               // msgIn.dispatch("/led", ledControl, 0);

                msgIn.dispatch("/oled", oledControl, 0);

                // type count
               // address = "/" + type;
                //msgIn.dispatch(address.c_str(), incIndex);

                msgIn.empty(); // free space occupied by message


            }
           else {   // just empty it if there was an error
                msgIn.empty(); // free space occupied by message
            }

            // get adc
        /*    ADC_ChannelConfig(ADC1, ADC_Channel_10, ADC_SampleTime_239_5Cycles);
            ADC_StartOfConversion(ADC1);
            while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET){;}
            adc = ADC_GetConversionValue(ADC1);

            OSCMessage msgKey("/key");
            msgKey.add((int32_t)adc);

            SLIPSerial.beginPacket();
            msgKey.send(SLIPSerial); // send the bytes to the SLIP stream
            SLIPSerial.endPacket(); // mark the end of the OSC Packet
            msgKey.empty(); // free space occupied by message

            // get enc
            int32_t enc1, enc2, enc3;
            enc1 = (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13)) ? 0 : 1;
            enc2 = (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14)) ? 0 : 1;
            enc3 = (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15)) ? 0 : 1;

            OSCMessage msgEnc("/enc");
            msgEnc.add((int32_t)enc1);
            msgEnc.add((int32_t)enc2);
            msgEnc.add((int32_t)enc3);

            SLIPSerial.beginPacket();
            msgEnc.send(SLIPSerial); // send the bytes to the SLIP stream
            SLIPSerial.endPacket(); // mark the end of the OSC Packet
            msgEnc.empty(); // free space occupied by message*/

    }
  // Infinite loop, never return.
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
