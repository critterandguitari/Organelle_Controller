#include "ssd1306.h"
#include "spi.h"
#include "Timer.h"

uint8_t pix_buf[1024];
extern uint8_t spi_out_buf_remaining;

//  line = pgm_read_byte(font+(c*5)+i);

uint8_t put_char_small(unsigned char c, uint8_t y, uint8_t x) {
	uint8_t i, j;
	//c -= 32;
	for (i = 0; i < 5; i++) {
		for (j = 0; j < 8; j++) {
			//if ((characters[(c * 5) + i] >> j) & 0x01)
			if ((font[(c * 5) + i] >> j) & 0x01)
				put_pixel(1, y + i, x + j);
			else
				put_pixel(0, y + i, x + j);
		}
	}
	return 5;
}

void put_pixel(uint8_t on, uint8_t x, uint8_t y) {

	uint8_t page = 0;
	uint8_t column = 0;
	uint8_t tmp8 = 0;

	x &= 0x7f;
	y &= 0x3f;

	// subtract cause its flipped
	// page = 7 - (y / 8);
	// column = 127 - x;

	page = y / 8;
	column = x;

	tmp8 = pix_buf[(page * 128) + column];

	if (on) {
		tmp8 |= (1 << (y & 0x7));
		//tmp8 |= (1 << (7 - (y & 0x7)));
	} else {
		tmp8 &= ~(1 << (y & 0x7));
		// tmp8 &= ~(1 << (7 - (y & 0x7)));
	}

	pix_buf[(page * 128) + column] = tmp8;
}

void ssd1306_cs(uint8_t stat) {
	if (stat)
		GPIO_SetBits(GPIOA, GPIO_Pin_10);
	else
		GPIO_ResetBits(GPIOA, GPIO_Pin_10);

}

void ssd1306_dc(uint8_t stat) {
	if (stat)
		GPIO_SetBits(GPIOA, GPIO_Pin_9);
	else
		GPIO_ResetBits(GPIOA, GPIO_Pin_9);
}

void ssd1306_rst(uint8_t stat) {
	if (stat)
		GPIO_SetBits(GPIOA, GPIO_Pin_8);
	else
		GPIO_ResetBits(GPIOA, GPIO_Pin_8);
}

void ssd1306_mosi(uint8_t stat) {
	if (stat)
		GPIO_SetBits(GPIOA, GPIO_Pin_7);
	else
		GPIO_ResetBits(GPIOA, GPIO_Pin_7);
}

void ssd1306_sck(uint8_t stat) {
	if (stat)
		GPIO_SetBits(GPIOA, GPIO_Pin_5);
	else
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
}

void CMD(uint8_t c) {
	ssd1306_cs(1);
	ssd1306_dc(0);
	ssd1306_cs(0);
	ssd1306_send_byte(c);
	ssd1306_cs(1);

}

void DATA(uint8_t c) {
	ssd1306_cs(1);
	ssd1306_dc(1);
	ssd1306_cs(0);
	ssd1306_send_byte(c);
	ssd1306_cs(1);
}

void ssd1306_send_byte(uint8_t byte) {

	SPI_SendData8(SPI1, byte);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY))
		;   // wait till done sending

	/*  for (i=7; i>=0; i--)
	 * 	int8_t i;
	 {
	 // Set clock pin low
	 ssd1306_sck(0);
	 // Set data pin high or low depending on the value of the current bit
	 if (byte & (1 << i)) ssd1306_mosi(1);
	 else ssd1306_mosi(0);
	 // Set clock pin high
	 ssd1306_sck(1);
	 }
	 ssd1306_sck(0);*/
}

void ssd1306_init(uint8_t vccstate) {

	spi_init();

	uint16_t i;
	for (i = 0; i < 1024; i++)
		pix_buf[i] = 0;

	// CMD(0xAA);
	// for(;;);

	// Reset the LCD
	ssd1306_rst(1);
	timer_sleep(5);
	ssd1306_rst(0);
	timer_sleep(5);
	ssd1306_rst(1);

	// Initialisation sequence
	CMD(SSD1306_DISPLAYOFF);                    // 0xAE
	CMD(SSD1306_SETLOWCOLUMN | 0x0);            // low col = 0
	CMD(SSD1306_SETHIGHCOLUMN | 0x0);           // hi col = 0
	CMD(SSD1306_SETSTARTLINE | 0x0);            // line #0
	CMD(SSD1306_SETCONTRAST);                   // 0x81
	if (vccstate == SSD1306_EXTERNALVCC) {
		CMD(0x9F);
	} else {
		CMD(0xCF);
	}
	CMD(0xa1);                                  // setment remap 95 to 0 (?)
	CMD(SSD1306_NORMALDISPLAY);                 // 0xA6
	CMD(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
	CMD(SSD1306_SETMULTIPLEX);                  // 0xA8
	CMD(0x3F);                                  // 0x3F 1/64 duty
	CMD(SSD1306_SETDISPLAYOFFSET);              // 0xD3
	CMD(0x0);                                   // no offset
	CMD(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
	CMD(0x80);                                  // the suggested ratio 0x80
	CMD(SSD1306_SETPRECHARGE);                  // 0xd9
	if (vccstate == SSD1306_EXTERNALVCC) {
		CMD(0x22);
	} else {
		CMD(0xF1);
	}
	CMD(SSD1306_SETCOMPINS);                    // 0xDA
	CMD(0x12);                                  // disable COM left/right remap
	CMD(SSD1306_SETVCOMDETECT);                 // 0xDB
	CMD(0x40);                                  // 0x20 is default?
	CMD(SSD1306_MEMORYMODE);                    // 0x20
	CMD(0x00);                                  // 0x0 act like ks0108
	CMD(SSD1306_SEGREMAP | 0x1);
	CMD(SSD1306_COMSCANDEC);
	CMD(SSD1306_CHARGEPUMP);                    //0x8D
	if (vccstate == SSD1306_EXTERNALVCC) {
		CMD(0x10);
	} else {
		CMD(0x14);
	}

	// Enabled the OLED panel
	CMD(SSD1306_DISPLAYON);
}

void ssd1306_refresh(void) {

	uint8_t byte;

	CMD(SSD1306_SETLOWCOLUMN | 0x0);  // low col = 0
	CMD(SSD1306_SETHIGHCOLUMN | 0x0);  // hi col = 0
	CMD(SSD1306_SETSTARTLINE | 0x0); // line #0

	ssd1306_cs(1);
	ssd1306_dc(1);
	ssd1306_cs(0);

	// send it all at once
	uint16_t i;
	for (i = 0; i < 1024; i++) {

		byte = pix_buf[i];

		SPI_SendData8(SPI1, byte);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY))
			;   // wait till done sending

	}
	ssd1306_cs(1);
}

void ssd1306_refresh_line(uint8_t page) {
	page &= 0x7;
	CMD(SSD1306_SETLOWCOLUMN | 0x0);  // low col = 0
	CMD(SSD1306_SETHIGHCOLUMN | 0x0);  // hi col = 0
	CMD(SSD1306_SETSTARTPAGE | page); // page 0 - 7

	static uint8_t c;
	c++;
	c &= 1;

	ssd1306_cs(1);
	ssd1306_dc(1);
	ssd1306_cs(0);

	uint16_t i;
	uint8_t byte;
	for (i = 0; i < 128; i++) {

		byte = pix_buf[i + (page * 128)];
		//spi_out_buf[i] = byte;

		SPI_SendData8(SPI1, byte);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY))
			;   // wait till done sending
	}

	spi_out_buf_remaining = 128;
	ssd1306_cs(1);
}

void println_16(char * line, int len, int x, int y) {
	int i, deltax;
	deltax = x;
	for (i = 0; i < len; i++) {
		deltax += put_char_arial16(line[i], deltax, y, 1);
		deltax += 2;
	}
}

void println_8(char * line, int len, int x, int y) {
	int i, deltax;
	deltax = x;
	for (i = 0; i < len; i++) {
		deltax += put_char_small(line[i], deltax, y);
		deltax += 1;
	}
}
void println_8_spacy(char * line, int len, int x, int y) {
	int i, deltax;
	deltax = x;
	for (i = 0; i < len; i++) {
		deltax += put_char_small(line[i], deltax, y);
		deltax += 2;
	}
}
unsigned int put_char_arial16(unsigned char character, unsigned int y,
		unsigned int x, unsigned int color) {
	int i;
	int j;
	int k;
	int charWidth;
	int charOffset;

	if (character == 32)
		return 4;

	character -= 33;

	charWidth = arial16Width[character + 1];
	charOffset = arial16Offset[character] * 2;

	for (i = 0; i < 2; i++) {
		for (j = 0; j < 8; j++) {
			for (k = 0; k < charWidth; k++) {
				if ((arial16[charOffset + k + (i * charWidth)] >> j) & 0x01)
					put_pixel(color, (y + k), (x + (i * 8) + j));
				else
					put_pixel(0, (y + k), (x + (i * 8) + j));
			}
		}
	}
	return charWidth + 1;
}

