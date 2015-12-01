/*
 * spi.h
 *
 *  Created on: May 24, 2014
 *      Author: owen
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32f0xx.h"

#define SPI_DATASIZE                     SPI_DataSize_8b

#define SPIx                             SPI1
#define SPIx_CLK                         RCC_APB2Periph_SPI1
#define SPIx_IRQn                        SPI1_IRQn
#define SPIx_IRQHandler                  SPI1_IRQHandler

#define SPIx_SCK_PIN                     GPIO_Pin_5
#define SPIx_SCK_GPIO_PORT               GPIOA
#define SPIx_SCK_GPIO_CLK                RCC_AHBPeriph_GPIOA
#define SPIx_SCK_SOURCE                  GPIO_PinSource5
#define SPIx_SCK_AF                      GPIO_AF_0

/*#define SPIx_MISO_PIN                    GPIO_Pin_14
 #define SPIx_MISO_GPIO_PORT              GPIOB
 #define SPIx_MISO_GPIO_CLK               RCC_AHBPeriph_GPIOB
 #define SPIx_MISO_SOURCE                 GPIO_PinSource14
 #define SPIx_MISO_AF                     GPIO_AF_0*/

#define SPIx_MOSI_PIN                    GPIO_Pin_7
#define SPIx_MOSI_GPIO_PORT              GPIOA
#define SPIx_MOSI_GPIO_CLK               RCC_AHBPeriph_GPIOA
#define SPIx_MOSI_SOURCE                 GPIO_PinSource7
#define SPIx_MOSI_AF                     GPIO_AF_0

/*#define SPIx_NSS_PIN                     GPIO_Pin_12
 #define SPIx_NSS_GPIO_PORT               GPIOB
 #define SPIx_NSS_GPIO_CLK                RCC_AHBPeriph_GPIOB
 #define SPIx_NSS_SOURCE                  GPIO_PinSource12
 #define SPIx_NSS_AF                      GPIO_AF_0*/

void spi_init(void);

#endif /* SPI_H_ */
