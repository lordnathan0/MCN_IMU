/*
 * spi.h
 *
 *  Created on: Dec 30, 2013
 *      Author: Nathan
 */

#ifndef SPI_H_
#define SPI_H_

unsigned int Send_SPI(unsigned char* a);
unsigned int Read_SPI(unsigned char* a);

void spi_fifo_init();
void SpiGpio();

#define 	SLAVEENSET()			GpioDataRegs.GPASET.bit.GPIO15 = 1
#define 	SLAVEENCLEAR()			GpioDataRegs.GPACLEAR.bit.GPIO15 = 1

#endif /* SPI_H_ */
