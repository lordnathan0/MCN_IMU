/*
 * spi.c
 *
 *  Created on: Dec 30, 2013
 *      Author: Nathan
 */



#include "all.h"
#include "spi.h"


char dummy;

void spi_fifo_init()
{
// Initialize SPI FIFO registers
   SpibRegs.SPICCR.bit.SPISWRESET=0; 	// Reset SPI

   SpibRegs.SPICCR.all=0x0007; 			//8-bit no loopback

   SpibRegs.SPICTL.all=0x0006;       	//// Enable master mode, normal phase,enable talk, and SPI int disabled.

   SpibRegs.SPICCR.bit.CLKPOLARITY = 1;
   SpibRegs.SPICTL.bit.CLK_PHASE = 0;

   SpibRegs.SPISTS.all=0x0000;
   SpibRegs.SPIBRR = 127; 				//Baudrate is slow as possible
  //SpiaRegs.SPIBRR=0x0063;           	// Baud rate

   SpibRegs.SPIFFCT.all=0x00;

   SpibRegs.SPIPRI.bit.FREE=1;

   SpibRegs.SPICCR.bit.SPISWRESET=1;  	// Enable SPI

  SpibRegs.SPIFFTX.bit.SPIFFENA = 0; 	//disable fifo
  SpibRegs.SPIFFRX.bit.RXFFIENA = 0;

}

void SpiGpio()
{

	EALLOW;

	/* Enable internal pull-up for the selected pins */
	// Pull-ups can be enabled or disabled disabled by the user.
	// This will enable the pullups for the specified pins.
	// Comment out other unwanted lines.

	//GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;     // Enable pull-up on GPIO12 (SPISIMOB)
	  GpioCtrlRegs.GPAPUD.bit.GPIO24 = 1;     // Enable pull-up on GPIO24 (SPISIMOB)

	GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;     // Enable pull-up on GPIO13 (SPISOMIB)
	//  GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;     // Enable pull-up on GPIO25 (SPISOMIB)

	GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1;     // Enable pull-up on GPIO14 (SPICLKB)
	//  GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;     // Enable pull-up on GPIO26 (SPICLKB)

	//    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;     // Enable pull-up on GPIO15 (SPISTEB)
	//  GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;     // Enable pull-up on GPIO27 (SPISTEB)


	/* Set qualification for selected pins to asynch only */
	// This will select asynch (no qualification) for the selected pins.
	// Comment out other unwanted lines.

	//GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3;   // Asynch input GPIO12 (SPISIMOB)
	  GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 3;   // Asynch input GPIO24 (SPISIMOB)

	GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 3;   // Asynch input GPIO13 (SPISOMIB)
	//  GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 3;   // Asynch input GPIO25 (SPISOMIB)

	GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 3;   // Asynch input GPIO14 (SPICLKB)
	//  GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 3;   // Asynch input GPIO26 (SPICLKB)

	//GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3;   // Asynch input GPIO15 (SPISTEB)
	//  GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 3;   // Asynch input GPIO27 (SPISTEB)

	/* Configure SPI-B pins using GPIO regs*/
	// This specifies which of the possible GPIO pins will be SPI functional pins.
	// Comment out other unwanted lines.

	//GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 3;    // Configure GPIO12 as SPISIMOB
	  GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 3;    // Configure GPIO24 as SPISIMOB

	GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 3;    // Configure GPIO13 as SPISOMIB
	//  GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 3;    // Configure GPIO25 as SPISOMIB

	GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 3;    // Configure GPIO14 as SPICLKB
	//  GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 3;    // Configure GPIO26 as SPICLKB

	//    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 3;    // Configure GPIO15 as SPISTEB
	//  GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 3;    // Configure GPIO27 as SPISTEB

	SLAVEENSET();

	EDIS;
}

unsigned int Send_SPI(unsigned char* a)
{
	SpibRegs.SPITXBUF=(*a)<<8;
	while((SpibRegs.SPISTS.bit.INT_FLAG !=1));
	SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag

	dummy  = SpibRegs.SPIRXBUF;
	return true;
}

unsigned int Read_SPI(unsigned char* a)
{
	SpibRegs.SPITXBUF=0;
	while((SpibRegs.SPISTS.bit.INT_FLAG !=1));
	SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag

	*a = SpibRegs.SPIRXBUF;
	SpibRegs.SPIRXBUF = 0;
	return true;
}

