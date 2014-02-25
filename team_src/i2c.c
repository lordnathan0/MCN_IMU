/*
 * i2c.c
 *
 *  Created on: Feb 3, 2014
 *      Author: Nathan
 */

#include "i2c.h"
#include "spi.h"
#include "all.h"

void I2CA_Init(void)
{
   // Initialize I2C

   I2caRegs.I2CPSC.all = 6;		    // Prescaler - need 7-12 Mhz on module clk
   I2caRegs.I2CCLKL = 15;			// NOTE: must be non zero
   I2caRegs.I2CCLKH = 10;			// NOTE: must be non zero
   I2caRegs.I2CIER.all = 0;			// Disable SCD & ARDY interrupts

   I2caRegs.I2CMDR.all = 0x0420;	// Take I2C out of reset
   									// Stop I2C when suspended
   	   	   	   	   	   	   	   	   	//  Master mode

   I2caRegs.I2CFFTX.bit.I2CFFEN = 0;// Disable mode and TXFIFO
   I2caRegs.I2CFFRX.all = 0;		// Dsiable rx fifo

   InitI2CGpio();
}



char I2C_readBit(unsigned char devAddr, unsigned char regAddr, unsigned char bitNum, unsigned char *d)
{
	unsigned char b;
	unsigned char count = I2C_readByte(devAddr, regAddr, &b);
    *d = b & (1 << bitNum);
    return count;
}

char I2C_readBits(unsigned char devAddr, unsigned char regAddr, unsigned char bitStart, unsigned char length, unsigned char *d)
{
	// 01101001 read byte
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	//    010   masked
	//   -> 010 shifted
	unsigned char count, b;
	if ((count = I2C_readByte(devAddr, regAddr, &b)) != 0)
	{
		unsigned char mask = ((1 << length) - 1) << (bitStart - length + 1);
		b &= mask;
		b >>= (bitStart - length+1);
		*d = b;
	}
	return count;
}

char I2C_readByte(unsigned char devAddr, unsigned char regAddr, unsigned char *d)
{
	return I2C_readBytes(devAddr, regAddr, 1, d);
}


char I2C_readBytes(unsigned char devAddr, unsigned char regAddr, unsigned int length, unsigned char *d)
{
	unsigned char a;
	int i;
	a = regAddr | 0x80; //write so change msb bit to 1
	SLAVEENCLEAR();
	Send_SPI(&a);

	if (length > 0)
	{
		Read_SPI(d);
		length--;

		for(i = 0; i<length; i++)
		{
			d++;
			Read_SPI(d);
		}
	}

	SLAVEENSET();
	return true;
}

//char I2C_readBytes(unsigned char devAddr, unsigned char regAddr, unsigned int length, unsigned char *d)
//{
//	// Wait until the STP bit is cleared from any previous master communication.
//	// Clearing of this bit by the module is delayed until after the SCD bit is
//	// set. If this bit is not checked prior to initiating a new message, the
//	// I2C could get confused.
//	int i;
//
//	// Setup slave address
//	I2caRegs.I2CSAR = devAddr;
//
//	// Check if bus busy
//	while (I2caRegs.I2CSTR.bit.BB == 1)
//	{
//		I2caRegs.I2CMDR.bit.IRS = 0;
//		I2caRegs.I2CMDR.bit.IRS = 1;
//	}
//
//	I2caRegs.I2CMDR.bit.MST = 1;
//	I2caRegs.I2CMDR.bit.TRX = 1; //Transmit mode
//	I2caRegs.I2CMDR.bit.RM = 1;	// repeat
//	I2caRegs.I2CMDR.bit.STT = 1;//start condition
//
//	while (I2caRegs.I2CMDR.bit.STT != 0) {} //detect start
//
//	I2caRegs.I2CSTR.bit.ARDY = 1;
//	I2caRegs.I2CDXR = regAddr;	//send reg to read
//	while (I2caRegs.I2CSTR.bit.ARDY != 1) {}
//
//	//I2caRegs.I2CMDR.bit.STP = 1;//stop condition
//
//
//
//	I2caRegs.I2CMDR.bit.TRX = 0; //Receiver mode
//	I2caRegs.I2CMDR.bit.STT = 1;//start condition
//
//	while(I2caRegs.I2CMDR.bit.STT != 0) {} // Detect a start
//
//	I2caRegs.I2CMDR.bit.RM = 0;	// repeat off
//	I2caRegs.I2CCNT = length; //this will do an nack when 0
//
//	for (i=0; i <length; i++)
//	{
//		if(I2caRegs.I2CSTR.bit.NACK != 0)
//		{
//			I2caRegs.I2CSTR.bit.NACK = 1;
//		}
//		while (I2caRegs.I2CSTR.bit.RRDY != 1) {}
//		d[i] = I2caRegs.I2CDRR;
//	}
//
//	while(I2caRegs.I2CSTR.bit.NACK != 0); // Detect NACK
//	I2caRegs.I2CSTR.bit.NACK = 1;
//
//	return i;
//}


unsigned char I2C_writeBit(unsigned char devAddr, unsigned char regAddr, unsigned char bitNum, unsigned char d)
{
	unsigned char b;
	I2C_readByte(devAddr, regAddr, &b);
	b = (d != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	I2C_writeByte(devAddr, regAddr, b);
	I2C_readByte(devAddr, regAddr, &b);
	return true;
}

unsigned char I2C_writeBits(unsigned char devAddr, unsigned char regAddr, unsigned char bitStart, unsigned char length, unsigned char d)
{
	//      010 value to write
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	unsigned char b;
	if (I2C_readByte(devAddr, regAddr, &b) != 0) {
		unsigned char mask = ((1 << length) - 1) << (bitStart - length + 1);
		d <<= (bitStart - length + 1); // shift d into correct position
		d &= mask; // zero all non-important bits in d
		b &= ~(mask); // zero all important bits in existing byte
		b |= d; // combine d with existing byte
		return I2C_writeByte(devAddr, regAddr, b);
	} else {
		return false;
	}
}

unsigned char I2C_writeByte(unsigned char devAddr, unsigned char regAddr, unsigned char d)
{
	return I2C_writeBytes(devAddr, regAddr, 1, &d);
}


unsigned char I2C_writeBytes(unsigned char devAddr, unsigned char regAddr, unsigned int length, unsigned char *d)
{
	int i;
	SLAVEENCLEAR();
	Send_SPI(&regAddr);
	Send_SPI(d);
	length--;

	for(i = 0; i<length; i++)
	{
		d++;
		Send_SPI(d);
	}

	SLAVEENSET();
	return true;

}

//unsigned char I2C_writeBytes(unsigned char devAddr, unsigned char regAddr, unsigned int length, unsigned char *d)
//{
//	// Wait until the STP bit is cleared from any previous master communication.
//	// Clearing of this bit by the module is delayed until after the SCD bit is
//	// set. If this bit is not checked prior to initiating a new message, the
//	// I2C could get confused.
//	int i;
//
//	while (I2caRegs.I2CSTR.bit.BB == 1)
//	{
//		I2caRegs.I2CMDR.bit.IRS = 0;
//		I2caRegs.I2CMDR.bit.IRS = 1;
//	}
//
//	// Setup slave address
//	I2caRegs.I2CSAR = devAddr;
//
//	I2caRegs.I2CMDR.bit.MST = 1;
//	I2caRegs.I2CMDR.bit.TRX = 1; //Transmitter mode
//	I2caRegs.I2CMDR.bit.RM = 1;	// repeat
//	I2caRegs.I2CMDR.bit.STT = 1;//start condition
//
//	while(I2caRegs.I2CMDR.bit.STT != 0) {} // Detect a start
//
//	I2caRegs.I2CSTR.bit.ARDY = 1;
//	I2caRegs.I2CDXR = regAddr;	//send reg to write to
//	while (I2caRegs.I2CSTR.bit.ARDY != 1) {}
//
//	for (i=0; i < length; i++)
//	{
//		I2caRegs.I2CSTR.bit.ARDY = 1;
//		I2caRegs.I2CDXR = d[i];
//		while (I2caRegs.I2CSTR.bit.ARDY != 1) {}
//	}
//
//	while (I2caRegs.I2CSTR.bit.NACK == 1) // Clear if NACK received
//	{
//		I2caRegs.I2CSTR.bit.NACK = 1;
//	}
//
//	I2caRegs.I2CMDR.bit.STP = 1;		//stop condition
//
//	while (I2caRegs.I2CMDR.bit.STP != 0) {}
//	while(I2caRegs.I2CSTR.bit.SCD != 1) {}
//	I2caRegs.I2CSTR.bit.SCD = 1; // Clear stop condition
//
//	I2caRegs.I2CMDR.bit.RM = 0;	// NO repeat
//
//	return i;
//}

unsigned char I2C_writeWord(unsigned char devAddr, unsigned char regAddr, unsigned int d)
{
	return I2C_writeWords(devAddr, regAddr, 1, &d);
}

unsigned char I2C_writeWords(unsigned char devAddr, unsigned char regAddr,unsigned int length, unsigned int* d)
{
	return I2C_writeBytes(devAddr, regAddr,length*2, (unsigned char*) d);
}
