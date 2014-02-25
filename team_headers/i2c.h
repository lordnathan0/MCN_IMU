/*
 * i2c.h
 *
 *  Created on: Feb 3, 2014
 *      Author: Nathan
 */

#ifndef I2C_H_
#define I2C_H_

#define false 						0
#define true						1

void I2CA_Init(void);
char I2C_readBit(unsigned char devAddr, unsigned char regAddr, unsigned char bitNum, unsigned char *d);
char I2C_readBits(unsigned char devAddr, unsigned char regAddr, unsigned char bitStart, unsigned char length, unsigned char *d);
char I2C_readByte(unsigned char devAddr, unsigned char regAddr, unsigned char *d);
char I2C_readBytes(unsigned char devAddr, unsigned char regAddr, unsigned int length, unsigned char *d);

unsigned char I2C_writeBit(unsigned char devAddr, unsigned char regAddr, unsigned char bitNum, unsigned char d);
unsigned char I2C_writeBits(unsigned char devAddr, unsigned char regAddr, unsigned char bitStart, unsigned char length, unsigned char d);
unsigned char I2C_writeByte(unsigned char devAddr, unsigned char regAddr, unsigned char d);
unsigned char I2C_writeBytes(unsigned char devAddr, unsigned char regAddr, unsigned int length, unsigned char *d);
unsigned char I2C_writeWord(unsigned char devAddr, unsigned char regAddr, unsigned int d);
unsigned char I2C_writeWords(unsigned char devAddr, unsigned char regAddr,unsigned int length, unsigned int* d);

#endif /* I2C_H_ */
