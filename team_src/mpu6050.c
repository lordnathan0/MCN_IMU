/*
 * mpu6050.c
 *
 *  Created on: Feb 2, 2014
 *      Author: Nathan
 */

#include "mpu6050.h"
#include "mpu6050dmp.h"
#include "i2c.h"
#include <string.h>
#include "all.h"

unsigned char buffer[22];
unsigned char devAddr;

extern unsigned char fifoBuffer[128];
extern const unsigned char dmpMemory[MPU6050_DMP_CODE_SIZE];
extern const unsigned char dmpConfig[MPU6050_DMP_CONFIG_SIZE];
extern const unsigned char dmpUpdates[MPU6050_DMP_UPDATES_SIZE];

unsigned char getDeviceID()
{
    I2C_readBits(devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, buffer);
    return buffer[0];
}

void switchSPIEnabled(char enabled)
{
    I2C_writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_IF_DIS_BIT, enabled);
}

void getMotion6(int* ax, int* ay, int* az, int* gx, int* gy, int* gz)
{
   I2C_readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
    *ax = (((int)buffer[0]) << 8) | buffer[1];
    *ay = (((int)buffer[2]) << 8) | buffer[3];
    *az = (((int)buffer[4]) << 8) | buffer[5];
    *gx = (((int)buffer[8]) << 8) | buffer[9];
    *gy = (((int)buffer[10]) << 8) | buffer[11];
    *gz = (((int)buffer[12]) << 8) | buffer[13];
}

void mpu_setup()
{
	devAddr = MPU6050_DEFAULT_ADDRESS;
	//switchSPIEnabled(true);
	DELAY_US(1*1000);
    setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
}

void setClockSource(unsigned char source)
{
    I2C_writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void setFullScaleGyroRange(unsigned char range)
{
    I2C_writeBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void setFullScaleAccelRange(unsigned char range)
{
    I2C_writeBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

void setSleepEnabled(unsigned char enabled)
{
    I2C_writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

void mpu_reset()
{
	//todo consider writing byte 0x80 without reading the register before since it is reset anyways
    I2C_writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true);
}

void setMemoryBank(unsigned char bank, unsigned char prefetchEnabled, unsigned char userBank)
{
    bank &= 0x1F;
    if (userBank) bank |= 0x20;
    if (prefetchEnabled) bank |= 0x40;
    I2C_writeByte(devAddr, MPU6050_RA_BANK_SEL, bank);
}

void setMemoryStartAddress(unsigned char address)
{
    I2C_writeByte(devAddr, MPU6050_RA_MEM_START_ADDR, address);
}

unsigned char readMemoryByte()
{
    I2C_readByte(devAddr, MPU6050_RA_MEM_R_W, buffer);
    return buffer[0];
}

unsigned char writeProgMemoryBlock(const unsigned char *d, unsigned int dataSize, unsigned char bank, unsigned char address, unsigned char verify)
{
	int i;
	for (i = 0; i < 7; i ++)
	{
		setMemoryBank(i, false, false); // bank number  = i
		setMemoryStartAddress(0);       // startaddress = 0 so start writing every DMP bank from the beginning

		I2C_writeBytes(devAddr,0x6f , 256, (unsigned char*) &d[i*256]);
	}

	// DMP bank 7 gets only 137 bytes:

	setMemoryBank(7, false, false); // bank number  = 7
	setMemoryStartAddress(0);       // startaddress = 0 so start writing also this DMP bank from the beginning

	I2C_writeBytes(devAddr, 0x6f , 137, (unsigned char*) &d[i*256]);

	return true; // end of writeDMPMemory reached
}

void readMemoryBlock(unsigned char *d, unsigned int dataSize, unsigned char bank, unsigned char address)
{
	I2C_readBytes(devAddr, MPU6050_RA_MEM_R_W, dataSize ,d);
}
//void readMemoryBlock(unsigned char *d, unsigned int dataSize, unsigned char bank, unsigned char address)
//{
//    setMemoryBank(bank,false,false);
//    setMemoryStartAddress(address);
//    unsigned char chunkSize;
//    unsigned int i;
//    for (i = 0; i < dataSize;) {
//        // determine correct chunk size according to bank position and d size
//        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;
//
//        // make sure we don't go past the d size
//        if (i + chunkSize > dataSize) chunkSize = dataSize - i;
//
//        // make sure this chunk doesn't go past the bank boundary (256 bytes)
//        if (chunkSize > 256 - address) chunkSize = 256 - address;
//
//        // read the chunk of d as specified
//        I2C_readBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, d + i);
//
//        // increase byte index by [chunkSize]
//        i += chunkSize;
//
//        // uint8_t automatically wraps to 0 at 256
//        address += chunkSize;
//
//        // if we aren't done, update bank (if necessary) and address
//        if (i < dataSize) {
//            if (address == 0) bank++;
//            setMemoryBank(bank,false,false);
//            setMemoryStartAddress(address);
//        }
//    }
//}

unsigned char getOTPBankValid()
{
    I2C_readBit(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, buffer);
    return buffer[0];
}

void setOTPBankValid(unsigned char enabled)
{
    I2C_writeBit(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}

unsigned char writeProgDMPConfigurationSet(const unsigned char *d, unsigned int dataSize)
{
    return writeDMPConfigurationSet(d, dataSize, false);
}

unsigned char writeMemoryBlock(const unsigned char *d, unsigned int dataSize, unsigned char bank, unsigned char address, unsigned char verify, unsigned char useProgMem)
{
	setMemoryBank(bank, false, false);
	setMemoryStartAddress(address);       // startaddress = 0 so start writing every DMP bank from the beginning

	I2C_writeBytes(devAddr,0x6f , dataSize, (unsigned char*) d);
	return true;
}

//unsigned char writeMemoryBlock(const unsigned char *d, unsigned int dataSize, unsigned char bank, unsigned char address, unsigned char verify, unsigned char useProgMem)
//{
//    setMemoryBank(bank,false,false);
//    setMemoryStartAddress(address);
//    unsigned char chunkSize;
//    unsigned char *verifyBuffer;
//    unsigned char *progBuffer;
//    unsigned int i;
//    unsigned char j;
//    if (verify) verifyBuffer = (unsigned char *)myMalloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
//    if (useProgMem) progBuffer = (unsigned char *)myMalloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
//    for (i = 0; i < dataSize;) {
//        // determine correct chunk size according to bank position and d size
//        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;
//
//        // make sure we don't go past the d size
//        if (i + chunkSize > dataSize) chunkSize = dataSize - i;
//
//        // make sure this chunk doesn't go past the bank boundary (256 bytes)
//        if (chunkSize > 256 - address) chunkSize = 256 - address;
//
//        if (useProgMem) {
//            // write the chunk of d as specified
//            for (j = 0; j < chunkSize; j++) progBuffer[j] = pgm_read_byte(d + i + j);
//        } else {
//            // write the chunk of d as specified
//            progBuffer = (unsigned char*)d + i;
//        }
//
//        I2C_writeBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, progBuffer);
//
//        // verify d if needed
//        if (verify && verifyBuffer) {
//            setMemoryBank(bank,false,false);
//            setMemoryStartAddress(address);
//            I2C_readBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer);
//            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
//                myFree((void*)verifyBuffer);
//                if (useProgMem) myFree(progBuffer);
//                return false; // uh oh.
//            }
//        }
//
//        // increase byte index by [chunkSize]
//        i += chunkSize;
//
//        // uint8_t automatically wraps to 0 at 256
//        address += chunkSize;
//
//        // if we aren't done, update bank (if necessary) and address
//        if (i < dataSize) {
//            if (address == 0) bank++;
//            setMemoryBank(bank,false,false);
//            setMemoryStartAddress(address);
//        }
//    }
//    if (verify) myFree(verifyBuffer);
//    if (useProgMem) myFree(progBuffer);
//    return true;
//}

unsigned char writeDMPConfigurationSet(const unsigned char *d, unsigned int dataSize, unsigned char useProgMem)
{
	unsigned char special;
	unsigned int i;
	// config set dmpConfig is a long string of blocks with the following structure:
	// [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
	unsigned char bank, offset, length;

	for (i = 0; i < MPU6050_DMP_CONFIG_SIZE;)
	{
		bank   = pgm_read_byte(dmpConfig + i++); // pgm_read_byte() is a macro that reads a byte of d stored in a specified address(PROGMEM area)
		offset = pgm_read_byte(dmpConfig + i++);
		length = pgm_read_byte(dmpConfig + i++);

		if (length > 0) // regular block of data to write
		{
			setMemoryBank(bank, false, false); // bank number  = bank
			setMemoryStartAddress(offset);     // startaddress = offset from the beginning (0) of the bank

			I2C_writeBytes(devAddr,0x6f , length, (unsigned char*) &dmpConfig[i]);

			i = i + length;
		}
		else // length = 0; special instruction to write
		{
			// NOTE: this kind of behavior (what and when to do certain things)
			// is totally undocumented. This code is in here based on observed
			// behavior only, and exactly why (or even whether) it has to be here
			// is anybody's guess for now.
			special = pgm_read_byte(dmpConfig + i++);
			if (special == 0x01)
			{
				// enable DMP-related interrupts (ZeroMotion, FIFOBufferOverflow, DMP)
				I2C_writeByte(devAddr, 0x38, 0x32); 	// write 00110010: ZMOT_EN, FIFO_OFLOW_EN, DMP_INT_EN true
				// by the way: this sets all other interrupt enables to false
			}
		}
	}
	return true;
}

void setSlaveAddress(unsigned char num, unsigned char address)
{
    if (num > 3) return;
    I2C_writeByte(devAddr, MPU6050_RA_I2C_SLV0_ADDR + num*3, address);
}

void setI2CMasterModeEnabled(unsigned char enabled)
{
    I2C_writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

void resetI2CMaster()
{
    I2C_writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, true);
}

void setIntEnabled(unsigned char enabled)
{
    I2C_writeByte(devAddr, MPU6050_RA_INT_ENABLE, enabled);
}

void setRate(unsigned char rate)
{
    I2C_writeByte(devAddr, MPU6050_RA_SMPLRT_DIV, rate);
}

void setExternalFrameSync(unsigned char sync)
{
    I2C_writeBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}

void setDLPFMode(unsigned char mode)
{
    I2C_writeBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

void setDMPConfig1 (unsigned char config)
{
    I2C_writeByte(devAddr, MPU6050_RA_DMP_CFG_1, config);
}

void setDMPConfig2(unsigned char config)
{
    I2C_writeByte(devAddr, MPU6050_RA_DMP_CFG_2, config);
}

void resetFIFO()
{
    I2C_writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true);
}

void getFIFOBytes(unsigned char *d, unsigned char length)
{
    I2C_readBytes(devAddr, MPU6050_RA_FIFO_R_W, length, d);
}

void setFIFOEnabled(unsigned char enabled)
{
    I2C_writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}

unsigned int getFIFOCount()
{
    I2C_readBytes(devAddr, MPU6050_RA_FIFO_COUNTH, 2, buffer);
    return (((unsigned int)buffer[0]) << 8) | buffer[1];
}

void setDMPEnabled(unsigned char enabled)
{
    I2C_writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}

void resetDMP()
{
    I2C_writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, true);
}

void setMotionDetectionThreshold(unsigned char threshold)
{
    I2C_writeByte(devAddr, MPU6050_RA_MOT_THR, threshold);
}

void setZeroMotionDetectionThreshold(unsigned char threshold)
{
    I2C_writeByte(devAddr, MPU6050_RA_ZRMOT_THR, threshold);
}

void setMotionDetectionDuration(unsigned char duration)
{
    I2C_writeByte(devAddr, MPU6050_RA_MOT_DUR, duration);
}

void setZeroMotionDetectionDuration(unsigned char duration)
{
    I2C_writeByte(devAddr, MPU6050_RA_ZRMOT_DUR, duration);
}

char getXGyroOffset()
{
    I2C_readBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}

void setXGyroOffset(char offset)
{
    I2C_writeBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}



char getYGyroOffset()
{
    I2C_readBits(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}

void setYGyroOffset(char offset)
{
    I2C_writeBits(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// ZG_OFFS_TC register

char getZGyroOffset()
{
    I2C_readBits(devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}

void setZGyroOffset(char offset)
{
    I2C_writeBits(devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

void setZAccelOffset(int offset)
{
    I2C_writeWord(devAddr, MPU6050_RA_ZA_OFFS_H, offset);
}

void setXAccelOffset(int offset)
{
    I2C_writeWord(devAddr, MPU6050_RA_XA_OFFS_H, offset);
}

void setYAccelOffset(int offset)
{
    I2C_writeWord(devAddr, MPU6050_RA_YA_OFFS_H, offset);
}

unsigned char getIntStatus()
{
    I2C_readByte(devAddr, MPU6050_RA_INT_STATUS, buffer);
    return buffer[0];
}
