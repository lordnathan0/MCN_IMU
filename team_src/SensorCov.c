/*
 * SensorCov().c
 *
 *  Created on: Oct 30, 2013
 *      Author: Nathan
 */

#include "all.h"
#include <string.h>
#include "mpu6050dmp.h"
#include "mpu6050.h"
#include "spi.h"
unsigned int fifoCount;

int ax, ay, az, gx, gy, gz;

float ypr[3];
Quaternion q;
VectorInt16 aa;
VectorFloat gravity;
VectorInt16 aaReal;

ops_struct ops_temp;
data_struct data_temp;

extern unsigned char fifoBuffer[128];

#define 	packetSize 		42
#define		MPUINT()		GpioDataRegs.GPADAT.bit.GPIO23

void SensorCov()
{
	SensorCovInit();
	while (ops.State == STATE_SENSOR_COV)
	{
		LatchStruct();
		SensorCovMeasure();
		UpdateStruct();
		FillCANData();
	}
	SensorCovDeInit();
}

void SensorCovInit()
{
	//todo USER: SensorCovInit()

	//set up mpu int pin
	EALLOW;
	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;         // GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO23 = 0;          // input
	GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 0;        //Synch to SYSCLKOUT only
	GpioCtrlRegs.GPAPUD.bit.GPIO23 = 1; 		//disable pull up
	EDIS;

	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;         // GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;          // input
	GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 0;        //Synch to SYSCLKOUT only
	GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1; 		//disable pull up
	EDIS;

	spi_fifo_init();
	SpiGpio();
//	I2CA_Init();
	int id = getDeviceID();
	//mpu_setup();
	//I2C_writeByte(0x68, 0x23, 0); // no fifo enable
	char status = dmpInitialize();

    setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    setFullScaleAccelRange(MPU6050_ACCEL_FS_16);

//    setXGyroOffset(-3500);
//    setYGyroOffset(-365);
//    setZGyroOffset(210);
//    setZAccelOffset(914);
//    setXAccelOffset(-980);
//    setYAccelOffset(630);

}


void LatchStruct()
{
	memcpy(&ops_temp, &ops, sizeof(struct OPERATIONS));
	memcpy(&data_temp, &data, sizeof(struct DATA));
	ops.Change.all = 0;	//clear change states
}

void SensorCovMeasure()
{

	//todo USER: Sensor Conversion
	//update data_temp and ops_temp
	//use stopwatch to catch timeouts
	//waiting should poll isStopWatchComplete() to catch timeout and throw StopWatchError
	unsigned char mpuIntStatus = getIntStatus();
	fifoCount = getFIFOCount();

	getMotion6(&data_temp.ax, &data_temp.ay, &data_temp.az, &data_temp.gx, &data_temp.gy, &data_temp.gz);
	if(fifoCount >= packetSize)
	{
		if (fifoCount == 1024)
		{
			// reset so we can continue cleanly
			resetFIFO();
		}
		else if (mpuIntStatus & 1)
		{
			// wait for correct available data length, should be a VERY short wait
			while (fifoCount < packetSize) fifoCount = getFIFOCount();

			getFIFOBytes(fifoBuffer, packetSize);
            dmpGetQuaternion(&q, fifoBuffer);
            dmpGetAccel(&aa, fifoBuffer);
            dmpGetGravity(&gravity, &q);
            dmpGetLinearAccel(&aaReal, &aa, &gravity);
            dmpGetYawPitchRoll(ypr, &q, &gravity);
			//probably not necessary
			//fifoCount -= packetSize;
		}
	}

}

void UpdateStruct()
{
	memcpy(&data, &data_temp, sizeof(struct DATA));

	//todo USER: UpdateStruct
	//update with node specific op changes

	//if ops is not changed outside of sensor conversion copy temp over, otherwise don't change

	//Change bit is only set by ops changes outside of SensorCov.
	if (ops.Change.bit.State == 0)
	{
		ops.State = ops_temp.State;
	}

	if (ops.Change.bit.Flags == 0)
	{
		//only cov error happens inside of conversion so all other changes are considered correct.
		//update accordingly to correct cov_errors
		ops.Flags.bit.cov_error = ops_temp.Flags.bit.cov_error;
	}
	ops.Change.all = 0;	//clear change states
}

void SensorCovDeInit()
{
	//todo USER: SensorCovDeInit()

}
