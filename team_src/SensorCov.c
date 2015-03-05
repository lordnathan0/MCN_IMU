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
#include <math.h>

#define QUANTA_TO_VOLT			(3.3/4096)

unsigned int fifoCount;



stopwatch_struct* conv_watch;
stopwatch_struct* conv_timer;

ops_struct ops_temp;
data_struct data_temp;

float s1;
float s2;

extern unsigned char devAddr;

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

//	spi_fifo_init();
//	SpiGpio();

	init_pwm_servo();
	s1 = 0.5;
	s2 = 0.5;

//	I2CA_Init();
//
//	devAddr = MPU6050_DEFAULT_ADDRESS;
//
//	mpu_reset();
//	int id = getDeviceID();
//	setSleepEnabled(false);
//	char status = dmpInitialize();
//	//setClockSource(MPU6050_CLOCK_PLL_XGYRO);
//
//    setFullScaleGyroRange(MPU6050_GYRO_FS_250);
//    setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
//
//
////    setXGyroOffset(-3500);
////    setYGyroOffset(-365);
////    setZGyroOffset(210);
////    setZAccelOffset(914);
////    setXAccelOffset(-980);
////    setYAccelOffset(630);
//
//
//
//	adcinit();
//
//	//CONFIG GP_BUTTON
//	ConfigGPButton();
//
//	//CONFIG LEDS
//	//led 0
//	ConfigLED0();
//	//led 1
//	ConfigLED1();
//	//CONFIG 12V SWITCH
//	Config12V();
//	conv_watch = StartStopWatch(50000);
//	conv_timer = StartStopWatch(500);
}


void LatchStruct()
{
	memcpy(&ops_temp, &ops, sizeof(struct OPERATIONS));
	memcpy(&data_temp, &data, sizeof(struct DATA));
	ops.Change.all = 0;	//clear change states
}

void SensorCovMeasure()
{
	unsigned char mpuIntStatus;
	set_pwm(s1, s2);

	//todo USER: Sensor Conversion
	//update data_temp and ops_temp
	//use stopwatch to catch timeouts
	//waiting should poll isStopWatchComplete() to catch timeout and throw StopWatchError

//	if(isStopWatchComplete(conv_timer) == 0)
//	{
//		return;
//	}
//
//
//	StopWatchRestart(conv_timer);
//
//	mpuIntStatus = getIntStatus();
//	fifoCount = getFIFOCount();
//
//	getMotion6(&data_temp.ax, &data_temp.ay, &data_temp.az, &data_temp.gx, &data_temp.gy, &data_temp.gz);
//	if(fifoCount >= packetSize)
//	{
//		if (fifoCount >= 1024)
//		{
//			// reset so we can continue cleanly
//			resetFIFO();
//		}
//		else if (mpuIntStatus & 1)
//		{
//			// wait for correct available data length, should be a VERY short wait
//			while (fifoCount < packetSize) fifoCount = getFIFOCount();
//
//			StopWatchRestart(conv_watch);
//
//			getFIFOBytes(fifoBuffer, packetSize);
//            dmpGetQuaternion(&data_temp.q, fifoBuffer);
//            dmpGetAccel(&data_temp.aa, fifoBuffer);
//            dmpGetGravity(&data_temp.gravity, &data_temp.q);
//            dmpGetYawPitchRoll(data_temp.ypr, &data_temp.q, &data_temp.gravity);
//			//probably not necessary
//			//fifoCount -= packetSize;
//		}
//	}


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
