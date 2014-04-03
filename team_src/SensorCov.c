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

	GPS_setup();

	spi_fifo_init();
	SpiGpio();
	int id = getDeviceID();
	char status = dmpInitialize();

    //setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    //setFullScaleAccelRange(MPU6050_ACCEL_FS_2);


//    setXGyroOffset(-3500);
//    setYGyroOffset(-365);
//    setZGyroOffset(210);
//    setZAccelOffset(914);
//    setXAccelOffset(-980);
//    setYAccelOffset(630);


	adcinit();

	//CONFIG GP_BUTTON
	ConfigGPButton();

	//CONFIG LEDS
	//led 0
	ConfigLED0();
	//led 1
	ConfigLED1();
	//CONFIG 12V SWITCH
	Config12V();
	conv_watch = StartStopWatch(50000);

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
	GPS_parse();
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

			StopWatchRestart(conv_watch);

			getFIFOBytes(fifoBuffer, packetSize);
            dmpGetQuaternion(&data_temp.q, fifoBuffer);
            dmpGetAccel(&data_temp.aa, fifoBuffer);
            dmpGetGravity(&data_temp.gravity, &data_temp.q);
            dmpGetYawPitchRoll(data_temp.ypr, &data_temp.q, &data_temp.gravity);
			//probably not necessary
			//fifoCount -= packetSize;
		}
	}

#define adc_ratio(x)	(((double)x)/(4096.0))
#define r_inf		(double)0.12885174498
#define B			(double)3375
#define r2  		(double)1470 //resistor before adc
#define r1			(double)1820 //resistor after adc
		double temp;


	readADC();



	//Controller cool side
	//thermistor
	temp = r1*(1/((1/adc_ratio(B0RESULT))-1))-r2;
	data_temp.post_controller.F32 = ((B)/(log((temp/r_inf)))) - 273.5;

	//Ambient
	//thermistor
	temp = r1*(1/((1/adc_ratio(B1RESULT))-1))-r2;
	data_temp.ambient.F32 = (B)/(log((temp/r_inf)))- 273.5;

	//Rear break pressure
	//Vout = ((.8*5)/3000) * P + .5
	//P = ((Vout-.5)*3000)/(.8*5)
	//Vm = (10/15.6)*Vout
	//Vout = (15.6/10) *Vm
	//Vm = ADC*QUANTA_TO_VOLT;
	//kpa = P * 6.8947
	temp = B2RESULT * QUANTA_TO_VOLT; 				//Vm
	temp = temp * (15.6/10); 						//Vout
	temp = ((temp-.5)*3000)/(4); 					// Psi
	data_temp.break_pressure.F32 = temp ;

	if (isStopWatchComplete(conv_watch) == 1)
	{
		ops_temp.Flags.bit.cov_error = 1;
	}
	else
	{
		ops_temp.Flags.bit.cov_error = 0;
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
