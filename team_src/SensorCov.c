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

canfloat s1;
float s2;
float sicr;
float accelcov;
float gyrocov;

int wdcount;
extern unsigned char devAddr;

extern unsigned char fifoBuffer[128];
extern sci_struct GPS;

#define 	packetSize 		42
#define		MPUINT()		GpioDataRegs.GPADAT.bit.GPIO23

void MPU_cov();
void MPU_setup();

void enablewatchdog()
{
		wdcount = 0;
	   EALLOW;
	   SysCtrlRegs.WDKEY = 0x55;                // Clear the WD counter
	   SysCtrlRegs.WDKEY = 0xAA;
	   SysCtrlRegs.WDCR = 0x0047;               // Enable watchdog module
	   SysCtrlRegs.SCSR = 0x02;					// watchdog is set up to cause isr

	   PieCtrlRegs.PIEIER1.bit.INTx8 = 1;          // Enable PIE Group 1 INT4
	   IFR &= ~M_INT1;
	   IER |= M_INT1;

	   EDIS;
}

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
	ConfigLED0();
	ConfigLED1();
	//todo USER: SensorCovInit()


	//init_pwm_servo();

	s1.F32 = 0.5;
	s2 = 0.5;
	sicr = .0015;

	MPU_setup();
	GPS_setup();


	conv_watch = StartStopWatch(50000);
	conv_timer = StartStopWatch(10);
	enablewatchdog();
}


void LatchStruct()
{
	memcpy(&ops_temp, &ops, sizeof(struct OPERATIONS));
	memcpy(&data_temp, &data, sizeof(struct DATA));
	ops.Change.all = 0;	//clear change states
}

void SensorCovMeasure()
{



//	//todo USER: Sensor Conversion
//	//update data_temp and ops_temp
//	//use stopwatch to catch timeouts
//	//waiting should poll isStopWatchComplete() to catch timeout and throw StopWatchError
//
	wdcount = 0;
	ServiceDog();
	GPS_parse();

	if(isStopWatchComplete(conv_timer) == 0)
	{
		return;
	}


//	if (s1.F32 >= 0.9 || s1.F32 <= 0.1 )
//	{
//		sicr = sicr * -1;
//	}
//
//
//	s1.F32 = s1.F32 + sicr;
//	s2 = s2 + sicr;
//
//
//	set_pwm(s1.F32, s2);

	SETLED0();
	MPU_cov();
	CLEARLED0();

	StopWatchRestart(conv_timer);



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

void MPU_cov()
{
	int ax,ay,az,gx,gy,gz;
	unsigned char mpuIntStatus;
	mpuIntStatus = getIntStatus();
	fifoCount = getFIFOCount();

	getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	data_temp.ax.F32 = accelcov * ax;
	data_temp.ay.F32  = accelcov * ay;
	data_temp.az.F32  = accelcov * az;
	data_temp.gx.F32  = gyrocov * gx;
	data_temp.gy.F32  = gyrocov * gy;
	data_temp.gz.F32  = gyrocov * gz;

	if(fifoCount >= packetSize)
	{
		if (fifoCount >= 1024)
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
            dmpGetYawPitchRoll(data_temp.ypr, &data_temp.q);
			//probably not necessary
			//fifoCount -= packetSize;
		}
	}
}

void MPU_setup()
{
	I2CA_Init();

	devAddr = MPU6050_DEFAULT_ADDRESS;

	mpu_reset();
	int id = getDeviceID();
	setSleepEnabled(false);
	char status = dmpInitialize();

    setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    accelcov = (2000)/32767.0;
    setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    gyrocov = (4)/32767.0;


//    setXGyroOffset(-60);
//    setYGyroOffset(-30);
//    setZGyroOffset(-35);
//    setZAccelOffset(0);
//   setXAccelOffset(0);
//   setYAccelOffset(0);
}

// INT1.8
__interrupt void  WAKEINT_ISR(void)    // WD, LOW Power
{
  if(wdcount > 1000)
  {
	  Restart();
  }
  wdcount++;
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
