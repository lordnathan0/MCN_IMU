/*
 * clock.h
 *
 *  Created on: Nov 12, 2013
 *      Author: Nathan
 */

#ifndef CLOCK_H_
#define CLOCK_H_

void ClockSetup();
void HeartBeat();

//todo USER: define clock
#define CLOCK_PERIOD 10000 //us so 0.01 seconds 100 hz

#define HEARTBEAT_TICKS		100		//1hz
#define GPS_TICKS			20		//5hz
#define IMU_TICKS			3		//~33hz close enough
#define TEMP_TICKS			50		//2hz


//todo USER: CLOCK_TICKS_CLEAR should have the same number of zeros as clock_struct has elements (as seen below)
#define CLOCK_TICKS_CLEAR	{0,0,0,0}

typedef struct CLOCK_TICKS
{
	int HeartBeat;
	int GPS;
	int IMU;
	int Temp;
}clock_struct;


#endif /* CLOCK_H_ */
