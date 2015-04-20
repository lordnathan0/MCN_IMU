/*
 * bb_serial.c
 *
 *  Created on: Apr 14, 2015
 *      Author: Nathan Lord
 */

#include "all.h"

sci_struct GPS;

extern void DSP28x_usDelay(Uint32 Count);

void ConfigTX()
{
	EALLOW;
	SETTX();
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;         // GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;          // output
	GpioCtrlRegs.GPAQSEL1.bit.GPIO1 = 0;        //Synch to SYSCLKOUT only
	GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1; 		//disable pull up
	EDIS;
}

void ConfigRX()
{
	   EALLOW;
	   GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;         // GPIO
	   GpioCtrlRegs.GPADIR.bit.GPIO3 = 0;          // input
	   GpioCtrlRegs.GPAQSEL1.bit.GPIO3 = 2;        // XINT1 Synch to SYSCLKOUT only
	   GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1; 			//disable pull up
	   EDIS;

	// GPIO3 is XINT1
	   EALLOW;
	   GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 3;   // XINT1 is GPIO3
	   EDIS;

	// Configure XINT1
	   XIntruptRegs.XINT1CR.bit.POLARITY = 0;      // Falling edge interrupt

	// Enable XINT1
	   XIntruptRegs.XINT1CR.bit.ENABLE = 1;        // Enable XINT1


	   PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4
	   IFR &= ~M_INT1;
	   IER |= M_INT1;
}

void bb_setup()
{
	ConfigTX();
	ConfigRX();
}

void bb_send_char(char c)
{
	char i;
	DINT;

  	//c = ~c;
  	CLEARTX();
  	DELAY_US(BB_SER_DELAY_US);
	for (i=0; i<8; i++)
  	{
    	if( c & 1 )
      		SETTX();
    	else
      		CLEARTX();
    	c >>= 1;
    	DELAY_US(BB_SER_DELAY_US);
 	}
	SETTX();
	DELAY_US(BB_SER_DELAY_US);

	EINT;
}

char bb_read_char()
{
	CLEARTX();
	char i;
	char c;
	c = 0;
	char b;
	DELAY_US(BB_SER_DELAY_US/8);
	for (i=0; i<8; i++ )
	{

		DELAY_US(BB_SER_DELAY_US);
		SETTX();
		b = READRX() << i;
		c = c | b;
		CLEARTX();
	}
	//DELAY_US(BB_SER_DELAY_US);
	SETTX();
	return c;
}

// INT1.4
//Falling edge. Indicates start Bit
__interrupt void  XINT1_ISR(void)
{
	char c = bb_read_char();
	if (c == '$')						//start of sentence start storing
	{
		GPS.startflag = 1;
		GPS.gps[0] = c;
		GPS.length = 1;
		SETLED1();
	}
	else if(c == '\n' && GPS.startflag)		// end of sentence parse and restart
	{
		GPS_Choose();
		StopWatchRestart(ops.GPS_stopwatch);
		GPS.startflag = 0;
		CLEARLED1();
	}
	else if(GPS.startflag)				//keep storing until end of sentence
	{

		GPS.gps[GPS.length] = c;
		GPS.length++;
	}

	PieCtrlRegs.PIEIFR1.bit.INTx4 = 0; //clear flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
