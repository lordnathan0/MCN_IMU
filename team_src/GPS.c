/*
 * SCI.c
 *
 *  Created on: Jan 28, 2014
 *      Author: Nathan
 */

#include "all.h"
#include "parse.h"
#include "GPS.h"
#include <string.h>

nmeaGPGGA gga;
nmeaGPGSA gsa;
nmeaGPGSV gsv;
nmeaGPRMC rmc;
nmeaGPVTG vtg;

sci_struct GPS;

const char choose[] = "$PMTK314,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; //choose gga, rmc, vtg
//const char choose[] = $PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28
//const char choose[] = "$PMTK314,0,1,1,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"; //choose gga, rmc, vtg, gsv
const char freq[] = "$PMTK220, 200*2C\r\n";	// 5hz


void GPS_setup()
{
	GPS_buffer_setup();
	InitSciGpio();
	GPS_fifo_init();
	GPS_Command();
	PieCtrlRegs.PIEIER9.bit.INTx1=1;     // PIE Group 9, INT1
	PieCtrlRegs.PIEIER9.bit.INTx2=1;     // PIE Group 9, INT2
	IER |= M_INT9;
}

void GPS_Command()
{
	GPS_send(choose);
	GPS_send(freq);
}

void GPS_stop()
{
	SciaRegs.SCICTL2.bit.RXBKINTENA = 0;
	SciaRegs.SCICTL1.all =0x0000;			//disable
	StopStopWatch(ops.GPS_stopwatch);
}

void GPS_buffer_setup()
{
	GPS.startflag = 0;
	ops.GPS_stopwatch = StartStopWatch(GPS_STOPWATCH);
}

void GPS_fifo_init()
{
   SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                  // No parity,8 char bits,
                                  // async mode, idle-line protocol
   SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                  // Disable RX ERR, SLEEP, TXWAKE
   SciaRegs.SCICTL2.bit.TXINTENA =0;
   SciaRegs.SCICTL2.bit.RXBKINTENA =1;
   SciaRegs.SCIHBAUD = 0;
   SciaRegs.SCILBAUD = 0xC2;
   SciaRegs.SCICCR.bit.LOOPBKENA =0; // Disable loop back
   //Disable FIFO
   SciaRegs.SCIFFTX.bit.SCIFFENA = 0;
   SciaRegs.SCIFFRX.bit.RXFFIENA = 0;

   SciaRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset

}

void GPS_Choose()
{
	switch(nmea_pack_type((const char *)GPS.gps,GPS.length))
	{
	case GPGGA:
		memcpy(GPS.gps_sentence[0],GPS.gps, sizeof(char)*GPS.length);
		GPS.gps_complete[0] = 1;
		break;
	case GPGSA:
		break;
	case GPGSV:
		break;
	case GPRMC: //not useful data
		memcpy(GPS.gps_sentence[1],GPS.gps, sizeof(char)*GPS.length);
		GPS.gps_complete[1] = 1;
		break;
	case GPVTG:
		memcpy(GPS.gps_sentence[2],GPS.gps, sizeof(char)*GPS.length);
		GPS.gps_complete[2] = 1;
		break;
	default:
		break;
	}
}
void GPS_parse()
{
	char i;
	for(i=0;i<3;i++)
	{
		if (GPS.gps_complete[i] == 1)
		{
			switch(nmea_pack_type((const char *)GPS.gps_sentence[i],GPS.sentence_length[i]))
			{
			case GPGGA:
				nmea_parse_GPGGA((const char *)GPS.gps_sentence[i],GPS.sentence_length[i], &gga);
				nmea_GPGGA2info(&gga, &GPS.gps_info);
				break;
			case GPGSA:
				nmea_parse_GPGSA((const char *)GPS.gps_sentence[i],GPS.sentence_length[i], &gsa);
				nmea_GPGSA2info(&gsa, &GPS.gps_info);
				break;
			case GPGSV:
				nmea_parse_GPGSV((const char *)GPS.gps_sentence[i],GPS.sentence_length[i],  &gsv);
				nmea_GPGSV2info(&gsv, &GPS.gps_info);
				break;
			case GPRMC: //not useful data
				nmea_parse_GPRMC((const char *)GPS.gps_sentence[i],GPS.sentence_length[i],  &rmc);
				nmea_GPRMC2info(&rmc, &GPS.gps_info);
				break;
			case GPVTG:
				nmea_parse_GPVTG((const char *)GPS.gps_sentence[i],GPS.sentence_length[i],  &vtg);
				nmea_GPVTG2info(&vtg, &GPS.gps_info);
				break;
			default:
				break;
			}
		}
		GPS.gps_complete[i] = 0;
	}
}

void GPS_send(const char * c)
{
	int i = 0;
	while (c[i] != '\0')
	{
	    while (SciaRegs.SCICTL2.bit.TXRDY != 1) {}
	    SciaRegs.SCITXBUF= c[i];
	    i++;
	}
}

// INT9.1
__interrupt void SCIRXINTA_ISR(void)     // SCI-A
{
  // Insert ISR Code here
	unsigned char c;
	c = SciaRegs.SCIRXBUF.all;
	if (c == '$')						//start of sentence start storing
	{
		GPS.startflag = 1;
		GPS.gps[0] = c;
		GPS.length = 1;
	}
	else if(c == '\n' && GPS.startflag)		// end of sentence parse and restart
	{
		GPS_Choose();
		StopWatchRestart(ops.GPS_stopwatch);
		GPS.startflag = 0;
	}
	else if(GPS.startflag)				//keep storing until end of sentence
	{

		GPS.gps[GPS.length] = c;
		GPS.length++;
	}

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}
