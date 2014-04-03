/*
 * SCI.h
 *
 *  Created on: Jan 28, 2014
 *      Author: Nathan
 */

#ifndef SCI_H_
#define SCI_H_

#include "info.h"

#define LSPCLK_FREQ (CPU_FREQ_MHZ*100000)/4
#define SCI_FREQ    9600
#define SCI_PRD     (long)(LSPCLK_FREQ/(SCI_FREQ*8))-1
#define GPS_STOPWATCH 	5000 //half second

typedef struct SCI
{
	unsigned char startflag : 1;
	unsigned char gps[SENTENCE_SIZE];
	unsigned char gps_sentence [3][SENTENCE_SIZE];
	unsigned char gps_complete[3];
	unsigned int length;
	unsigned int sentence_length[3];
	nmeaINFO gps_info;
}sci_struct;


void GPS_Choose();
void GPS_setup();
void GPS_stop();
void GPS_buffer_setup();
void GPS_fifo_init();
void GPS_parse();
void GPS_Command();
void GPS_send(const char * c);

#endif /* SCI_H_ */
