/*
 * bb_serial.h
 *
 *  Created on: Apr 14, 2015
 *      Author: Nathan Lord
 */

#ifndef BB_SERIAL_H_
#define BB_SERIAL_H_

#define 	CLEARTX()			GpioDataRegs.GPACLEAR.bit.GPIO1 = 1
#define		SETTX()				GpioDataRegs.GPASET.bit.GPIO1 = 1
#define 	TOGGLETX()			GpioDataRegs.GPATOGGLE.bit.GPIO1 = 1

#define		READRX()			GpioDataRegs.GPADAT.bit.GPIO3

#define UART_SOFT_BAUD 9600UL

#define BB_SER_DELAY_US (Uint32) (1000000.0/((float) UART_SOFT_BAUD))

void ConfigTX();
void ConfigRX();
void bb_setup();
void bb_send_char(char c);
char bb_read_char();

#endif /* BB_SERIAL_H_ */
