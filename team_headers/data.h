/*
 * data.h
 *
 *  Created on: Oct 30, 2013
 *      Author: Nathan
 */

#ifndef DATA_H_
#define DATA_H_


typedef struct DATA
{
	int ax;
	int ay;
	int az;
	int gx;
	int gy;
	int gz;
	double post_motor;
	double post_controller;
	double ambient;
	double motor1;
	double motor2;
} data_struct;

#endif /* DATA_H_ */
