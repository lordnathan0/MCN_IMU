/*
 * data.h
 *
 *  Created on: Oct 30, 2013
 *      Author: Nathan
 */

#ifndef DATA_H_
#define DATA_H_


typedef union CANFLOAT
{
	float F32;
	unsigned long U32;
} canfloat;

typedef struct DATA
{
	int ax;
	int ay;
	int az;
	int gx;
	int gy;
	int gz;
	canfloat post_motor;
	canfloat post_controller;
	canfloat ambient;
	canfloat motor1;
	canfloat motor2;
} data_struct;

#endif /* DATA_H_ */
