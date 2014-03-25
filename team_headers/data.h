/*
 * data.h
 *
 *  Created on: Oct 30, 2013
 *      Author: Nathan
 */

#ifndef DATA_H_
#define DATA_H_

#include "mpu6050dmp.h"

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
	canfloat break_pressure;
	float ypr[3];
	Quaternion q;
	VectorInt16 aa;
	VectorFloat gravity;
} data_struct;

#endif /* DATA_H_ */
