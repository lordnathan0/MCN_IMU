/*
 * mpu6050dmptypes.h
 *
 *  Created on: Mar 17, 2015
 *      Author: Nathan Lord
 */

#ifndef MPU6050DMPTYPES_H_
#define MPU6050DMPTYPES_H_

typedef struct QUATERNION
{
    float w;
    float x;
    float y;
    float z;
}Quaternion;

typedef struct VECTORFLOAT
{
    float x;
    float y;
    float z;
}VectorFloat;

typedef struct VECTORINT16
{
    int x;
    int y;
    int z;
}VectorInt16;


#endif /* MPU6050DMPTYPES_H_ */
