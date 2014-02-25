/*
 * mpu6050dmp.h
 *
 *  Created on: Feb 2, 2014
 *      Author: Nathan
 */

#ifndef MPU6050DMP_H_
#define MPU6050DMP_H_

#include <string.h>

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

void Quaternion_getConjugate(Quaternion *q, Quaternion *ret);
void Quaternion_getProduct(Quaternion *q1, Quaternion *q2,Quaternion *ret );
void VectorInt16_Rotate(VectorInt16 *v,Quaternion *q);
unsigned char dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
unsigned char dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
unsigned char dmpGetAccel(VectorInt16 *v, const unsigned char* packet);
unsigned char dmpGetGravity(VectorFloat *v, Quaternion *q);
unsigned char int_dmpGetQuaternion(int *d, const unsigned char* packet);
unsigned char dmpGetQuaternion(Quaternion *q, const unsigned char* packet);
unsigned char dmpGetYawPitchRoll(float *d, Quaternion *q, VectorFloat *gravity);
unsigned char dmpInitialize();

#define strcpy_P(dest, src) strcpy((dest), (src))
#define strcat_P(dest, src) strcat((dest), (src))
#define strcmp_P(a, b) strcmp((a), (b))

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#define pgm_read_float(addr) (*(const float *)(addr))

#define pgm_read_byte_near(addr) pgm_read_byte(addr)
#define pgm_read_word_near(addr) pgm_read_word(addr)
#define pgm_read_dword_near(addr) pgm_read_dword(addr)
#define pgm_read_float_near(addr) pgm_read_float(addr)
#define pgm_read_byte_far(addr) pgm_read_byte(addr)
#define pgm_read_word_far(addr) pgm_read_word(addr)
#define pgm_read_dword_far(addr) pgm_read_dword(addr)
#define pgm_read_float_far(addr) pgm_read_float(addr)

#define MPU6050_DMP_CODE_SIZE       1929    // dmpMemory[]
#define MPU6050_DMP_CONFIG_SIZE     192     // dmpConfig[]
#define MPU6050_DMP_UPDATES_SIZE    47      // dmpUpdates[]


#endif /* MPU6050DMP_H_ */
