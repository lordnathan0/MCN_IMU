/*
 * mpu6050dmp.c
 *
 *  Created on: Feb 2, 2014
 *      Author: Nathan
 */

#include <math.h>
#include <string.h>
#include "mpu6050dmp.h"
#include "mpu6050.h"
#include "i2c.h"
#include "all.h"

extern unsigned char devAddr;
unsigned char fifoBuffer[128];
extern const unsigned char dmpMemory[MPU6050_DMP_CODE_SIZE];
extern const unsigned char dmpConfig[MPU6050_DMP_CONFIG_SIZE];
extern const unsigned char dmpUpdates[MPU6050_DMP_UPDATES_SIZE];
float PI = 3.1415926359;
void Quaternion_getConjugate(Quaternion *q, Quaternion *ret)
{
	ret->x = -(q->x);
	ret->y = -(q->y);
	ret->z = -(q->z);
	//return Quaternion(w, -x, -y, -z);
}

void Quaternion_getProduct(Quaternion *q1, Quaternion *q2,Quaternion *ret )
{
	ret->w = q1->w*q2->w - q1->x*q2->x - q1->y*q2->y - q1->z*q2->z;
	ret->x = q1->w*q2->x + q1->x*q2->w + q1->y*q2->z - q1->z*q2->y;
	ret->y = q1->w*q2->y - q1->x*q2->z + q1->y*q2->w + q1->z*q2->x;
	ret->z = q1->w*q2->z + q1->x*q2->y - q1->y*q2->x + q1->z*q2->w;

}

void VectorInt16_Rotate(VectorInt16 *v,Quaternion *q)
{
	// http://www.cprogramming.com/tutorial/3d/quaternions.html
	// http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
	// http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
	// ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1

	// P_out = q * P_in * conj(q)
	// - P_out is the output vector
	// - q is the orientation quaternion
	// - P_in is the input vector (a*aReal)
	// - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
	Quaternion p;
	p.w = 0;
	p.x = (float)v->x;
	p.y = (float)v->y;
	p.z = (float)v->z;

	Quaternion qtemp;

	// quaternion multiplication: q * p, stored back in p
	Quaternion_getProduct(q,&p,&p);

	// quaternion multiplication: p * conj(q), stored back in p
	Quaternion_getConjugate(q,&qtemp);
	Quaternion_getProduct(&p,&qtemp,&p);

	// p quaternion is now [0, x', y', z']
	v->x = (int)p.x;
	v->y = (int)p.y;
	v->z = (int)p.z;
}

unsigned char dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
    // rotate measured 3D acceleration vector into original state
    // frame of reference based on orientation quaternion
    memcpy(v, vReal, sizeof(VectorInt16));

    VectorInt16_Rotate(v,q);
    return 0;
}

unsigned char dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
    // get rid of the gravity component full scale is 16 so 2048/g
    v -> x = vRaw -> x - gravity -> x*970;
    v -> y = vRaw -> y - gravity -> y*970;
    v -> z = vRaw -> z - gravity -> z*970;
    return 0;
}

unsigned char dmpGetAccel(VectorInt16 *v, const unsigned char* packet)
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    v -> x = (packet[28] << 8) + packet[29];
    v -> y = (packet[32] << 8) + packet[33];
    v -> z = (packet[36] << 8) + packet[37];
    return 0;
}

unsigned char dmpGetGravity(VectorFloat *v, Quaternion *q)
{
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}

unsigned char dmpGetYawPitchRoll(canfloat *d, Quaternion *q)
{
//    // yaw: (about Z axis)
//    d[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
//    // pitch: (nose up/down, about Y axis)
//    d[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
//    // roll: (tilt left/right, about X axis)
//    d[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));

   d[0].F32 = atan2(2.0f * (q -> x * q -> y + q -> w * q -> z), q -> w * q -> w + q -> x * q -> x - q -> y * q -> y - q -> z * q -> z);
   d[1].F32 = -asin(2.0f * (q -> x * q -> z - q -> w * q -> y));
   d[2].F32 = atan2(2.0f * (q -> w * q -> x + q -> y * q -> z), q -> w * q -> w - q -> x * q -> x - q -> y * q -> y + q -> z * q -> z);
   d[1].F32 *= 180.0f / PI;
   d[0].F32 *= 180.0f / PI;
   d[2].F32 *= 180.0f / PI;
    return 0;
}

unsigned char int_dmpGetQuaternion(int *d, const unsigned char* packet)
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    d[0] = ((packet[0] << 8) + packet[1]);
    d[1] = ((packet[4] << 8) + packet[5]);
    d[2] = ((packet[8] << 8) + packet[9]);
    d[3] = ((packet[12] << 8) + packet[13]);
    return 0;
}

unsigned char dmpGetQuaternion(Quaternion *q, const unsigned char* packet)
{
	int qI[4];
	unsigned char status = int_dmpGetQuaternion(qI, packet);
    if (status == 0) {
        q -> w = (float)qI[0] / 16384.0f;
        q -> x = (float)qI[1] / 16384.0f;
        q -> y = (float)qI[2] / 16384.0f;
        q -> z = (float)qI[3] / 16384.0f;
        return 0;
    }
    return status; // int16 return value, indicates error if this line is reached
}

unsigned char dmpInitialize()
{
    // reset device
	mpu_reset();
	DELAY_US(30*1000);

    // disable sleep mode

    setSleepEnabled(false);

    // get MPU hardware revision
    setMemoryBank(0x10, true, true);
    setMemoryStartAddress(0x06);
    unsigned char hwRevision = readMemoryByte();
    setMemoryBank(0, false, false);

    // check OTP bank valid
    unsigned char otpValid = getOTPBankValid();

    // get X/Y/Z gyro offsets
    char xgOffsetTC = getXGyroOffset();
    char ygOffsetTC = getYGyroOffset();
    char zgOffsetTC = getZGyroOffset();

    // setup weird slave stuff (?)
//    setSlaveAddress(0, 0x7F);
//    setI2CMasterModeEnabled(false);
//    setSlaveAddress(0, 0x68);
//    resetI2CMaster();

    DELAY_US(20*1000);

    // load DMP code into memory banks

    if (writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE,0,0,false))
    {
        if (writeProgDMPConfigurationSet(dmpConfig, MPU6050_DMP_CONFIG_SIZE)) {
            setClockSource(MPU6050_CLOCK_PLL_ZGYRO);

            setIntEnabled(0x12);

            setRate(4); // 1khz / (1 + 4) = 200 Hz

            setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);

            setDLPFMode(MPU6050_DLPF_BW_42);

//            setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
//            setFullScaleAccelRange(MPU6050_ACCEL_FS_16);


            setDMPConfig1(0x03);
            setDMPConfig2(0x00);

            setOTPBankValid(false);

//            setXGyroOffset(xgOffsetTC);
//            setYGyroOffset(ygOffsetTC);
//            setZGyroOffset(zgOffsetTC);

            unsigned char dmpUpdate[16], j;
            unsigned int pos = 0;

            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true, false);

            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true, false);


            resetFIFO();

            unsigned int fifoCount = getFIFOCount();

            if (fifoCount > 128)
            {
            	fifoCount = 128;
            }

            getFIFOBytes(fifoBuffer, fifoCount);

            setMotionDetectionThreshold(2);

            setZeroMotionDetectionThreshold(156);

            setMotionDetectionDuration(80);

            setZeroMotionDetectionDuration(0);

            resetFIFO();

            setFIFOEnabled(true);

            setDMPEnabled(true);

            resetDMP();


            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true, false);

            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true, false);

            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true, false);

//            while ((getFIFOCount()) < 3) {DELAY_US(500);}
//
//            fifoCount = getFIFOCount();
//
//            getFIFOBytes(fifoBuffer, fifoCount);

            unsigned char mpuIntStatus = getIntStatus();

            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

//            while ((getFIFOCount()) < 3) {DELAY_US(500);}
//
//            fifoCount = getFIFOCount();
//            getFIFOBytes(fifoBuffer, fifoCount);

            mpuIntStatus = getIntStatus();


            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], true, false);

            setDMPEnabled(true);


            resetFIFO();
            getIntStatus();
        } else {
           return 2; // configuration block loading failed
        }
    } else {
        return 1; // main binary block loading failed
    }
    return 0; // success
}

