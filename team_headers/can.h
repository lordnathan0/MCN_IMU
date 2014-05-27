/*
 * can.h
 *
 *  Created on: Nov 13, 2013
 *      Author: Nathan
 */

#ifndef CAN_H_
#define CAN_H_


void BUS_OFF();

struct CANmsg {
   char MBox;
   union CANMSGCTRL_REG   MSGCTRL;
   union CANMDL_REG       MDL;
   union CANMDH_REG       MDH;
};

struct TRS_REG {
	union CANTRS_REG	TRS;
};

void CANSetup();
char FillCAN(unsigned int Mbox);
void SendCAN(unsigned int Mbox);
void FillCANData();
void FillSendCAN(unsigned int Mbox);
void ClearMailBoxes();

//todo USER: DEFINE IDs and mailboxes for output
#define COMMAND_ID 		0xA1
#define COMMAND_BOX 	0

#define HEARTBEAT_ID 	0x41
#define HEARTBEAT_BOX 	1

#define CURRENT_TIME_ID 	0x10e
#define CURRENT_TIME_BOX	3

#define ALT_ACCR_ID			0x10d
#define ALT_ACCR_BOX		4

#define LAT_VAL_ID			0x10b
#define LAT_VAL_BOX			5

#define LONG_ID				0x10c
#define LONG_BOX			6

#define ACCEL_ID			0x10F
#define ACCEL_BOX			7

#define GYRO_ID				0x110
#define GYRO_BOX			8

#define POST_CON_ID			0x108
#define POST_CON_BOX		10

#define AMBIENT_ID			0x10A
#define AMBIENT_BOX			11

#define R_BRAKE_PRESSURE_ID 	0x102
#define R_BRAKE_PRESSURE_BOX 	12

#define IMU1_ID 	0x113
#define IMU1_BOX 	13

#define IMU2_ID 	0x114
#define IMU2_BOX 	14

#define IMU3_ID 	0x115
#define IMU3_BOX 	15

#define IMU4_ID 	0x116
#define IMU4_BOX 	16

#define IMU5_ID 	0x117
#define IMU5_BOX 	17

#define GPS_SPEED_ID	0x118
#define GPS_SPEED_BOX 	18


#endif /* CAN_H_ */
