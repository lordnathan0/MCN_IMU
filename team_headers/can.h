/*
 * can.h
 *
 *  Created on: Nov 13, 2013
 *      Author: Nathan
 */

#ifndef CAN_H_
#define CAN_H_




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
#define COMMAND_ID 		0x1
#define COMMAND_BOX 	0

#define HEARTBEAT_ID 	0x0
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


#endif /* CAN_H_ */
