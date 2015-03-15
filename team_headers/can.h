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

#define XAxisIMU_ID			0x15E
#define XAxisIMU_BOX			7

#define YAxisIMU_ID			0x15F
#define YAxisIMU_BOX			8

#define ZAxisIMU_ID			0x160
#define ZAxisIMU_BOX			9

#define GPS_SPEED_ID	0x118
#define GPS_SPEED_BOX 	18


#endif /* CAN_H_ */
