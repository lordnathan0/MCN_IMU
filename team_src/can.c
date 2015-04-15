/*
 * can.c
 *
 *  Created on: Nov 12, 2013
 *      Author: Nathan
 */
#include "all.h"

unsigned long mask;
stopwatch_struct* can_watch;
struct ECAN_REGS ECanaShadow;
extern sci_struct GPS;
extern canfloat s1;

void BUS_OFF()
{
	EALLOW;
	ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;


	ECanaShadow.CANMC.bit.CCR = 0;
	ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

	ECanaShadow.CANES.all = ECanaRegs.CANES.all;
	while (ECanaShadow.CANES.bit.CCE != 0)
	{
		ECanaShadow.CANES.all = ECanaRegs.CANES.all;
	}

	EDIS;
}

void CANSetup()
{

	InitECanaGpio();
	InitECana();

	ClearMailBoxes();

	ECanaShadow.CANMIM.all = 0;
	ECanaShadow.CANMIL.all = 0;
	ECanaShadow.CANGIM.all = 0;
	ECanaShadow.CANGAM.bit.AMI = 0; //must be standard
	ECanaShadow.CANGIM.bit.I1EN = 1;  // enable I1EN
	ECanaShadow.CANMD.all = ECanaRegs.CANMD.all;
	ECanaShadow.CANME.all = ECanaRegs.CANME.all;

	//todo USER: Node specifc CAN setup
	EALLOW;

	// create mailbox for all Receive and transmit IDs
	// MBOX0 - MBOX31

	//Command RECEIVE
	ECanaMboxes.MBOX0.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX0.MSGID.bit.AME = 0;	// all bit must match
	ECanaMboxes.MBOX0.MSGID.bit.AAM = 0; 	// no RTR AUTO TRANSMIT
	ECanaMboxes.MBOX0.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX0.MSGID.bit.STDMSGID = COMMAND_ID;
	ECanaShadow.CANMD.bit.MD0 = 1;			//receive
	ECanaShadow.CANME.bit.ME0 = 1;			//enable
	ECanaShadow.CANMIM.bit.MIM0  = 1; 		//int enable
	ECanaShadow.CANMIL.bit.MIL0  = 1;  		// Int.-Level MB#0  -> I1EN

	//Heart TRANSMIT
	ECanaMboxes.MBOX1.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX1.MSGID.bit.AME = 0; 	// all bit must match
	ECanaMboxes.MBOX1.MSGID.bit.AAM = 1; 	//RTR AUTO TRANSMIT
	ECanaMboxes.MBOX1.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX1.MSGID.bit.STDMSGID = HEARTBEAT_ID;
	ECanaShadow.CANMD.bit.MD1 = 0; 			//transmit
	ECanaShadow.CANME.bit.ME1 = 1;			//enable


	//Current Time TRANSMIT
	ECanaMboxes.MBOX3.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX3.MSGID.bit.AME = 0; 	// all bit must match
	ECanaMboxes.MBOX3.MSGID.bit.AAM = 1; 	//RTR AUTO TRANSMIT
	ECanaMboxes.MBOX3.MSGCTRL.bit.DLC = 7;
	ECanaMboxes.MBOX3.MSGID.bit.STDMSGID = CURRENT_TIME_ID;
	ECanaShadow.CANMD.bit.MD3 = 0; 			//transmit
	ECanaShadow.CANME.bit.ME3 = 1;			//enable

	//Alt and Accuracy TRANSMIT
	ECanaMboxes.MBOX4.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX4.MSGID.bit.AME = 0; 	// all bit must match
	ECanaMboxes.MBOX4.MSGID.bit.AAM = 1; 	//RTR AUTO TRANSMIT
	ECanaMboxes.MBOX4.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX4.MSGID.bit.STDMSGID = ALT_ACCR_ID;
	ECanaShadow.CANMD.bit.MD4 = 0; 			//transmit
	ECanaShadow.CANME.bit.ME4 = 1;			//enable

	//Lat and Validity TRANSMIT
	ECanaMboxes.MBOX5.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX5.MSGID.bit.AME = 0; 	// all bit must match
	ECanaMboxes.MBOX5.MSGID.bit.AAM = 1; 	//RTR AUTO TRANSMIT
	ECanaMboxes.MBOX5.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX5.MSGID.bit.STDMSGID = LAT_VAL_ID;
	ECanaShadow.CANMD.bit.MD5 = 0; 			//transmit
	ECanaShadow.CANME.bit.ME5 = 1;			//enable

	//LONG TRANSMIT
	ECanaMboxes.MBOX6.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX6.MSGID.bit.AME = 0; 	// all bit must match
	ECanaMboxes.MBOX6.MSGID.bit.AAM = 1; 	//RTR AUTO TRANSMIT
	ECanaMboxes.MBOX6.MSGCTRL.bit.DLC = 7;
	ECanaMboxes.MBOX6.MSGID.bit.STDMSGID = LONG_ID;
	ECanaShadow.CANMD.bit.MD6 = 0; 			//transmit
	ECanaShadow.CANME.bit.ME6 = 1;			//enable

	//XIMU ACCEL TRANSMIT
	ECanaMboxes.MBOX7.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX7.MSGID.bit.AME = 0; 	// all bit must match
	ECanaMboxes.MBOX7.MSGID.bit.AAM = 1; 	//RTR AUTO TRANSMIT
	ECanaMboxes.MBOX7.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX7.MSGID.bit.STDMSGID = XAxisIMU_ID;
	ECanaShadow.CANMD.bit.MD7 = 0; 			//transmit
	ECanaShadow.CANME.bit.ME7 = 1;			//enable

	//YIMU ACCEL TRANSMIT
	ECanaMboxes.MBOX8.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX8.MSGID.bit.AME = 0; 	// all bit must match
	ECanaMboxes.MBOX8.MSGID.bit.AAM = 1; 	//RTR AUTO TRANSMIT
	ECanaMboxes.MBOX8.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX8.MSGID.bit.STDMSGID = YAxisIMU_ID;
	ECanaShadow.CANMD.bit.MD8 = 0; 			//transmit
	ECanaShadow.CANME.bit.ME8 = 1;			//enable

	//ZIMU ACCEL TRANSMIT
	ECanaMboxes.MBOX9.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX9.MSGID.bit.AME = 0; 	// all bit must match
	ECanaMboxes.MBOX9.MSGID.bit.AAM = 1; 	//RTR AUTO TRANSMIT
	ECanaMboxes.MBOX9.MSGCTRL.bit.DLC = 8;
	ECanaMboxes.MBOX9.MSGID.bit.STDMSGID = ZAxisIMU_ID;
	ECanaShadow.CANMD.bit.MD9 = 0; 			//transmit
	ECanaShadow.CANME.bit.ME9 = 1;			//enable

	//testing TRANSMIT
	ECanaMboxes.MBOX10.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX10.MSGID.bit.AME = 0; 	// all bit must match
	ECanaMboxes.MBOX10.MSGID.bit.AAM = 1; 	//RTR AUTO TRANSMIT
	ECanaMboxes.MBOX10.MSGCTRL.bit.DLC =8;
	ECanaMboxes.MBOX10.MSGID.bit.STDMSGID = 500;
	ECanaShadow.CANMD.bit.MD10 = 0; 			//transmit
	ECanaShadow.CANME.bit.ME10 = 1;			//enable


	// GPS speed TRANSMIT
	ECanaMboxes.MBOX18.MSGID.bit.IDE = 0; 	//standard id
	ECanaMboxes.MBOX18.MSGID.bit.AME = 0; 	// all bit must match
	ECanaMboxes.MBOX18.MSGID.bit.AAM = 1; 	//RTR AUTO TRANSMIT
	ECanaMboxes.MBOX18.MSGCTRL.bit.DLC = 4;
	ECanaMboxes.MBOX18.MSGID.bit.STDMSGID = GPS_SPEED_ID;
	ECanaShadow.CANMD.bit.MD18 = 0; 			//transmit
	ECanaShadow.CANME.bit.ME18 = 1;			//enable

	ECanaRegs.CANGAM.all = ECanaShadow.CANGAM.all;
	ECanaRegs.CANGIM.all = ECanaShadow.CANGIM.all;
	ECanaRegs.CANMIM.all = ECanaShadow.CANMIM.all;
	ECanaRegs.CANMIL.all = ECanaShadow.CANMIL.all;
	ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;
    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.STM = 0;    // No self-test mode
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
    EDIS;

    //ENABLE PIE INTERRUPTS
    IER |= M_INT9;
    PieCtrlRegs.PIEIER9.bit.INTx6= 1;

    can_watch = StartStopWatch(SENDCAN_STOPWATCH);
}

void ClearMailBoxes()
{
    ECanaMboxes.MBOX0.MDH.all = 0;
    ECanaMboxes.MBOX0.MDL.all = 0;
    ECanaMboxes.MBOX1.MDH.all = 0;
    ECanaMboxes.MBOX1.MDL.all = 0;
    ECanaMboxes.MBOX2.MDH.all = 0;
    ECanaMboxes.MBOX2.MDL.all = 0;
    ECanaMboxes.MBOX3.MDH.all = 0;
    ECanaMboxes.MBOX3.MDL.all = 0;
    ECanaMboxes.MBOX4.MDH.all = 0;
    ECanaMboxes.MBOX4.MDL.all = 0;
    ECanaMboxes.MBOX5.MDH.all = 0;
    ECanaMboxes.MBOX5.MDL.all = 0;
    ECanaMboxes.MBOX6.MDH.all = 0;
    ECanaMboxes.MBOX6.MDL.all = 0;
    ECanaMboxes.MBOX7.MDH.all = 0;
    ECanaMboxes.MBOX7.MDL.all = 0;
    ECanaMboxes.MBOX8.MDH.all = 0;
    ECanaMboxes.MBOX8.MDL.all = 0;
    ECanaMboxes.MBOX9.MDH.all = 0;
    ECanaMboxes.MBOX9.MDL.all = 0;
    ECanaMboxes.MBOX10.MDH.all = 0;
    ECanaMboxes.MBOX10.MDL.all = 0;
    ECanaMboxes.MBOX11.MDH.all = 0;
    ECanaMboxes.MBOX11.MDL.all = 0;
    ECanaMboxes.MBOX12.MDH.all = 0;
    ECanaMboxes.MBOX12.MDL.all = 0;
    ECanaMboxes.MBOX13.MDH.all = 0;
    ECanaMboxes.MBOX13.MDL.all = 0;
    ECanaMboxes.MBOX14.MDH.all = 0;
    ECanaMboxes.MBOX14.MDL.all = 0;
    ECanaMboxes.MBOX15.MDH.all = 0;
    ECanaMboxes.MBOX15.MDL.all = 0;
    ECanaMboxes.MBOX16.MDH.all = 0;
    ECanaMboxes.MBOX16.MDL.all = 0;
    ECanaMboxes.MBOX17.MDH.all = 0;
    ECanaMboxes.MBOX17.MDL.all = 0;
    ECanaMboxes.MBOX18.MDH.all = 0;
    ECanaMboxes.MBOX18.MDL.all = 0;
    ECanaMboxes.MBOX19.MDH.all = 0;
    ECanaMboxes.MBOX19.MDL.all = 0;
    ECanaMboxes.MBOX20.MDH.all = 0;
    ECanaMboxes.MBOX20.MDL.all = 0;
    ECanaMboxes.MBOX21.MDH.all = 0;
    ECanaMboxes.MBOX21.MDL.all = 0;
    ECanaMboxes.MBOX22.MDH.all = 0;
    ECanaMboxes.MBOX22.MDL.all = 0;
    ECanaMboxes.MBOX23.MDH.all = 0;
    ECanaMboxes.MBOX23.MDL.all = 0;
    ECanaMboxes.MBOX24.MDH.all = 0;
    ECanaMboxes.MBOX24.MDL.all = 0;
    ECanaMboxes.MBOX25.MDH.all = 0;
    ECanaMboxes.MBOX25.MDL.all = 0;
    ECanaMboxes.MBOX26.MDH.all = 0;
    ECanaMboxes.MBOX26.MDL.all = 0;
    ECanaMboxes.MBOX27.MDH.all = 0;
    ECanaMboxes.MBOX27.MDL.all = 0;
    ECanaMboxes.MBOX28.MDH.all = 0;
    ECanaMboxes.MBOX28.MDL.all = 0;
    ECanaMboxes.MBOX29.MDH.all = 0;
    ECanaMboxes.MBOX30.MDL.all = 0;
    ECanaMboxes.MBOX30.MDH.all = 0;
    ECanaMboxes.MBOX31.MDL.all = 0;
    ECanaMboxes.MBOX31.MDH.all = 0;
}

char FillCAN(unsigned int Mbox)
{
	//todo USER: setup for all transmit MBOXs
	struct ECAN_REGS ECanaShadow;
	ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	switch (Mbox)								//choose mailbox
	{
	case HEARTBEAT_BOX:
		//todo Nathan define heartbeat
		EALLOW;
		ECanaShadow.CANMC.bit.MBNR = Mbox;
		ECanaShadow.CANMC.bit.CDR = 1;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		ECanaMboxes.MBOX1.MDH.all = 0;
		ECanaMboxes.MBOX1.MDL.all = 0;
		ECanaMboxes.MBOX1.MDL.word.LOW_WORD = ops.Flags.all;
		ECanaShadow.CANMC.bit.MBNR = 0;
		ECanaShadow.CANMC.bit.CDR = 0;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		EDIS;
		return 1;
	case CURRENT_TIME_BOX:
		EALLOW;
		ECanaShadow.CANMC.bit.MBNR = Mbox;
		ECanaShadow.CANMC.bit.CDR = 1;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		ECanaMboxes.MBOX3.MDL.word.LOW_WORD = GPS.gps_info.utc.year + 2000;
		ECanaMboxes.MBOX3.MDL.byte.BYTE1 = GPS.gps_info.utc.mon;
		ECanaMboxes.MBOX3.MDL.byte.BYTE0 = GPS.gps_info.utc.day;
		ECanaMboxes.MBOX3.MDH.byte.BYTE7 = GPS.gps_info.utc.hour;
		ECanaMboxes.MBOX3.MDH.byte.BYTE5 = GPS.gps_info.utc.sec;
		ECanaMboxes.MBOX3.MDH.byte.BYTE6 = GPS.gps_info.utc.min;
		ECanaShadow.CANMC.bit.CDR = 0;
		ECanaShadow.CANMC.bit.MBNR = 0;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		EDIS;
		return 1;
	case ALT_ACCR_BOX:
		EALLOW;
		ECanaShadow.CANMC.bit.MBNR = Mbox;
		ECanaShadow.CANMC.bit.CDR = 1;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		ECanaMboxes.MBOX4.MDH.all = GPS.gps_info.elv.U32;
		ECanaMboxes.MBOX4.MDL.all = GPS.gps_info.PDOP.U32;
		ECanaShadow.CANMC.bit.CDR = 0;
		ECanaShadow.CANMC.bit.MBNR = 0;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		EDIS;
		return 1;
	case LAT_VAL_BOX:
		EALLOW;
		ECanaShadow.CANMC.bit.MBNR = Mbox;
		ECanaShadow.CANMC.bit.CDR = 1;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		ECanaMboxes.MBOX5.MDL.all = GPS.gps_info.lat.U32;
		if (GPS.gps_info.sig > 0)
		{
			ECanaMboxes.MBOX5.MDH.byte.BYTE7 = 1;
		}
		else
		{
			ECanaMboxes.MBOX5.MDH.byte.BYTE7 = 0;
		}
		ECanaShadow.CANMC.bit.CDR = 0;
		ECanaShadow.CANMC.bit.MBNR = 0;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		EDIS;
		return 1;
	case LONG_BOX:
		EALLOW;
		ECanaShadow.CANMC.bit.MBNR = Mbox;
		ECanaShadow.CANMC.bit.CDR = 1;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		ECanaMboxes.MBOX6.MDL.all = GPS.gps_info.lon.U32;
		ECanaShadow.CANMC.bit.CDR = 0;
		ECanaShadow.CANMC.bit.MBNR = 0;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		EDIS;
		return 1;
	case XAxisIMU_BOX:
		EALLOW;
		ECanaShadow.CANMC.bit.MBNR = Mbox;
		ECanaShadow.CANMC.bit.CDR = 1;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		ECanaMboxes.MBOX7.MDL.all = data.ypr[2].U32;
		ECanaMboxes.MBOX7.MDH.all = data.ax.U32;
		ECanaShadow.CANMC.bit.CDR = 0;
		ECanaShadow.CANMC.bit.MBNR = 0;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		EDIS;
		return 1;
	case YAxisIMU_BOX:
		EALLOW;
		ECanaShadow.CANMC.bit.MBNR = Mbox;
		ECanaShadow.CANMC.bit.CDR = 1;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		ECanaMboxes.MBOX8.MDL.all = data.ypr[1].U32;
		ECanaMboxes.MBOX8.MDH.all= data.ay.U32;
		ECanaShadow.CANMC.bit.CDR = 0;
		ECanaShadow.CANMC.bit.MBNR = 0;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		return 1;
	case ZAxisIMU_BOX:
		EALLOW;
		ECanaShadow.CANMC.bit.MBNR = Mbox;
		ECanaShadow.CANMC.bit.CDR = 1;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		ECanaMboxes.MBOX9.MDL.all = data.ypr[0].U32;
		ECanaMboxes.MBOX9.MDH.all = data.az.U32;
		ECanaShadow.CANMC.bit.CDR = 0;
		ECanaShadow.CANMC.bit.MBNR = 0;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		return 1;
	case GPS_SPEED_BOX:
		EALLOW;
		ECanaShadow.CANMC.bit.MBNR = Mbox;
		ECanaShadow.CANMC.bit.CDR = 1;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		ECanaMboxes.MBOX18.MDL.all = GPS.gps_info.speed.U32;
		ECanaShadow.CANMC.bit.MBNR = 0;
		ECanaShadow.CANMC.bit.CDR = 0;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		EDIS;
		return 1;
	case 10: //testing
		//todo Nathan define heartbeat
		EALLOW;
		ECanaShadow.CANMC.bit.MBNR = Mbox;
		ECanaShadow.CANMC.bit.CDR = 1;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		ECanaMboxes.MBOX10.MDH.all = 0;
		ECanaMboxes.MBOX10.MDL.all = 0;
		ECanaMboxes.MBOX10.MDL.all = s1.U32;
		ECanaShadow.CANMC.bit.MBNR = 0;
		ECanaShadow.CANMC.bit.CDR = 0;
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		EDIS;
		return 1;
	}
	return 0;
}

void FillSendCAN(unsigned Mbox)
{
	if (FillCAN(Mbox) == 1)
	{
		SendCAN(Mbox);
	}
}

void SendCAN(unsigned int Mbox)
{
	mask = 1UL << Mbox;
	ECanaRegs.CANTRS.all = mask;

	ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	if (ECanaShadow.CANMC.bit.CCR == 1)
	{
		BUS_OFF();
	}
	//todo Nathan: calibrate sendcan stopwatch
	StopWatchRestart(can_watch);

	do{ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;}
	while(((ECanaShadow.CANTA.all & mask) != mask) && (isStopWatchComplete(can_watch) == 0)); //wait to send or hit stop watch

	ECanaRegs.CANTA.all = mask;						//clear flag
	if (isStopWatchComplete(can_watch) == 1)					//if stopwatch flag
	{
		ops.Flags.bit.can_error = 1;
	}
	else if (ops.Flags.bit.can_error == 1)		//if no stopwatch and flagged reset
	{
		ops.Flags.bit.can_error = 0;
	}
}


void FillCANData()
{
	//todo USER: use FillCAN to put data into correct mailboxes
//	FillCAN(CURRENT_TIME_BOX);
//	FillCAN(ALT_ACCR_BOX);
//	FillCAN(LAT_VAL_BOX	);
//	FillCAN(LONG_BOX);
	FillCAN(XAxisIMU_BOX);
	FillCAN(YAxisIMU_BOX);
	FillCAN(ZAxisIMU_BOX);
	FillCAN(10);
//	FillCAN(GPS_SPEED_BOX);
}

// INT9.6
__interrupt void ECAN1INTA_ISR(void)  // eCAN-A
{
	Uint32 ops_id;
	Uint32 dummy;
  	unsigned int mailbox_nr;
  	ECanaShadow.CANGIF1.bit.MIV1 =  ECanaRegs.CANGIF1.bit.MIV1;
  	mailbox_nr = ECanaShadow.CANGIF1.bit.MIV1;
  	//todo USER: Setup ops command
  	if(mailbox_nr == COMMAND_BOX)
  	{
  		//todo Nathan: Define Command frame
  		//proposed:
  		//HIGH 4 BYTES = Uint32 ID
  		//LOW 4 BYTES = Uint32 change to
  		ops_id = ECanaMboxes.MBOX0.MDH.all;
  		dummy = ECanaMboxes.MBOX0.MDL.all;
		switch (ops_id)
		{
		case OPS_ID_STATE:
			memcpy(&ops.State,&dummy,sizeof ops.State);
			ops.Change.bit.State = 1;
			break;
		case OPS_ID_STOPWATCHERROR:
			memcpy(&ops.Flags.all,&dummy,sizeof ops.Flags.all);
			ops.Change.bit.Flags = 1;
			break;
		}
		ECanaRegs.CANRMP.bit.RMP0 = 1;
  	}
  	//todo USER: Setup other reads

  	//To receive more interrupts from this PIE group, acknowledge this interrupt
  	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

