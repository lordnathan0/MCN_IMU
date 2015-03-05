/*
 * pwm_servo.c
 *
 *  Created on: Mar 2, 2015
 *      Author: Nathan
 */


#include "all.h"


void init_pwm_servo()
{

    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)

/* Configure EPWM-1 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPWM1 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B


	EPwm1Regs.TBPRD = 669; // Period = 601 TBCLK counts (20 ms)
	EPwm1Regs.CMPA.half.CMPA = 50; // Compare A = 350 TBCLK counts
	EPwm1Regs.CMPB = 50; // Compare B = 200 TBCLK counts
	EPwm1Regs.TBPHS.all = 0; // Set Phase register to zero
	EPwm1Regs.TBCTR = 0; // clear TB counter
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Phase loading disabled
	EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0x7; // TBCLK about 33.47 khz
	EPwm1Regs.TBCTL.bit.CLKDIV = 0x7;	//
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR = Zero
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR = Zero
	EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;
	EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;
	EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
}

void set_pwm(float servo1, float servo2)
{
	// servo percent
	EPwm1Regs.CMPA.half.CMPA = (int) (33 + 34*servo1);
	EPwm1Regs.CMPB = (int) (33 + 34*servo2);
}
