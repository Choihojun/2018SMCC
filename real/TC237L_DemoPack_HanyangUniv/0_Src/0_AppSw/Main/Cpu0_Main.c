/**
 * \file Cpu0_Main.c
 * \brief Main function definition for Cpu core 0 .
 *
 * \copyright Copyright (c) 2012 Infineon Technologies AG. All rights reserved.
 *
 *
 *
 *                                 IMPORTANT NOTICE
 *
 *
 * Infineon Technologies AG (Infineon) is supplying this file for use
 * exclusively with Infineon's microcontroller products. This file can be freely
 * distributed within development tools that are supporting such microcontroller
 * products.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 */

#include "Tricore/Cpu/Std/Ifx_Types.h"
#include "Tricore/Cpu/Std/IfxCpu_Intrinsics.h"
#include "Tricore/Scu/Std/IfxScuWdt.h"
#include <Stm/Std/IfxStm.h>
#include <Port/Std/IfxPort.h>

#include "Test_Irq.h"
#include "Test_ModuleInit.h"
#include "Test_Pwm.h"
#include "Test_Uart.h"
#include "Test_Adc.h"


// normally shorter signal is left longer signal is right
// maybe reverse
#define STEER_MID		650U
#define STEER_LEFT		740U
#define STEER_RIGHT		560U

#define CW				0
#define CCW				1

volatile uint16 inittest = 0;

uint16 test8 = 0;
uint16 test9 = 0;
uint16 teststop = 0;

volatile uint16 initStartCount = 0;
extern void Uart_TEST(void);


// Make PWM using DIO on/off
void set_ServoAngle(uint16 value)
{
	int i;

	for(i=0;i<10;i++)
	{
		P33_OUT.B.P10 = 1;
		Delay_us(value);

		P33_OUT.B.P10 = 0;
		Delay_us(23000);
	}
}

void action_Steering_1(void)
{
	int i;
	for(i=0; i<540; i+=20)
	{
		P33_OUT.B.P10 = 1;
		Delay_us(1320 + i);
		P33_OUT.B.P10 = 0;
		Delay_us(23000);
	}
}

void action_Steering_2(void)
{
	int i;
	for(i=0; i<=270; i+=5)
	{
		P33_OUT.B.P10 = 1;
		Delay_us(1590 - i);
		P33_OUT.B.P10 = 0;
		Delay_us(23000);
	}
	for(i=0; i<=540; i+=10)
	{
		P33_OUT.B.P10 = 1;
		Delay_us(1320 + i);
		P33_OUT.B.P10 = 0;
		Delay_us(23000);
	}
	for(i=0; i<=270; i+=5)
	{
		P33_OUT.B.P10 = 1;
		Delay_us(1860 - i);
		P33_OUT.B.P10 = 0;
		Delay_us(23000);
	}
}

void action_CarRuning(uint16 Speed,uint16 Direction){

	Pwm_MotorDutyAndDirectionControl(Speed, Direction);
}

void action_CarStop(void)
{
	P02_OUT.B.P0 = 0;
	P00_OUT.B.P2 = 0;
	P02_OUT.B.P2 = 0;
	P33_OUT.B.P5 = 0;
}

void RcCarInit(void)
{
	uint16 i = 0;

	Uart_TEST();
	Test_VadcAutoScan(IfxVadc_GroupId_0);
	Test_VadcAutoScan(IfxVadc_GroupId_1);

P13_OUT.B.P0 = 0;

	Pwm_ServoDutyUpdate(STEER_MID);
	Delay_ms(500);

	Pwm_ServoDutyUpdate(STEER_LEFT);
	Delay_ms(500);

	Pwm_ServoDutyUpdate(STEER_MID);
	Delay_ms(500);

	Pwm_ServoDutyUpdate(STEER_RIGHT);
	Delay_ms(500);

	Pwm_ServoDutyUpdate(STEER_MID);
	Delay_ms(500);

P13_OUT.B.P1 = 0;

    for(i=0;i<1800;i+=18)
    {
    	action_CarRuning(i, CW);
		Delay_ms(4);
    }
    Delay_ms(1000);

P13_OUT.B.P2 = 0;

    for(i=1800;i>1;i-=18)
    {
    	action_CarRuning(i, CW);
		Delay_ms(4);
    }
    action_CarStop();
    Delay_ms(400);

P13_OUT.B.P2 = 1;
P13_OUT.B.P1 = 1;
P13_OUT.B.P0 = 1;
}

void core0_main (void)
{
    __enable ();
    /*
     * !!WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE!!
     * Enable the watchdog in the demo if it is required and also service the watchdog periodically
     * */
    IfxScuWdt_disableCpuWatchdog (IfxScuWdt_getCpuWatchdogPassword ());
    IfxScuWdt_disableSafetyWatchdog (IfxScuWdt_getSafetyWatchdogPassword ());

    Test_ModuleInit();


    while (1)
    {

    	if(inittest==1){
    		RcCarInit();

			inittest=0;
			initStartCount = 0;
    	}
    }

}

void SecondTimer_Initialization(void)
{
    volatile float       stm_freq;
    Ifx_STM             *stm = &MODULE_STM0;
    IfxStm_CompareConfig stmCompareConfig;

    /* suspend by debugger enabled */
    IfxStm_enableOcdsSuspend(stm);
    /* get current STM frequency : debug purpose only*/
    stm_freq = IfxScuCcu_getStmFrequency();
    /* constructor of configuration */
    IfxStm_initCompareConfig(&stmCompareConfig);
	stmCompareConfig.triggerPriority = ISR_PRIORITY_STM;
	stmCompareConfig.ticks = 100000000;
	stmCompareConfig.typeOfService = IfxSrc_Tos_cpu0;
    /* Now Compare functionality is initialized */
    IfxStm_initCompare(stm, &stmCompareConfig);

} // End of TaskScheduler_Initialization()

//*********************************************************************************************
// @Function	 	void UsrIsr_Stm_0(void)
// @Description   	STM0 Interrupt for system tick generation
// @Returnvalue		None
// @Parameters    	None
//*********************************************************************************************
IFX_INTERRUPT (SecondTimer_Isr, 0, ISR_PRIORITY_STM);
void SecondTimer_Isr(void)
{
    Ifx_STM *stm = &MODULE_STM0;

    /* Set next 1ms scheduler tick alarm */
    IfxStm_updateCompare(stm, IfxStm_Comparator_0, IfxStm_getLower(stm) + 100000000);


    P13_OUT.B.P3 = !P13_OUT.B.P3;

    initStartCount++;

    // every 4sec after
    if(initStartCount>5){

    	inittest = 1;
    	initStartCount = 0;

    }

    __enable();

}


