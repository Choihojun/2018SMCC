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
#include "Bsp/Bsp.h"
#include "IfxAsclin_Asc.h"

#include "Test_Irq.h"
#include "Test_ModuleInit.h"
#include "Test_Pwm.h"
#include "Test_Uart.h"
#include "Test_Adc.h"
#include "Direction_RPM.h"
#include "School.h"
#include "Math.h"

#include "Gpt12/Std/IfxGpt12.h"


// normally shorter signal is left longer signal is right
// maybe reverse
#define STEER_MID		650U
#define STEER_LEFT		740U
#define STEER_RIGHT		560U

#define	_ON				0
#define	_OFF			1

#define _LEFT			0
#define _RIGHT			1

#define	FORWARD			0
#define	BACKWARD		1

#define	kp				4.8
#define	ki				0.002
#define	kd				0.7

//********   motor_control     *********//

volatile sint32	spd_pulse32 = 0;
volatile uint16	pulse_k = 0;
volatile uint16	pulse_k_1 = 0;
volatile uint16	speed = 0;
int wdir = 0;
sint32 spd_err_k = 0;
sint32 spd_err_k_1 = 0;
sint32 dspd_err = 0;
sint32 sum_spd_err = 0;

volatile uint32 PurPose_Pulse = 200;

float32 up = 0;
float32 ui = 0;
float32 ud = 0;
float32 usum = 0;
float32	u_in = 0;

// ************************************ //

//********   Acceleration     *********//
#define MAX_SPEED       800
#define GP				4.0

volatile uint16 accel_speed =0;
// ************************************ //


////----------------------------------
volatile	uint8	time_flag = 0;  // 20ms 제어주기를 위한 변수
////----------------------------------

volatile uint16 inittest = 0;
volatile uint16 initStartCount = 0;

//extern void Uart_TEST(void);
uint16	steer_val = 360;
uint16	steer_dir = 0;

volatile int Turn_Flag=0;
volatile int Time_Count_Obstacle_First = 0, First_Flag = 0;
volatile int Time_Count_Obstacle_Second = 0, Second_Flag = 0;
volatile int Time_Count_Obstacle_Third = 0, Third_Flag = 0;


volatile int i = 0, j=0, flag = 0 , Final_Flag = 0;

volatile int IR[6] = {0,};
//state----------------------
uint16 state = 0;
uint16 school_flag = 0;
volatile uint16 pose_pulse = 0;
volatile uint16 pose_pulse_l = 0;
volatile uint16 pose;
double sum_linecam_r = 0;
double sum_linecam_l = 0;
double sum_linecam = 0;
uint16 IR_flag = 0;
//---------------------------


void Motor_control(sint32 ref_spd_pulse32)
{
	spd_err_k = ref_spd_pulse32 - spd_pulse32;
	dspd_err = spd_err_k - spd_err_k_1;
	sum_spd_err += spd_err_k;

	up = kp * (float32)spd_err_k;
	ui = ki * (float32)sum_spd_err;
	ud = kd * (float32)dspd_err;
	usum = up + ui + ud;

	spd_err_k_1 = spd_err_k;

	if(usum < 0){
		//u_in = usum * (-1);
		u_in = 0;
		sum_spd_err = 0;
		wdir = BACKWARD;
	}
	else{
		u_in = usum;
		wdir = FORWARD;
	}

	if(u_in > 2000){
		u_in = 2000;
	}
	speed = (uint16)u_in;
	u_in = 0;

	Pwm_MotorDutyAndDirectionControl(speed, 0);
}

void Acceleration_Motor_control(double servo){

   //STEER_MID = 650U;
   //STEER_LEFT = 740U;    왼쪽으로는 더해줘야함
   //STEER_RIGHT = 560U;   오른쪽으로는 빼줘야함

   if(servo < 630 || servo > 670)
   {
      accel_speed = MAX_SPEED - GP * abs(servo - 650);
   }

   else
   {
      accel_speed = MAX_SPEED;
   }

   Motor_control(accel_speed);

}
void IR_Digital(void)
{
	// P41.3(AN15)
	// P41.4(AN16)
	// P40.11 (AN11)
	// P41.5 (AN17)
	// P41.8 (AN20)
	// P41.9 (AN21)
	IR[0] = IfxPort_getPinState(&MODULE_P41,3); //AN15 A/D2
	IR[1] = IfxPort_getPinState(&MODULE_P41,4); //AN16 A/D2
	IR[2] = IfxPort_getPinState(&MODULE_P40,11); //AN11 A/D1
	IR[3] = IfxPort_getPinState(&MODULE_P41,5); // AN17 A/D1
	IR[4] = IfxPort_getPinState(&MODULE_P41,8); // AN20 A/D1
	IR[5] = IfxPort_getPinState(&MODULE_P41,9); // AN21 A/D1
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

    initTime();

    P13_OUT.B.P2 = 0;
    P13_OUT.B.P1 = 0;
    P13_OUT.B.P0 = 0;
    //Delay_us(3000000);
	waitTime(5 * TimeConst_1us);

    P13_OUT.B.P2 = 1;
    P13_OUT.B.P1 = 1;
    P13_OUT.B.P0 = 1;

    Gpt12_init();

    // 스쿨존 속도 범위 -> 167 ~ 235 (지우지 마시오) -> 200
    // 평소 주행 속도 -> 400 예상
    // 중요 ************** Don't use the Delay_us() function.
    //main cycle 0.0025ms
    //360~840U 제한
    //button_function
  //  test_pin = IfxPort_getPinState(&MODULE_P14,5);
	//STEER_MID = 650U;
	//STEER_LEFT = 740U;    왼쪽으로는 더해줘야함
	//STEER_RIGHT = 560U;   오른쪽으로는 빼줘야함
    Pwm_HEADDutyUpdate(800);
    Pwm_ServoDutyUpdate(STEER_MID);
    //PurPose_Pulse = 500;
    while (1)
    {

    	if(time_flag){

    		TestLineScan();
			Average_Filter();
			Min_Extract();
			PSD_Sensor();
			sum_linecam = 0;
			sum_linecam_r = 0;
			sum_linecam_l = 0;

			//Pwm_BuzzerDutyUpdate(100U);
			//Dotted_Line_Pos_Detect();
			if(Got_it == 0 && School_Flag == 1)
			{
				Dotted_Line_Pos_Detect();
			}


			for (i = 95 ; i > 25 ; i--) // 왼쪽
			{
				sum_linecam_l += Sig_Result[0][i]/10;
				sum_linecam_r += Sig_Result[1][i]/10;
				//sum_linecam = sum_linecam/140;
			}
			if(sum_linecam_l < sum_linecam_r) sum_linecam = sum_linecam_l;
			else sum_linecam = sum_linecam_r;

    	   	time_flag = 0;
    	   	pulse_k = IfxGpt12_T4_getTimerValue(&MODULE_GPT120);
    	    spd_pulse32 = (sint32)pulse_k - (sint32)pulse_k_1;
    	   	pulse_k_1 = pulse_k;
    	   //	Motor_control(0);

    	   	Motor_control(PurPose_Pulse);
    	   	// 검은색 12 ~ 15 ( 밝은 곳 )
    	   	// 하얀색 27 ~ 33 ( 밝은 곳 )

    	   	// 검은색 0 ~ 12
    	   	// 하얀색


    	   	// 바로 3으로 가는 문제 그리고 왼쪽 점선이라고 인식했을때 탈선하는 문제 (직진을함)
    	   	// 언덕 최대점에서 멈췄다가 내려올때 가속해서 나가리 되는 문제 (언덕이라는걸 알아야함)
    	   	// IR 하나만 이용해서 그런지 스쿨존 인식이 덜되는 건지 그리고 카메라 평균이 틀린 문제
    	   	// 스쿨존 점선인식 문제 ...
    	   	// 어디가 4095 나오는지 체킹을 못함.
    	   	// 11000~ 12000 글서 sum_linecam = 13000으로 하면 state = 3으로는 못 넘어감.

    	    //7/11 18000 test
    	    switch(state){
			case 0:				//라인트레이싱
				PurPose_Pulse = 510;
				POS_MID();

				P13_OUT.B.P1 = 1;
				P13_OUT.B.P0 = 1;
				P13_OUT.B.P2 = 1;
				if(Distance < 50 && Distance > 15)	state = 1;
				//if(sum_linecam < 20000){ // 13000 - 임의의값(스쿨존 인식이 잘 안됨
				if(sum_linecam < 12000){
					state = 2;
					IR_flag = 0;
				}
				break;
			case 1:				//AEB
				if(Distance < 50 && Distance > 15){

					PurPose_Pulse = 0;
					Pwm_MotorDutyAndDirectionControl(0, 0);
					Pwm_BuzzerDutyUpdate(100U);
					}
				else {
					 state = 0;
					 Pwm_BuzzerDutyUpdate(0);
					 }
				break;
			case 2:
				pose_pulse = pulse_k;
				duty_servo = 650U - duty_servo;
				duty_servo = 650U + duty_servo;
				Pwm_ServoDutyUpdate(duty_servo);
				PurPose_Pulse = 200;
				state = 3;
				//Pwm_MotorDutyAndDirectionControl(0, 0);
				break;
			case 3:				//스쿨존 판단
				IR_Sensor();
				pose = pulse_k - pose_pulse;
				if(pose > 1000) Pwm_ServoDutyUpdate(650U);
				//else POS_MID();
				//if(IR_1 > 3000 || IR_2 > 3000) IR_flag = 1; // 더 낮춰보자
				if(ir_val) IR_flag = 1;
				//if(IR_2 > 2500) IR_flag = 1;
				if(sum_linecam >= 12000){
					if(pose> 800 && IR_flag == 1){
						if(School_Flag == 0){
							School_Flag = 1;
							state = 4;
						}
						else{ state = 0;
							  School_Flag = 0;}
					}
					else{
						if(School_Flag == 0)
						{state = 0;
						P13_OUT.B.P1 = 1;
						P13_OUT.B.P0 = 1;
						P13_OUT.B.P2 = 1;
						}
						else state = 5;
					}
					IR_flag = 0;
				}
				break;
			case 4:
				//P13_OUT.B.P0 = 0;
				//Dotted_Line_Pos_Detect();
				P13_OUT.B.P1 = 0;
				if(Got_it == 1){
					state = 5;
					PurPose_Pulse = 200;
			    }
				else POS_MID_School();
				break;
			case 5:
				//P13_OUT.B.P0 = 0;
				Plus_Lane_Detect_Count_R = 0;
				Plus_Lane_Detect_Count_L = 0;
				Plus_Lane_Detect();
				POS_MID_School();
				Pwm_BuzzerDutyUpdate(0);
				PurPose_Pulse = 200;
				if(Distance < 60 && Distance > 15) 		state = 6;
				if(sum_linecam < 12000){
					state = 2;
					IR_flag = 0;
				}

				if(Plus_Lane_Detect_Count_R > 20 || Plus_Lane_Detect_Count_L > 20){ // 왼쪽으로 적당히 가야함
					state = 9;
				}
				break;
			case 6:
				pose_pulse_l = pulse_k;
				if(Dotted_Line_Pos == _LEFT)  state = 7;
				else						  state = 8;
				break;
			case 7:
				pose = (pulse_k - pose_pulse_l);
				Pwm_BuzzerDutyUpdate(150U);
				Dotted_Line_Pos = _RIGHT;
				P13_OUT.B.P0 = 1;
				P13_OUT.B.P2 = 0;
				if(pose < 12000U) Pwm_ServoDutyUpdate(560U);
				else if(pose >= 12000U && pose < 23000U)
				{
					Pwm_ServoDutyUpdate(740U);

				}
				else state = 5;
				break;
			case 8:
				pose = (pulse_k - pose_pulse_l);
				Pwm_BuzzerDutyUpdate(150U);
				Dotted_Line_Pos = _LEFT;
				P13_OUT.B.P0 = 0;
				P13_OUT.B.P2 = 1;
				if(pose < 15500U) Pwm_ServoDutyUpdate(740U);
				else if((pose >= 15500U) && pose < 26000U )
				{
					Pwm_ServoDutyUpdate(560U);
				}
				else state = 5;
				break;
			case 9:
				pose_pulse_l = pulse_k;
				state = 10;
				break;
			case 10:
				pose = pulse_k - pose_pulse_l;
				if(pose < 7000U && Dotted_Line_Pos == _LEFT){
					Pwm_ServoDutyUpdate(560);
				}
				else if(pose > 7000U && Dotted_Line_Pos == _LEFT){
					Dotted_Line_Pos = _RIGHT;
					state = 5;
				}

				else if(Dotted_Line_Pos == _RIGHT && pose < 7000U){ // 오른쪽으로 적당히 가야함
					Pwm_ServoDutyUpdate(740U);

				}

				else if(pose > 7000U && Dotted_Line_Pos == _RIGHT){
					Dotted_Line_Pos = _LEFT;
					state = 5;
				}

				break;
			}
    	}


    }

}

//짜야할것 - > 최외각 인덱스값만 체킹하는 것


void Gpt12_init(void)
{
	/* Initialize global clocks */
	/* FIXME Global prescaller should not be set by the driver as they are global resources */

	IfxGpt12_enableModule(&MODULE_GPT120);

	// Gpt1 block clock 설정
	IfxGpt12_setGpt1BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt1BlockPrescaler_8);
	// Gpt2 block clock 설정
	IfxGpt12_setGpt2BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt2BlockPrescaler_4);

	IfxGpt12_initTxInPin(&IfxGpt120_T4INA_P02_8_IN, IfxPort_InputMode_noPullDevice);
	IfxGpt12_T4_setTimerValue(&MODULE_GPT120, 0);

	IfxGpt12_T4_setMode(&MODULE_GPT120, IfxGpt12_Mode_counter);
	IfxGpt12_T4_setDirectionSource(&MODULE_GPT120, IfxGpt12_TimerDirectionSource_internal);

	//IfxGpt12_T4_setIncrementalInterfaceInputMode(&MODULE_GPT120, IfxGpt12_IncrementalInterfaceInputMode_bothEdgesTxIN);

	IfxGpt12_T4_setTimerDirection(&MODULE_GPT120, IfxGpt12_TimerDirection_up);
	IfxGpt12_T4_setCounterInputMode(&MODULE_GPT120, IfxGpt12_CounterInputMode_bothEdgesTxIN);
	//xGpt12_T4_enableOutput(&MODULE_GPT120, 1);
	IfxGpt12_T4_run(&MODULE_GPT120, IfxGpt12_TimerRun_start);

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
	//stmCompareConfig.ticks = 100000000;
	stmCompareConfig.ticks = 2000000;
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
    IfxStm_updateCompare(stm, IfxStm_Comparator_0, IfxStm_getLower(stm) + 2000000);


    //P13_OUT.B.P3 = !P13_OUT.B.P3;

    initStartCount++;
    time_flag = 1; // 제어주기를 위한 주기 ( 0 <-> 1 )

    if(School_Flag == 1 )
    {
    	Time_Count_School_Out++;
    	Time_Count_School_In = 0;

    }
    if(School_Flag == 0)
    {
    	Time_Count_School_In++;
        Time_Count_School_Out = 0;
    }

    // *************** Obstacle Avoiding  ******************//
    if(First_Flag == 0 && Turn_Flag == 1)
    {
        Time_Count_Obstacle_First++;
    }
    if(Second_Flag == 1)
    {
        Time_Count_Obstacle_Second++;
    }
    if(Third_Flag == 1)
    {
        Time_Count_Obstacle_Third++;
    }
    /////////////////////////////////////////////////////////

    if(Time_School_Zone_Flag == 1)
    {
    	Time_Count_School_Zone++;
    }

   // Time_Count_School_Zone, Time_School_Zone_Flag
    //Time_Count_Obstacle_First = 0, First_Flag = 0;
    //Time_Count_Obstacle_Second = 0, Second_Flag = 0;
    //Time_Count_Obstacle_Third = 0, Third_Flag = 0;
    __enable();

}
