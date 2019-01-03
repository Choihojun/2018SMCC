/*
 * School.c
 *
 *  Created on: 2018. 6. 29.
 *      Author: 최호준
 */


#include "School.h"
#include "Test_Pwm.h"
#include "Direction_RPM.h"

#include <stdio.h>
#include "Test_Adc.h"
#include "Bsp/Bsp.h"
#include "IfxPort.h"

#include "Ifx_Cfg.h"                   /* Do not remove this include */
#include "Cpu/Std/IfxCpu_Intrinsics.h" /* Do not remove this include */
#include "_Utilities/Ifx_Assert.h"     /* Do not remove this include */

#include "Cpu/Std/IfxCpu.h"
#include "Port/Std/IfxPort.h"
#include "Src/Std/IfxSrc.h"
#include "Stm/Std/IfxStm.h"
#include "Scu/Std/IfxScuCcu.h"
#include "math.h"
#define STEER_MID		650U
#define STEER_LEFT		740U
#define STEER_RIGHT		560U


// 스쿨존 진입과 아웃을 위한 변수
volatile int School_Flag = 0; //0 -> 1(스쿨존 진입)    1 -> 0 (스쿨존 탈출)    AEB - 급정지(스쿨존 아웃)    장애물 회피 (스쿨존 인)
volatile int Time_Count_School_In =0,Time_Count_School_Out = 0;

// 점선 라인 포지션 파악 (왼쪽인지 오른쪽인지) - (0 - 왼쪽) & (1 - 오른쪽)
volatile int Dotted_Line_Pos = 0;
volatile int Got_it = 0; // 점선라인 위치 파악을 한후 더이상 안해도 되는 상황을 나타내주는 플래그

volatile int Time_Count_School_Zone = 0, Time_School_Zone_Flag = 0;


volatile int IR_ADC_OFF_Flag = 0;  // 0 - ADC 작동, 1 - ADC 멈춤



void School_Zone_Detect(void)
{

	if( ( IR_1 > 2800 || IR_2 > 2800 ) && School_Flag == 0 && Time_Count_School_In > 50 )  // 진입
	{
			School_Flag = 1;
			Time_School_Zone_Flag = 1;
			//Pwm_ServoDutyUpdate(STEER_MID);
	}
	if(School_Flag == 1 && Time_Count_School_Out > 50 && (IR_1 > 2800 || IR_2 > 2800) )  // 진입
	{
			School_Flag = 0;
			Time_School_Zone_Flag = 1;
			//Pwm_ServoDutyUpdate(STEER_MID);
	}
}
//Time_Count_School_Zone, Time_School_Zone_Flag

void Dotted_Line_Pos_Detect(void)
{
	if(Left_Dotted_Line_Sum > 30 || Right_Dotted_Line_Sum > 30)
	{
		if(Left_Dotted_Line_Sum > Right_Dotted_Line_Sum)
		{
			Dotted_Line_Pos = 0;	// 왼쪽
			Left_Dotted_Line_Sum = 0;
			Right_Dotted_Line_Sum = 0;
			Got_it = 1;
			P13_OUT.B.P0 = 0;
			P13_OUT.B.P2 = 1;
		}


		else if (Left_Dotted_Line_Sum < Right_Dotted_Line_Sum)
		{
			Dotted_Line_Pos = 1;	// 오른쪽
			Left_Dotted_Line_Sum = 0;
			Right_Dotted_Line_Sum = 0;
			Got_it = 1;
			P13_OUT.B.P0 = 1;
			P13_OUT.B.P2 = 0;
		}

		else
			Got_it = 0;
		//else 같은 경우
	}
}
