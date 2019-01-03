/*
 * direction.c
 *
 *  Created on: 2018. 6. 28.
 *      Author: 최호준
 */



#include "Direction_RPM.h"
#include "School.h"

#include "Tricore/Cpu/Std/Ifx_Types.h"
#include "Tricore/Cpu/Std/IfxCpu_Intrinsics.h"
#include "Tricore/Scu/Std/IfxScuWdt.h"
#include <Stm/Std/IfxStm.h>
#include <Port/Std/IfxPort.h>
#include "Test_Irq.h"
#include "Test_Pwm.h"
#include "Test_Adc.h"
#include "IfxPort_reg.h"
#include "IfxPort.h"
#include "Math.h"



#include "Bsp/Bsp.h"
#include "IfxAsclin_Asc.h"

#include "Test_Irq.h"
#include "Test_ModuleInit.h"


#include "Gpt12/Std/IfxGpt12.h"

#define	FORWARD			0
#define	BACKWARD		1

// 서보 모터 게인 값
#define     P_Gain      1.3
#define     D_Gain      5


//// DC 모터 게인 값

///////////////////////////////////////////////////////////////////////
//						서보모터 변수들									//
volatile int STEER_MID = 650U;
volatile int STEER_LEFT = 740U;
volatile int STEER_RIGHT = 560U;

// 차 중앙 기준 검은색 라인 인덱스
volatile int LEFT_CAM_ORG_blacklineindex = 60;
volatile int RIGHT_CAM_ORG_blacklineindex = 60;

// 차 중앙 기준 중앙선 라인 인
volatile int LEFT_CAM_MID_index = 100;
volatile int RIGHT_CAM_MID_index = 20;

volatile int LEFT_DIF = 0;
volatile int RIGHT_DIF = 0;

//volatile int DIF_AVG = 0;
volatile double duty_servo = 0;


volatile int Current_Left_Pos = 0;
volatile int Current_Right_Pos = 0;
/////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////
//   							블랙 라인 디텍팅								//
volatile int Left_Detected_Pos = 0;
volatile int Right_Detected_Pos = 0;


volatile int Edge_Count = 0;

uint32 difference_norm = 30;

volatile int Error_Past = 0, Error_Present = 0, Error_PaPre=0;




volatile int pwm = 0;
volatile int dir = CCW;
// 서보 조향 함수
void POS_MID(void)
{

	//참고
	//STEER_MID = 650U;
	//STEER_LEFT = 740U;    왼쪽으로는 더해줘야함
	//STEER_RIGHT = 560U;   오른쪽으로는 빼줘야함


	LEFT_DIF  =  LEFT_CAM_ORG_blacklineindex  - Left_Detected_Pos;	//60 -
	RIGHT_DIF =  RIGHT_CAM_ORG_blacklineindex - Right_Detected_Pos; //60 -

	Error_Present = LEFT_DIF + RIGHT_DIF;
	Error_PaPre = Error_Past - Error_Present;
	// 400 - * 1
	// 500 -> * 0.7(부족) -> 0.8
	//350 -> 0.8 ( 크다 ) 0.7 이상함 (작다)
	//0.77 350
	//400 0.83
	//450 0.9
	//500 0.97 괜춘 급출발 발생
	//470 0.93 구
	//500 -> 350 -> 200 (후보)

	//470(7.71V -> 0.89)
	//470(8.4V -> 잘됨 ( 0.93 ) 4번 돌렸더니 8.4V -> 8.02V
	duty_servo = STEER_MID - (Error_Present) * P_Gain * 1.2 + (Error_Past - Error_Present) * D_Gain;

	if(duty_servo < 560)
		duty_servo = 560;

	if(duty_servo > 760)
		duty_servo = 760;

	Pwm_ServoDutyUpdate(duty_servo);

	Error_Past = Error_Present;

}

void POS_MID_School(void)
{
	//참고
	//STEER_MID = 650U;
	//STEER_LEFT = 740U;    왼쪽으로는 더해줘야함
	//STEER_RIGHT = 560U;   오른쪽으로는 빼줘야함

	LEFT_DIF  =  LEFT_CAM_ORG_blacklineindex  - Left_Detected_Pos;	//60 -
	RIGHT_DIF =  RIGHT_CAM_ORG_blacklineindex - Right_Detected_Pos; //60 -

	if(Dotted_Line_Pos == 1) // 오른쪽에 점선이 있을때
	{
		RIGHT_DIF = 0; // 오른쪽에 점선이 있을때
	}

	else if(Dotted_Line_Pos == 0) // 왼쪽에 점선이 있을때
	{
		LEFT_DIF = 0;
	}
	//Dotted_Line_Pos = 0,1	// 0 - 왼쪽, 1- 오른쪽
	//왼쪽이면 더 해주고 //오른쪽이면 오른쪽으로 더 해주고
	//if()
	// 2.8 -> 10(실패)
	Error_Present = LEFT_DIF + RIGHT_DIF;
	Error_PaPre = Error_Past - Error_Present;
	//2.7
	// 연습주행 2.8
	duty_servo = STEER_MID - (Error_Present) * P_Gain * 5 + (Error_Past - Error_Present) * D_Gain;

	if(duty_servo < 560)
		duty_servo = 560;

	if(duty_servo > 760)
		duty_servo = 760;

	Pwm_ServoDutyUpdate(duty_servo);

	Error_Past = Error_Present;
}


void Black_Line_Detect(void)
{
	//Nomal_Result[][], Line_Result[][], LEFT_CAM_MID_index , Current_Left_Pos, LEFT_CAM_ORG_blacklineindex

	int j;

			for (j = 18; j <= LEFT_CAM_MID_index; j++) //좌측 카메라 중앙선 기준 좌측 값만 활용
			{
				//falling edge
				if (abs((Nomal_Result[0][j-4] - Nomal_Result[0][j])) > difference_norm || abs((Nomal_Result[0][j+4] - Nomal_Result[0][j])) > difference_norm)
				{
					Left_Detected_Pos = j;
					//Vect_Flag1 = 0;
					Edge_Count = 1;
				}
				if(Edge_Count == 0)
				{
					Left_Detected_Pos = 14;
					Right_Detected_Pos = RIGHT_CAM_MID_index;
				}
			}
			Edge_Count = 0;
			for (j = 100; j >= RIGHT_CAM_MID_index ; j--) //우측 카메라 중앙선 기준 우측 값만 활용
			{
				if (abs((Nomal_Result[1][j-4] - Nomal_Result[1][j])) > difference_norm || abs((Nomal_Result[1][j+4] - Nomal_Result[1][j])) > difference_norm)
				{
					Right_Detected_Pos = j;
					Edge_Count = 1;
				}
				if(Edge_Count == 0)
				{
					Left_Detected_Pos = LEFT_CAM_MID_index;
					Right_Detected_Pos = 100;

				}
			}
			Edge_Count = 0;
			//if ((abs(Nomal_Result[1][j] - Nomal_Result[1][j-3]) > difference_norm) || (abs(Nomal_Result[1][j] - Nomal_Result[1][j-4]) > difference_norm))
											//rising edge
}
void Obstacle_Stop_OR_Avoid(int School_Flag)
{
	if(School_Flag == 0)	// 스쿨존 아웃 상태    ->  AEB(급정지)
	{

	}
	if(School_Flag == 1)	// 스쿨존 인 상태	->  장애물 회피
	{

	}
}




