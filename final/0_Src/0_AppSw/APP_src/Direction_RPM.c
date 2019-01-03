/*
 * direction.c
 *
 *  Created on: 2018. 6. 28.
 *      Author: ��ȣ��
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

// ���� ���� ���� ��
#define     P_Gain      1.3
#define     D_Gain      5


//// DC ���� ���� ��

///////////////////////////////////////////////////////////////////////
//						�������� ������									//
volatile int STEER_MID = 650U;
volatile int STEER_LEFT = 740U;
volatile int STEER_RIGHT = 560U;

// �� �߾� ���� ������ ���� �ε���
volatile int LEFT_CAM_ORG_blacklineindex = 60;
volatile int RIGHT_CAM_ORG_blacklineindex = 60;

// �� �߾� ���� �߾Ӽ� ���� ��
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
//   							�� ���� ������								//
volatile int Left_Detected_Pos = 0;
volatile int Right_Detected_Pos = 0;


volatile int Edge_Count = 0;

uint32 difference_norm = 30;

volatile int Error_Past = 0, Error_Present = 0, Error_PaPre=0;




volatile int pwm = 0;
volatile int dir = CCW;
// ���� ���� �Լ�
void POS_MID(void)
{

	//����
	//STEER_MID = 650U;
	//STEER_LEFT = 740U;    �������δ� ���������
	//STEER_RIGHT = 560U;   ���������δ� �������


	LEFT_DIF  =  LEFT_CAM_ORG_blacklineindex  - Left_Detected_Pos;	//60 -
	RIGHT_DIF =  RIGHT_CAM_ORG_blacklineindex - Right_Detected_Pos; //60 -

	Error_Present = LEFT_DIF + RIGHT_DIF;
	Error_PaPre = Error_Past - Error_Present;
	// 400 - * 1
	// 500 -> * 0.7(����) -> 0.8
	//350 -> 0.8 ( ũ�� ) 0.7 �̻��� (�۴�)
	//0.77 350
	//400 0.83
	//450 0.9
	//500 0.97 ���� ����� �߻�
	//470 0.93 ��
	//500 -> 350 -> 200 (�ĺ�)

	//470(7.71V -> 0.89)
	//470(8.4V -> �ߵ� ( 0.93 ) 4�� ���ȴ��� 8.4V -> 8.02V
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
	//����
	//STEER_MID = 650U;
	//STEER_LEFT = 740U;    �������δ� ���������
	//STEER_RIGHT = 560U;   ���������δ� �������

	LEFT_DIF  =  LEFT_CAM_ORG_blacklineindex  - Left_Detected_Pos;	//60 -
	RIGHT_DIF =  RIGHT_CAM_ORG_blacklineindex - Right_Detected_Pos; //60 -

	if(Dotted_Line_Pos == 1) // �����ʿ� ������ ������
	{
		RIGHT_DIF = 0; // �����ʿ� ������ ������
	}

	else if(Dotted_Line_Pos == 0) // ���ʿ� ������ ������
	{
		LEFT_DIF = 0;
	}
	//Dotted_Line_Pos = 0,1	// 0 - ����, 1- ������
	//�����̸� �� ���ְ� //�������̸� ���������� �� ���ְ�
	//if()
	// 2.8 -> 10(����)
	Error_Present = LEFT_DIF + RIGHT_DIF;
	Error_PaPre = Error_Past - Error_Present;
	//2.7
	// �������� 2.8
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

			for (j = 18; j <= LEFT_CAM_MID_index; j++) //���� ī�޶� �߾Ӽ� ���� ���� ���� Ȱ��
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
			for (j = 100; j >= RIGHT_CAM_MID_index ; j--) //���� ī�޶� �߾Ӽ� ���� ���� ���� Ȱ��
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
	if(School_Flag == 0)	// ������ �ƿ� ����    ->  AEB(������)
	{

	}
	if(School_Flag == 1)	// ������ �� ����	->  ��ֹ� ȸ��
	{

	}
}




