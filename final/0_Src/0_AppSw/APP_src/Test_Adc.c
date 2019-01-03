/*
 * Test_Adc.c
 *
 *  Created on: 2015. 3. 26.
 *      Author: kimwayne
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include <stdio.h>
#include "Test_Adc.h"
#include <Cpu/Std/IfxCpu.h>
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
#include "Direction_RPM.h"
#include "School.h"

//#include "Bsp.h"


#define TSL1401_AO_1				9
#define TSL1401_AO_2				10

#define _LEFT 						0
#define _RIGHT						1
/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/
App_VadcAutoScan VadcAutoScan; /**< \brief Demo information */

/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/
void Vadc_InitConfig(IfxVadc_GroupId GroupId,App_VadcAutoScan *Vadc);
void Vadc_ChConfig(IfxVadc_GroupId GroupId, App_VadcAutoScan *VadcAutoScan);

void TestLineScan(void);
void Nomalization(void);
void Average_Filter(void);
void Gaussian_Filter(void);
void PSD_Sensor(void);
void Min_Extract(void);
void IR_Sensor(void);

uint32 ADCresult(IfxVadc_GroupId GroupId, uint16 i);


uint32 White_max_data = 2750;
uint32 Black_min_data = 1000;

uint32 chnIx_LineCam1 = 9;
uint32 chnIx_LineCam2 = 10;



volatile int IR_1 = 0, IR_2 = 0,IR_3 = 0,IR_4 = 0,IR_5 = 0 ,IR_6 = 0;
volatile int Min2=0, Min1=0;

volatile int Line_Result[2][128] = {{0},};
double Sig_Result[2][128] = {{0},};

volatile int Nomal_Result[2][128] = {{0},};
volatile int Gauss_Result[2][128] = {{0},};

uint8 Right_Dotted_Line_Sum = 0;
uint8 Right_Dotted_Check = 0;

uint8 Left_Dotted_Line_Sum = 0;
uint8 Left_Dotted_Check = 0;

uint16 Plus_Lane_Detect_Count_L = 0;
uint16 Plus_Lane_Detect_Count_R = 0;

uint8	ir_val = 0;

// ********* PSD Average Filter ********** //
int PSD_Idx = 0;
int PSD_init_flag = 0;
int PSD_new_data_flag = 0;
float PSD_Dist_Q[5] = {0};
float Distance = 0;
float volt = 0;
float temp_sum = 0;
float temp1 = 0, temp2 =0;
// *************************************** //


/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/
uint32 	Adc_Result_Scan[MaxModuleNum][MaxChNum]	= {{0},{0}};
const uint8 adc_ch_cfg[][MaxChNum] =
{
	  /*  { CH0, CH1, CH2, CH3, CH4, CH5, CH6, CH7, CH8, CH9, CH10, CH11 } */
 /*G0*/   { OFF , OFF , ON , ON , ON , ON , ON , ON , ON , ON , ON  , ON   },
 /*G1*/   { OFF , OFF , ON , ON , ON , ON , OFF , OFF , OFF , OFF , OFF  , OFF   }

};

IfxVadc_Adc_Channel       adcChannel[MaxModuleNum][MaxChNum];

/** \brief Demo init API
 *
 * This function is called from main during initialization phase
 */
void Adc_Initialization(void)
{

	 Vadc_InitConfig(IfxVadc_GroupId_0,&VadcAutoScan);
	 Vadc_ChConfig(IfxVadc_GroupId_0,&VadcAutoScan);

	 Vadc_InitConfig(IfxVadc_GroupId_1,&VadcAutoScan);
	 Vadc_ChConfig(IfxVadc_GroupId_1,&VadcAutoScan);

}



void Vadc_InitConfig(IfxVadc_GroupId GroupId,App_VadcAutoScan *Vadc){

    /* VADC Configuration */

    /* create configuration */
    IfxVadc_Adc_Config adcConfig;
    IfxVadc_Adc_initModuleConfig(&adcConfig, &MODULE_VADC);

    /* initialize module */
    IfxVadc_Adc_initModule(&Vadc->vadc, &adcConfig);

    /* create group config */
    IfxVadc_Adc_GroupConfig adcGroupConfig;
    IfxVadc_Adc_initGroupConfig(&adcGroupConfig, &Vadc->vadc);

    /* with group 0 */
    adcGroupConfig.groupId = GroupId;
    adcGroupConfig.master  = adcGroupConfig.groupId;

    /* enable scan source */
    adcGroupConfig.arbiter.requestSlotScanEnabled = TRUE;

    /* enable auto scan */
    adcGroupConfig.scanRequest.autoscanEnabled = TRUE;

    /* enable all gates in "always" mode (no edge detection) */
    adcGroupConfig.scanRequest.triggerConfig.gatingMode = IfxVadc_GatingMode_always;

    /* initialize the group */
    /*IfxVadc_Adc_Group adcGroup;*/    //declared globally
    IfxVadc_Adc_initGroup(&Vadc->adcGroup[GroupId], &adcGroupConfig);



}
void Vadc_ChConfig(IfxVadc_GroupId GroupId, App_VadcAutoScan *VadcAutoScan){

    uint8  chnIxNum = MaxChNum;
    uint32 chnIx;
    /* create channel config */
    IfxVadc_Adc_ChannelConfig adcChannelConfig[chnIxNum];


    for (chnIx = 0; chnIx < chnIxNum; ++chnIx)
    {

    	if(adc_ch_cfg[GroupId][chnIx] != OFF){
        IfxVadc_Adc_initChannelConfig(&adcChannelConfig[chnIx], &VadcAutoScan->adcGroup[GroupId]);

        adcChannelConfig[chnIx].channelId      = (IfxVadc_ChannelId)(chnIx);
        adcChannelConfig[chnIx].resultRegister = (IfxVadc_ChannelResult)(chnIx);  /* use dedicated result register */

        /* initialize the channel */
        IfxVadc_Adc_initChannel(&adcChannel[GroupId][chnIx], &adcChannelConfig[chnIx]);

        /* add to scan */
        unsigned channels = (1 << adcChannelConfig[chnIx].channelId);
        unsigned mask     = channels;
        IfxVadc_Adc_setScan( &VadcAutoScan->adcGroup[GroupId], channels, mask);
    	}

    }

}


/** \brief Demo run API
 *
 * This function is called from main, background loop
 */
void Test_VadcAutoScan(IfxVadc_GroupId GroupId)
{
    uint32  chnIx;
    uint8 	chnIxNum = MaxChNum;
        /* start autoscan */
        IfxVadc_Adc_startScan(&VadcAutoScan.adcGroup[GroupId]);

        /* check results */
        for (chnIx = 0; chnIx < chnIxNum; ++chnIx)
        {

        	if(adc_ch_cfg[GroupId][chnIx] != OFF){
            unsigned     group   = adcChannel[GroupId][chnIx].group->groupId;
            unsigned     channel = adcChannel[GroupId][chnIx].channel;

            /* wait for valid result */
            Ifx_VADC_RES conversionResult;

            do
            {
                conversionResult = IfxVadc_Adc_getResult(&adcChannel[GroupId][chnIx]);
            } while (!conversionResult.B.VF);

            Adc_Result_Scan[GroupId][chnIx] = conversionResult.B.RESULT;
        	}

        }
}

void TestLineScan(void)
{

	uint32 idx = 0;


	// P14-6 ( SI - CAM 1,2 )
	// P14-7 ( CLK - CAM 1,2 )

	P14_OUT.B.P6 = 1;   // SI - CAM1 ON
	P14_OUT.B.P7 = 0;	// CLK - CAM1 OFF
	waitTime(5 * TimeConst_100ns);

	P14_OUT.B.P6 = 1;	// SI_1 ON
	P14_OUT.B.P7 = 1;	// CLK_1 ON
	waitTime(5 * TimeConst_100ns);

	P14_OUT.B.P6 = 0;	// SI_1 OFF
	P14_OUT.B.P7 = 1;	// CLK_1 ON
	waitTime(5 * TimeConst_100ns);


	IfxVadc_Adc_startScan(&VadcAutoScan.adcGroup[0]);

	for(idx = 0; idx < 128 ; ++idx)
	{

			P14_OUT.B.P6 = 0;	// SI_1 OFF
			P14_OUT.B.P7 = 0;	// CLK_1 OFF
			waitTime(3 * TimeConst_1us);

			P14_OUT.B.P6 = 0;	// SI_1 OFF
			P14_OUT.B.P7 = 1;	// CLK_1 ON
			waitTime(2 * TimeConst_1us);


	    	Ifx_VADC_RES conversionResult;

			do
			{
				conversionResult = IfxVadc_Adc_getResult(&adcChannel[0][chnIx_LineCam1]);
			} while (!conversionResult.B.VF);

			Line_Result[0][idx] = conversionResult.B.RESULT;

			do
			{
				conversionResult = IfxVadc_Adc_getResult(&adcChannel[0][chnIx_LineCam2]);
			} while (!conversionResult.B.VF);

			Line_Result[1][idx] = conversionResult.B.RESULT;




	}
	P14_OUT.B.P6 = 0;	// SI_1 OFF
	P14_OUT.B.P7 = 0;	// CLK_1 OFF
	waitTime(1 * TimeConst_10ms);

}
void Average_Filter(void)
{
	uint32 idx = 0;
	uint32 i = 0;
	for(i = 0; i<2; i++)
	{
		for(idx = 7; idx < 121 ; ++idx)
		{
				if(idx < 4)
				{
					Sig_Result[i][idx] = Line_Result[i][idx];
				}
				else if(idx >= 4 || idx <=125)
				{
					Sig_Result[i][idx-2] = (Line_Result[i][idx-4] + Line_Result[i][idx-3] + Line_Result[i][idx-2] + Line_Result[i][idx-1] + Line_Result[i][idx])/5;
				}
				else if(idx > 125)
				{
					Sig_Result[i][idx] = Line_Result[i][idx];
				}
		}
	}
}


uint32 ADCresult(IfxVadc_GroupId GroupId, uint16 i)
{
	 uint32  chnIx ;

		 chnIx = i;

	        /* start autoscan */
	        IfxVadc_Adc_startScan(&VadcAutoScan.adcGroup[GroupId]);

	        /* check results */
	        if(adc_ch_cfg[GroupId][chnIx] != OFF)
	        {

	        	unsigned     group   = adcChannel[GroupId][chnIx].group->groupId;
	            unsigned     channel = adcChannel[GroupId][chnIx].channel;

	            /* wait for valid result */
	            Ifx_VADC_RES conversionResult;

	            do
	            {
	                conversionResult = IfxVadc_Adc_getResult(&adcChannel[GroupId][chnIx]);
	            } while (!conversionResult.B.VF);

	            Adc_Result_Scan[GroupId][chnIx] = conversionResult.B.RESULT;
	        }

	        return Adc_Result_Scan[GroupId][chnIx];
	        //return Adc_Result_Scan[GroupId][chnIx];
}

/*
void PSD_Sensor(void)
{
		volt =  ADCresult(IfxVadc_GroupId_0,3);
		Distance = exp(8.5841-log(volt)) * 10;
		if(Distance > 80)
			Distance = 80;
		if(Distance < 24 )
			Distance = 24;
}*/
//21,20,11,17
void IR_Sensor(void)
{	//AN15, 16
	if(IR_ADC_OFF_Flag == 0)
	{
		IR_1 = ADCresult(IfxVadc_GroupId_1,3); // P41.3(AN15)
		IR_2 = ADCresult(IfxVadc_GroupId_1,4); // P41.4(AN16)
		IR_3 = ADCresult(IfxVadc_GroupId_0,11); // P40.11 (AN11)
		//IR_4 = 								// P41.5 (AN17)
		//IR_5 =								// P41.8 (AN20)
		//IR_6 =								// P41.9 (AN21)

		ir_val = 0x00;
		if(IR_1>2500){
			ir_val |= 0x01;
		}
		if(IR_2>2500){
			ir_val |= 0x02;
		}
		if(IR_3>2500){
			ir_val |= 0x04;
		}
	}

}

void Super_Min_Extract(void)
{
	int i = 0;
	Min1 = 4095, Min2 = 4095;


}

void Min_Extract(void) // 검은색 도출 및 점선 라인 위치 파악
{
		//uint8 Left_Dotted_Line_sum = 0;
		//uint8 Left_Dotted_Check = 0;

		int i;
        Min1 = 4095, Min2 = 4095;

		for (i = 93 ; i > 27 ; i--) // 왼쪽
		{
			if( Sig_Result[0][i] < Min1 )
				Min1 = Sig_Result[0][i];
		}
		for ( i = 93 ; i > 27 ; i--)
		{
			if(Min1 == Sig_Result[0][i] && Min1 < 2300) //2300(연습트랙 돌릴때) 5공 밝은환경(오후 6시쯤) - 2800
			{

				Left_Detected_Pos = i;
				Left_Dotted_Check = 0;
				break;

			}
			else
				Left_Dotted_Check = 1;

		}


		// 추가로 더 짜야할것 라이징을 얻어야한다.
		for (i = 27 ; i < 93 ; i++) // 왼쪽
		{
			if( Sig_Result[1][i] < Min2 )
				Min2 = Sig_Result[1][i];
		}
		for ( i = 27 ; i < 93 ; i++)
		{
			if(Min2 == Sig_Result[1][i] && Min2 < 2300)
			{
				Right_Detected_Pos = i;
				Right_Dotted_Check = 0;
				break;
			}
			else
				Right_Dotted_Check = 1;

		}
		if(School_Flag == 1) // 플래그=0 (스쿨존 x) & 플래그 =1 (스쿨존ㅇ)
		{
			Left_Dotted_Line_Sum += Left_Dotted_Check;
			Right_Dotted_Line_Sum += Right_Dotted_Check;
		}

}

//Plus_Lane_Detect_Count_L = 0;
//Plus_Lane_Detect_Count_R = 0;
void Plus_Lane_Detect(void)
{
	int i =0;
	if(Dotted_Line_Pos == _LEFT)	//점선이 왼쪽에 있을때
	{
		for( i = 60; i < 90; i++ )
		{
			if(Sig_Result[1][i] < 2000)
				Plus_Lane_Detect_Count_R++;
		}
	}
	else
	{
		for( i = 60; i< 90; i++)	//점선이 오른쪽에 있을때
		{
			if(Sig_Result[0][i] < 2000)
				Plus_Lane_Detect_Count_L++;
		}
	}

}


void PSD_Sensor(void)
{
      volt =  ADCresult(IfxVadc_GroupId_0,3);
      if(PSD_init_flag == 0){     // PSD값 5개 인덱스 채워주는 작업
         PSD_init();             // init_flag는 한번쓰고 맘
      }
      else if(PSD_init_flag == 1)
      {
		  PSD_Average(PSD_Dist_Q, 5);  //PSD값 평균

		  if(Distance > 80)
			 Distance = 80;
		  if(Distance < 15 )
			 Distance = 15;

		  if(PSD_new_data_flag == 0)
		  {
			 PSD_FIFO(PSD_Dist_Q, 5);   //새로운 데이터 들어오면 인덱스 4번값 버리고 한칸씩 옮김, 새로운 값 인덱스 0 에 대입
		  }
      }

}


///////////////////////////////////////////////////////////
void PSD_init()
{
   temp1 = exp(8.5841-log(volt)) * 10;
   PSD_Dist_Q[PSD_Idx] = temp1;
   PSD_Idx = PSD_Idx+1;

   if(PSD_Idx == 5){
      PSD_init_flag = 1;
      PSD_Idx = 0;
   }
}

void PSD_Average(float *PSD_Dist, int n){
   int i = 0;
   for(i =0; i<n; i++){
      temp_sum = temp_sum + PSD_Dist[i];
   }
   Distance = temp_sum/n;
   PSD_new_data_flag = 0;
   temp_sum = 0;
}

void PSD_FIFO(float *PSD_Dist, int n){
   int i =0;
  temp2 = exp(8.5841-log(volt)) * 10;

   for(i=0; i<n-1; i++){
       PSD_Dist[i+1] = PSD_Dist[i];
   }
   PSD_Dist[0] = temp2;
   PSD_new_data_flag = 1;
}

