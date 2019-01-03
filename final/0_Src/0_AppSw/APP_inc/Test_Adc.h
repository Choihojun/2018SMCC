/*
 * Test_Adc.h
 *
 *  Created on: 2015. 3. 26.
 *      Author: kimwayne
 */

#ifndef TEST_ADC_H_
#define TEST_ADC_H_

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include <Vadc/Std/IfxVadc.h>
#include <Vadc/Adc/IfxVadc_Adc.h>

/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/
#define MaxChNum            (12U)
#define MaxModuleNum        (2U)

typedef struct
{
    IfxVadc_Adc vadc; /* VADC handle */
    IfxVadc_Adc_Group adcGroup[MaxModuleNum];
} App_VadcAutoScan;



#define ON 		1
#define OFF		0
#define NONE	0

uint32 Line_result(IfxVadc_GroupId GroupId);

IFX_EXTERN void TestLineScan(void);
IFX_EXTERN void Average_Filter(void);

IFX_EXTERN void PSD_Sensor(void);
IFX_EXTERN void Min_Extract(void);
IFX_EXTERN void IR_Sensor(void);

IFX_EXTERN void PSD_init(void);
IFX_EXTERN void PSD_Average(float *PSD_Dist,int n);
IFX_EXTERN void PSD_FIFO(float *PSD_Dist, int n);

IFX_EXTERN uint32 ADCresult(IfxVadc_GroupId GroupId, uint16 i);
IFX_EXTERN void Plus_Lane_Detect(void);

/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/
IFX_EXTERN App_VadcAutoScan VadcAutoScan;

/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/

IFX_EXTERN void Test_VadcAutoScan(IfxVadc_GroupId GroupId);

extern float Distance;
extern float volt;
extern int PSD_init_flag;
extern int PSD_new_data_flag;
extern float temp1, temp2;
extern float temp_sum;

extern volatile int Line_Result[2][128];
extern double Sig_Result[2][128];
extern volatile int Nomal_Result[2][128];
extern volatile int Gauss_Result[2][128];

extern volatile int Min1,Min2;
extern volatile int IR_1, IR_2 , IR_3;

extern uint8 Right_Dotted_Line_Sum;
extern uint8 Right_Dotted_Check;

extern uint8 Left_Dotted_Line_Sum;
extern uint8 Left_Dotted_Check;

extern uint16 Plus_Lane_Detect_Count_L;
extern uint16 Plus_Lane_Detect_Count_R;

extern uint8	ir_val;


//const uint8 adc_ch_cfg[][MaxChNum]
#endif /* TEST_ADC_H_ */
