/*
 * Test_Pwm.c
 *
 *  Created on: 2015. 3. 24.
 *      Author: kimwayne
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include <stdio.h>
#include "Test_Pwm.h"
#include "Test_Irq.h"

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

App_GtmTomPwmHl GtmTomPwmHl; /**< \brief Demo information */
App_GtmTomPwmHl GtmTomPwmHlDCmotor; /**< \brief Demo information */
App_GtmTomPwmHl GtmTomServoTimer; /**< \brief Demo information */
App_GtmTomTimer GtmTomTimer; /**< \brief Demo information */
App_GtmTomTimer GtmTomMsTimer; /**< \brief Demo information */
App_GtmTomPwmHl GtmTomValve; /**< \brief Demo information */

uint16	TestDuty;
unsigned int	Gpt_TimerOverflow;
unsigned int	Gpt_MsTimerOverflow;
/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/



void Delay_us(uint32 DelayTime)
{

	Gpt_TimerOverflow = 0;
    IfxGtm_Tom_Timer_run(&GtmTomTimer.drivers.timerOneMs);

    while(Gpt_TimerOverflow < DelayTime){

    	__nop();

    }

    IfxGtm_Tom_Timer_stop(&GtmTomTimer.drivers.timerOneMs);

}


/** \name Interrupts for Timer
 * \{ */

IFX_INTERRUPT(Gpt_Notification_SystemTick_1us, 0, ISR_PRIORITY_TIMER_1us);
void Gpt_Notification_SystemTick_1us(void)
{
    IfxCpu_enableInterrupts();

    IfxGtm_Tom_Timer_acknowledgeTimerIrq(&GtmTomTimer.drivers.timerOneMs);
    Gpt_TimerOverflow++;
}


void Delay_ms(uint32 DelayTime)
{

	Gpt_MsTimerOverflow = 0;
    IfxGtm_Tom_Timer_run(&GtmTomMsTimer.drivers.timerOneMs);

    while(Gpt_MsTimerOverflow < (DelayTime*10)){

    	__nop();

    }

    IfxGtm_Tom_Timer_stop(&GtmTomMsTimer.drivers.timerOneMs);

}

/** \name Interrupts for Timer
 * \{ */

IFX_INTERRUPT(Gpt_Notification_SystemTick_1ms, 0, ISR_PRIORITY_TIMER_1ms);
void Gpt_Notification_SystemTick_1ms(void)
{
    IfxCpu_enableInterrupts();

    IfxGtm_Tom_Timer_acknowledgeTimerIrq(&GtmTomMsTimer.drivers.timerOneMs);
    Gpt_MsTimerOverflow++;

}

void Pwm_MotorDutyAndDirectionControl(uint16 DutyCycle, uint8 Direction)
{
	/* set Inhibit pin high active */
	P02_OUT.B.P0 = 1;
	P00_OUT.B.P2 = 1;
	P02_OUT.B.P2 = 1;
	P33_OUT.B.P5 = 1;

	/* motor rev direction set up */
	switch(Direction)
	{
		case 0:          // CW
			GTM_TOM0_CH9_SR0.B.SR0 = FREQ_20KHZ;
			GTM_TOM0_CH9_SR1.B.SR1 = FREQ_20KHZ;
			GTM_TOM0_CH10_SR0.B.SR0 = FREQ_20KHZ;
			GTM_TOM0_CH10_SR1.B.SR1 =  FREQ_20KHZ-DutyCycle;
			break;
		case 1:         // CCW
			GTM_TOM0_CH9_SR0.B.SR0 = FREQ_20KHZ;
			GTM_TOM0_CH9_SR1.B.SR1 = FREQ_20KHZ-DutyCycle;
			GTM_TOM0_CH10_SR0.B.SR0 = FREQ_20KHZ;
			GTM_TOM0_CH10_SR1.B.SR1 =  FREQ_20KHZ;
			break;
		default:
			break;
	}

}

void GtmTomPwmHl_initTimer_DCmotor(void)
{
    {   /* GTM TOM configuration */
        IfxGtm_Tom_Timer_Config timerConfig;
        IfxGtm_Tom_PwmHl_Config pwmHlConfig;

        IfxGtm_Tom_Timer_initConfig(&timerConfig, &MODULE_GTM);
        timerConfig.base.frequency                  = 20000;  //20Khz
//        timerConfig.base.isrPriority                = ISR_PRIORITY_TIMER;	//For Interrupt Enable
        timerConfig.base.isrPriority                = 0;
        timerConfig.base.isrProvider                = 0;
        timerConfig.base.minResolution              = (1.0 / timerConfig.base.frequency) / 1000;
        timerConfig.base.trigger.enabled            = FALSE;
        timerConfig.tom                             = IfxGtm_Tom_0;
        timerConfig.timerChannel                    = IfxGtm_Tom_Ch_0;
        timerConfig.clock                           = IfxGtm_Cmu_Clk_0;

        timerConfig.triggerOut                      = NULL_PTR;	//For Trigger Signal output port
        timerConfig.base.trigger.outputEnabled      = TRUE;
        timerConfig.base.trigger.enabled            = FALSE;	//For Trigger Signal Check via Oscilloscope
        timerConfig.base.trigger.triggerPoint       = 2500;
        timerConfig.base.trigger.risingEdgeAtPeriod = TRUE;

        IfxGtm_Tom_Timer_init(&GtmTomPwmHlDCmotor.drivers.timer, &timerConfig);


        IfxGtm_Tom_ToutMapP ccx[2]   = {&IfxGtm_TOM0_9_TOUT1_P02_1_OUT,
        								&IfxGtm_TOM0_10_TOUT12_P00_3_OUT
        								};

        IfxGtm_Tom_ToutMapP coutx[2] = {&IfxGtm_TOM0_11_TOUT3_P02_3_OUT,
        		                        &IfxGtm_TOM0_2_TOUT28_P33_6_OUT
        		                        };

        pwmHlConfig.timer                 = &GtmTomPwmHlDCmotor.drivers.timer;
        pwmHlConfig.tom                   = timerConfig.tom;
//      pwmHlConfig.base.deadtime         = 1e-8;
//      pwmHlConfig.base.minPulse         = 0;
        pwmHlConfig.base.channelCount     = 2;
        pwmHlConfig.base.emergencyEnabled = FALSE;
        pwmHlConfig.base.outputMode       = IfxPort_OutputMode_pushPull;
        pwmHlConfig.base.outputDriver     = IfxPort_PadDriver_cmosAutomotiveSpeed1;
        pwmHlConfig.base.ccxActiveState   = Ifx_ActiveState_high;
        pwmHlConfig.base.coutxActiveState = Ifx_ActiveState_low;
        pwmHlConfig.ccx                   = ccx;
        pwmHlConfig.coutx                 = coutx;

        IfxGtm_Tom_PwmHl_init(&GtmTomPwmHlDCmotor.drivers.pwm, &pwmHlConfig);

        GTM_TOM0_TGC0_OUTEN_CTRL.U = 0x0000AAAA;
        GTM_TOM0_TGC0_OUTEN_STAT.U = 0x0000AAAA;
        GTM_TOM0_TGC0_ENDIS_CTRL.U = 0x0000AAAA;
        GTM_TOM0_TGC0_ENDIS_STAT.U = 0x0000AAAA;

        GTM_TOM0_TGC1_OUTEN_STAT.U = 0x0000AAAA;
        GTM_TOM0_TGC1_OUTEN_CTRL.U = 0x0000AAAA;
        GTM_TOM0_TGC1_ENDIS_CTRL.U = 0x0000AAAA;
        GTM_TOM0_TGC1_ENDIS_STAT.U = 0x0000AAAA;

        GTM_TOM0_TGC0_GLB_CTRL.U = 0xAAAA0000;
        GTM_TOM0_TGC1_GLB_CTRL.U = 0xAAAA0000;

     //   IfxGtm_Tom_Timer_run(&GtmTomPwmHlDCmotor.drivers.timer);

    }
}

void GtmTomTimer_initTimer_SERVOmotor(void)
{
	{   /* GTM TOM configuration */
			IfxGtm_Tom_Timer_Config ServotimerConfig;
	        IfxGtm_Tom_Timer_initConfig(&ServotimerConfig, &MODULE_GTM);

//	        ServotimerConfig.base.frequency       = 20000;
//	        ServotimerConfig.base.isrPriority     = 0;
	        ServotimerConfig.base.isrProvider     = 0;
	        ServotimerConfig.base.minResolution   = (1.0 / ServotimerConfig.base.frequency) / 1000;
	        ServotimerConfig.base.trigger.enabled = FALSE;

	        ServotimerConfig.tom                  = IfxGtm_Tom_0;
	        ServotimerConfig.timerChannel         = IfxGtm_Tom_Ch_7;
	        ServotimerConfig.clock                = IfxGtm_Cmu_Fxclk_2;


	        ServotimerConfig.triggerOut                      = &IfxGtm_TOM0_7_TOUT32_P33_10_OUT;	//For Trigger Signal output port
	        ServotimerConfig.base.trigger.outputEnabled      = TRUE;
	        ServotimerConfig.base.trigger.enabled            = TRUE;	//For Trigger Signal Check via Oscilloscope
//	        ServotimerConfig.base.trigger.triggerPoint       = 1500;
	        ServotimerConfig.base.trigger.risingEdgeAtPeriod = TRUE;

	        IfxGtm_Tom_Timer_init(&GtmTomServoTimer.drivers.timer, &ServotimerConfig);
	        GTM_TOM0_CH7_SR0.B.SR0 = 	SERVO_PERIOD_20MS;
	        GTM_TOM0_CH7_SR1.B.SR1 = 	650U;
	        IfxGtm_Tom_Timer_run(&GtmTomServoTimer.drivers.timer);

	}
}

void Pwm_ServoDutyUpdate(uint16 DutyCycle)
{
    IfxGtm_Tom_Timer *timer = &GtmTomServoTimer.drivers.timer;

	IfxGtm_Tom_Timer_disableUpdate(timer);

	GTM_TOM0_CH7_SR0.U = SERVO_PERIOD_20MS;
	GTM_TOM0_CH7_SR1.U = DutyCycle;

	IfxGtm_Tom_Timer_applyUpdate(timer);

} /* Pwm_MotorDrive */


void GtmTomTimer_initTimer(void)
{
    {   /* GTM TOM configuration */
        IfxGtm_Tom_Timer_Config timerConfig;
        IfxGtm_Tom_Timer_initConfig(&timerConfig, &MODULE_GTM);
        timerConfig.base.frequency       = 100000;
        timerConfig.base.isrPriority     = ISR_PRIORITY_TIMER_1us;
        timerConfig.base.isrProvider     = 0;
        timerConfig.base.minResolution   = (1.0 / timerConfig.base.frequency) / 1000;
        timerConfig.base.trigger.enabled = FALSE;

        //IfxGtm_TOM1_0_TOUT32_P33_10_OUT
        timerConfig.tom                  = IfxGtm_Tom_1;
        timerConfig.timerChannel         = IfxGtm_Tom_Ch_0;
        timerConfig.clock                = IfxGtm_Cmu_Fxclk_0;

        IfxGtm_Tom_Timer_init(&GtmTomTimer.drivers.timerOneMs, &timerConfig);
        GTM_TOM1_CH0_SR0.B.SR0 = 	100; //1500;			// 1500 -> 1.5ms
        IfxGtm_Tom_Timer_run(&GtmTomTimer.drivers.timerOneMs);

    }
}

void GtmTomTimer_initMsTimer(void)
{
    {   /* GTM TOM configuration */
        IfxGtm_Tom_Timer_Config MstimerConfig;
        IfxGtm_Tom_Timer_initConfig(&MstimerConfig, &MODULE_GTM);
        MstimerConfig.base.frequency       = 100000;
        MstimerConfig.base.isrPriority     = ISR_PRIORITY_TIMER_1ms;
        MstimerConfig.base.isrProvider     = 0;
        MstimerConfig.base.minResolution   = (1.0 / MstimerConfig.base.frequency) / 1000;
        MstimerConfig.base.trigger.enabled = FALSE;

        MstimerConfig.tom                  = IfxGtm_Tom_0;
        MstimerConfig.timerChannel         = IfxGtm_Tom_Ch_14;
        MstimerConfig.clock                = IfxGtm_Cmu_Fxclk_0;

        IfxGtm_Tom_Timer_init(&GtmTomMsTimer.drivers.timerOneMs, &MstimerConfig);
        GTM_TOM0_CH14_SR0.B.SR0 = 	10000; //1500;			// 1500 -> 1.5ms
        IfxGtm_Tom_Timer_run(&GtmTomMsTimer.drivers.timerOneMs);

    }

}

uint32 Gpt_GetTimeElapsed(void)
{

return	GTM_TOM1_CH0_CN0.B.CN0;

}

/** \brief Demo init API
 *
 * This function is called from main during initialization phase
 */
void Pwm_Initialization(void)
{
    /* disable interrupts */
    boolean  interruptState = IfxCpu_disableInterrupts();

    /** - GTM clocks */
    Ifx_GTM *gtm = &MODULE_GTM;
    GtmTomPwmHl.info.gtmFreq = IfxGtm_Cmu_getModuleFrequency(gtm);
    IfxGtm_enable(gtm);

    /* Set the global clock frequencies */
    IfxGtm_Cmu_setGclkFrequency(gtm, GtmTomPwmHl.info.gtmFreq);
    GtmTomPwmHl.info.gtmGclkFreq = IfxGtm_Cmu_getGclkFrequency(gtm);

    //GTM_CMU_CLK_2_CTRL.B.CLK_CNT = 0x100;
   IfxGtm_Cmu_setClkFrequency(gtm, IfxGtm_Cmu_Clk_0,1000000);

    GtmTomPwmHl.info.gtmCmuClk0Freq = IfxGtm_Cmu_getClkFrequency(gtm, IfxGtm_Cmu_Clk_0, TRUE);
    GtmTomPwmHl.info.stateDeadline  = now();

    /** - Initialise the DC_MOTOR part */
    GtmTomPwmHl_initTimer_DCmotor();
    /** - Initialise the DC_MOTOR part */
	GtmTomTimer_initTimer_SERVOmotor();


    /** - Initialise the GTM part */
    GtmTomTimer_initTimer();
    /** - Initialise the GTM part */
    GtmTomTimer_initMsTimer();

    /* enable interrupts again */
    IfxCpu_restoreInterrupts(interruptState);

    IfxGtm_Cmu_enableClocks(gtm, IFXGTM_CMU_CLKEN_FXCLK |IFXGTM_CMU_CLKEN_CLK0);

}
