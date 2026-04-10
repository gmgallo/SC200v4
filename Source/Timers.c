/*
 * Timers.c
 *
 *  Created on: Sep 7, 2021
 *      Author: Guillermo
 *
 * Example How to configure ECO clock for HF clock
 * https://forum.digikey.com/t/sampling-audio-at-44-1-khz-with-the-cy8cproto-062-4343w-board/13177
 *
 *
 * Rev.3.23: Sep. 2025  - Updated IMU records time mark and documentation
 * 						- Changed uSec_Timer ISR to adjust counter clock variations using PPS timing (works very well!)
 */
#include "common.h"
/*
#define ENABLE_USEC_TIMER
#define USEC_TIMER_TP	TP7

#define ENABLE_PPS_ISR
#define PPS_ISR_TP  	TP6
*/

volatile bool gps_time_sync = false;					// True when received GPS time from GPS

#define PPS_IN_ISR_PRIORITY		  4U
#define USec_Timer_INTR_PRIORITY  5U
#define PPS_Counter_INTR_PRIORITY 7U
#define GPTIMERS1_INTR_PRIORITY	  6U
#define HARTBIT_INTR_PRIORITY 	  7U
#define TDAS_PWM_INTR_PRIORITY    5U


/*-------------------------------------------------------------------------------------------------------
 *  IMU Trigger Time Synchronization
 *
 *  Each FSAS IMU record received @200Hz must be time tagged with the second and fraction of a second to 1ms.
 *  That clock is precisely aligned with the PPS signal.
 *  To allow a precise timing sync of the 200 IMU records/sec  the PPS signal is used to adjust the uSec_Counter.
 *  In the present board production the external clock oscillator are disabled (wrong assembly orientation)
 *  All clocks are generated out of the Internal Main Oscillator (IMO) which shifts with temperature.
 *
 *  - The PPS pulse that marks the start of the second and is connected to PPS_IN (P6.5) which generates GPIO Interrupt.
 *    This interrupt is tightly aligned within microseconds with the external signal.
 *    The PPS_IN interrupt keeps track of the whole seconds counter PPS_WeekSeconds and flags the start of each second.
 *
 *  - NovAtel MARK1 output programmed as 200Hz IMU TRIGGER clock in sync with PPS also wired to GPS_EVENT_OUT_1 (P4.1)
 *
 * 	- USec_Timer is clocked at 100KHz. Its ISR uses the start of the second flag to calculate its clock drift from
 * 	   the 100KHz nominal value and saves the compensation value for each IMU trigger. Resets it counter to zero at PPS time.
 * 	   It saves the adjusted fraction of a second trigger time for the each IMU record coming next.
 * 	   The capture contains the fraction of the second of each IMU trigger with a 1/100000 resolution
 * 	   The captured time stores it in the UsecEventTime structure user later to complete the IMU record.
 *
 *                      ________________
 *  PPS _______________|               |__________________ (1 per second, duration 1ms exactly)
 *
 *  PPS_INT____________|__________________________________ ( pps interrupt advances the whole seconds counter)
 *             ____________________________________________
 *  IMU_TRIGGER        | (#1 T0)           | (#2 T0 + 5ms) ( 10 us low pulse, 200Hz from NovAtel Event out 1 )
 *
    US_CAPTURE - Obsolete. The new logic doesn't use this PWM timer anymore.
 */

/*------------------------------------------------------------------------- MARK1 PRESCALER Settings */


/*-------------------------------------------------------------------------
 * 	PPS Counter	- TCPMW[0].5
 *
 *	- Configured as event counter, increments the count with each PPS raising edge
 *	- The counter is preset to the Seconds of the Week (Week Time) by a
 *	  callback from the Novatel decoding function.
 *--------------------------------------------------------------------------*/
#define FULL_WEEK_SECONDS	(604800U)			// Constant for week count rollover saturday midnight

volatile bool sync_prescaler    = true;
volatile bool PPS_SOW_InSync    = false;		// True when the counter has true SOW count
volatile uint16_t PPS_WeekNr	= 0;			// Will be updated along with the SOW count
volatile uint32_t PPS_WeekSeconds = 0;		    // full week seconds updated by PPS pulse
volatile bool pps_sec_updated 	= false;

volatile bool pps_hi	= true;

/*---------------------------------------------------------------------------------
 *  MonitorPPSTime()
 *  Will be called from Novatel if GPS time is FINESTEARING well 10 ms after PPS
 *  To set the PPS counter back in sync with the current seconds of the week.
 *--------------------------------------------------------------------------------*/
void MonitorPPSTime(uint16_t gps_week, uint32_t gps_seconds )
{
	if ( PPS_WeekSeconds != gps_seconds)
	{
		pps_sec_updated = true;				// prevent double increment

		PPS_SOW_InSync  = true;
		PPS_WeekSeconds = gps_seconds;		// will update on next PPS.
		PPS_WeekNr = gps_week;				// WeekNr may be out of sync too.
	}
}

/*-----------------------------------------------------------------------------------
 *  Init_PPS_Counter_Time()
 *  Will be called from Novatel decode once GPS time is FINESTEARING
 *  It only needs to be called once after PPS_Counter is started
 *---------------------------------------------------------------------------------*/
void Init_PPS_Counter_Time(uint16_t gps_week, uint32_t gps_seconds )
{
	pps_sec_updated = true;

	PPS_WeekNr = gps_week;
	PPS_WeekSeconds = gps_seconds;
	//Set_PPS_Counter( gps_seconds );					// Program the PPS week counter.
	RegisterNovatelTimeNotify( MonitorPPSTime ); 	// no longer needed.
}


/*--------------------------------------------------------------------------------*/
void PPS_In_GPIO_callback(void *arg, cyhal_gpio_event_t event)
{
    (void) arg;

    if(event & CYHAL_GPIO_IRQ_RISE)			// We get interrupts on both edges!
    {
#ifdef ENABLE_PPS_ISR
	SET_DEBUG_TP(PPS_ISR_TP);
#endif
			PPS_WeekSeconds++;
			pps_hi = true;
		/*
		 * Handle here the week rollover saturday midnight
		 */
		if (PPS_WeekSeconds == FULL_WEEK_SECONDS )
		{
			PPS_WeekSeconds = 0;
			PPS_WeekNr++;
		}

#ifdef ENABLE_PPS_ISR
	CLEAR_DEBUG_TP(PPS_ISR_TP);
#endif
    }
    else
    {
    	pps_hi = false;
    }
}

/* must be global */
cyhal_gpio_callback_data_t pps_in_cb_data =
{
		PPS_In_GPIO_callback,
		NULL, 0, 0,
};

/*--------------------------------------------------------------------------------*/
void Init_PPS_Counter()
{
	/* we use the PPS_IN GPIO ISR to update the PPS_WeekSeconds counter */

	cyhal_gpio_register_callback(PPS_IN, &pps_in_cb_data);
    cyhal_gpio_enable_event(PPS_IN, CYHAL_GPIO_IRQ_BOTH, PPS_IN_ISR_PRIORITY, true);

	/* Set the GPS receiver to update the counter with the correct SOW and
	 * request a time update from the receiver
	 */
	PPS_SOW_InSync = false;
	RegisterNovatelTimeNotify( Init_PPS_Counter_Time );
}


/*-------------------------------------------------------------------------
 * 	uSec_Timer - TCPMW[0].0	-  Microsecond Timer
 *
 *	- Resets with a rising PPS pulse applied to Reload input
 *	- Samples the counter time and generates Interrupt with a rising pulse to its Capture input
 * 	- Programmed as a counter restarted by GPS_EVENT_OUT_1/IMU_TRIGGER pulse from OEM7700 @ 200Hz
 * 	- Each capture has a time resolution of 1/10000 of a second with GPS clock accuracy
 * 	- Terminal count 1 seconds. The last sample should and will happen 5ms before that thanks
 * 	  to the IMU_TRIGGER generated by OEM7700 in sync with PPS.
 *
 *  Clock: 100KHz (from 16 bit divider 1)
 *
 * 	Signal input:
 * 		- IMU_TRIGGER 	Capture triggered by IMU_TRIGGER  input P4[1])
 *
 * 	Signal outputs:
 *
 *  ISR start: ~20 uSec after capture event
 *	ISR Duration: < 20 uSec
 *
 *	CLOCK SHIFT ADJUTMET
 *	Because the 100Khz clock running uSec_Timer is running from IMO Internal Main Oscillator
 *	that has over 1% error and changes with temperature, it needs to be adjusted.
 *	The timing of the 200Hz IMU trigger generated by the GNSS receiver is precisely aligned with the PPS.
  * The nominal count at the PPS time should be 100,000 counts/second with a 100KHz clock.
 *  Tdif = 100,000 - Tcapture @ PPS.
 *  With Ftrigger of 200Hz each capture after PPS must be adjusted by
 *  Tusec = Tcapture + (TdifAccuumulated + Tdif /200);
 *
 *----------------------------------------------------------------------------*/
#define USTIMER_CLK	100000.0		// CLOCK 100KHz
#define USTIMER_CLK_MSEC 100		// CLOCK/1000

volatile double ClockDif    = 0;	// Total clock deviation per second
volatile double CountAdjust = 0; 	// Counter adjust increment for each interrupt
volatile double AdjutAccum  = 0; 	// Accumulated adjustment

volatile event_time_t UsecEventTime;

event_time_t *pEventTime = NULL;

void Set_Event_Time_Report(event_time_t *ptime)
{
	pEventTime = ptime;
}





/*----------------------------------------------------------------------------*/
void USec_Timer_Isr(void)
{
#ifdef ENABLE_USEC_TIMER
	SET_DEBUG_TP(USEC_TIMER_TP);
#endif

    /* Get all the enabled pending interrupts */
    uint32_t interrupts = Cy_TCPWM_GetInterruptStatusMasked(USec_Timer_HW, USec_Timer_NUM);

    /* Handle the Capture event trigger by US_CAPTURE to time-tag the next IMU reading */
    if (0UL != (CY_TCPWM_INT_ON_CC & interrupts))
	{
		/* Calculate
		 * next time mark will happen at PPS + 5ms -> PPS will be low
		 */
    	double t_mark;

	    uint32_t tcap = Cy_TCPWM_Counter_GetCapture(USec_Timer_HW, USec_Timer_NUM);

    	if (pps_hi == true) // Calculate the IMO clock correction and reset the counter
    	{
    	    Cy_TCPWM_TriggerStopOrKill_Single(USec_Timer_HW, USec_Timer_NUM);

			ClockDif = USTIMER_CLK - tcap;

			CountAdjust = ClockDif / IMU_Clock;
			AdjutAccum = 0;
			t_mark = 0.0;

    	    uint32_t status = Cy_TCPWM_Counter_GetStatus(USec_Timer_HW, USec_Timer_NUM);

    	    // Wait for the counter to stop!
    	    while(CY_TCPWM_COUNTER_STATUS_COUNTER_RUNNING & status)
    	    		status = Cy_TCPWM_Counter_GetStatus(USec_Timer_HW, USec_Timer_NUM);

    	    Cy_TCPWM_Counter_SetCounter(USec_Timer_HW, USec_Timer_NUM, 2 ); // consider offset of reset counter
       	    Cy_TCPWM_TriggerStart_Single(USec_Timer_HW, USec_Timer_NUM);
    	}
    	else
    	{
    		AdjutAccum += CountAdjust;
    		t_mark = AdjutAccum + tcap;
    	}
    	/* we assume that GPS message keeps checking and updating PPSTime once a second */

    	UsecEventTime.TimeType = ( PPS_SOW_InSync == true )? nStatus.TimeStatus : TIME_TYPE_LOCAL;

        UsecEventTime.WeekSeconds = PPS_WeekSeconds + t_mark / USTIMER_CLK;
        UsecEventTime.WeekMilliSeconds = PPS_WeekSeconds*1000 + t_mark/USTIMER_CLK_MSEC;
        UsecEventTime.GPSWeek = PPS_WeekNr;

       	if (pEventTime != NULL)				// update reporting record time
       	{
       		memcpy(pEventTime,(event_time_t*) &UsecEventTime, sizeof(event_time_t));
       	}
	}

	Cy_TCPWM_ClearInterrupt(USec_Timer_HW, USec_Timer_NUM, interrupts );

#ifdef ENABLE_USEC_TIMER
    CLEAR_DEBUG_TP(USEC_TIMER_TP);
#endif
}


void Init_USec_Timer()
{
	/* Init the capture PWM */
	// US_CAPTURE provides a delayed capture event to USec_Timer so it doesn't delay with
	// the Reload signal provided by PPS
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(US_CAPTURE_HW, US_CAPTURE_NUM, &US_CAPTURE_config))
    {
        /* Handle possible errors */
    }
    Cy_TCPWM_PWM_Enable(US_CAPTURE_HW, US_CAPTURE_NUM);
    Cy_TCPWM_TriggerReloadOrIndex_Single(US_CAPTURE_HW, US_CAPTURE_NUM);

    // now init the timer
	if (CY_TCPWM_SUCCESS !=  Cy_TCPWM_Counter_Init(USec_Timer_HW, USec_Timer_NUM, &USec_Timer_config))
	{
		/* Handle possible errors */
	}
	Cy_TCPWM_Counter_Enable(USec_Timer_HW, USec_Timer_NUM);
	ConfigureInterrupt(USec_Timer_IRQ, USec_Timer_INTR_PRIORITY, USec_Timer_Isr );
	Cy_TCPWM_TriggerStart_Single(USec_Timer_HW, USec_Timer_NUM);
}


/*-------------------------------------------------------------------------
 *  PPS_MON - TCPMW[1].19 - PWM Timer for LED indicator
 *
 *  - Handles the only LED under PSOC control, it has dual function:
 *  - During initialization it generates blink codes,
 *	- in normal operation generates a 500 us blink for PPS indicator LED
 *	- It is reload (reset) by the PPS pulse from the GPS in sync with USec_Timer
 *
 * 	Signal inputs:
 * 		- PPS (reload counter) 		P6.5
 *
 * 	Signal outputs:
 * 		- PPS_MONITOR	P3.1 - LED indicator
 *------------------------------------------------------------------------*/
const uint32_t _pps_mon_clock	= (10000U);		// 10 KHz

#define MIN_PPS_MON_FREQUENCY		(0.2)		// 1 Pulse every 5 sec.
#define MAX_PPS_MON_FREQUENCY		(100.0)		// 100 Hertz
#define MIN_ON_TIME					(500U)		// 500 ms blink

cy_stc_tcpwm_pwm_config_t active_pps_mon_config;

#define RELOAD_DISABLE 	(PPS_MON_INPUT_DISABLED & 0x3U);
#define NO_RELOAD_VALUE	(OU)

uint32_t pps_mon_reload_value;
uint32_t pps_mon_reload_mode;

void Init_PPS_MONITOR_PWM(bool oneShot)
{
	 memcpy(&active_pps_mon_config, &PPS_MON_config, sizeof(cy_stc_tcpwm_pwm_config_t));

	 active_pps_mon_config.runMode = oneShot? CY_TCPWM_PWM_ONESHOT: CY_TCPWM_PWM_CONTINUOUS;

	if (CY_TCPWM_SUCCESS ==  Cy_TCPWM_PWM_Init(PPS_MON_HW, PPS_MON_NUM, &active_pps_mon_config) )
	{
		/* Enable the initialized PWM */
		Cy_TCPWM_PWM_Enable(PPS_MON_HW,PPS_MON_NUM);

		/* if not one shot start timer at once */
		Cy_TCPWM_TriggerReloadOrIndex_Single(PPS_MON_HW, PPS_MON_NUM);
		/* Handle possible errors */
	}
}

void StopPPSMonitor()
{
	Cy_TCPWM_TriggerStopOrKill_Single(PPS_MON_HW, PPS_MON_NUM);

	/* wait for the counter to stop */
	while ((Cy_TCPWM_PWM_GetStatus(PPS_MON_HW, PPS_MON_NUM) & CY_TCPWM_PWM_STATUS_COUNTER_RUNNING) )
		;
}

void Deinit_PPS_Monitor_PWM()
{
	StopPPSMonitor();
    Cy_TCPWM_PWM_DeInit(PPS_MON_HW, PPS_MON_NUM, &PPS_MON_config);
}

void TriggerPPSMonitor()
{
	Cy_TCPWM_TriggerReloadOrIndex_Single(PPS_MON_HW, PPS_MON_NUM);
}

bool SetSPPSMonintorFrequency(float _new_frequency, uint32_t on_time)
{
	uint32_t compare;

	/* validate frequency ranges */
	if (_new_frequency < MIN_PPS_MON_FREQUENCY || _new_frequency > MAX_PPS_MON_FREQUENCY)
		return true;

	Cy_TCPWM_TriggerStopOrKill_Single(PPS_MON_HW, PPS_MON_NUM);

	uint32_t new_period =(uint32_t)( (float)_pps_mon_clock/_new_frequency);

	uint32_t on_period = ( on_time * (_pps_mon_clock/1000));

	if (on_period < MIN_ON_TIME )
	{
		on_period = MIN_ON_TIME;
	}

	if ( (on_period + 1000) < new_period ) // min of 100 ms ON time
	{
		compare = new_period - on_period;
	}
	else
	{
		compare = new_period /2;
	}

	/* wait for the counter to stop */
	while ((Cy_TCPWM_PWM_GetStatus(PPS_MON_HW, PPS_MON_NUM) & CY_TCPWM_PWM_STATUS_COUNTER_RUNNING) )
		;

	/* program the new frequency */
	Cy_TCPWM_PWM_SetCounter(PPS_MON_HW, PPS_MON_NUM, 0);
	Cy_TCPWM_PWM_SetPeriod0(PPS_MON_HW, PPS_MON_NUM, new_period);
	Cy_TCPWM_PWM_SetCompare0Val(PPS_MON_HW, PPS_MON_NUM, compare);

	Cy_TCPWM_TriggerReloadOrIndex_Single(PPS_MON_HW, PPS_MON_NUM);

	return false;
}

void ChangePPSMonitorRunMode(bool oneShot)
{
	StopPPSMonitor();

    /* Disable the counter prior to deinitializing */
    Cy_TCPWM_PWM_Disable(PPS_MON_HW, PPS_MON_NUM);
    Cy_TCPWM_PWM_DeInit(PPS_MON_HW, PPS_MON_NUM, &active_pps_mon_config);
    Init_PPS_MONITOR_PWM(oneShot);
}

void EnablePPSMonitor()
{
    /* Enable the initialized PWM */
	Cy_TCPWM_PWM_Enable(PPS_MON_HW,PPS_MON_NUM);
}

void DisablePPSMonitor()
{
	Cy_TCPWM_PWM_Disable(PPS_MON_HW,PPS_MON_NUM);
}



/*-------------------------------------------------------------------------
 *  HARTBT - TCPMW[0].3	-  PWM for Hartbit functions
 *
 *	- Generates a 500 microseconds pulse to turn on the PPS indicator LED
 *	-
 *	- It is reload (reset) by the PPS pulse from the GPS in sync with USec_Timer
 *
 * 	Signal inputs:
 *
 * 	Signal outputs:
 * 		- Interrupt for code activation.
 *------------------------------------------------------------------------*/

volatile uint32_t _hartbits = 0;

fpaction _hbaction = NULL;

void SetHartBitAction(fpaction action) { _hbaction = action; }


void HartBit_Isr()
{
    /* Get all the enabled pending interrupts */
    uint32_t interrupts = Cy_TCPWM_GetInterruptStatusMasked(HARTBIT_HW, HARTBIT_NUM);

	_hartbits++;	//GPS Week roll over at 604800 seconds

	if (_hbaction != NULL)
		_hbaction(_hartbits);

	Cy_TCPWM_ClearInterrupt(HARTBIT_HW, HARTBIT_NUM, interrupts);
}


void Init_HARTBIT_PWM(fpaction action)
{
	if (CY_TCPWM_SUCCESS !=  Cy_TCPWM_PWM_Init(HARTBIT_HW, HARTBIT_NUM, &HARTBIT_config) )
	{
		/* Handle possible errors */
	}
	    /* Enable the initialized PWM */
	Cy_TCPWM_PWM_Enable(HARTBIT_HW, HARTBIT_NUM);

    /* configure interrupt one level over USec timer */
	ConfigureInterrupt(HARTBIT_IRQ, HARTBIT_INTR_PRIORITY, HartBit_Isr );

	_hartbits = 0;
	_hbaction = action;

   	/* start timer at once */
	Cy_TCPWM_TriggerReloadOrIndex_Single(HARTBIT_HW, HARTBIT_NUM);
}


up_time_t GetUpTime()
{
	// may need a critical section to avoid off by 1 sec
	uint32_t H = _hartbits;

	uint32_t msec = Cy_TCPWM_PWM_GetCounter(HARTBIT_HW, HARTBIT_NUM);

	if (msec >= 1000)
		msec -= 1000;

	up_time_t T = { 0,0,0,0 };

	T.Millisec = msec;								// fraction of a second in milliseconds

	T.Seconds  = (uint8_t) H % 60U;					// fractional of a minute in seconds
	H = (H - T.Seconds)/60;							// convert to full minutes
	T.Minutes  = (uint8_t) H % 60U;					// fractional hours in minutes
	T.Hours    = (uint16_t) (H -T.Minutes)/60U; 	// hours in 6 operations

	return T;
}

static char _uptime_str[20] = "";

char* GetUpTimeStr()
{
	up_time_t T = GetUpTime();

	snprintf(_uptime_str,ARRAY_SIZE(_uptime_str),"%d:%.2d:%.2d.%.3d", T.Hours, T.Minutes, T.Seconds, T.Millisec );
	return _uptime_str;
}

/*-------------------------------------------------------------------------
 *  IMU_INIT - TCPMW[0].2	-  PWM drive the IMU_INIT_BIT
 *
 *	- Generates a 100 milliseconds low pulse to the IMU_INIT_BIT line
 *
 * 	Signal outputs:
 * 		- P[3].0 INIT_BIT
 *
 * 	NOTE: This signals has no apparent effect on the IMU
 *
 *------------------------------------------------------------------------*/
void Init_IMU_INIT_BIT_PWM()
{
	if (CY_TCPWM_SUCCESS !=  Cy_TCPWM_PWM_Init(INIT_BIT_PWM_HW, INIT_BIT_PWM_NUM, &INIT_BIT_PWM_config) )
	{

	}
	Cy_TCPWM_PWM_Enable(INIT_BIT_PWM_HW, INIT_BIT_PWM_NUM);

}

void Trigger_INIT_BIT()
{
   	/* start timer at once */
	Cy_TCPWM_TriggerReloadOrIndex_Single(INIT_BIT_PWM_HW, INIT_BIT_PWM_NUM);
}

void Enable_IMU_INIT_BIT_PWM(bool enable)
{
	if (enable)
		Cy_TCPWM_PWM_Enable(INIT_BIT_PWM_HW, INIT_BIT_PWM_NUM);
	else
		Cy_TCPWM_PWM_Disable(INIT_BIT_PWM_HW, INIT_BIT_PWM_NUM);
}

/*-----------------------------------------------------------------------------
 * GPTIMER - General purpose timers run by GPTIMER1 Isr
 *
 * These functions create a timer with a callback function called at timeout
 * they can be one shot or free running.
 *
 *NOTE
 * Functions:
 * 		StartPeriodicTask() - High level function to create periodic tasks
 * 		StopPeriodicTask()
 *
 * 		CreateGPTimer() - creates a one shot or free running timer with callback
 * 						Returns: a timer handle
 *
 * 		ReleaseGPTimer() - release resource with the handle returned by CreateGPTimer()
 *
 * 		StartGPTimer() - Starts a free running or triggers a single shot timer.
 *
 * 		StopGPTimer()	- Pauses the timer. Does not reset counter
 *
 * 		ResumeGPTimer()	- Continues from the stopped counter
 *
 * 		GetGPTImerCount() - Returns the current timer count in 1/10th of millisecond
 *
 *-----------------------------------------------------------------------------*/

typedef struct
{
	bool used;
	bool running;
	bool reload;
	uint32_t period;
	uint32_t count;
	GPT_CALLBACK callback;
	void*	args;

} gptimer_t;

gptimer_t gptimers_table[MAX_TIMERS] =		// array of up to 10 timers
{
		{false},{false},{false},{false},{false},
		{false},{false},{false},{false},{false}
};

/*
 * GPTIMERS1 PWM ISR runs @ 1khz frequency
 */
void GPTimers1_Isr()
{
    /* Get all the enabled pending interrupts */
    uint32_t interrupts = Cy_TCPWM_GetInterruptStatusMasked(GPTIMERS1_HW, GPTIMERS1_NUM);

   for( int i=0; i < MAX_TIMERS; i++)
   {
	   gptimer_t *pt = &gptimers_table[i];

	   if(pt->running == true)
	   {
		   if( --pt->count == 0 )
		   {
			   if (pt->callback != NULL)
				   pt->callback(pt->args);

			   if (pt->reload == false) // if one shoot release the timer
			   {
				   pt->running = false;
				   pt->used = false;
			   }
			   else
			   {
				   pt->count = pt->period;
			   }
		   }
 	   }
   }
   Cy_TCPWM_ClearInterrupt(GPTIMERS1_HW, GPTIMERS1_NUM, interrupts);
}

void Init_GPTIMER1_PWM()
{
	if (CY_TCPWM_SUCCESS !=  Cy_TCPWM_PWM_Init(GPTIMERS1_HW, GPTIMERS1_NUM, &GPTIMERS1_config) )
	{
		/* Handle possible errors */
	}
	    /* Enable the initialized PWM */
	Cy_TCPWM_PWM_Enable(GPTIMERS1_HW, GPTIMERS1_NUM);

    /* configure interrupt one level over USec timer */
	ConfigureInterrupt(GPTIMERS1_IRQ, GPTIMERS1_INTR_PRIORITY, GPTimers1_Isr );


   	/* start timer at once */
	Cy_TCPWM_TriggerReloadOrIndex_Single(GPTIMERS1_HW, GPTIMERS1_NUM);
}


GPT_HANDLE _get_next_timer()
{
	for (int i = 0; i < MAX_TIMERS; i++ )
	{
		if (gptimers_table[i].used == false )
		{
			gptimers_table[i].used = true;
			return i;
		}
	}

	printf("MAX TIMERS = %d REACHED\n", MAX_TIMERS);
	return INVALID_GPTIMER_HANDLE;
}

int TimersFree()
{
	int tf= 0;

	for (int i = 0; i < MAX_TIMERS; i++ )
	{
		if (gptimers_table[i].used == false )
			tf++;
	}
	return tf;
}

/*----------------------------------------------------------------------------*/
uint32_t GpTimerEllapsedMs(GPT_HANDLE thandle)
{
	if ( thandle < MAX_TIMERS)
	{
		if (gptimers_table[thandle].used == true )
		{
			return (gptimers_table[thandle].period - gptimers_table[thandle].count)/10;
		}
	}
	return 0;
}

/*----------------------------------------------------------------------------*/
void ReleaseAllGPTimers()
{
	for( int i = 0; i < MAX_TIMERS; i++)
	{
		gptimers_table[i].running = false;
		gptimers_table[i].used = false;
	}
}

/*----------------------------------------------------------------------------*/
void ReleaseGPTimer(GPT_HANDLE thandle)
{
	if ( thandle < MAX_TIMERS)
	{
		if (gptimers_table[thandle].used == true )
		{
			gptimers_table[thandle].running = false;
			gptimers_table[thandle].used = false;
		}
	}
}

/*----------------------------------------------------------------------------*/
void StartGPTimer(GPT_HANDLE thandle) // starts from zero
{
	if ( thandle < MAX_TIMERS)
	{
		if (gptimers_table[thandle].used == true )
		{
			gptimers_table[thandle].count = gptimers_table[thandle].period;
			gptimers_table[thandle].running = true;
		}
	}
}


/*----------------------------------------------------------------------------*/
void ResumeGPTimer(GPT_HANDLE thandle) // Continues from the last Stop
{
	if ( thandle < MAX_TIMERS)
	{
		if (gptimers_table[thandle].used == true )
		{
			gptimers_table[thandle].running = true;
		}
	}
}

/*----------------------------------------------------------------------------*/
void StopGPTimer(GPT_HANDLE thandle)
{
	if ( thandle < MAX_TIMERS)
	{
		if (gptimers_table[thandle].used == true )
		{
			gptimers_table[thandle].running = false;
		}
	}
}

/*----------------------------------------------------------------------------*/
uint32_t GetGPTImerCount (GPT_HANDLE thandle) // Returns the current timer count in 1/10th of millisecondGet
{
	uint32_t retval = 0;
	if ( thandle < MAX_TIMERS)
	{
		if (gptimers_table[thandle].used == true )
		{
			retval = gptimers_table[thandle].count;
		}
	}
	return retval;
}

bool IsGPTimerRunning(GPT_HANDLE thandle)
{
	if ( thandle < MAX_TIMERS)
	{
		return (gptimers_table[thandle].running == true );
	}
	return false;
}

/*----------------------------------------------------------------------------*/
uint32_t GetGPTImerPeriod(GPT_HANDLE thandle) // Returns the current timer count in 1/10th of millisecondGet
{
	uint32_t retval = 0;
	if ( thandle < MAX_TIMERS)
	{
		if (gptimers_table[thandle].used == true )
		{
			retval = gptimers_table[thandle].period;
		}
	}
	return retval;
}

/*----------------------------------------------------------------------------*/
GPT_HANDLE CreateGPTimer(GPT_CALLBACK callback, void* cbarg, uint16_t timeout_ms, bool one_shot)
{
	GPT_HANDLE thandle = _get_next_timer();

	if (thandle != INVALID_GPTIMER_HANDLE )
	{
		gptimer_t *pt = &gptimers_table[thandle];

		pt->running = false;
		pt->callback =callback;
		pt->args = cbarg;
		pt->period = timeout_ms;
		pt->reload = !one_shot;
	}

	return thandle;
}

bool StartPeriodicTask(GPT_TASK_t *P)
{
	if ( P->period == 0 ) // start a periodic timer
		return true;

	/* disable previous GPTimer */
	if ( P->thandle != INVALID_GPTIMER_HANDLE)
		ReleaseGPTimer(P->thandle);

	P->thandle = CreateGPTimer(P->callback, P->cbarg, P->period, P->oneshot);

	if ( P->thandle != INVALID_GPTIMER_HANDLE)
		StartGPTimer(P->thandle);

	return (P->thandle == INVALID_GPTIMER_HANDLE);
}

void StopPeriodicTask(GPT_TASK_t *P)
{
	if ( P->thandle != INVALID_GPTIMER_HANDLE )
	{
		StopGPTimer( P->thandle);
		ReleaseGPTimer( P->thandle);
		P->thandle = INVALID_GPTIMER_HANDLE;
	}
}

void GPTimerDelay(uint16_t timeout_ms)
{
	GPT_HANDLE thandle =  CreateGPTimer(NULL, NULL, timeout_ms, true);

	if (thandle != INVALID_GPTIMER_HANDLE )
	{
		StartGPTimer( thandle );
		while ( IsGPTimerRunning( thandle) );
	}
}

/* EOF */
