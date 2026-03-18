/*
 * Timers.h
 *
 *  Created on: Sep 7, 2021
 *      Author: Guillermo
 *
 * Rev.3.1: Oct. 2024
 *
 */

#ifndef TIMERS_H_
#define TIMERS_H_

#include "globaldefs.h"

/*-----------------------------------------------------------------
 * USec_Timer - Microsecond timer to capture TDAS events time
 *
 * The event time is recorded a few microseconds after the trigger
 * signal. It will be ready to use long before the IMU data
 * completed transmission and is decoded.
 *----------------------------------------------------------------*/
typedef struct tevent_time
{
	uint16_t	TimeType;			// From GPS data
	uint16_t	GPSWeek;
	uint32_t	WeekMilliSeconds;	// Week millisencods
	double 		WeekSeconds;		// SOW in seconds, not milliseconds

}event_time_t;

extern volatile event_time_t UsecEventTime;

void Init_USec_Timer();
void Set_Event_Time_Report(event_time_t *ptime);


/*-------------------------------------------------------------------------
 * 	PPS Counter	- TCPMW[0].5
 *
 *	- Configured as event counter, increments the count with each PPS raising edge.
 *	- The counter is preset to the Seconds of the Week (Week Time) by a
 *	  callback from Novatel data decoder.
 *	- The period is set to (604800-1) which is the total # of seconds in a week
 *	- A timer overflow signs the start of a new GPS week.
 *	- The interrurpt on overflow increments the GPS week in case the equipment
 *	- is running at Sunday midnight of GPS time.
 *
 *--------------------------------------------------------------------------*/
extern volatile bool PPS_Counter_Error;		// True if init not possible
extern volatile bool PPS_SOW_InSync;		// True when the counter has true SOW count
extern volatile uint32_t PPS_WeekSeconds;	// full week seconds updated by PPS pulse

void Set_PPS_Counter(uint32_t count);
void Init_PPS_Counter();
uint32_t Get_SecondsOfTheWeek000();			// returns the GPS Seconds of the week (no fractions

extern volatile uint32_t GPS_WeekSeconds;
inline bool IsGpsTimeInSync() { return PPS_WeekSeconds == GPS_WeekSeconds; }

/*---------------------------------------------
 * TDAS - IMU strobe signal
 *--------------------------------------------*/
void Init_TDAS_PWM();
bool SetTDASStrobeFrequency(uint32_t _timer_frequency);

void StopTDASStrobe();
void StartTDASStrobe();

extern volatile bool TDAS_PWM_Running;

/*---------------------------------------------
 * PPS Monitor - Controls PPS LED
 *--------------------------------------------*/
#define DEFAULT_PPS_ON_TIME		(200u)				// 200 ms on time
#define SHORT_PPS_MONITOR_PULSE (50U)
#define LONG_PPS_MONITOR_PULSE	(500U)
#define SLOW_PPS_MONITOR_FREQ	(0.2F)				// 5 sec. period
#define FAST_PPS_MONITOR_FREQ	(10.0F)				// 10 HZ

void Init_PPS_MONITOR_PWM(bool oneShot);			// The PPS indicator LED timer
void TriggerPPSMonitor();
bool SetSPPSMonintorFrequency(float _new_frequency, uint32_t on_time);
void ChangePPSMonitorRunMode(bool oneShot);
void DisablePPSMonitor();
void EnablePPSMonitor();


/*---------------------------------------------
 * Hartbit PWM timer
 *--------------------------------------------*/
typedef void (*fpaction)(uint32_t);
void Init_HARTBIT_PWM(fpaction action);				// action will be called with the _hartbit count since started
void SetHartBitAction(fpaction action);

typedef struct _up_time
{
	uint16_t Hours;
	uint8_t  Minutes;
	uint8_t  Seconds;
	uint16_t Millisec;

} up_time_t;

up_time_t GetUpTime();

char* GetUpTimeStr();

/*-------------------------------------------------------------------------
 *  IMU_INIT - TCPMW[0].2	-  PWM drive the IMU_INIT_BIT
 *
 *	- Generates a 500 milliseconds low pulse to the IMU_INIT_BIT line
 *
 * 	Signal outputs:
 * 		- P[3].0 INIT_BIT
 *
 * 	NOTE: This signals has no apparent effect on the FSAS IMU
 *
 *------------------------------------------------------------------------*/
void Init_IMU_INIT_BIT_PWM();
void Trigger_INIT_BIT();
void Enable_IMU_INIT_BIT_PWM(bool enable);

/*-----------------------------------------------------------------------------
 * GPTIMER - General purpose timers
 *
 * These functions create a timer with a callback function called at timeout
 * they can be one shot or free running.
 *
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

/* timer callback prototype */
typedef void (*GPT_CALLBACK)(void* cbarg );

typedef uint32_t GPT_HANDLE;

typedef struct _gpt_task	// GP Timer Periodic task
{
	// input parameters

	uint32_t		period;		// Timer period in milliseconds
	GPT_CALLBACK 	callback;	// Timer callback at end of period
	void 			*cbarg;		// Callback arguments
	bool 			oneshot;	// set to false for free running timer
	// output
	GPT_HANDLE 		thandle;	// to be set by GP Timer

} GPT_TASK_t;


enum
{
	 MAX_TIMERS			  = 10,
	INVALID_GPTIMER_HANDLE = 0xFFFFU,
};


bool StartPeriodicTask(GPT_TASK_t *P);
void StopPeriodicTask(GPT_TASK_t *P);
inline bool GPTaskRunning(GPT_TASK_t *P) { return (P->thandle != INVALID_GPTIMER_HANDLE); }

void Init_GPTIMER1_PWM();
GPT_HANDLE CreateGPTimer(GPT_CALLBACK callback, void* cbarg, uint16_t timeout_ms, bool one_shot);
void ReleaseGPTimer(GPT_HANDLE thandle);
void ReleaseAllGPTimers();
void StartGPTimer(GPT_HANDLE thandle);
void StopGPTimer(GPT_HANDLE thandle);
void ResumeGPTimer(GPT_HANDLE thandle);
uint32_t GetGPTImerCount(GPT_HANDLE thandle);
uint32_t GetGPTImerPeriod(GPT_HANDLE thandle);
uint32_t GpTimerEllapsedMs(GPT_HANDLE thandle);
bool IsGPTimerRunning(GPT_HANDLE thandle);
void GPTimerDelay(uint16_t timeout_ms);

int TimersFree();

#endif /* TIMERS_H_ */
