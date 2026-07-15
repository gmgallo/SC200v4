/**
 * Tests and Benchmarks
 *
 */

#include "common.h"


/*------------------------------------------------------------------------------------ TESTS */
char* test_imu_loopback(char**tokens, int cnt, _ports_t port );
char* test_log_imu(char**tokens, int cnt, _ports_t port );
char* test_forward_imu( char* port );
char* test_bswap( _ports_t port );
char* test_init_bit( _ports_t port );
char* test_imu_pop( _ports_t port );
char* Start_TDAS_Frequency(char**tokens, int cnt, _ports_t port );

char *test_cmd(char**tokens, int cnt, _ports_t port)
{
	char *buf = CmdAnswer;
	size_t size = ARRAY_SIZE(CmdAnswer);

	if (cnt < 2)
	{
			snprintf(buf, size, "Test help:\n"
					"\tTEST 1 [n] - IMU port loopback. n = record length, default 1\n"
					"\tTEST 2 - STIM300 Log to port _port_name_.\n"
					"\tTEST 3 - STIM300 FORWARD TO PORT.\n"
					"\tTEST 4 - Dequeue IMU Records.\n"
					"\tTEST 5 - Enable INIT_BIT PPS Output.\n"
					"\tTEST 6 - Start NovAtel TDAS Frequency.\n"
					);

			return buf;
	}

	int32_t t1;

	if ( ToInt32(tokens[1], &t1) == 0 )
		return PrintCmdError( "bad parameter 1.");

	switch(t1)
	{
		case 1:
		{
			buf = test_imu_loopback(tokens, cnt, port);
			break;
		}
		case 2:
		{
			buf = test_log_imu(tokens, cnt, port);
			break;
		}

		case 3:
		{
			if (cnt >= 3)
				buf = test_forward_imu( tokens[2] );
			else
				return PrintCmdError("Forward STIM300 missing port param\n");
			break;
		}

		case 4:
			{
				buf = test_imu_pop( port );
				break;
			}
		case 5:
		{
			test_init_bit(port);
			break;
		}
		case 6:
		{
			buf = Start_TDAS_Frequency(tokens, cnt, port);
			break;
		}

		default:
			snprintf(buf, size, "Incorrect test # [%ld]\n", t1);
	}
	return buf;
}



/*----------------------------------------------- Compose Novatel MARTK1 Frequency  
* Output on TP2 -> IMU TDAS output                                      
*/
char* Start_TDAS_Frequency(char**tokens, int cnt, _ports_t port )
{
	char buf[100];

	if ( cnt < 3)
	{
		snprintf(CmdAnswer, sizeof(CmdAnswer), "\nUsage: TEST 6 frequency[0-1000] pulse_width[ms] [polarity[0-1]]\n");
		return CmdAnswer;
	}

	bool _polarity = true;
	uint32_t _frequency = 0, _pulse_width = 0;

	if ( cnt >= 3)
	{
		uint32_t freq;
		if ( ToUint32(tokens[2], &freq) == 0 )
		{
			return PrintCmdError("Invalid frequency parameter\n");
		}
		_frequency = freq;

		if (_frequency == 0)
		{
			Stop_IMU_Trigger_Frequency();
			int j = ComposeMarkOutDisable( buf, 100,1);
			 Uart_Oem7700_Send((uint8_t*)buf,j);

			return "TDAS Frequency Stopped\n";
		}
	}


	if( cnt >= 4)
	{
		uint32_t pw;

		if ( ToUint32(tokens[3], &pw) == 0 || pw < 1 )
		{
			return PrintCmdError("Invalid pulse width parameter\n");
		}
		_pulse_width = pw;
	}

	if (cnt >= 5)
	{
		int pol = find_binaryParam(tokens[4]);

		if (pol == IS_ON || pol == IS_OFF)
		{
			_polarity = (pol == IS_ON);
		}
		else
		{
			return PrintCmdError("Invalid polarity parameter\n");
		}
	}

	/* prevent out of valid range settings */

	if (_frequency > 1000  )
		_frequency = 1000;
	else if (_frequency < 1 )
		_frequency = 1;

	int j = ComposeMarkOutCommand(buf, ARRAY_SIZE(buf), 1, _frequency, _pulse_width, _polarity);
	Uart_Oem7700_Send((uint8_t*)buf,j);
	snprintf(CmdAnswer, sizeof(CmdAnswer), "START TDAS Frequency %ld Hz, pulse width %ld ms, polarity %s\n", _frequency, _pulse_width, _polarity? "HIGH":"LOW");
	return CmdAnswer;	
}

/*----------------------------------------------- TEST - IMU PORT LOOPBACK */
char *GrabConsoleInput(uint8_t* buf, _ports_t port)
{
	if (buf[0] == 'x' || buf[0] == 'X') // Test for grab end
	{
		_release_console();
		return "Console disconnected.\n";
	}
	//int cnt = strlen((char*)buf);
	
	Uart_IMU_SendString((char_t*)buf); 	// Send line to IMU port

	return NULL;
}

/*----------------------------------------*/
fnMessageProcessor 	old_processor;
uint32_t	old_isr_level;

void DisconnectLoopback()
{
	Reconfig_Uart_IMU(old_isr_level, false );
	SetImuMsgProcessor(old_processor);
	Enable_Uart_IMU();
}

/*----------------------------------------*/
uint8_t testBuf[128];
#define MAX_TEST_BUF 127
int tbIndex = 0;
extern bauds_t imu_bauds;

void TestMsgProcessor(uint8_t* buf, size_t cnt)
{
	memset(testBuf, 0, ARRAY_SIZE(testBuf) );
	memcpy(testBuf, buf, MIN(cnt,MAX_TEST_BUF));
	printf("IMU message: %d chars [%s]\n", cnt, testBuf );
	tbIndex =0;
}

char* test_imu_loopback(char**tokens, int cnt, _ports_t port )
{
	uint32_t record_length = 1;

	if (cnt > 2)
	{
		uint32_t n = 0;
		
		if (ToUint32(tokens[2], &n) != 1)
		{
			return PrintCmdError("Invalid record length parameter\n");
		}
		record_length = n;
	}
	snprintf(CmdAnswer,  ARRAY_SIZE(CmdAnswer), "IMU LOOPBACK TEST with Record length %ld\nBidge TX+ to RX+ and TX- to RX-\n"
						"Send strings over this terminal, or 'X' to end\n", record_length);

	Init_Uart_IMU( imu_bauds );
	Set_IMU_COM_Target(Target_PSOC);
	old_isr_level = Reconfig_Uart_IMU(record_length -1, false); // One interrupt per 2 characters

	old_processor = SetImuMsgProcessor(TestMsgProcessor );
	Enable_Uart_IMU();

	_grab_console(GrabConsoleInput ,DisconnectLoopback);

	return CmdAnswer;
}


/*------------------------------------------------------------------------------------- IMUREDIR */
/* IMUREDIR - Redirects the raw IMU output to a port for diagnostics
 *
 * Command format:
 *
 * 	IMUREDIR [port]
 *
 *----------------------------------------------------*/
_ports_t redir_target;

void ImuRedirMsgProcessor(uint8_t* buf, size_t cnt)
{
	char str[100];

	SendToPort( redir_target,buf, cnt);
	GetUartErrorStr(str,100, Uart_IMU_Error);

	if ( Uart_IMU_Error != 0)
	{
		printf("IMU message: %d bytes, UART Error: %s\n", cnt, str );
		Uart_IMU_Error = 0;
	}
	//tbIndex =0;
}


char *imuredir_cmd(char**tokens, int cnt, _ports_t port)	// debug command
{
	redir_target = port;		// default to calling port;

	if (cnt > 1)
	{
		_ports_t _p = FindPortID(tokens[1]);
		if (_p != INVALID_PORT)
		{
			redir_target = _p;
		}
	}
	snprintf(CmdAnswer,  ARRAY_SIZE(CmdAnswer), "IMU REDIR TO PORT\n"
						"Send commands over this terminal, or 'X' to end\n");

	Set_IMU_COM_Target(Target_PSOC);

	if (SysConfig.imu_type == IMUType_FSAS )
	{
		old_isr_level = Reconfig_Uart_IMU(FSAS_SN_RECORD_SIZE-1, false);
	}
	else if (SysConfig.imu_type == IMUType_STIM300 )
	{
		old_isr_level = Reconfig_Uart_IMU(STIM_RECORD_SIZE-1, false);
	}
	else if(SysConfig.imu_type == IMUType_KVH )
	{
		old_isr_level = Reconfig_Uart_IMU(KVH_RECORD_SIZE-1, false);
	}
	old_processor = SetImuMsgProcessor(ImuRedirMsgProcessor );

	_grab_console(GrabConsoleInput,DisconnectLoopback);

	Enable_Uart_IMU();

	return CmdAnswer;
}

/****************************************************************************
* BAUDSMON PWM
*
* Generate accurate pulses on TP6 for the duration of # of bytes
* Use to compare the actual baud rate received from the INU device
*****************************************************************************/
#define BAUDSMON_INTR_PRIORITY  CYHAL_ISR_PRIORITY_DEFAULT // = 3
#define BAUDSMON_TP	TP6
bool baudsmon_init = false;

void _bauds_monitor_Isr(void)
{
	uint32_t interrupts = Cy_TCPWM_GetInterruptStatusMasked(BAUDSMON_HW, BAUDSMON_NUM);

	/* Handle the compare event trigger */
	if (0UL != (CY_TCPWM_INT_ON_CC & interrupts))
	{
		SET_DEBUG_TP(BAUDSMON_TP);
	}
	Cy_TCPWM_ClearInterrupt(BAUDSMON_HW, BAUDSMON_NUM, interrupts );
}

void InitBaudsMonitor()
{
	if (baudsmon_init)
		return;

    if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(BAUDSMON_HW, BAUDSMON_NUM, &BAUDSMON_config))
    {
        /* Handle possible errors */
    }
    /* Enable the initialized PWM */
    Cy_TCPWM_PWM_Enable(BAUDSMON_HW, BAUDSMON_NUM);

	ConfigureInterrupt(BAUDSMON_IRQ, BAUDSMON_INTR_PRIORITY, _bauds_monitor_Isr );

	baudsmon_init = true;
}

void TriggerBaudsMonitor(uint32_t compare)
{
	Cy_TCPWM_PWM_SetCompare0Val(BAUDSMON_HW, BAUDSMON_NUM, compare);
	CLEAR_DEBUG_TP(BAUDSMON_TP);
	Cy_TCPWM_TriggerReloadOrIndex(BAUDSMON_HW, (1UL << BAUDSMON_NUM));
}

/*====================================================================== STIM300 TEST */

char *stim_status(char** tokens, int cnt, _ports_t port)
{
	GetStimStatus(CmdAnswer,  ARRAY_SIZE(CmdAnswer));
	return CmdAnswer;
}

char *GrabInput2(uint8_t* buf, _ports_t port)
{
	if (buf[0] == 'x' || buf[0] == 'X')
	{
		_release_console();
		Init_IMU_Interface(SysConfig.imu_type, SysConfig.imu_connect);

		return "End of Test.\n";
	}
	return "";
}

char* test_log_imu(char**tokens, int cnt, _ports_t port )
{
	Init_IMU_Interface(IMUType_STIM300, Target_PSOC);
	_grab_console(GrabInput2, Stim_Stop_Logging);

	if(cnt < 3)
		return "Missing port ID to forward.\n";

	return test_forward_imu( tokens[2] );

}


char* test_forward_imu( char* port )
{
	char *buf = CmdAnswer;
	size_t size = ARRAY_SIZE(CmdAnswer);

	_grab_console(GrabInput2, Stim_Stop_Logging);

	_ports_t log_port = FindPortID(port);

	if (log_port == INVALID_PORT )
	{
		snprintf(buf, size, "STIM300 FORWARD: %s Not a valid port (COM1,USB1,USB2,CONSOLE,UART_J6)\n",port );
		return buf;
	}

	Stim_Log_To_Port(log_port);

	snprintf(buf, size, "STIM300 LOG to PORT %s STARTED - X to end.\n", port );

	return (buf);
}


/*------------------------------------------------------------------------
 * Find error in IMU records pop during LOG command
 */

extern RDBUF_HANDLE HandleImuRecords;

char* test_imu_pop( _ports_t port )
{
	printf("Reading back Queued IMU records.\n");

	if (HandleImuRecords == NULL)
	{
		return "ERROR: HandleImuRecords == NULL\n";
	}

	int records = FlexRingBufferItems( HandleImuRecords );
	uint16_t cnt = 0;

	printf("HandleImuRecords Record count: %d\n", records);

	uint8_t  *pdn=0;

	while ( records != 0 )
	{
		SET_DEBUG_TP(TP6);
		cnt++;

		printf("Reading record %d of %d ...\n", cnt, records);

		ring_data_t R;
		PopFromFlexRingBuffer(HandleImuRecords, &R);

		uint16_t size = R.Size;
		uint8_t  *pd = R.Data;
		pdn = pd + size;

		CLEAR_DEBUG_TP(TP6);

		char * roll = (pd != pdn)? "OK" : "Roll";

		printf("\tRecord size: %d. %hhn Next: %hhn - %s\n", size, pd, pdn, roll);

		records = FlexRingBufferItems( HandleImuRecords );
	}
	return "---< DONE >----\n";
}

/*------------------------------------------------------------------------------------- CLOCKMONITOR */
cy_stc_tcpwm_pwm_config_t _clk_monitor_config;
#define CLK_MONITOR_CLOCK PCLK_TCPWM1_CLOCKS21 // Peripheral clock for the TCPWM[1] counter[21]

void Start_ClockMonitor();
void Stop_ClockMonitor();
void DeInit_ClockMonitor();

char* clock_monitor_cmd(char** tokens, int cnt, _ports_t port)
{
	if (cnt == 1)
	{
		return "Usage: CLOCKMONITOR [ON|OFF]\n"
				"      Outputs a square wave frequency == PERICLCK/100 (1 MHz) on TP6\n";
	}
	action_t action = find_Action(tokens[1]);

	if (action == ACTION_ON)
	{
		Start_ClockMonitor();
		return "CLOCKMONITOR ON: 1MHz at TP6\n";
	}
	else if (action == ACTION_OFF)
	{
		Stop_ClockMonitor();
		return "CLOCKMONITOR OFF\n";
	}

	return "Invalid arguments. Usage: CLOCKMONITOR [ON|OFF]\n";
}


bool Init_ClockMonitor(uint32_t clk_hw, uint32_t clk_num, bool continuous)
{
	memcpy(&_clk_monitor_config, &CLK_MONITOR_config, sizeof(cy_stc_tcpwm_pwm_config_t) );
	uint32_t pwmode = continuous? CY_TCPWM_PWM_CONTINUOUS : CY_TCPWM_PWM_ONESHOT;

	_clk_monitor_config.pwmMode = pwmode;

		 // Re-initialize with new mode
	if(	 CY_TCPWM_SUCCESS == Cy_TCPWM_PWM_Init(CLK_MONITOR_HW, CLK_MONITOR_NUM, &_clk_monitor_config))
	{
		Cy_GPIO_Pin_FastInit(TP6_PORT, TP6_PORT_NUM, CY_GPIO_DM_STRONG,	0, TP6_HSIOM);   // HSIOM selection for PWM line
	    Cy_SysClk_PeriphAssignDivider(CLK_MONITOR_CLOCK, clk_hw, clk_num);
		Cy_TCPWM_PWM_Enable(CLK_MONITOR_HW,CLK_MONITOR_NUM);
		return true;
	}

	return false;
	
}

void DeInit_ClockMonitor()
{
	Cy_TCPWM_PWM_Disable(CLK_MONITOR_HW, CLK_MONITOR_NUM);
	Cy_TCPWM_PWM_DeInit(CLK_MONITOR_HW, CLK_MONITOR_NUM, &_clk_monitor_config);
	
	Cy_GPIO_Pin_FastInit(TP6_PORT, TP6_PORT_NUM, CY_GPIO_DM_STRONG, 0, HSIOM_SEL_GPIO);  // HSIOM selection back t GPIO
}

void Start_ClockMonitor()
{
	if (Init_ClockMonitor(CLK6_100MHZ_HW, CLK6_100MHZ_NUM, true) )
	{
		/* if not one shot start timer at once */
		Cy_TCPWM_TriggerReloadOrIndex_Single(CLK_MONITOR_HW, CLK_MONITOR_NUM);
		/* Handle possible errors */
	}
}

void Stop_ClockMonitor()
{
	DeInit_ClockMonitor();
}


/*----------------------------------------------------------------------------- TEST MULTIPLE TIMERS */
GPT_HANDLE TimerHandles[MAX_TIMERS] =
{
		INVALID_GPTIMER_HANDLE, INVALID_GPTIMER_HANDLE, INVALID_GPTIMER_HANDLE, INVALID_GPTIMER_HANDLE, INVALID_GPTIMER_HANDLE,
		INVALID_GPTIMER_HANDLE, INVALID_GPTIMER_HANDLE, INVALID_GPTIMER_HANDLE, INVALID_GPTIMER_HANDLE, INVALID_GPTIMER_HANDLE,
};


void TimerTest_Callback(void *arg)
{
	GPT_HANDLE handle = *((GPT_HANDLE*)arg);
	uint32_t T = GetGPTImerPeriod(handle);
	printf("TIMER %ld - Elapsed: %ld\n", handle, T);
}

char* create_timer_cmd(char** tokens, int cnt, _ports_t port)
{
	if (cnt == 1)
	{
		return "Usage: CREATETIMER duration_in_ms [TRUE for one shot]\n";
	}

	bool once = false;
	int32_t T;


	if ( ToInt32(tokens[1], &T) == 0 )
	{
		return "INVALID TIMER DURATION";
	}

	if (cnt == 3)
	{
		bool B =find_binaryParam(tokens[2]);
		if(B == IS_ON)
		{
			once = B;
		}
	}

	for(int i=0; i < ARRAY_SIZE(TimerHandles); i++)
	{
		if (TimerHandles[i] == INVALID_GPTIMER_HANDLE) // find first available timer
		{
			GPT_HANDLE h = CreateGPTimer(TimerTest_Callback, TimerHandles + i, T, once );
			if (h != INVALID_GPTIMER_HANDLE)
			{
				TimerHandles[i] = h;
				StartGPTimer(h);

				snprintf(CmdAnswer, ARRAY_SIZE(CmdAnswer), "TIMER %ld CREATED\n", h);
				return CmdAnswer;
			}
		}
	}
	return "NO MORE TIMERS AVAILABLE\n";
}


char* release_timer_cmd(char** tokens, int cnt, _ports_t port)
{
	if (cnt == 2)
	{
		int32_t T;

		if ( ToInt32(tokens[1], &T) == 0 )
		{
			return "INVALID TIMER NUMBER";
		}
		for(int i=0; i < ARRAY_SIZE(TimerHandles); i++ )
		{
			if(T == TimerHandles[i])
			{
				StopGPTimer(T);
				ReleaseGPTimer(T);
				TimerHandles[i] = INVALID_GPTIMER_HANDLE;

				snprintf(CmdAnswer, ARRAY_SIZE(CmdAnswer), "TIMER %ld RELEASED\n", T);
				return CmdAnswer;
			}
		}
		snprintf(CmdAnswer, ARRAY_SIZE(CmdAnswer), "TIMER %ld NOT FOUND\n", T);
		return CmdAnswer;
	}
	return "Usage: RELEASETIMER timer_nr\n";
}

/*------------------------------------------------------------------------------------ BENCHMARKS */
void BenchmarkMalloc(size_t bufsize, int count);
void BenchmarkRingBuffer(int bufsize, int count);
char* BenchmarkFlexRingBuffer(char**tokens,int cnt,_ports_t port);
char* BenchmarkTimeToDistance(char**tokens,int cnt,_ports_t port);
char* BenchmarkStopWatch(char**tokens,int cnt,_ports_t port);

char *benchmark_cmd(char**tokens,int cnt,_ports_t port)
{
	if (cnt < 2)
		return PrintCmdError(
				"BENCHMARK p1 p2 p3 pn\n"
				" p1 = 1 - BenchmarkMalloc()\n"
				" p1 = 2 - BenchmarkRingBuffer()\n"
				" p1 = 3 - BenchmarkFlexRingBuffer()\n"
				" p1 = 4 - BenchmarkTimeToDistance()\n"
				" p1 = 5 - BenchmarkStopWatch()\n"
				" p2, pn - benchmark parameters\n"
				);

	int32_t p1=0, p2=0, p3=0;

	if ( ToInt32(tokens[1], &p1) == 0 )
		return PrintCmdError( "bad parameter 1.");

	if ( cnt > 3 && ToInt32(tokens[2], &p2) == 0 )
		return PrintCmdError( "bad parameter 2.");

	if ( cnt > 4 && ToInt32(tokens[3], &p3) == 0 )
		return PrintCmdError( "bad parameter 3.");

	switch(p1)
	{
		case 1:
		printf("Malloc benchmark %ld %ld\n", p2, p3);
		BenchmarkMalloc(p2,p3);
		break;

		case 2:
		printf("BenchmarkRingBuffer  %ld %ld\n", p2, p3);
		BenchmarkRingBuffer(p2,p3);
		break;

		case 3:
		printf("BenchmarkFlexRingBuffer %ld %ld\n", p2, p3);
		return BenchmarkFlexRingBuffer(tokens, cnt, port);
		break;

		case 4:
		printf("BenchmarkTimeToDistance %ld \n", p2);
		return BenchmarkTimeToDistance(tokens, cnt, port);
		break;
		case 5:
		printf("BenchmarkStopWatch %ld seconds\n", p2);
		return BenchmarkStopWatch(tokens, cnt, port);
		break;
	}

	return PrintCmdOK();
}

/*---------------------------------------------------------------- BENCHMARK TIME TO DISTANCE */
double TimeToDistance(double speed, double accel, double distance);
double DistanceTravelled(double speed, double accel, double period);

static double report_frequency = 10.0;			// default 10 Hz
static double trigger_dist = 0;

static _ports_t delta_dist_target = INVALID_PORT;


static double distance_travelled = 0.0;
static double last_update_time = 0.0;

bool distance_report_process(pvelocity_rep_t vel)
{
	 char report_buf[256];

	if (last_update_time == 0.0)
	{
		last_update_time = vel->WeekSeconds;
		return false;
	}

	double period = vel->WeekSeconds - last_update_time; // period in seconds
	last_update_time = vel->WeekSeconds;

	if (period <= 0.0 || trigger_dist <= 0.0)
	{
		return false; // Invalid period, skip this report
	}

	// Update the distance travelled based on the current speed and acceleration
	double travel = DistanceTravelled(vel->HorSpeed, vel->HorAccel, period); // period in seconds

	distance_travelled += travel;

	// Calculate the time required to reach the target distance
	if (distance_travelled >= trigger_dist)
	{
		int cnt = snprintf(report_buf, ARRAY_SIZE(report_buf), 
			"+DELTADIST,%6.3lf, %.3lf, %.3lf, %.3lf, %.3lf\n", 
			vel->WeekSeconds, 	// current week seconds
			distance_travelled,	// distance travelled since last report
			0.0,				// residual time to reach target or oveshoot
			vel->HorSpeed, 		// current horizontal speed
			vel->HorAccel 		// current horizontal acceleration
			);

		distance_travelled -= trigger_dist; // Reset distance travelled after reaching the target

		// Already reached the target distance
		if (cnt > 0 && cnt < ARRAY_SIZE(report_buf))
			SendToPort(delta_dist_target, (uint8_t*)report_buf, cnt);

		return true; // Indicate that the target distance has been reached
	}
		
	double timetodist = TimeToDistance(vel->HorSpeed, vel->HorAccel, trigger_dist - distance_travelled);
	
	if (timetodist < period/2.0)
	{	
		int cnt = snprintf(report_buf, ARRAY_SIZE(report_buf), 
			"-DELTADIST,%6.3lf, %.3lf, %.3lf, %.3lf, %.3lf\n", 
			vel->WeekSeconds, 	// current week seconds
			distance_travelled,	// distance travelled since last report
			timetodist,			// residual time to reach target or oveshoot
			vel->HorSpeed, 		// current horizontal speed
			vel->HorAccel 		// current horizontal acceleration
			);

		distance_travelled = 0; // Already reached the target distance

		if (cnt > 0 && cnt < ARRAY_SIZE(report_buf))
			SendToPort(delta_dist_target, (uint8_t*)report_buf, cnt);

		return true; // Indicate that the target distance has been reached

	}
	return false; // Target distance not yet reached
}

static inline double frand(double min, double max)
{
    // uniform in [0,1]
    double u = (double)rand() / (double)RAND_MAX;

    // scale to [min, max]
    return min + (max - min) * u;
}


char* BenchmarkTimeToDistance(char**tokens,int cnt, _ports_t port)
{
	char buf[100];

	delta_dist_target = port;
	distance_travelled = 0.0;
 	last_update_time= 0.0;

	velocity_rep_t vel = {
		.WeekSeconds = 1000.0,
		.HorSpeed = 0.0,
		.HorAccel = 0.0,
		.VertSpeed = 0.0,
		.VertAccel = 0.0
	};

	if (cnt < 4)	
		return PrintCmdError("Usage: BENCHMARK 4 <loops> <trigger_distance> <initial_speed>\n");

	double speed=10, accel;
	if (cnt >= 5)
	{
		if ( ToDouble(tokens[4], &speed) == 0 )
			return PrintCmdError( "bad initial speed parameter 4.");
		if (speed < 0.0)
			speed = 0.0;
	}
	int32_t count;

	if ( ToInt32(tokens[2], &count) == 0 )
		return PrintCmdError( "bad loops parameter 2.");

	if (count < 1)
		return PrintCmdError( "bad loops parameter 2. Must be > 1.");

	if ( ToDouble(tokens[3], &trigger_dist) == 0 )
		return PrintCmdError( "bad trigger distance parameter 3.");

	if (trigger_dist < 1.0)
		trigger_dist = 1.0;

	if ( ToDouble(tokens[4], &speed) == 0 )
		return PrintCmdError( "bad initial speed parameter 4.");

	if (speed < 1.0)
		speed = 1.0;

	InitStopWatch(true);

	//speed = frand(0.5, 10.0);	// random speed between 0.1 and 10.0 m/s
	accel = frand(-1.0, 1.0);	// random acceleration between -1.0 and 1.0 m/s^2
	
	// calculate distance traveled in time_s
	//dist = (speed + (0.5 * accel * time_s * time_s)) * time_s;


	double period = 1.0 / report_frequency; // period in seconds
	
	for (int i = 0; i < count; i++)
	{
		vel.WeekSeconds += period;
		vel.HorSpeed = speed;
		vel.HorAccel = accel;

		distance_report_process(&vel);
		
		speed += (0.5 * accel * period); // Update speed based on acceleration and time)
		if (speed < 0.0)
			speed = 0.0; // Prevent negative speed

		accel = frand(-2.0, 2.0);
	}

	double elapsed = 1000 * GetStopWatchTime();
	StopStopWatch();

	snprintf(CmdAnswer, SIZEOF_CMD_BUFFER, "TimeToDistance() %ld loops in %.3lf ms\n", count, elapsed);
	return CmdAnswer;
}


/*---------------------------------------------------------------------------------------------- STOPWATCH BENCHMARK */
char* BenchmarkStopWatch(char**tokens,int cnt, _ports_t port)
{
	char buf[100];

	if (cnt < 3)
		return PrintCmdError("Usage: BENCHMARK 5 <milliseconds>\n");

	int32_t milliseconds;

	if ( ToInt32(tokens[2], &milliseconds) == 0 )
		return PrintCmdError( "bad parameter 2.");

	if (milliseconds < 1)
		return PrintCmdError( "bad parameter 2. Must be > 0.");

	if (!InitStopWatch(true))
		return PrintCmdError( "Failed to initialize stopwatch.");

	cyhal_system_delay_ms(milliseconds);	// wait for 1 second to stabilize the timer

	double total_elapsed = GetStopWatchTime();
	StopStopWatch();

	snprintf(CmdAnswer, SIZEOF_CMD_BUFFER, "BenchmarkStopWatch completed %ld milliseconds in %.6lf seconds\n", milliseconds, total_elapsed);
	return CmdAnswer;
}


/*====================================================================== BYTESWAP TEST */

char *GrabInput3(uint8_t* buf, _ports_t port)
{
	char *ans = CmdAnswer;
	size_t size = ARRAY_SIZE(CmdAnswer);


	if (buf[0] == 'x' || buf[0] == 'X')
	{
		_release_console();
		SysConfig.imu_connect = Target_NovAtel;

		return "End of Test.\n";
	}

	uint32_t val, c3;

	sscanf((char*)buf,"%lu", &val);
	c3 = _bswap32(val);

	snprintf(ans, size, "Received  \t %.8lX\n__bswap32()\t = %.8lX\n",val,c3);

	return ans;
}

char* test_bswap( _ports_t port )
{
	char *buf = CmdAnswer;
	size_t size = ARRAY_SIZE(CmdAnswer);

	snprintf(buf, size,"Byte swap test. Enter a 32 bit number - X or x to end\n");

	_grab_console(GrabInput3, NULL);

	return buf;
}
/*----------------------------------------------------------------------------- GENOUTPUT */
/* GenerateS a random string with a CRC at the end "RANDOMCHARS[CRC]\n"
 */
char* genoutput_cmd(char** tokens, int cnt, _ports_t port)
{
	_ports_t target = port;
	int32_t nch = 25;

	if(cnt < 2)
		return "Usage: GENOUTPUT nchars [port]\n";

	/* scan arguments */
	for (int i = 1; i < cnt; i++ )
	{
		_ports_t _p = FindPortID(tokens[i]);

		if (_p != INVALID_PORT)
		{
			target = _p;
			continue;
		}
		else
		{
			if ( ToInt32(tokens[i], &nch) == 0 )
				return  "bad parameter.\n";
		}
	}

	/* generate a buffer random chars and CRC */
	uint8_t* pbuf = malloc(nch+1);

	for (int i = 0; i < nch; i++)
	{
		pbuf[i] = '@' + rand() % 26;
	}
	pbuf[nch] = '\n';

	uint16_t crc = crc16_ccitt(pbuf,nch);
	SendToPort(target, pbuf,nch);
	int n = snprintf(CmdBuffer,sizeof(CmdBuffer),"[%u]\n", crc );
	SendToPort(target, (uint8_t*)CmdBuffer,(size_t)n);

	return  PrintCmdOK();
}
