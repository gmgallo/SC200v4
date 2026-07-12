/**
 * Tests and Benchmarks
 * Copyright (C) 2024 - 2026, G2 AIRBORNE SYSTEMS
 */
#pragma once
#include "system.h"

char *benchmark_cmd(char**tokens,int cnt,_ports_t port);
char *test_cmd(char**tokens, int cnt, _ports_t port);
char *imuredir_cmd(char**tokens, int cnt, _ports_t port);
void TriggerBaudsMonitor(uint32_t compare);
char *Start_TDAS_Frequency(char**tokens, int cnt, _ports_t port);
char* clock_monitor_cmd(char** tokens, int cnt, _ports_t port);
char* genoutput_cmd(char** tokens, int cnt, _ports_t port);