/*****************************************************************************
 *
 *  Copyright (C) 2009-2010  Moehwald GmbH B. Benner
 *                     2011  IgH Andreas Stewering-Bone
 *                     2012  Florian Pose <fp@igh.de>
 *                     2025  Cix Technology Group Co., Ltd. (EL2008, latency/jitter stats)
 *
 *  This file is part of the IgH EtherCAT master
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT master. If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************************/

#include <ctype.h>
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sched.h>
#ifdef XENOMAI_API_V3
#include <alchemy/task.h>
#include <alchemy/sem.h>
#include <alchemy/timer.h>
enum {T_FPU = 0};
#else
#include <rtdm/rtdm.h>
#include <native/task.h>
#include <native/sem.h>
#include <native/mutex.h>
#include <native/timer.h>
#include <rtdk.h>
#endif
#include <pthread.h>

#include "ecrt.h"

RT_TASK my_task;

static volatile sig_atomic_t run = 1;

/****************************************************************************/
/* Timing / latency stats
 * CRITICAL: In RT task use Xenomai clock (rt_timer_read), NOT Linux
 * CLOCK_MONOTONIC. rt_task_wait_period() is driven by Xenomai core clock;
 * mixing Linux monotonic causes large systematic jitter (~8ms offset).
 */
#define DEFAULT_FREQUENCY 1000
#define DEFAULT_RT_CPU 9
#define WARMUP_CYCLES 1500
/* Runtime (CLI can override); names kept same as original macros. */
static unsigned int FREQUENCY = DEFAULT_FREQUENCY;
static unsigned long long CYCLE_TIME_NS = 1000000000ULL / DEFAULT_FREQUENCY;
static int rt_cpu = DEFAULT_RT_CPU;

static inline long long ts_to_ns(const struct timespec *ts)
{
	return (long long)ts->tv_sec * 1000000000LL + (long long)ts->tv_nsec;
}

static inline void gettime_ns(struct timespec *ts)
{
	clock_gettime(CLOCK_MONOTONIC, ts);
}

#ifdef XENOMAI_API_V3
/* Use Xenomai core clock in RT task - same time base as rt_task_wait_period() */
static inline void gettime_rt_ns(struct timespec *ts)
{
	long long ns = (long long)rt_timer_read();
	ts->tv_sec = (time_t)(ns / 1000000000LL);
	ts->tv_nsec = (long)(ns % 1000000000LL);
	if (ts->tv_nsec < 0) {
		ts->tv_sec--;
		ts->tv_nsec += 1000000000L;
	}
}
#else
#define gettime_rt_ns gettime_ns
#endif

static inline long long diff_ns(const struct timespec *end, const struct timespec *start)
{
	return ts_to_ns(end) - ts_to_ns(start);
}

static inline void ns_to_timespec(long long ns, struct timespec *ts)
{
	ts->tv_sec = (time_t)(ns / 1000000000LL);
	ts->tv_nsec = (long)(ns % 1000000000LL);
	if (ts->tv_nsec < 0) {
		ts->tv_sec--;
		ts->tv_nsec += 1000000000L;
	}
}

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static uint8_t *domain1_pd = NULL;

static ec_slave_config_t *sc_dig_out_01 = NULL;

/****************************************************************************/

// process data

#define BusCoupler01_Pos  0, 0
#define DigOutSlave01_Pos 0, 1

#define Beckhoff_EK1100 0x00000002, 0x044c2c52
#define Beckhoff_EL2008 0x00000002, 0x07D83052

// offsets for PDO entries (EL2008: one byte for 8 channels)
static unsigned int off_dig_out_8 = 0;

const static ec_pdo_entry_reg_t domain1_regs[] = {
	{DigOutSlave01_Pos, Beckhoff_EL2008, 0x3001, 1, &off_dig_out_8, NULL},
	{}
};

/****************************************************************************/

/* Slave 1, "EL2008" - 8-channel digital output
 * Vendor ID:       0x00000002
 * Product code:    0x07D83052
 */

const ec_pdo_entry_info_t slave_1_pdo_entries[] = {
	{0x3001, 1, 1}, /* Ch1 */
	{0x3001, 2, 1}, /* Ch2 */
	{0x3001, 3, 1}, /* Ch3 */
	{0x3001, 4, 1}, /* Ch4 */
	{0x3001, 5, 1}, /* Ch5 */
	{0x3001, 6, 1}, /* Ch6 */
	{0x3001, 7, 1}, /* Ch7 */
	{0x3001, 8, 1}, /* Ch8 */
};

const ec_pdo_info_t slave_1_pdos[] = {
	{0x1600, 1, slave_1_pdo_entries + 0},
	{0x1601, 1, slave_1_pdo_entries + 1},
	{0x1602, 1, slave_1_pdo_entries + 2},
	{0x1603, 1, slave_1_pdo_entries + 3},
	{0x1604, 1, slave_1_pdo_entries + 4},
	{0x1605, 1, slave_1_pdo_entries + 5},
	{0x1606, 1, slave_1_pdo_entries + 6},
	{0x1607, 1, slave_1_pdo_entries + 7},
};

ec_sync_info_t slave_1_syncs[] = {
	{0, EC_DIR_OUTPUT, 8, (ec_pdo_info_t *)slave_1_pdos, EC_WD_ENABLE},
	{0xff}
};

/****************************************************************************/
/* Latency / jitter statistics for cyclic task */
struct ethercat_stats {
	long long min_jitter_ns;
	long long max_jitter_ns;
	long long avg_jitter_ns;
	long long min_signed_jitter_ns;
	long long max_signed_jitter_ns;
	long long avg_signed_jitter_ns;
	long long min_duration_ns;
	long long max_duration_ns;
	long long avg_duration_ns;
	struct timespec expected_wake;
	struct timespec actual_wake;
	struct timespec start_time;
	struct timespec end_time;
	unsigned long cycle_count;
	unsigned long long total_jitter_ns;
	long long total_signed_jitter_ns;
	unsigned long long total_duration_ns;
	/* Jitter distribution (ns) */
	unsigned long jitter_0_5us;
	unsigned long jitter_5_10us;
	unsigned long jitter_10_20us;
	unsigned long jitter_20us_plus;
	/* Duration distribution */
	unsigned long duration_0_5us;
	unsigned long duration_5_10us;
	unsigned long duration_10_20us;
	unsigned long duration_20_50us;
	unsigned long duration_50us_plus;
};

static struct ethercat_stats stats;

#define LLONG_MAX_JITTER  ((long long)(0x7FFFFFFFFFFFFFFFLL))
#define LLONG_MIN_JITTER  ((long long)(-0x7FFFFFFFFFFFFFFFLL - 1LL))

static void clear_stats(void)
{
	memset(&stats, 0, sizeof(stats));
	stats.min_jitter_ns = LLONG_MAX_JITTER;
	stats.min_signed_jitter_ns = LLONG_MAX_JITTER;
	stats.max_signed_jitter_ns = LLONG_MIN_JITTER;
	stats.min_duration_ns = LLONG_MAX_JITTER;
}

static void update_statistics(void)
{
	long long jitter_ns, abs_jitter_ns, duration_ns;

	if (stats.cycle_count < WARMUP_CYCLES) {
		stats.cycle_count++;
		return;
	}

	jitter_ns = diff_ns(&stats.actual_wake, &stats.expected_wake);
	abs_jitter_ns = (jitter_ns < 0) ? -jitter_ns : jitter_ns;

	if (abs_jitter_ns > stats.max_jitter_ns)
		stats.max_jitter_ns = abs_jitter_ns;
	if (abs_jitter_ns < stats.min_jitter_ns)
		stats.min_jitter_ns = abs_jitter_ns;

	stats.total_jitter_ns += abs_jitter_ns;
	stats.total_signed_jitter_ns += jitter_ns;

	if (jitter_ns < stats.min_signed_jitter_ns)
		stats.min_signed_jitter_ns = jitter_ns;
	if (jitter_ns > stats.max_signed_jitter_ns)
		stats.max_signed_jitter_ns = jitter_ns;

	if (abs_jitter_ns <= 5000)
		stats.jitter_0_5us++;
	else if (abs_jitter_ns <= 10000)
		stats.jitter_5_10us++;
	else if (abs_jitter_ns <= 20000)
		stats.jitter_10_20us++;
	else
		stats.jitter_20us_plus++;

	duration_ns = diff_ns(&stats.end_time, &stats.start_time);
	if (duration_ns > stats.max_duration_ns)
		stats.max_duration_ns = duration_ns;
	if (duration_ns < stats.min_duration_ns)
		stats.min_duration_ns = duration_ns;

	stats.total_duration_ns += duration_ns;

	if (duration_ns <= 5000)
		stats.duration_0_5us++;
	else if (duration_ns <= 10000)
		stats.duration_5_10us++;
	else if (duration_ns <= 20000)
		stats.duration_10_20us++;
	else if (duration_ns <= 50000)
		stats.duration_20_50us++;
	else
		stats.duration_50us_plus++;

	stats.cycle_count++;
}

static void dump_stats(void)
{
	unsigned long count;
	int pct_0_5us, pct_5_10us, pct_10_20us, pct_20us_plus;
	int pct_dur_0_5us, pct_dur_5_10us, pct_dur_10_20us, pct_dur_20_50us, pct_dur_50us_plus;
	char buf[2048];
	int n;
	const char *path;
	FILE *fp;

	if (stats.cycle_count <= WARMUP_CYCLES)
		return;

	count = stats.cycle_count - WARMUP_CYCLES;
	stats.avg_jitter_ns = (long long)(stats.total_jitter_ns / count);
	stats.avg_signed_jitter_ns = (long long)(stats.total_signed_jitter_ns / (long long)count);
	stats.avg_duration_ns = (long long)(stats.total_duration_ns / count);

	pct_0_5us = (int)((stats.jitter_0_5us * 10000) / count);
	pct_5_10us = (int)((stats.jitter_5_10us * 10000) / count);
	pct_10_20us = (int)((stats.jitter_10_20us * 10000) / count);
	pct_20us_plus = (int)((stats.jitter_20us_plus * 10000) / count);
	pct_dur_0_5us = (int)((stats.duration_0_5us * 10000) / count);
	pct_dur_5_10us = (int)((stats.duration_5_10us * 10000) / count);
	pct_dur_10_20us = (int)((stats.duration_10_20us * 10000) / count);
	pct_dur_20_50us = (int)((stats.duration_20_50us * 10000) / count);
	pct_dur_50us_plus = (int)((stats.duration_50us_plus * 10000) / count);

	n = 0;
	n += snprintf(buf + n, sizeof(buf) - n, "========================================\n");
	n += snprintf(buf + n, sizeof(buf) - n,
		"Xenomai3 cyclic task (period=%uus) statistics over %lu cycles:\n",
		(unsigned)(CYCLE_TIME_NS / 1000), count);
	n += snprintf(buf + n, sizeof(buf) - n, "----------------------------------------\n");
	n += snprintf(buf + n, sizeof(buf) - n,
		"Sched Jitter(ns)     Min:%10lld Avg:%10lld Max:%10lld\n",
		stats.min_jitter_ns == LLONG_MAX_JITTER ? 0 : stats.min_jitter_ns,
		stats.avg_jitter_ns, stats.max_jitter_ns);
	n += snprintf(buf + n, sizeof(buf) - n,
		"Signed Jitter(ns)    Min:%10lld Avg:%10lld Max:%10lld\n",
		stats.min_signed_jitter_ns == LLONG_MAX_JITTER ? 0 : stats.min_signed_jitter_ns,
		stats.avg_signed_jitter_ns,
		stats.max_signed_jitter_ns == LLONG_MIN_JITTER ? 0 : stats.max_signed_jitter_ns);
	n += snprintf(buf + n, sizeof(buf) - n,
		"Task Duration(ns)    Min:%10lld Avg:%10lld Max:%10lld\n",
		stats.min_duration_ns == LLONG_MAX_JITTER ? 0 : stats.min_duration_ns,
		stats.avg_duration_ns, stats.max_duration_ns);
	n += snprintf(buf + n, sizeof(buf) - n, "Sched Jitter Distribution:\n");
	n += snprintf(buf + n, sizeof(buf) - n,
		"  0-5us     : %10lu (%d.%02d%%)\n",
		stats.jitter_0_5us, pct_0_5us / 100, pct_0_5us % 100);
	n += snprintf(buf + n, sizeof(buf) - n,
		"  5-10us    : %10lu (%d.%02d%%)\n",
		stats.jitter_5_10us, pct_5_10us / 100, pct_5_10us % 100);
	n += snprintf(buf + n, sizeof(buf) - n,
		"  10-20us   : %10lu (%d.%02d%%)\n",
		stats.jitter_10_20us, pct_10_20us / 100, pct_10_20us % 100);
	n += snprintf(buf + n, sizeof(buf) - n,
		"  >20us     : %10lu (%d.%02d%%)\n",
		stats.jitter_20us_plus, pct_20us_plus / 100, pct_20us_plus % 100);
	n += snprintf(buf + n, sizeof(buf) - n, "Duration Distribution:\n");
	n += snprintf(buf + n, sizeof(buf) - n,
		"  0-5us     : %10lu (%d.%02d%%)\n",
		stats.duration_0_5us, pct_dur_0_5us / 100, pct_dur_0_5us % 100);
	n += snprintf(buf + n, sizeof(buf) - n,
		"  5-10us    : %10lu (%d.%02d%%)\n",
		stats.duration_5_10us, pct_dur_5_10us / 100, pct_dur_5_10us % 100);
	n += snprintf(buf + n, sizeof(buf) - n,
		"  10-20us   : %10lu (%d.%02d%%)\n",
		stats.duration_10_20us, pct_dur_10_20us / 100, pct_dur_10_20us % 100);
	n += snprintf(buf + n, sizeof(buf) - n,
		"  20-50us   : %10lu (%d.%02d%%)\n",
		stats.duration_20_50us, pct_dur_20_50us / 100, pct_dur_20_50us % 100);
	n += snprintf(buf + n, sizeof(buf) - n,
		"  >50us     : %10lu (%d.%02d%%)\n",
		stats.duration_50us_plus, pct_dur_50us_plus / 100, pct_dur_50us_plus % 100);
	n += snprintf(buf + n, sizeof(buf) - n, "========================================\n");

	/* Write to file so kernel printk (EtherCAT "Master thread exited" etc.) cannot
	 * interleave; stdout shares console with kernel and gets interrupted. */
	path = getenv("EC_STATS_FILE");
	if (!path)
		path = "ec_cyclic_stats.txt";
	fp = fopen(path, "w");
	if (fp) {
		fwrite(buf, 1, (size_t)n, fp);
		fclose(fp);
		printf("Statistics written to %s\n", path);
	} else {
		fprintf(stderr, "Failed to write stats to %s: %s\n", path, strerror(errno));
		flockfile(stdout);
		fputs(buf, stdout);
		funlockfile(stdout);
	}
}

/****************************************************************************/

void rt_check_domain_state(void)
{
	ec_domain_state_t ds = {};

	ecrt_domain_state(domain1, &ds);

	if (ds.working_counter != domain1_state.working_counter) {
		rt_printf("Domain1: WC %u.\n", ds.working_counter);
	}

	if (ds.wc_state != domain1_state.wc_state) {
		rt_printf("Domain1: State %u.\n", ds.wc_state);
	}

	domain1_state = ds;
}

/****************************************************************************/

void rt_check_master_state(void)
{
	ec_master_state_t ms;

	ecrt_master_state(master, &ms);

	if (ms.slaves_responding != master_state.slaves_responding) {
		rt_printf("%u slave(s).\n", ms.slaves_responding);
	}

	if (ms.al_states != master_state.al_states) {
		rt_printf("AL states: 0x%02X.\n", ms.al_states);
	}

	if (ms.link_up != master_state.link_up) {
		rt_printf("Link is %s.\n", ms.link_up ? "up" : "down");
	}

	master_state = ms;
}

/****************************************************************************/

void my_task_proc(void *arg)
{
	unsigned int blink = 0;
	long long next_ns;
#ifdef XENOMAI_API_V3
	long long first_release_ns;
#endif

#ifdef XENOMAI_API_V3
	/* Align expected wakeup with periodic schedule start exactly. */
	first_release_ns = (long long)rt_timer_read() + CYCLE_TIME_NS;
	rt_task_set_periodic(NULL, first_release_ns, CYCLE_TIME_NS);
	ns_to_timespec(first_release_ns, &stats.expected_wake);
#else
	rt_task_set_periodic(NULL, TM_NOW, CYCLE_TIME_NS);
	gettime_rt_ns(&stats.expected_wake);
	next_ns = ts_to_ns(&stats.expected_wake) + CYCLE_TIME_NS;
	ns_to_timespec(next_ns, &stats.expected_wake);
#endif

	while (run) {
		rt_task_wait_period(NULL);

		gettime_rt_ns(&stats.actual_wake);
		stats.start_time = stats.actual_wake;

		/* receive EtherCAT frames */
		ecrt_master_receive(master);
		ecrt_domain_process(domain1);

		rt_check_domain_state();

		if (!(stats.cycle_count % 1000)) {
			rt_check_master_state();
		}

		if (!(stats.cycle_count % 200)) {
			blink = !blink;
		}

		/* EL2008: one byte drives all 8 outputs */
		EC_WRITE_U8(domain1_pd + off_dig_out_8, blink ? 0xFF : 0x00);

		/* send process data */
		ecrt_domain_queue(domain1);
		ecrt_master_send(master);

		gettime_rt_ns(&stats.end_time);

		update_statistics();

		/* next expected wake */
		next_ns = ts_to_ns(&stats.expected_wake) + CYCLE_TIME_NS;
		ns_to_timespec(next_ns, &stats.expected_wake);
	}
}

/****************************************************************************
 * Signal handler
 ***************************************************************************/

void signal_handler(int sig)
{
	run = 0;
}

/****************************************************************************
 * Command-line parsing
 ***************************************************************************/

static void print_usage(const char *prog)
{
	fprintf(stderr,
		"Usage: %s [-c CPU] [-p PERIOD] [-h]\n"
		"\n"
		"Options:\n"
		"  -c, --cpu CPU       Bind process/RT task to CPU (default: %d)\n"
		"  -p, --period PERIOD Cycle period (default: 1ms)\n"
		"                      Supported: 1ms, 125us, 32us\n"
		"                      Also accepts: Nms / Nus / plain us number\n"
		"  -h, --help          Show this help\n"
		"\n"
		"Examples:\n"
		"  %s -c 9 -p 1ms\n"
		"  %s --cpu 9 --period 125us\n"
		"  %s -p 32us\n"
		"  %s 9                 (legacy: CPU only)\n",
		prog, DEFAULT_RT_CPU, prog, prog, prog, prog);
}

/* Parse period string into nanoseconds. Returns 0 on success. */
static int parse_period(const char *str, unsigned long long *period_ns)
{
	char *end = NULL;
	unsigned long long value;
	const char *p;
	char unit[8];
	size_t i, n;

	if (!str || !*str || !period_ns)
		return -1;

	if (!strcasecmp(str, "1ms") || !strcasecmp(str, "1000us")) {
		*period_ns = 1000000ULL;
		return 0;
	}
	if (!strcasecmp(str, "125us")) {
		*period_ns = 125000ULL;
		return 0;
	}
	if (!strcasecmp(str, "32us")) {
		*period_ns = 32000ULL;
		return 0;
	}

	value = strtoull(str, &end, 10);
	if (end == str)
		return -1;

	p = end;
	while (*p && isspace((unsigned char)*p))
		p++;

	if (*p == '\0') {
		/* Bare number: treat as microseconds */
		*period_ns = value * 1000ULL;
		return 0;
	}

	n = strlen(p);
	if (n >= sizeof(unit))
		return -1;
	for (i = 0; i < n; i++)
		unit[i] = (char)tolower((unsigned char)p[i]);
	unit[n] = '\0';

	if (!strcmp(unit, "ns"))
		*period_ns = value;
	else if (!strcmp(unit, "us") || !strcmp(unit, "usec"))
		*period_ns = value * 1000ULL;
	else if (!strcmp(unit, "ms") || !strcmp(unit, "msec"))
		*period_ns = value * 1000000ULL;
	else
		return -1;

	return 0;
}

static int parse_args(int argc, char *argv[])
{
	static const struct option long_opts[] = {
		{"cpu",    required_argument, NULL, 'c'},
		{"period", required_argument, NULL, 'p'},
		{"help",   no_argument,       NULL, 'h'},
		{NULL, 0, NULL, 0}
	};
	int opt;
	unsigned long long period_ns = CYCLE_TIME_NS;
	char *end = NULL;
	long cpu;

	/* Legacy: single numeric positional arg means CPU affinity */
	if (argc == 2 && argv[1][0] != '-') {
		cpu = strtol(argv[1], &end, 10);
		if (end != argv[1] && *end == '\0' && cpu >= 0) {
			rt_cpu = (int)cpu;
			return 0;
		}
	}

	while ((opt = getopt_long(argc, argv, "c:p:h", long_opts, NULL)) != -1) {
		switch (opt) {
		case 'c':
			cpu = strtol(optarg, &end, 10);
			if (end == optarg || *end != '\0' || cpu < 0) {
				fprintf(stderr, "Invalid CPU: %s\n", optarg);
				return -1;
			}
			rt_cpu = (int)cpu;
			break;
		case 'p':
			if (parse_period(optarg, &period_ns)) {
				fprintf(stderr, "Invalid period: %s\n", optarg);
				fprintf(stderr, "Use 1ms, 125us, 32us (or Nms/Nus).\n");
				return -1;
			}
			if (period_ns == 0) {
				fprintf(stderr, "Period must be > 0\n");
				return -1;
			}
			break;
		case 'h':
			print_usage(argv[0]);
			exit(0);
		default:
			print_usage(argv[0]);
			return -1;
		}
	}

	if (optind < argc) {
		fprintf(stderr, "Unexpected argument: %s\n", argv[optind]);
		print_usage(argv[0]);
		return -1;
	}

	CYCLE_TIME_NS = period_ns;
	FREQUENCY = (unsigned int)(1000000000ULL / CYCLE_TIME_NS);
	return 0;
}

/****************************************************************************
 * Main function
 ***************************************************************************/

int main(int argc, char *argv[])
{
	ec_slave_config_t *sc;
	int ret;
	cpu_set_t cpuset;

#ifndef XENOMAI_API_V3
	rt_print_auto_init(1);
#endif

	signal(SIGTERM, signal_handler);
	signal(SIGINT, signal_handler);

	if (parse_args(argc, argv) < 0)
		return -1;

	mlockall(MCL_CURRENT | MCL_FUTURE);
	/* Bind process to RT core */
	CPU_ZERO(&cpuset);
	CPU_SET(rt_cpu, &cpuset);
	if (sched_setaffinity(0, sizeof(cpuset), &cpuset) < 0)
		fprintf(stderr, "Warning: sched_setaffinity(CPU%d) failed: %s\n",
			rt_cpu, strerror(errno));
	else
		printf("Process bound to CPU %d.\n", rt_cpu);

	clear_stats();

	printf("Requesting master...\n");
	master = ecrt_request_master(0);
	if (!master) {
		return -1;
	}

	domain1 = ecrt_master_create_domain(master);
	if (!domain1) {
		return -1;
	}

	printf("Creating slave configurations...\n");

	sc = ecrt_master_slave_config(master, BusCoupler01_Pos, Beckhoff_EK1100);
	if (!sc) {
		return -1;
	}

	sc_dig_out_01 =
		ecrt_master_slave_config(master, DigOutSlave01_Pos, Beckhoff_EL2008);
	if (!sc_dig_out_01) {
		fprintf(stderr, "Failed to get slave configuration for EL2008.\n");
		return -1;
	}

	if (ecrt_slave_config_pdos(sc_dig_out_01, EC_END, slave_1_syncs)) {
		fprintf(stderr, "Failed to configure PDOs for EL2008.\n");
		return -1;
	}

	if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
		fprintf(stderr, "PDO entry registration failed!\n");
		return -1;
	}

	printf("Activating master...\n");
	if (ecrt_master_activate(master)) {
		return -1;
	}

	if (!(domain1_pd = ecrt_domain_data(domain1))) {
		fprintf(stderr, "Failed to get domain data pointer.\n");
		return -1;
	}

#ifdef XENOMAI_API_V3
	ret = rt_task_create(&my_task, "my_task", 0, 80, T_JOINABLE);
#else
	ret = rt_task_create(&my_task, "my_task", 0, 80, T_FPU);
#endif
	if (ret < 0) {
		fprintf(stderr, "Failed to create task: %s\n", strerror(-ret));
		return -1;
	}

	ret = rt_task_set_affinity(&my_task, &cpuset);
	if (ret < 0)
		fprintf(stderr, "Failed to set task affinity to CPU%d: %s\n", rt_cpu, strerror(-ret));
	else
		printf("RT task bound to CPU %d.\n", rt_cpu);

	printf("Starting cyclic task (period=%lluus / %u Hz, warmup %d cycles)...\n",
		CYCLE_TIME_NS / 1000ULL, FREQUENCY, WARMUP_CYCLES);
	ret = rt_task_start(&my_task, &my_task_proc, NULL);
	if (ret < 0) {
		fprintf(stderr, "Failed to start task: %s\n", strerror(-ret));
		return -1;
	}

	/* Poll run; SIGINT may be delivered to the Alchemy thread. */
	while (run) {
		struct timespec wait_ts = { .tv_sec = 0, .tv_nsec = 100000000L };

		nanosleep(&wait_ts, NULL);
	}

	printf("Stopping realtime task...\n");
#ifdef XENOMAI_API_V3
	ret = rt_task_join(&my_task);
	if (ret < 0)
		fprintf(stderr, "Failed to join task: %s\n", strerror(-ret));
#else
	rt_task_delete(&my_task);
#endif

	dump_stats();
	clear_stats();

	printf("Releasing master...\n");
	ecrt_release_master(master);

	printf("End of Program\n");
	return 0;
}

/****************************************************************************/
