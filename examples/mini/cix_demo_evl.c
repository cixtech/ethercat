// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Cix Technology Group Co., Ltd.
 *
 * EtherCAT master demo for Xenomai 4 (EVL)
 * Author: Lihua Liu <Lihua.Liu@cixtech.com>
 *
 * This version uses Xenomai 4 EVL (Embedded Virtual Linux) API
 * for ultra-low latency real-time performance.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/cpu.h>

/* Xenomai 4 EVL headers */
#include <evl/thread.h>
#include <evl/timer.h>
#include <evl/clock.h>
#include <evl/flag.h>
#include <evl/sched.h>

#include "../../include/ecrt.h"  // EtherCAT realtime interface

/****************************************************************************/

// Module parameters
#define FREQUENCY 1000
#define WARMUP_CYCLES 1500
#define CYCLE_TIME_NS (NSEC_PER_SEC / FREQUENCY)

// Optional features
#define CONFIGURE_PDOS 1
#define EXTERNAL_MEMORY 1
#define SDO_ACCESS 0
#define VOE_ACCESS 0

#define PFX "cix_evl: "

static int cpu_id = 11;
module_param(cpu_id, int, 0444);
MODULE_PARM_DESC(cpu_id, "CPU to pin EVL kthread and timer (default 11)");

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_dig_out = NULL;
static ec_slave_config_state_t sc_dig_out_state = {};

/* EVL real-time objects */
static struct evl_kthread cyclic_kthread;
static struct evl_timer cyclic_timer;
static struct evl_flag timer_flag;

static int thread_running = 0;

/****************************************************************************/

// process data
static uint8_t *domain1_pd;  // process data memory

// Slave positions
#define EK1100Pos 0, 0
#define EL2008Pos 0, 1

// Vendor ID and product codes
#define Beckhoff_EK1100 0x00000002, 0x044c2c52
#define Beckhoff_EL2008 0x00000002, 0x07D83052

// offsets for PDO entries
static unsigned int off_dig_out_8;

static const ec_pdo_entry_reg_t domain1_regs[] = {
        {EL2008Pos, Beckhoff_EL2008, 0x3001, 1, &off_dig_out_8},
        {}};

static unsigned int counter = 0;
static unsigned int blink = 0;

struct ethercat_stats
{
        s64 min_jitter_ns;
        s64 max_jitter_ns;
        s64 avg_jitter_ns;
        s64 min_duration_ns;
        s64 max_duration_ns;
        s64 avg_duration_ns;
        ktime_t start_time;
        ktime_t end_time;
        ktime_t expected_wake;     // Next cycle expected wake time
        ktime_t current_expected;  // Current cycle expected time (for jitter
                                   // calculation)
        ktime_t actual_wake;
        unsigned long cycle_count;
        unsigned long total_jitter_ns;
        unsigned long total_duration_ns;
        // Jitter distribution in nanoseconds
        unsigned long jitter_0_5us;
        unsigned long jitter_5_10us;
        unsigned long jitter_10us_plus;
        // Duration distribution
        unsigned long duration_0_5us;
        unsigned long duration_5_10us;
        unsigned long duration_10_20us;
        unsigned long duration_20_50us;
        unsigned long duration_50us_plus;
        // EVL-specific stats
        s64 max_timer_latency_ns;
        s64 max_schedule_latency_ns;
        unsigned long total_timer_latency_ns;
        unsigned long total_schedule_latency_ns;
        ktime_t timer_fire_time;
};

static struct ethercat_stats stats;

/****************************************************************************/

#if CONFIGURE_PDOS
// Digital out
static ec_pdo_entry_info_t el2008_channels[] = {
        {0x3001, 1, 1},
        {0x3001, 2, 1},
        {0x3001, 3, 1},
        {0x3001, 4, 1},
        {0x3001, 5, 1},
        {0x3001, 6, 1},
        {0x3001, 7, 1},
        {0x3001, 8, 1}};

static ec_pdo_info_t el2008_pdos[] = {
        {0x1600, 1, &el2008_channels[0]},
        {0x1601, 1, &el2008_channels[1]},
        {0x1602, 1, &el2008_channels[2]},
        {0x1603, 1, &el2008_channels[3]},
        {0x1604, 1, &el2008_channels[4]},
        {0x1605, 1, &el2008_channels[5]},
        {0x1606, 1, &el2008_channels[6]},
        {0x1607, 1, &el2008_channels[7]}};

static ec_sync_info_t el2008_syncs[] = {
        {0, EC_DIR_OUTPUT, 8, el2008_pdos},
        {1, EC_DIR_INPUT},
        {0xff}};
#endif

/****************************************************************************/

#if SDO_ACCESS
static ec_sdo_request_t *sdo;
#endif

#if VOE_ACCESS
static ec_voe_handler_t *voe;
#endif

/****************************************************************************/

void check_domain1_state(void);
void check_master_state(void);
void check_slave_config_states(void);
void send_callback(void *);
void receive_callback(void *);

/****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        printk(KERN_INFO PFX "Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state) {
        printk(KERN_INFO PFX "Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        printk(KERN_INFO PFX "%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states) {
        printk(KERN_INFO PFX "AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up) {
        printk(KERN_INFO PFX "Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/****************************************************************************/

void check_slave_config_states(void)
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc_dig_out, &s);

    if (s.al_state != sc_dig_out_state.al_state) {
        printk(KERN_INFO PFX "EL2008: State 0x%02X.\n", s.al_state);
    }
    if (s.online != sc_dig_out_state.online) {
        printk(KERN_INFO PFX "EL2008: %s.\n",
               s.online ? "online" : "offline");
    }
    if (s.operational != sc_dig_out_state.operational) {
        printk(KERN_INFO PFX "EL2008: %soperational.\n",
               s.operational ? "" : "Not ");
    }

    sc_dig_out_state = s;
}

/****************************************************************************/

#if SDO_ACCESS
void read_sdo(void)
{
    switch (ecrt_sdo_request_state(sdo)) {
        case EC_REQUEST_UNUSED:
            ecrt_sdo_request_read(sdo);
            break;
        case EC_REQUEST_BUSY:
            break;
        case EC_REQUEST_SUCCESS:
            // printk not allowed in EVL out-of-band context
            // printk(KERN_INFO PFX "SDO value: 0x%04X\n",
            //         EC_READ_U16(ecrt_sdo_request_data(sdo)));
            ecrt_sdo_request_read(sdo);
            break;
        case EC_REQUEST_ERROR:
            // printk not allowed in EVL out-of-band context
            // printk(KERN_INFO PFX "Failed to read SDO!\n");
            ecrt_sdo_request_read(sdo);
            break;
    }
}
#endif

/****************************************************************************/

#if VOE_ACCESS
void read_voe(void)
{
    switch (ecrt_voe_handler_execute(voe)) {
        case EC_REQUEST_UNUSED:
            ecrt_voe_handler_read(voe);
            break;
        case EC_REQUEST_BUSY:
            break;
        case EC_REQUEST_SUCCESS:
            // printk not allowed in EVL out-of-band context
            // printk(KERN_INFO PFX "VoE received.\n");
            ecrt_voe_handler_read(voe);
            break;
        case EC_REQUEST_ERROR:
            // printk not allowed in EVL out-of-band context
            // printk(KERN_INFO PFX "Failed to read VoE data!\n");
            ecrt_voe_handler_read(voe);
            break;
    }
}
#endif

/****************************************************************************/

void update_statistics(void)
{
    s64 jitter_ns;
    s64 duration_ns;

    // Warmup period
    if (stats.cycle_count < WARMUP_CYCLES) {
        stats.cycle_count++;
        return;
    }

    // Calculate jitter (precision in nanoseconds)
    // Use current_expected which was set in timer callback for THIS cycle
    jitter_ns =
            ktime_to_ns(ktime_sub(stats.actual_wake, stats.current_expected));
    if (jitter_ns < 0) {
        jitter_ns = -jitter_ns;
    }

    if (jitter_ns > stats.max_jitter_ns) {
        stats.max_jitter_ns = jitter_ns;
    }
    if (jitter_ns < stats.min_jitter_ns) {
        stats.min_jitter_ns = jitter_ns;
    }

    stats.total_jitter_ns += jitter_ns;

    // Jitter distribution
    if (jitter_ns <= 5000) {
        stats.jitter_0_5us++;
    }
    else if (jitter_ns <= 10000) {
        stats.jitter_5_10us++;
    }
    else {
        stats.jitter_10us_plus++;
    }

    duration_ns = ktime_to_ns(ktime_sub(stats.end_time, stats.start_time));
    if (duration_ns > stats.max_duration_ns) {
        stats.max_duration_ns = duration_ns;
    }
    if (duration_ns < stats.min_duration_ns) {
        stats.min_duration_ns = duration_ns;
    }
    stats.total_duration_ns += duration_ns;

    // Duration distribution
    if (duration_ns <= 5000) {
        stats.duration_0_5us++;
    }
    else if (duration_ns <= 10000) {
        stats.duration_5_10us++;
    }
    else if (duration_ns <= 20000) {
        stats.duration_10_20us++;
    }
    else if (duration_ns <= 50000) {
        stats.duration_20_50us++;
    }
    else {
        stats.duration_50us_plus++;
    }

    stats.cycle_count++;
}

/****************************************************************************/

void cyclic_handler(void)
{
    ktime_t now;
    s64 latency_ns;

    // Early exit if thread is stopping - avoid spurious latency from manual
    // wakeup
    if (!thread_running) {
        return;
    }

    now = ktime_get();
    stats.actual_wake = now;
    stats.start_time = now;

    // Measure latency from expected wake time
    if (stats.cycle_count >= WARMUP_CYCLES) {
        latency_ns = ktime_to_ns(ktime_sub(now, stats.timer_fire_time));
        if (latency_ns < 0) {
            latency_ns = -latency_ns;
        }
        if (latency_ns > stats.max_schedule_latency_ns) {
            stats.max_schedule_latency_ns = latency_ns;
        }

        stats.total_schedule_latency_ns += latency_ns;
    }

    // Receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    if (counter) {
        counter--;
    }
    else {
        counter = FREQUENCY;
        blink = !blink;

        /*
         * NOTE: check_master_state() and check_slave_config_states()
         * use printk which can cause scheduling in EVL out-of-band context.
         * Disabled in cyclic path for real-time performance.
         * Run these checks only during init/cleanup if needed.
         */
        // check_master_state();
        // check_slave_config_states();

#if SDO_ACCESS
        read_sdo();
#endif

#if VOE_ACCESS
        read_voe();
#endif
    }

    // Write process data - blink all 8 channels
    EC_WRITE_U8(domain1_pd + off_dig_out_8, blink ? 0xFF : 0x00);

    // Send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);

    stats.end_time = ktime_get();

    update_statistics();
}

/****************************************************************************/

/* EVL timer callback - runs in out-of-band context (hard IRQ) */
static void cyclic_timer_handler(struct evl_timer *timer)
{
    ktime_t now;
    s64 timer_latency_ns;
    static int timer_cpu_printed = 0;

    /* Print timer CPU once for debugging */
    if (!timer_cpu_printed) {
        printk(KERN_ERR PFX "Timer IRQ running on CPU%d\n",
               raw_smp_processor_id());
        timer_cpu_printed = 1;
    }

    now = ktime_get();
    stats.timer_fire_time = now;

    // Save current cycle's expected wake time for jitter calculation
    // MUST be done BEFORE raising flag to avoid race condition
    stats.current_expected = stats.expected_wake;

    // Measure timer latency
    if (stats.cycle_count >= WARMUP_CYCLES) {
        timer_latency_ns = ktime_to_ns(ktime_sub(now, stats.expected_wake));
        if (timer_latency_ns < 0) {
            timer_latency_ns = -timer_latency_ns;
        }

        if (timer_latency_ns > stats.max_timer_latency_ns) {
            stats.max_timer_latency_ns = timer_latency_ns;
        }

        stats.total_timer_latency_ns += timer_latency_ns;
    }

    // Update next expected wake time
    stats.expected_wake = ktime_add_ns(stats.expected_wake, CYCLE_TIME_NS);

    // Signal the cyclic thread (MUST be last to ensure all state is updated)
    evl_raise_flag(&timer_flag);
}

/****************************************************************************/

/* EVL cyclic thread - runs in out-of-band (real-time) context */
static void cyclic_thread_func(void *arg)
{
    int ret;

    /* Thread is automatically switched to out-of-band mode by EVL */
    while (thread_running) {
        /* Wait for timer signal - out-of-band wait, ultra-low latency */
        ret = evl_wait_flag(&timer_flag);
        if (ret) {
            break;
        }

        /* Execute cyclic task in out-of-band context */
        cyclic_handler();
    }

    printk(KERN_INFO PFX "EVL cyclic thread exiting\n");
    return;
}

/****************************************************************************/

void send_callback(void *cb_data)
{
    ec_master_t *m = (ec_master_t *) cb_data;
    ecrt_master_send_ext(m);
}

/****************************************************************************/

void receive_callback(void *cb_data)
{
    ec_master_t *m = (ec_master_t *) cb_data;
    ecrt_master_receive(m);
}

/****************************************************************************/

void clear_stats(void)
{
    memset(&stats, 0, sizeof(stats));
    stats.min_jitter_ns = S64_MAX;
    stats.min_duration_ns = S64_MAX;
}

/****************************************************************************/

int __init init_cix_demo_module(void)
{
    int ret = -1;
#if EXTERNAL_MEMORY
    unsigned int size;
#endif
    ktime_t start_time;

    printk(KERN_INFO PFX "Starting Xenomai 4 (EVL) demo...\n");
    clear_stats();

    master = ecrt_request_master(0);
    if (!master) {
        ret = -EBUSY;
        printk(KERN_ERR PFX "Requesting master 0 failed.\n");
        goto out_return;
    }

    ecrt_master_callbacks(master, send_callback, receive_callback, master);

    printk(KERN_INFO PFX "Registering domain...\n");
    if (!(domain1 = ecrt_master_create_domain(master))) {
        printk(KERN_ERR PFX "Domain creation failed!\n");
        goto out_release_master;
    }

    // Configure EL2008
    if (!(sc_dig_out = ecrt_master_slave_config(
                  master,
                  EL2008Pos,
                  Beckhoff_EL2008))) {
        printk(KERN_ERR PFX
               "Failed to get slave configuration for EL2008.\n");
        goto out_release_master;
    }

#if CONFIGURE_PDOS
    printk(KERN_INFO PFX "Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc_dig_out, EC_END, el2008_syncs)) {
        printk(KERN_ERR PFX "Failed to configure PDOs for EL2008.\n");
        goto out_release_master;
    }
#endif

#if SDO_ACCESS
    printk(KERN_INFO PFX "Creating SDO requests...\n");
    if (!(sdo = ecrt_slave_config_create_sdo_request(
                  sc_dig_out,
                  0x3001,
                  1,
                  1))) {
        printk(KERN_ERR PFX "Failed to create SDO request.\n");
        goto out_release_master;
    }
    ecrt_sdo_request_timeout(sdo, 500);
#endif

#if VOE_ACCESS
    printk(KERN_INFO PFX "Creating VoE handlers...\n");
    if (!(voe = ecrt_slave_config_create_voe_handler(sc_dig_out, 1000))) {
        printk(KERN_ERR PFX "Failed to create VoE handler.\n");
        goto out_release_master;
    }
#endif

    printk(KERN_INFO PFX "Registering PDO entries...\n");
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        printk(KERN_ERR PFX "PDO entry registration failed!\n");
        goto out_release_master;
    }

#if EXTERNAL_MEMORY
    if ((size = ecrt_domain_size(domain1))) {
        if (!(domain1_pd = (uint8_t *) kmalloc(size, GFP_KERNEL))) {
            printk(KERN_ERR PFX
                   "Failed to allocate %u bytes of process data!\n",
                   size);
            goto out_release_master;
        }
        ecrt_domain_external_memory(domain1, domain1_pd);
    }
#endif

    printk(KERN_INFO PFX "Activating master...\n");
    if (ecrt_master_activate(master)) {
        printk(KERN_ERR PFX "Failed to activate master!\n");
#if EXTERNAL_MEMORY
        goto out_free_process_data;
#else
        goto out_release_master;
#endif
    }

#if !EXTERNAL_MEMORY
    domain1_pd = ecrt_domain_data(domain1);
#endif

    /* Initialize EVL flag for timer signaling */
    printk(KERN_INFO PFX "Initializing EVL flag...\n");
    evl_init_flag(&timer_flag);

    /* Create EVL real-time thread */
    printk(KERN_INFO PFX "Creating EVL thread...\n");
    thread_running = 1;

    /* Run kthread with priority 99, bound to CPU */
    ret = evl_run_kthread_on_cpu(
            &cyclic_kthread,
            cpu_id,
            cyclic_thread_func,
            NULL,
            99, /* FIFO priority */
            0,  /* clone_flags */
            "ec_cyclic");
    if (ret) {
        printk(KERN_ERR PFX "Failed to create EVL thread: %d\n", ret);
        goto out_destroy_flag;
    }

    /* Initialize EVL timer on the SAME CPU as the thread to avoid IPI latency
     */
    printk(KERN_INFO PFX "Initializing EVL timer on CPU%d...\n", cpu_id);
    evl_init_timer_on_cpu(&cyclic_timer, cpu_id, cyclic_timer_handler);

    /* Start timer - first cycle */
    start_time = ktime_add_ns(ktime_get(), CYCLE_TIME_NS);
    stats.expected_wake = start_time;
    evl_start_timer(&cyclic_timer, start_time, ns_to_ktime(CYCLE_TIME_NS));

    printk(KERN_INFO PFX "EVL cyclic timer started at %d Hz on CPU%d\n",
           FREQUENCY,
           cpu_id);
    printk(KERN_INFO PFX "Started.\n");
    return 0;

out_destroy_flag:
    evl_destroy_flag(&timer_flag);
out_deactivate:
    ecrt_master_deactivate(master);
#if EXTERNAL_MEMORY
out_free_process_data:
    kfree(domain1_pd);
#endif
out_release_master:
    printk(KERN_ERR PFX "Releasing master...\n");
    ecrt_release_master(master);
out_return:
    printk(KERN_ERR PFX "Failed to load. Aborting.\n");
    return ret;
}

/****************************************************************************/

void dump_stats(void)
{
    int pct_0_5us, pct_5_10us, pct_10us_plus;
    int pct_dur_0_5us, pct_dur_5_10us, pct_dur_10_20us, pct_dur_20_50us,
            pct_dur_50us_plus;
    s64 avg_timer_latency_ns, avg_schedule_latency_ns;

    if (stats.cycle_count > WARMUP_CYCLES) {
        stats.cycle_count -= WARMUP_CYCLES;
        stats.avg_duration_ns = stats.total_duration_ns / stats.cycle_count;
        stats.avg_jitter_ns = stats.total_jitter_ns / stats.cycle_count;
        avg_timer_latency_ns =
                stats.total_timer_latency_ns / stats.cycle_count;
        avg_schedule_latency_ns =
                stats.total_schedule_latency_ns / stats.cycle_count;

        pct_0_5us = (stats.jitter_0_5us * 10000) / stats.cycle_count;
        pct_5_10us = (stats.jitter_5_10us * 10000) / stats.cycle_count;
        pct_10us_plus = (stats.jitter_10us_plus * 10000) / stats.cycle_count;

        printk(KERN_INFO PFX "========================================\n");
        printk(KERN_INFO PFX
               "EVL Cyclic task (period=%ldus) statistics over %lu cycles:\n",
               CYCLE_TIME_NS / 1000,
               stats.cycle_count);
        printk(KERN_INFO PFX "Timer Latency(ns)    Avg:%10lld Max:%10lld\n",
               avg_timer_latency_ns,
               stats.max_timer_latency_ns);
        printk(KERN_INFO PFX "Thread Wakeup(ns)    Avg:%10lld Max:%10lld\n",
               avg_schedule_latency_ns,
               stats.max_schedule_latency_ns);
        printk(KERN_INFO PFX "----------------------------------------\n");
        printk(KERN_INFO PFX
               "Sched Latency(ns)    Min:%10lld Avg:%10lld Max:%10lld\n",
               (stats.min_jitter_ns == S64_MAX) ? 0 : stats.min_jitter_ns,
               stats.avg_jitter_ns,
               stats.max_jitter_ns);
        printk(KERN_INFO PFX
               "Task Duration(ns)    Min:%10lld Avg:%10lld Max:%10lld\n",
               (stats.min_duration_ns == S64_MAX) ? 0 : stats.min_duration_ns,
               stats.avg_duration_ns,
               stats.max_duration_ns);
        printk(KERN_INFO PFX "Sched Latency Distribution:\n");

        printk(KERN_INFO PFX "0-5us     : %10lu(%d.%02d%%)\n",
               stats.jitter_0_5us,
               pct_0_5us / 100,
               pct_0_5us % 100);
        printk(KERN_INFO PFX "5-10us    : %10lu(%d.%02d%%)\n",
               stats.jitter_5_10us,
               pct_5_10us / 100,
               pct_5_10us % 100);
        printk(KERN_INFO PFX ">10us     : %10lu(%d.%02d%%)\n",
               stats.jitter_10us_plus,
               pct_10us_plus / 100,
               pct_10us_plus % 100);

        // Duration distribution
        pct_dur_0_5us = (stats.duration_0_5us * 10000) / stats.cycle_count;
        pct_dur_5_10us = (stats.duration_5_10us * 10000) / stats.cycle_count;
        pct_dur_10_20us =
                (stats.duration_10_20us * 10000) / stats.cycle_count;
        pct_dur_20_50us =
                (stats.duration_20_50us * 10000) / stats.cycle_count;
        pct_dur_50us_plus =
                (stats.duration_50us_plus * 10000) / stats.cycle_count;

        printk(KERN_INFO PFX "Duration Distribution:\n");
        printk(KERN_INFO PFX "0-5us     : %10lu(%d.%02d%%)\n",
               stats.duration_0_5us,
               pct_dur_0_5us / 100,
               pct_dur_0_5us % 100);
        printk(KERN_INFO PFX "5-10us    : %10lu(%d.%02d%%)\n",
               stats.duration_5_10us,
               pct_dur_5_10us / 100,
               pct_dur_5_10us % 100);
        printk(KERN_INFO PFX "10-20us   : %10lu(%d.%02d%%)\n",
               stats.duration_10_20us,
               pct_dur_10_20us / 100,
               pct_dur_10_20us % 100);
        printk(KERN_INFO PFX "20-50us   : %10lu(%d.%02d%%)\n",
               stats.duration_20_50us,
               pct_dur_20_50us / 100,
               pct_dur_20_50us % 100);
        printk(KERN_INFO PFX ">50us     : %10lu(%d.%02d%%)\n",
               stats.duration_50us_plus,
               pct_dur_50us_plus / 100,
               pct_dur_50us_plus % 100);
    }
}

/****************************************************************************/

void __exit cleanup_cix_demo_module(void)
{
    printk(KERN_INFO PFX "Stopping...\n");

    /* Stop timer */
    evl_stop_timer(&cyclic_timer);
    evl_destroy_timer(&cyclic_timer);

    /* Stop thread */
    thread_running = 0;
    evl_raise_flag(&timer_flag); /* Wake up thread to exit */
    evl_stop_kthread(&cyclic_kthread);

    /* Destroy flag */
    evl_destroy_flag(&timer_flag);

    dump_stats();
    clear_stats();

    printk(KERN_INFO PFX "Deactivating master...\n");
    ecrt_master_deactivate(master);

#if EXTERNAL_MEMORY
    kfree(domain1_pd);
#endif

    printk(KERN_INFO PFX "Releasing master...\n");
    ecrt_release_master(master);

    printk(KERN_INFO PFX "Unloading.\n");
}

/****************************************************************************/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lihua Liu <Lihua.Liu@cixtech.com>");
MODULE_DESCRIPTION("EtherCAT demo for Xenomai 4 (EVL)");

module_init(init_cix_demo_module);
module_exit(cleanup_cix_demo_module);

/****************************************************************************/
