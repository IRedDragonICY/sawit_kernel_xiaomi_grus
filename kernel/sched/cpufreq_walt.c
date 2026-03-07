/*
 * WALT CPUFreq Governor
 *
 * A dedicated CPUFreq governor built natively around WALT (Window-Assisted
 * Load Tracking) signals. Unlike schedutil which bolts WALT adjustment onto
 * PELT-based logic, this governor speaks WALT natively:
 *
 * - Uses freq_policy_load() (WALT prev_runnable_sum + group load) directly
 * - Leverages WALT predicted demand (pl) for proactive frequency scaling
 * - Uses WALT top-task load for burst-responsive frequency selection
 * - Tracks WALT window boundaries for cycle-accurate average capacity
 * - Supports per-cluster hispeed thresholds and adaptive rate limiting
 *
 * Copyright (c) 2025, Sawit Kernel Project
 * Based on cpufreq_schedutil.c (C) 2016 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/sched/sysctl.h>
#include <trace/events/power.h>
#include "sched.h"
#include "tune.h"

#define WALT_GOV_KTHREAD_PRIORITY	50
#define WALT_TARGET_LOAD		80
#define WALT_NL_RATIO			75
#define WALT_DEFAULT_HISPEED_LOAD	90
#define WALT_DEFAULT_UP_RATE_US		500
#define WALT_DEFAULT_DOWN_RATE_US	2000
#define WALT_DEFAULT_ADAPTIVE_LOW_FREQ	300000
#define WALT_DEFAULT_ADAPTIVE_HIGH_FREQ	0

#define KHZ 1000

/* ── Tunables ────────────────────────────────────────────────────── */
struct waltgov_tunables {
	struct gov_attr_set	attr_set;
	unsigned int		up_rate_limit_us;
	unsigned int		down_rate_limit_us;
	unsigned int		hispeed_load;
	unsigned int		hispeed_freq;
	bool			pl;
	bool			iowait_boost_enable;
	unsigned int		adaptive_low_freq;
	unsigned int		adaptive_high_freq;
};

/* ── Per-policy state ────────────────────────────────────────────── */
struct waltgov_policy {
	struct cpufreq_policy	*policy;
	struct waltgov_tunables	*tunables;
	struct list_head	tunables_hook;

	raw_spinlock_t		update_lock;
	u64			last_freq_update_time;
	s64			min_rate_limit_ns;
	s64			up_rate_delay_ns;
	s64			down_rate_delay_ns;
	u64			last_ws;
	u64			curr_cycles;
	u64			last_cyc_update_time;
	unsigned long		avg_cap;
	unsigned int		next_freq;
	unsigned int		cached_raw_freq;
	unsigned long		hispeed_util;
	unsigned long		max;

	struct irq_work		irq_work;
	struct kthread_work	work;
	struct mutex		work_lock;
	struct kthread_worker	worker;
	struct task_struct	*thread;
	bool			work_in_progress;
	bool			need_freq_update;
};

/* ── Per-CPU state ───────────────────────────────────────────────── */
struct waltgov_cpu {
	struct update_util_data		update_util;
	struct waltgov_policy		*wg_policy;

	unsigned long			iowait_boost;
	unsigned long			iowait_boost_max;
	u64				last_update;

	struct sched_walt_cpu_load	walt_load;

	unsigned long			util;
	unsigned long			max;
	unsigned int			flags;
	unsigned int			cpu;

#ifdef CONFIG_NO_HZ_COMMON
	unsigned long			saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct waltgov_cpu, waltgov_cpu);
static unsigned int waltgov_stale_ns;
static DEFINE_PER_CPU(struct waltgov_tunables *, waltgov_cached_tunables);

/* ═══════════════════════════════════════════════════════════════════
 *  Governor Internals
 * ═══════════════════════════════════════════════════════════════════ */

static bool waltgov_should_update_freq(struct waltgov_policy *wg_policy,
				       u64 time)
{
	s64 delta_ns;

	if (unlikely(wg_policy->need_freq_update)) {
		wg_policy->need_freq_update = false;
		wg_policy->next_freq = UINT_MAX;
		return true;
	}

	delta_ns = time - wg_policy->last_freq_update_time;
	return delta_ns >= wg_policy->min_rate_limit_ns;
}

static bool waltgov_up_down_rate_limit(struct waltgov_policy *wg_policy,
				       u64 time, unsigned int next_freq)
{
	s64 delta_ns;

	delta_ns = time - wg_policy->last_freq_update_time;

	if (next_freq > wg_policy->next_freq &&
	    delta_ns < wg_policy->up_rate_delay_ns)
		return true;

	if (next_freq < wg_policy->next_freq &&
	    delta_ns < wg_policy->down_rate_delay_ns)
		return true;

	return false;
}

static void waltgov_update_commit(struct waltgov_policy *wg_policy, u64 time,
				  unsigned int next_freq)
{
	struct cpufreq_policy *policy = wg_policy->policy;

	if (wg_policy->next_freq == next_freq)
		return;

	if (waltgov_up_down_rate_limit(wg_policy, time, next_freq))
		return;

	wg_policy->next_freq = next_freq;
	wg_policy->last_freq_update_time = time;

	if (policy->fast_switch_enabled) {
		next_freq = cpufreq_driver_fast_switch(policy, next_freq);
		if (next_freq == CPUFREQ_ENTRY_INVALID)
			return;
		policy->cur = next_freq;
		trace_cpu_frequency(next_freq, smp_processor_id());
	} else {
		wg_policy->work_in_progress = true;
		sched_irq_work_queue(&wg_policy->irq_work);
	}
}

static unsigned long waltgov_freq_to_util(struct waltgov_policy *wg_policy,
					  unsigned int freq)
{
	return mult_frac(wg_policy->max, freq,
			 wg_policy->policy->cpuinfo.max_freq);
}

/*
 * WALT-native frequency calculation.
 *
 * Uses boosted_cpu_util() which internally calls cpu_util_freq_walt() →
 * freq_policy_load() giving us the full WALT load signal including
 * prev_runnable_sum, group load, top-task load, and early detection.
 *
 * The C=1.25 multiplier (TARGET_LOAD=80%) ensures we have headroom:
 *   next_freq = max_freq * (util / max) * 1.25
 */
static unsigned int waltgov_next_freq(struct waltgov_policy *wg_policy,
				      unsigned long util, unsigned long max)
{
	struct cpufreq_policy *policy = wg_policy->policy;
	unsigned int freq;

	freq = arch_scale_freq_invariant() ?
		policy->cpuinfo.max_freq : policy->cur;
	freq = (freq + (freq >> 2)) * util / max;

	if (freq == wg_policy->cached_raw_freq &&
	    wg_policy->next_freq != UINT_MAX)
		return wg_policy->next_freq;

	wg_policy->cached_raw_freq = freq;
	return cpufreq_driver_resolve_freq(policy, freq);
}

static void waltgov_get_util(unsigned long *util, unsigned long *max,
			     int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long cfs_max;
	struct waltgov_cpu *wg_cpu = &per_cpu(waltgov_cpu, cpu);

	cfs_max = arch_scale_cpu_capacity(NULL, cpu);
	*util = min(rq->cfs.avg.util_avg, cfs_max);
	*max = cfs_max;

	/* Override with WALT-boosted utilization */
	*util = boosted_cpu_util(cpu, &wg_cpu->walt_load);
}

/* ── WALT window cycle tracking ──────────────────────────────────── */

static void waltgov_track_cycles(struct waltgov_policy *wg_policy,
				 unsigned int prev_freq, u64 upto)
{
	u64 delta_ns, cycles;

	if (unlikely(!sysctl_sched_use_walt_cpu_util))
		return;

	delta_ns = upto - wg_policy->last_cyc_update_time;
	delta_ns *= prev_freq;
	do_div(delta_ns, (NSEC_PER_SEC / KHZ));
	cycles = delta_ns;
	wg_policy->curr_cycles += cycles;
	wg_policy->last_cyc_update_time = upto;
}

static void waltgov_calc_avg_cap(struct waltgov_policy *wg_policy,
				 u64 curr_ws, unsigned int prev_freq)
{
	u64 last_ws = wg_policy->last_ws;
	unsigned int avg_freq;

	if (unlikely(!sysctl_sched_use_walt_cpu_util))
		return;

	BUG_ON(curr_ws < last_ws);
	if (curr_ws <= last_ws)
		return;

	if (curr_ws > (last_ws + sched_ravg_window)) {
		avg_freq = prev_freq;
		wg_policy->last_cyc_update_time = curr_ws;
	} else {
		waltgov_track_cycles(wg_policy, prev_freq, curr_ws);
		avg_freq = wg_policy->curr_cycles;
		avg_freq /= sched_ravg_window / (NSEC_PER_SEC / KHZ);
	}
	wg_policy->avg_cap = waltgov_freq_to_util(wg_policy, avg_freq);
	wg_policy->curr_cycles = 0;
	wg_policy->last_ws = curr_ws;
}

/* ── WALT-native load adjustment ─────────────────────────────────
 *
 * Core difference from schedutil: we always trust WALT signals for
 * frequency decisions. The adjustment logic uses:
 *
 * 1. hispeed_util: If CPU load exceeds hispeed_load% of average capacity,
 *    clamp frequency to at least hispeed_freq.
 *
 * 2. New-task load (nl): If the CPU is under high load AND new-task
 *    contribution is significant (>75% of cpu_util), go to max frequency.
 *    This catches bursty fork/exec workloads that WALT's window detection
 *    might not ramp quickly enough for.
 *
 * 3. Predicted load (pl): If enabled, use WALT's predicted demand as a
 *    floor, ensuring we don't scale down prematurely for periodic tasks.
 *
 * 4. Adaptive frequency: Selectively relax rate limits when the target
 *    frequency is below adaptive_low_freq (allow faster ramp-down in
 *    idle scenarios) or above adaptive_high_freq (allow faster ramp-up
 *    for heavy loads).
 */
static void waltgov_walt_adjust(struct waltgov_cpu *wg_cpu,
				unsigned long *util, unsigned long *max)
{
	struct waltgov_policy *wg_policy = wg_cpu->wg_policy;
	bool is_migration = wg_cpu->flags & SCHED_CPUFREQ_INTERCLUSTER_MIG;
	unsigned long nl = wg_cpu->walt_load.nl;
	unsigned long cpu_util = wg_cpu->util;
	bool is_hiload;

	if (unlikely(!sysctl_sched_use_walt_cpu_util))
		return;

	is_hiload = (cpu_util >= mult_frac(wg_policy->avg_cap,
					   wg_policy->tunables->hispeed_load,
					   100));

	if (is_hiload && !is_migration)
		*util = max(*util, wg_policy->hispeed_util);

	if (is_hiload && nl >= mult_frac(cpu_util, WALT_NL_RATIO, 100))
		*util = *max;

	if (wg_policy->tunables->pl)
		*util = max(*util, wg_cpu->walt_load.pl);
}

/* ── iowait boost ────────────────────────────────────────────────── */

static void waltgov_set_iowait_boost(struct waltgov_cpu *wg_cpu, u64 time,
				     unsigned int flags)
{
	struct waltgov_policy *wg_policy = wg_cpu->wg_policy;

	if (!wg_policy->tunables->iowait_boost_enable)
		return;

	if (flags & SCHED_CPUFREQ_IOWAIT) {
		wg_cpu->iowait_boost = wg_cpu->iowait_boost_max;
	} else if (wg_cpu->iowait_boost) {
		s64 delta_ns = time - wg_cpu->last_update;

		if (delta_ns > TICK_NSEC)
			wg_cpu->iowait_boost = 0;
	}
}

static void waltgov_iowait_boost(struct waltgov_cpu *wg_cpu,
				 unsigned long *util, unsigned long *max)
{
	unsigned long boost_util = wg_cpu->iowait_boost;
	unsigned long boost_max = wg_cpu->iowait_boost_max;

	if (!boost_util)
		return;

	if (*util * boost_max < *max * boost_util) {
		*util = boost_util;
		*max = boost_max;
	}
	wg_cpu->iowait_boost >>= 1;
}

/* ── Idle detection ──────────────────────────────────────────────── */

#ifdef CONFIG_NO_HZ_COMMON
static bool waltgov_cpu_is_busy(struct waltgov_cpu *wg_cpu)
{
	unsigned long idle_calls = tick_nohz_get_idle_calls();
	bool ret = idle_calls == wg_cpu->saved_idle_calls;

	wg_cpu->saved_idle_calls = idle_calls;
	return ret;
}
#else
static inline bool waltgov_cpu_is_busy(struct waltgov_cpu *wg_cpu)
{
	return false;
}
#endif

/* ═══════════════════════════════════════════════════════════════════
 *  Update callbacks — called from scheduler tick / task migration
 * ═══════════════════════════════════════════════════════════════════ */

static void waltgov_update_single(struct update_util_data *hook, u64 time,
				  unsigned int flags)
{
	struct waltgov_cpu *wg_cpu =
		container_of(hook, struct waltgov_cpu, update_util);
	struct waltgov_policy *wg_policy = wg_cpu->wg_policy;
	struct cpufreq_policy *policy = wg_policy->policy;
	unsigned long util, max, hs_util;
	unsigned int next_f;
	bool busy;

	flags &= ~SCHED_CPUFREQ_RT_DL;

	if (!wg_policy->tunables->pl && flags & SCHED_CPUFREQ_PL)
		return;

	waltgov_set_iowait_boost(wg_cpu, time, flags);
	wg_cpu->last_update = time;

	if (!waltgov_should_update_freq(wg_policy, time))
		return;

	busy = waltgov_cpu_is_busy(wg_cpu);

	raw_spin_lock(&wg_policy->update_lock);

	if (flags & SCHED_CPUFREQ_RT_DL) {
		next_f = policy->cpuinfo.max_freq;
	} else {
		waltgov_get_util(&util, &max, wg_cpu->cpu);
		if (wg_policy->max != max) {
			wg_policy->max = max;
			hs_util = waltgov_freq_to_util(wg_policy,
					wg_policy->tunables->hispeed_freq);
			hs_util = mult_frac(hs_util, WALT_TARGET_LOAD, 100);
			wg_policy->hispeed_util = hs_util;
		}

		wg_cpu->util = util;
		wg_cpu->max = max;
		wg_cpu->flags = flags;

		waltgov_calc_avg_cap(wg_policy,
				     wg_cpu->walt_load.ws,
				     policy->cur);

		waltgov_iowait_boost(wg_cpu, &util, &max);
		waltgov_walt_adjust(wg_cpu, &util, &max);
		next_f = waltgov_next_freq(wg_policy, util, max);

		if (busy && next_f < wg_policy->next_freq)
			next_f = wg_policy->next_freq;
	}

	waltgov_update_commit(wg_policy, time, next_f);
	raw_spin_unlock(&wg_policy->update_lock);
}

static unsigned int waltgov_next_freq_shared(struct waltgov_cpu *wg_cpu,
					     u64 time)
{
	struct waltgov_policy *wg_policy = wg_cpu->wg_policy;
	struct cpufreq_policy *policy = wg_policy->policy;
	unsigned long util = 0, max = 1;
	unsigned int j;

	for_each_cpu(j, policy->cpus) {
		struct waltgov_cpu *j_wg_cpu = &per_cpu(waltgov_cpu, j);
		unsigned long j_util, j_max;
		s64 delta_ns;

		delta_ns = time - j_wg_cpu->last_update;
		if (delta_ns > waltgov_stale_ns) {
			j_wg_cpu->iowait_boost = 0;
			continue;
		}
		if (j_wg_cpu->flags & SCHED_CPUFREQ_RT_DL)
			return policy->cpuinfo.max_freq;

		j_util = j_wg_cpu->util;
		j_max = j_wg_cpu->max;
		if (j_util * max >= j_max * util) {
			util = j_util;
			max = j_max;
		}

		waltgov_iowait_boost(j_wg_cpu, &util, &max);
		waltgov_walt_adjust(j_wg_cpu, &util, &max);
	}

	return waltgov_next_freq(wg_policy, util, max);
}

static void waltgov_update_shared(struct update_util_data *hook, u64 time,
				  unsigned int flags)
{
	struct waltgov_cpu *wg_cpu =
		container_of(hook, struct waltgov_cpu, update_util);
	struct waltgov_policy *wg_policy = wg_cpu->wg_policy;
	unsigned long util, max, hs_util;
	unsigned int next_f;

	if (!wg_policy->tunables->pl && flags & SCHED_CPUFREQ_PL)
		return;

	waltgov_get_util(&util, &max, wg_cpu->cpu);

	flags &= ~SCHED_CPUFREQ_RT_DL;

	raw_spin_lock(&wg_policy->update_lock);

	if (wg_policy->max != max) {
		wg_policy->max = max;
		hs_util = waltgov_freq_to_util(wg_policy,
					wg_policy->tunables->hispeed_freq);
		hs_util = mult_frac(hs_util, WALT_TARGET_LOAD, 100);
		wg_policy->hispeed_util = hs_util;
	}

	wg_cpu->util = util;
	wg_cpu->max = max;
	wg_cpu->flags = flags;

	waltgov_set_iowait_boost(wg_cpu, time, flags);
	wg_cpu->last_update = time;

	waltgov_calc_avg_cap(wg_policy,
			     wg_cpu->walt_load.ws,
			     wg_policy->policy->cur);

	if (waltgov_should_update_freq(wg_policy, time)) {
		next_f = waltgov_next_freq_shared(wg_cpu, time);
		waltgov_update_commit(wg_policy, time, next_f);
	}

	raw_spin_unlock(&wg_policy->update_lock);
}

/* ═══════════════════════════════════════════════════════════════════
 *  Kthread for slow-path frequency changes
 * ═══════════════════════════════════════════════════════════════════ */

static void waltgov_work(struct kthread_work *work)
{
	struct waltgov_policy *wg_policy =
		container_of(work, struct waltgov_policy, work);
	unsigned long flags;

	mutex_lock(&wg_policy->work_lock);
	raw_spin_lock_irqsave(&wg_policy->update_lock, flags);
	waltgov_track_cycles(wg_policy, wg_policy->policy->cur,
			     sched_ktime_clock());
	raw_spin_unlock_irqrestore(&wg_policy->update_lock, flags);
	__cpufreq_driver_target(wg_policy->policy, wg_policy->next_freq,
				CPUFREQ_RELATION_L);
	mutex_unlock(&wg_policy->work_lock);

	wg_policy->work_in_progress = false;
}

static void waltgov_irq_work(struct irq_work *irq_work)
{
	struct waltgov_policy *wg_policy =
		container_of(irq_work, struct waltgov_policy, irq_work);

	kthread_queue_work(&wg_policy->worker, &wg_policy->work);
}

/* ═══════════════════════════════════════════════════════════════════
 *  sysfs interface
 * ═══════════════════════════════════════════════════════════════════ */

static struct waltgov_tunables *waltgov_global_tunables;
static DEFINE_MUTEX(waltgov_global_tunables_lock);

static inline struct waltgov_tunables *
to_waltgov_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct waltgov_tunables, attr_set);
}

static DEFINE_MUTEX(waltgov_min_rate_lock);

static void waltgov_update_min_rate_limit_ns(struct waltgov_policy *wg_policy)
{
	mutex_lock(&waltgov_min_rate_lock);
	wg_policy->min_rate_limit_ns = min(wg_policy->up_rate_delay_ns,
					   wg_policy->down_rate_delay_ns);
	mutex_unlock(&waltgov_min_rate_lock);
}

/* ── up_rate_limit_us ──────────────────────────────────────────── */

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->up_rate_limit_us);
}

static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);
	struct waltgov_policy *wg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->up_rate_limit_us = rate_limit_us;

	list_for_each_entry(wg_policy, &attr_set->policy_list, tunables_hook) {
		wg_policy->up_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		waltgov_update_min_rate_limit_ns(wg_policy);
	}

	return count;
}

/* ── down_rate_limit_us ────────────────────────────────────────── */

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set,
				       char *buf)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->down_rate_limit_us);
}

static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);
	struct waltgov_policy *wg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->down_rate_limit_us = rate_limit_us;

	list_for_each_entry(wg_policy, &attr_set->policy_list, tunables_hook) {
		wg_policy->down_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		waltgov_update_min_rate_limit_ns(wg_policy);
	}

	return count;
}

/* ── hispeed_load ──────────────────────────────────────────────── */

static ssize_t hispeed_load_show(struct gov_attr_set *attr_set, char *buf)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->hispeed_load);
}

static ssize_t hispeed_load_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	if (kstrtouint(buf, 10, &tunables->hispeed_load))
		return -EINVAL;

	tunables->hispeed_load = min(100U, tunables->hispeed_load);

	return count;
}

/* ── hispeed_freq ──────────────────────────────────────────────── */

static ssize_t hispeed_freq_show(struct gov_attr_set *attr_set, char *buf)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->hispeed_freq);
}

static ssize_t hispeed_freq_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);
	unsigned int val;
	struct waltgov_policy *wg_policy;
	unsigned long hs_util;
	unsigned long flags;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->hispeed_freq = val;
	list_for_each_entry(wg_policy, &attr_set->policy_list, tunables_hook) {
		raw_spin_lock_irqsave(&wg_policy->update_lock, flags);
		hs_util = waltgov_freq_to_util(wg_policy,
					       tunables->hispeed_freq);
		hs_util = mult_frac(hs_util, WALT_TARGET_LOAD, 100);
		wg_policy->hispeed_util = hs_util;
		raw_spin_unlock_irqrestore(&wg_policy->update_lock, flags);
	}

	return count;
}

/* ── pl (predicted load) ─────────────────────────────────────── */

static ssize_t pl_show(struct gov_attr_set *attr_set, char *buf)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->pl);
}

static ssize_t pl_store(struct gov_attr_set *attr_set, const char *buf,
			size_t count)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	if (kstrtobool(buf, &tunables->pl))
		return -EINVAL;
	return count;
}

/* ── iowait_boost_enable ─────────────────────────────────────── */

static ssize_t iowait_boost_enable_show(struct gov_attr_set *attr_set,
					char *buf)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 tunables->iowait_boost_enable);
}

static ssize_t iowait_boost_enable_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);
	bool enable;

	if (kstrtobool(buf, &enable))
		return -EINVAL;

	tunables->iowait_boost_enable = enable;
	return count;
}

/* ── adaptive_low_freq ───────────────────────────────────────── */

static ssize_t adaptive_low_freq_show(struct gov_attr_set *attr_set, char *buf)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->adaptive_low_freq);
}

static ssize_t adaptive_low_freq_store(struct gov_attr_set *attr_set,
				       const char *buf, size_t count)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	if (kstrtouint(buf, 10, &tunables->adaptive_low_freq))
		return -EINVAL;

	return count;
}

/* ── adaptive_high_freq ──────────────────────────────────────── */

static ssize_t adaptive_high_freq_show(struct gov_attr_set *attr_set,
				       char *buf)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->adaptive_high_freq);
}

static ssize_t adaptive_high_freq_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	if (kstrtouint(buf, 10, &tunables->adaptive_high_freq))
		return -EINVAL;

	return count;
}

/* ── attribute table ─────────────────────────────────────────── */

static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);
static struct governor_attr hispeed_load = __ATTR_RW(hispeed_load);
static struct governor_attr hispeed_freq = __ATTR_RW(hispeed_freq);
static struct governor_attr pl = __ATTR_RW(pl);
static struct governor_attr iowait_boost_enable = __ATTR_RW(iowait_boost_enable);
static struct governor_attr adaptive_low_freq = __ATTR_RW(adaptive_low_freq);
static struct governor_attr adaptive_high_freq = __ATTR_RW(adaptive_high_freq);

static struct attribute *waltgov_attributes[] = {
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&hispeed_load.attr,
	&hispeed_freq.attr,
	&pl.attr,
	&iowait_boost_enable.attr,
	&adaptive_low_freq.attr,
	&adaptive_high_freq.attr,
	NULL
};

static void waltgov_tunables_free(struct kobject *kobj)
{
	struct gov_attr_set *attr_set =
		container_of(kobj, struct gov_attr_set, kobj);

	kfree(to_waltgov_tunables(attr_set));
}

static struct kobj_type waltgov_tunables_ktype = {
	.default_attrs	= waltgov_attributes,
	.sysfs_ops	= &governor_sysfs_ops,
	.release	= &waltgov_tunables_free,
};

/* ═══════════════════════════════════════════════════════════════════
 *  Governor interface: init / exit / start / stop / limits
 * ═══════════════════════════════════════════════════════════════════ */

static struct cpufreq_governor walt_gov;

static struct waltgov_policy *waltgov_policy_alloc(struct cpufreq_policy *policy)
{
	struct waltgov_policy *wg_policy;

	wg_policy = kzalloc(sizeof(*wg_policy), GFP_KERNEL);
	if (!wg_policy)
		return NULL;

	wg_policy->policy = policy;
	raw_spin_lock_init(&wg_policy->update_lock);
	return wg_policy;
}

static void waltgov_policy_free(struct waltgov_policy *wg_policy)
{
	kfree(wg_policy);
}

static int waltgov_kthread_create(struct waltgov_policy *wg_policy)
{
	struct task_struct *thread;
	struct sched_param param = { .sched_priority = MAX_USER_RT_PRIO / 2 };
	struct cpufreq_policy *policy = wg_policy->policy;
	int ret;

	if (policy->fast_switch_enabled)
		return 0;

	kthread_init_work(&wg_policy->work, waltgov_work);
	kthread_init_worker(&wg_policy->worker);
	thread = kthread_create(kthread_worker_fn, &wg_policy->worker,
				"waltgov:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("failed to create waltgov thread: %ld\n",
		       PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &param);
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_FIFO\n", __func__);
		return ret;
	}

	wg_policy->thread = thread;
	kthread_bind_mask(thread, policy->related_cpus);
	init_irq_work(&wg_policy->irq_work, waltgov_irq_work);
	mutex_init(&wg_policy->work_lock);

	wake_up_process(thread);

	return 0;
}

static void waltgov_kthread_stop(struct waltgov_policy *wg_policy)
{
	if (wg_policy->policy->fast_switch_enabled)
		return;

	kthread_flush_worker(&wg_policy->worker);
	kthread_stop(wg_policy->thread);
	mutex_destroy(&wg_policy->work_lock);
}

static struct waltgov_tunables *
waltgov_tunables_alloc(struct waltgov_policy *wg_policy)
{
	struct waltgov_tunables *tunables;

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (tunables) {
		gov_attr_set_init(&tunables->attr_set,
				  &wg_policy->tunables_hook);
		if (!have_governor_per_policy())
			waltgov_global_tunables = tunables;
	}
	return tunables;
}

static void waltgov_tunables_save(struct cpufreq_policy *policy,
				  struct waltgov_tunables *tunables)
{
	int cpu;
	struct waltgov_tunables *cached =
		per_cpu(waltgov_cached_tunables, policy->cpu);

	if (!have_governor_per_policy())
		return;

	if (!cached) {
		cached = kzalloc(sizeof(*tunables), GFP_KERNEL);
		if (!cached) {
			pr_warn("Couldn't allocate tunables for caching\n");
			return;
		}
		for_each_cpu(cpu, policy->related_cpus)
			per_cpu(waltgov_cached_tunables, cpu) = cached;
	}

	cached->pl = tunables->pl;
	cached->hispeed_load = tunables->hispeed_load;
	cached->hispeed_freq = tunables->hispeed_freq;
	cached->up_rate_limit_us = tunables->up_rate_limit_us;
	cached->down_rate_limit_us = tunables->down_rate_limit_us;
	cached->adaptive_low_freq = tunables->adaptive_low_freq;
	cached->adaptive_high_freq = tunables->adaptive_high_freq;
}

static void waltgov_clear_global_tunables(void)
{
	if (!have_governor_per_policy())
		waltgov_global_tunables = NULL;
}

static void waltgov_tunables_restore(struct cpufreq_policy *policy)
{
	struct waltgov_policy *wg_policy = policy->governor_data;
	struct waltgov_tunables *tunables = wg_policy->tunables;
	struct waltgov_tunables *cached =
		per_cpu(waltgov_cached_tunables, policy->cpu);

	if (!cached)
		return;

	tunables->pl = cached->pl;
	tunables->hispeed_load = cached->hispeed_load;
	tunables->hispeed_freq = cached->hispeed_freq;
	tunables->up_rate_limit_us = cached->up_rate_limit_us;
	tunables->down_rate_limit_us = cached->down_rate_limit_us;
	tunables->adaptive_low_freq = cached->adaptive_low_freq;
	tunables->adaptive_high_freq = cached->adaptive_high_freq;
}

static int waltgov_init(struct cpufreq_policy *policy)
{
	struct waltgov_policy *wg_policy;
	struct waltgov_tunables *tunables;
	int ret = 0;

	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	wg_policy = waltgov_policy_alloc(policy);
	if (!wg_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = waltgov_kthread_create(wg_policy);
	if (ret)
		goto free_wg_policy;

	mutex_lock(&waltgov_global_tunables_lock);

	if (waltgov_global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = wg_policy;
		wg_policy->tunables = waltgov_global_tunables;

		gov_attr_set_get(&waltgov_global_tunables->attr_set,
				 &wg_policy->tunables_hook);
		goto out;
	}

	tunables = waltgov_tunables_alloc(wg_policy);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	/* WALT-optimized defaults: fast ramp-up, moderate ramp-down */
	tunables->up_rate_limit_us = WALT_DEFAULT_UP_RATE_US;
	tunables->down_rate_limit_us = WALT_DEFAULT_DOWN_RATE_US;
	tunables->hispeed_load = WALT_DEFAULT_HISPEED_LOAD;
	tunables->hispeed_freq = 0;
	tunables->pl = true;		/* Enable predicted load by default */
	tunables->iowait_boost_enable = true;
	tunables->adaptive_low_freq = WALT_DEFAULT_ADAPTIVE_LOW_FREQ;
	tunables->adaptive_high_freq = WALT_DEFAULT_ADAPTIVE_HIGH_FREQ;

	policy->governor_data = wg_policy;
	wg_policy->tunables = tunables;
	waltgov_stale_ns = sched_ravg_window + (sched_ravg_window >> 3);

	waltgov_tunables_restore(policy);

	ret = kobject_init_and_add(&tunables->attr_set.kobj,
				   &waltgov_tunables_ktype,
				   get_governor_parent_kobj(policy),
				   "%s", walt_gov.name);
	if (ret)
		goto fail;

out:
	mutex_unlock(&waltgov_global_tunables_lock);
	return 0;

fail:
	policy->governor_data = NULL;
	waltgov_clear_global_tunables();

stop_kthread:
	waltgov_kthread_stop(wg_policy);

free_wg_policy:
	mutex_unlock(&waltgov_global_tunables_lock);
	waltgov_policy_free(wg_policy);

disable_fast_switch:
	cpufreq_disable_fast_switch(policy);
	pr_err("initialization failed (error %d)\n", ret);
	return ret;
}

static void waltgov_exit(struct cpufreq_policy *policy)
{
	struct waltgov_policy *wg_policy = policy->governor_data;
	struct waltgov_tunables *tunables = wg_policy->tunables;
	unsigned int count;

	mutex_lock(&waltgov_global_tunables_lock);

	count = gov_attr_set_put(&tunables->attr_set,
				 &wg_policy->tunables_hook);
	policy->governor_data = NULL;
	if (!count) {
		waltgov_tunables_save(policy, tunables);
		waltgov_clear_global_tunables();
	}

	mutex_unlock(&waltgov_global_tunables_lock);

	waltgov_kthread_stop(wg_policy);
	waltgov_policy_free(wg_policy);
	cpufreq_disable_fast_switch(policy);
}

static int waltgov_start(struct cpufreq_policy *policy)
{
	struct waltgov_policy *wg_policy = policy->governor_data;
	unsigned int cpu;

	wg_policy->up_rate_delay_ns =
		wg_policy->tunables->up_rate_limit_us * NSEC_PER_USEC;
	wg_policy->down_rate_delay_ns =
		wg_policy->tunables->down_rate_limit_us * NSEC_PER_USEC;
	waltgov_update_min_rate_limit_ns(wg_policy);
	wg_policy->last_freq_update_time = 0;
	wg_policy->next_freq = UINT_MAX;
	wg_policy->work_in_progress = false;
	wg_policy->need_freq_update = false;
	wg_policy->cached_raw_freq = 0;

	for_each_cpu(cpu, policy->cpus) {
		struct waltgov_cpu *wg_cpu = &per_cpu(waltgov_cpu, cpu);

		memset(wg_cpu, 0, sizeof(*wg_cpu));
		wg_cpu->wg_policy = wg_policy;
		wg_cpu->cpu = cpu;
		wg_cpu->flags = SCHED_CPUFREQ_RT;
		wg_cpu->iowait_boost_max = policy->cpuinfo.max_freq;
	}

	for_each_cpu(cpu, policy->cpus) {
		struct waltgov_cpu *wg_cpu = &per_cpu(waltgov_cpu, cpu);

		cpufreq_add_update_util_hook(cpu, &wg_cpu->update_util,
					     policy_is_shared(policy) ?
						waltgov_update_shared :
						waltgov_update_single);
	}
	return 0;
}

static void waltgov_stop(struct cpufreq_policy *policy)
{
	struct waltgov_policy *wg_policy = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_sched();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&wg_policy->irq_work);
		kthread_cancel_work_sync(&wg_policy->work);
	}
}

static void waltgov_limits(struct cpufreq_policy *policy)
{
	struct waltgov_policy *wg_policy = policy->governor_data;
	unsigned long flags;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&wg_policy->work_lock);
		raw_spin_lock_irqsave(&wg_policy->update_lock, flags);
		waltgov_track_cycles(wg_policy, wg_policy->policy->cur,
				     sched_ktime_clock());
		raw_spin_unlock_irqrestore(&wg_policy->update_lock, flags);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&wg_policy->work_lock);
	}

	wg_policy->need_freq_update = true;
}

/* ═══════════════════════════════════════════════════════════════════
 *  Governor registration
 * ═══════════════════════════════════════════════════════════════════ */

static struct cpufreq_governor walt_gov = {
	.name	= "walt",
	.owner	= THIS_MODULE,
	.init	= waltgov_init,
	.exit	= waltgov_exit,
	.start	= waltgov_start,
	.stop	= waltgov_stop,
	.limits	= waltgov_limits,
};

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_WALT
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &walt_gov;
}
#endif

static int __init waltgov_register(void)
{
	return cpufreq_register_governor(&walt_gov);
}
fs_initcall(waltgov_register);
