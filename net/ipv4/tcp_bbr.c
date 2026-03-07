/* BBR (Bottleneck Bandwidth and RTT) congestion control - Version 3
 *
 * Backported to Linux 4.9.337 from Google's BBR v3 branch.
 * Original: https://github.com/google/bbr (branch: v3)
 *
 * BBR is a model-based congestion control algorithm that aims for low queues,
 * low loss, and (bounded) Reno/CUBIC coexistence. To maintain a model of the
 * network path, it uses measurements of bandwidth and RTT, as well as (if they
 * occur) packet loss and/or shallow-threshold ECN signals.
 *
 * The model has both higher and lower bounds for the operating range:
 *   lo: bw_lo, inflight_lo: conservative short-term lower bound
 *   hi: bw_hi, inflight_hi: robust long-term upper bound
 *
 * State transition diagram for BBR:
 *
 *             |
 *             V
 *    +---> STARTUP  ----+
 *    |        |         |
 *    |        V         |
 *    |      DRAIN   ----+
 *    |        |         |
 *    |        V         |
 *    +---> PROBE_BW ----+
 *    |      ^    |      |
 *    |      |    |      |
 *    |      +----+      |
 *    |                  |
 *    +---- PROBE_RTT <--+
 *
 * Backport notes for Linux 4.9.337:
 * - tcp_snd_cwnd()/tcp_snd_cwnd_set() compat inlines (defined in tcp.h)
 * - tcp_jiffies32 mapped to tcp_time_stamp
 * - No EDT; uses tcp_packets_in_flight()
 * - No per-SKB lost/delivered_ce (tcp_skb_cb 24-byte constraint)
 * - No PLB, No BPF/BTF
 * - skb_mstamp instead of u64 usec timestamps
 * - prandom_u32_max() instead of get_random_u32_below()
 * - sk_pacing_shift hardcoded to 10
 * - No skb_marked_lost callback in tcp_congestion_ops
 * - bbr_can_use_ecn() simplified: no per-route TCP_ECN_LOW
 *
 * Authors (original BBR):
 *   Van Jacobson, Neal Cardwell, Yuchung Cheng, Soheil Hassas Yeganeh,
 *   Priyaranjan Jha, Yousuk Seung, Kevin Yang, Arjun Roy, David Morley
 *
 * Backported to Linux 4.9.337 by:
 *   Mohammad Farid Hendianto (IRedDragonICY) <hendik.suwoto@gmail.com>
 */
#include <linux/module.h>
#include <net/tcp.h>
#include <linux/inet_diag.h>
#include <linux/inet.h>
#include <linux/random.h>
#include <linux/win_minmax.h>

#define BBR_VERSION	3

#define bbr_param(sk, name)	(bbr_ ## name)

#define BW_SCALE	24
#define BW_UNIT		(1 << BW_SCALE)

#define BBR_SCALE	8
#define BBR_UNIT	(1 << BBR_SCALE)

enum bbr_mode {
	BBR_STARTUP,
	BBR_DRAIN,
	BBR_PROBE_BW,
	BBR_PROBE_RTT,
};

enum bbr_ack_phase {
	BBR_ACKS_INIT,
	BBR_ACKS_REFILLING,
	BBR_ACKS_PROBE_STARTING,
	BBR_ACKS_PROBE_FEEDBACK,
	BBR_ACKS_PROBE_STOPPING,
};

struct bbr {
	u32 min_rtt_us;
	u32 min_rtt_stamp;
	u32 probe_rtt_done_stamp;
	u32 probe_rtt_min_us;
	u32 probe_rtt_min_stamp;
	u32 next_rtt_delivered;
	struct skb_mstamp cycle_mstamp;
	u32 mode:2,
	    prev_ca_state:3,
	    round_start:1,
	    ce_state:1,
	    bw_probe_up_rounds:5,
	    try_fast_path:1,
	    idle_restart:1,
	    probe_rtt_round_done:1,
	    init_cwnd:7,
	    unused_1:10;
	u32 pacing_gain:10,
	    cwnd_gain:10,
	    full_bw_reached:1,
	    full_bw_cnt:2,
	    cycle_idx:2,
	    has_seen_rtt:1,
	    unused_2:6;
	u32 prior_cwnd;
	u32 full_bw;

	struct skb_mstamp ack_epoch_mstamp;
	u16 extra_acked[2];
	u32 ack_epoch_acked:20,
	    extra_acked_win_rtts:5,
	    extra_acked_win_idx:1,
	    full_bw_now:1,
	    startup_ecn_rounds:2,
	    loss_in_cycle:1,
	    ecn_in_cycle:1,
	    unused_3:1;
	u32 loss_round_delivered;
	u32 undo_bw_lo;
	u32 undo_inflight_lo;
	u32 undo_inflight_hi;
	u32 bw_latest;
	u32 bw_lo;
	u32 bw_hi[2];
	u32 inflight_latest;
	u32 inflight_lo;
	u32 inflight_hi;
	u32 bw_probe_up_cnt;
	u32 bw_probe_up_acks;
	u32 probe_wait_us;
	u32 prior_rcv_nxt;
	u32 ecn_eligible:1,
	    ecn_alpha:9,
	    bw_probe_samples:1,
	    prev_probe_too_high:1,
	    stopped_risky_probe:1,
	    rounds_since_probe:8,
	    loss_round_start:1,
	    loss_in_round:1,
	    ecn_in_round:1,
	    ack_phase:3,
	    loss_events_in_round:4,
	    initialized:1;
	u32 alpha_last_delivered;
	u32 alpha_last_delivered_ce;
};

struct bbr_context {
	u32 sample_bw;
};

/* --- Module parameters / constants --- */

static const u32 bbr_min_rtt_win_sec = 10;
static const u32 bbr_probe_rtt_mode_ms = 200;
static const u32 bbr_probe_rtt_win_ms = 5000;
static const u32 bbr_probe_rtt_cwnd_gain = BBR_UNIT * 1 / 2;
static const u32 bbr_tso_rtt_shift = 9;
static const int bbr_pacing_margin_percent = 1;

static const int bbr_startup_pacing_gain = BBR_UNIT * 277 / 100 + 1;
static const int bbr_startup_cwnd_gain = BBR_UNIT * 2;
static const int bbr_drain_gain = BBR_UNIT * 1000 / 2885;
static const int bbr_cwnd_gain = BBR_UNIT * 2;
static const int bbr_pacing_gain[] = {
	BBR_UNIT * 5 / 4,	/* UP */
	BBR_UNIT * 91 / 100,	/* DOWN */
	BBR_UNIT,		/* CRUISE */
	BBR_UNIT,		/* REFILL */
};
enum bbr_pacing_gain_phase {
	BBR_BW_PROBE_UP   = 0,
	BBR_BW_PROBE_DOWN = 1,
	BBR_BW_PROBE_CRUISE = 2,
	BBR_BW_PROBE_REFILL = 3,
};

static const u32 bbr_cwnd_min_target = 4;
static const u32 bbr_full_bw_thresh = BBR_UNIT * 5 / 4;
static const u32 bbr_full_bw_cnt = 3;
static const int bbr_extra_acked_gain = BBR_UNIT;
static const u32 bbr_extra_acked_win_rtts = 5;
static const u32 bbr_ack_epoch_acked_reset_thresh = 1U << 20;
static const u32 bbr_extra_acked_max_us = 100 * 1000;

static const u32 bbr_ecn_max_rtt_us = 5000;
static const u32 bbr_beta = BBR_UNIT * 30 / 100;
static const u32 bbr_ecn_alpha_gain = BBR_UNIT * 1 / 16;
static const u32 bbr_ecn_alpha_init = BBR_UNIT;
static const u32 bbr_ecn_factor = BBR_UNIT * 1 / 3;
static const u32 bbr_ecn_thresh = BBR_UNIT * 1 / 2;
static const u32 bbr_ecn_reprobe_gain = BBR_UNIT * 1 / 2;
static const u32 bbr_loss_thresh = BBR_UNIT * 2 / 100;
static const u32 bbr_full_loss_cnt = 6;
static const u32 bbr_full_ecn_cnt = 2;
static const u32 bbr_inflight_headroom = BBR_UNIT * 15 / 100;
static const u32 bbr_bw_probe_cwnd_gain = 1;
static const u32 bbr_bw_probe_max_rounds = 63;
static const u32 bbr_bw_probe_rand_rounds = 2;
static const u32 bbr_bw_probe_base_us = 2 * USEC_PER_SEC;
static const u32 bbr_bw_probe_rand_us = 1 * USEC_PER_SEC;
static const bool bbr_fast_path = true;
static const bool bbr_fast_ack_mode = true;
static const bool bbr_precise_ece_ack = true;

/* Forward declarations */
static u32 bbr_max_bw(const struct sock *sk);
static u32 bbr_bw(const struct sock *sk);
static void bbr_exit_probe_rtt(struct sock *sk);
static void bbr_reset_congestion_signals(struct sock *sk);
static void bbr_check_probe_rtt_done(struct sock *sk);

static bool bbr_can_use_ecn(const struct sock *sk)
{
	return tcp_sk(sk)->ecn_flags & TCP_ECN_OK;
}

static bool bbr_full_bw_reached(const struct sock *sk)
{
	const struct bbr *bbr = inet_csk_ca(sk);
	return bbr->full_bw_reached;
}

static u32 bbr_max_bw(const struct sock *sk)
{
	const struct bbr *bbr = inet_csk_ca(sk);
	return max(bbr->bw_hi[0], bbr->bw_hi[1]);
}

static u32 bbr_bw(const struct sock *sk)
{
	const struct bbr *bbr = inet_csk_ca(sk);
	return min(bbr_max_bw(sk), bbr->bw_lo);
}

static u16 bbr_extra_acked(const struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);
	return max(bbr->extra_acked[0], bbr->extra_acked[1]);
}

static u64 bbr_rate_bytes_per_sec(struct sock *sk, u64 rate, int gain,
				  int margin)
{
	unsigned int mss = tcp_sk(sk)->mss_cache;

	rate *= mss;
	rate *= gain;
	rate >>= BBR_SCALE;
	rate *= USEC_PER_SEC / 100 * (100 - margin);
	rate >>= BW_SCALE;
	rate = max(rate, 1ULL);
	return rate;
}

static u64 bbr_bw_bytes_per_sec(struct sock *sk, u64 rate)
{
	return bbr_rate_bytes_per_sec(sk, rate, BBR_UNIT, 0);
}

static u32 bbr_bw_to_pacing_rate(struct sock *sk, u32 bw, int gain)
{
	u64 rate = bw;

	rate = bbr_rate_bytes_per_sec(sk, rate, gain,
				      bbr_pacing_margin_percent);
	rate = min_t(u64, rate, sk->sk_max_pacing_rate);
	return rate;
}

static void bbr_init_pacing_rate_from_rtt(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	u64 bw;
	u32 rtt_us;

	if (tp->srtt_us) {
		rtt_us = max(tp->srtt_us >> 3, 1U);
		bbr->has_seen_rtt = 1;
	} else {
		rtt_us = USEC_PER_MSEC;
	}
	bw = (u64)tcp_snd_cwnd(tp) * BW_UNIT;
	do_div(bw, rtt_us);
	sk->sk_pacing_rate = bbr_bw_to_pacing_rate(sk, bw,
					bbr_param(sk, startup_pacing_gain));
}

static void bbr_set_pacing_rate(struct sock *sk, u32 bw, int gain)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	u32 rate = bbr_bw_to_pacing_rate(sk, bw, gain);

	if (unlikely(!bbr->has_seen_rtt && tp->srtt_us))
		bbr_init_pacing_rate_from_rtt(sk);
	if (bbr_full_bw_reached(sk) || rate > sk->sk_pacing_rate)
		sk->sk_pacing_rate = rate;
}

static u32 bbr_tso_segs_generic(struct sock *sk, unsigned int mss_now,
				u32 gso_max_size)
{
	struct bbr *bbr = inet_csk_ca(sk);
	u32 segs, r;
	u64 bytes;

	bytes = sk->sk_pacing_rate >> 10;
	if (bbr_param(sk, tso_rtt_shift)) {
		r = bbr->min_rtt_us >> bbr_param(sk, tso_rtt_shift);
		if (r < BITS_PER_TYPE(u32))
			bytes += GSO_MAX_SIZE >> r;
	}
	bytes = min_t(u32, bytes, gso_max_size - 1 - MAX_TCP_HEADER);
	segs = max_t(u32, bytes / mss_now, 1);
	return segs;
}

static u32 bbr_tso_segs_goal(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	return bbr_tso_segs_generic(sk, tp->mss_cache, GSO_MAX_SIZE);
}

static void bbr_save_cwnd(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);

	if (bbr->prev_ca_state < TCP_CA_Recovery && bbr->mode != BBR_PROBE_RTT)
		bbr->prior_cwnd = tcp_snd_cwnd(tp);
	else
		bbr->prior_cwnd = max(bbr->prior_cwnd, tcp_snd_cwnd(tp));
}

/* CE state tracking for delivered_ce accounting. */
static void bbr_ece_ack_update(struct sock *sk, enum tcp_ca_event ev,
			       u32 *prior_rcv_nxt, u32 *ce_state)
{
	struct tcp_sock *tp = tcp_sk(sk);

	if (ev == CA_EVENT_ECN_IS_CE) {
		if (!*ce_state)
			*ce_state = 1;
		tp->delivered_ce++;
	} else if (ev == CA_EVENT_ECN_NO_CE) {
		*ce_state = 0;
	}
	*prior_rcv_nxt = tp->rcv_nxt;
}

static void bbr_cwnd_event(struct sock *sk, enum tcp_ca_event event)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);

	if (event == CA_EVENT_TX_START) {
		if (!tp->app_limited)
			return;
		bbr->idle_restart = 1;
		bbr->ack_epoch_mstamp = tp->delivered_mstamp;
		bbr->ack_epoch_acked = 0;
		if (bbr->mode == BBR_PROBE_BW)
			bbr_set_pacing_rate(sk, bbr_bw(sk), BBR_UNIT);
		else if (bbr->mode == BBR_PROBE_RTT)
			bbr_check_probe_rtt_done(sk);
	} else if ((event == CA_EVENT_ECN_IS_CE ||
		    event == CA_EVENT_ECN_NO_CE) &&
		   bbr_can_use_ecn(sk) &&
		   bbr_param(sk, precise_ece_ack)) {
		u32 state = bbr->ce_state;
		bbr_ece_ack_update(sk, event, &bbr->prior_rcv_nxt, &state);
		bbr->ce_state = state;
	}
}

static u32 bbr_bdp(struct sock *sk, u32 bw, int gain)
{
	struct bbr *bbr = inet_csk_ca(sk);
	u32 bdp;
	u64 w;

	if (unlikely(bbr->min_rtt_us == ~0U))
		return bbr->init_cwnd;

	w = (u64)bw * bbr->min_rtt_us;
	bdp = (((w * gain) >> BBR_SCALE) + BW_UNIT - 1) / BW_UNIT;
	return bdp;
}

static u32 bbr_quantization_budget(struct sock *sk, u32 cwnd)
{
	struct bbr *bbr = inet_csk_ca(sk);
	u32 tso_segs_goal;

	tso_segs_goal = 3 * bbr_tso_segs_goal(sk);
	cwnd = max_t(u32, cwnd, tso_segs_goal);
	cwnd = max_t(u32, cwnd, bbr_param(sk, cwnd_min_target));
	if (bbr->mode == BBR_PROBE_BW && bbr->cycle_idx == BBR_BW_PROBE_UP)
		cwnd += 2;
	return cwnd;
}

static u32 bbr_inflight(struct sock *sk, u32 bw, int gain)
{
	u32 inflight;
	inflight = bbr_bdp(sk, bw, gain);
	inflight = bbr_quantization_budget(sk, inflight);
	return inflight;
}

/* 4.9: no EDT, fall back to plain in-flight count */
static u32 bbr_packets_in_net_at_edt(struct sock *sk, u32 inflight_now)
{
	return inflight_now;
}

static u32 bbr_ack_aggregation_cwnd(struct sock *sk)
{
	u32 max_aggr_cwnd, aggr_cwnd = 0;

	if (bbr_param(sk, extra_acked_gain)) {
		max_aggr_cwnd = ((u64)bbr_bw(sk) * bbr_extra_acked_max_us)
				/ BW_UNIT;
		aggr_cwnd = (bbr_param(sk, extra_acked_gain) *
			     bbr_extra_acked(sk)) >> BBR_SCALE;
		aggr_cwnd = min(aggr_cwnd, max_aggr_cwnd);
	}
	return aggr_cwnd;
}

static u32 bbr_probe_rtt_cwnd(struct sock *sk)
{
	return max_t(u32, bbr_param(sk, cwnd_min_target),
		     bbr_bdp(sk, bbr_bw(sk),
			     bbr_param(sk, probe_rtt_cwnd_gain)));
}

static void bbr_set_cwnd(struct sock *sk, const struct rate_sample *rs,
			 u32 acked, u32 bw, int gain, u32 cwnd,
			 struct bbr_context *ctx)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	u32 target_cwnd = 0;

	if (!acked)
		goto done;

	target_cwnd = bbr_bdp(sk, bw, gain);
	target_cwnd += bbr_ack_aggregation_cwnd(sk);
	target_cwnd = bbr_quantization_budget(sk, target_cwnd);

	bbr->try_fast_path = 0;
	if (bbr_full_bw_reached(sk)) {
		cwnd += acked;
		if (cwnd >= target_cwnd) {
			cwnd = target_cwnd;
			bbr->try_fast_path = 1;
		}
	} else if (cwnd < target_cwnd || cwnd < 2 * bbr->init_cwnd) {
		cwnd += acked;
	} else {
		bbr->try_fast_path = 1;
	}

	cwnd = max_t(u32, cwnd, bbr_param(sk, cwnd_min_target));
done:
	tcp_snd_cwnd_set(tp, min(cwnd, tp->snd_cwnd_clamp));
	if (bbr->mode == BBR_PROBE_RTT)
		tcp_snd_cwnd_set(tp, min_t(u32, tcp_snd_cwnd(tp),
					   bbr_probe_rtt_cwnd(sk)));
}

static void bbr_reset_startup_mode(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);
	bbr->mode = BBR_STARTUP;
}

static u32 bbr_update_round_start(struct sock *sk,
				  const struct rate_sample *rs,
				  struct bbr_context *ctx)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	u32 round_delivered = 0;

	bbr->round_start = 0;
	if (rs->interval_us > 0 &&
	    !before(rs->prior_delivered, bbr->next_rtt_delivered)) {
		round_delivered = tp->delivered - bbr->next_rtt_delivered;
		bbr->next_rtt_delivered = tp->delivered;
		bbr->round_start = 1;
	}
	return round_delivered;
}

static void bbr_calculate_bw_sample(struct sock *sk,
				    const struct rate_sample *rs,
				    struct bbr_context *ctx)
{
	u64 bw = 0;

	if (rs->interval_us > 0) {
		if (WARN_ONCE(rs->delivered < 0,
			      "negative delivered: %d interval_us: %ld\n",
			      rs->delivered, rs->interval_us))
			return;
		bw = (u64)rs->delivered * BW_UNIT + rs->interval_us - 1;
		do_div(bw, rs->interval_us);
	}
	ctx->sample_bw = bw;
}

static void bbr_update_ack_aggregation(struct sock *sk,
				       const struct rate_sample *rs)
{
	u32 epoch_us, expected_acked, extra_acked;
	struct bbr *bbr = inet_csk_ca(sk);
	struct tcp_sock *tp = tcp_sk(sk);
	u32 extra_acked_win_rtts_thresh = bbr_param(sk, extra_acked_win_rtts);

	if (!bbr_param(sk, extra_acked_gain) || rs->acked_sacked <= 0 ||
	    rs->delivered < 0 || rs->interval_us <= 0)
		return;

	if (bbr->round_start) {
		bbr->extra_acked_win_rtts =
			min(0x1F, bbr->extra_acked_win_rtts + 1);
		if (!bbr_full_bw_reached(sk))
			extra_acked_win_rtts_thresh = 1;
		if (bbr->extra_acked_win_rtts >=
		    extra_acked_win_rtts_thresh) {
			bbr->extra_acked_win_rtts = 0;
			bbr->extra_acked_win_idx =
				bbr->extra_acked_win_idx ? 0 : 1;
			bbr->extra_acked[bbr->extra_acked_win_idx] = 0;
		}
	}

	/* 4.9 backport: use tcp_stamp_us_delta_mstamp */
	epoch_us = tcp_stamp_us_delta_mstamp(tp->delivered_mstamp,
					     bbr->ack_epoch_mstamp);
	expected_acked = ((u64)bbr_bw(sk) * epoch_us) / BW_UNIT;

	if (bbr->ack_epoch_acked <= expected_acked ||
	    (bbr->ack_epoch_acked + rs->acked_sacked >=
	     bbr_ack_epoch_acked_reset_thresh)) {
		bbr->ack_epoch_acked = 0;
		bbr->ack_epoch_mstamp = tp->delivered_mstamp;
		expected_acked = 0;
	}

	bbr->ack_epoch_acked = min_t(u32, 0xFFFFF,
				     bbr->ack_epoch_acked + rs->acked_sacked);
	extra_acked = bbr->ack_epoch_acked - expected_acked;
	extra_acked = min(extra_acked, tcp_snd_cwnd(tp));
	if (extra_acked > bbr->extra_acked[bbr->extra_acked_win_idx])
		bbr->extra_acked[bbr->extra_acked_win_idx] = extra_acked;
}

static void bbr_check_probe_rtt_done(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);

	if (!(bbr->probe_rtt_done_stamp &&
	      after(tcp_jiffies32, bbr->probe_rtt_done_stamp)))
		return;

	bbr->probe_rtt_min_stamp = tcp_jiffies32;
	tcp_snd_cwnd_set(tp, max(tcp_snd_cwnd(tp), bbr->prior_cwnd));
	bbr_exit_probe_rtt(sk);
}

static void bbr_update_min_rtt(struct sock *sk, const struct rate_sample *rs)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	bool probe_rtt_expired, min_rtt_expired;
	u32 expire;

	expire = bbr->probe_rtt_min_stamp +
		 msecs_to_jiffies(bbr_param(sk, probe_rtt_win_ms));
	probe_rtt_expired = after(tcp_jiffies32, expire);
	if (rs->rtt_us >= 0 &&
	    (rs->rtt_us < bbr->probe_rtt_min_us || probe_rtt_expired)) {
		bbr->probe_rtt_min_us = rs->rtt_us;
		bbr->probe_rtt_min_stamp = tcp_jiffies32;
	}

	expire = bbr->min_rtt_stamp + bbr_param(sk, min_rtt_win_sec) * HZ;
	min_rtt_expired = after(tcp_jiffies32, expire);
	if (bbr->probe_rtt_min_us <= bbr->min_rtt_us || min_rtt_expired) {
		bbr->min_rtt_us = bbr->probe_rtt_min_us;
		bbr->min_rtt_stamp = bbr->probe_rtt_min_stamp;
	}

	if (bbr_param(sk, probe_rtt_mode_ms) > 0 && probe_rtt_expired &&
	    !bbr->idle_restart && bbr->mode != BBR_PROBE_RTT) {
		bbr->mode = BBR_PROBE_RTT;
		bbr_save_cwnd(sk);
		bbr->probe_rtt_done_stamp = 0;
		bbr->ack_phase = BBR_ACKS_PROBE_STOPPING;
		bbr->next_rtt_delivered = tp->delivered;
	}

	if (bbr->mode == BBR_PROBE_RTT) {
		tp->app_limited =
			(tp->delivered + tcp_packets_in_flight(tp)) ? : 1;
		if (!bbr->probe_rtt_done_stamp &&
		    tcp_packets_in_flight(tp) <= bbr_probe_rtt_cwnd(sk)) {
			bbr->probe_rtt_done_stamp = tcp_jiffies32 +
				msecs_to_jiffies(
					bbr_param(sk, probe_rtt_mode_ms));
			bbr->probe_rtt_round_done = 0;
			bbr->next_rtt_delivered = tp->delivered;
		} else if (bbr->probe_rtt_done_stamp) {
			if (bbr->round_start)
				bbr->probe_rtt_round_done = 1;
			if (bbr->probe_rtt_round_done)
				bbr_check_probe_rtt_done(sk);
		}
	}

	if (rs->delivered > 0)
		bbr->idle_restart = 0;
}

static void bbr_update_gains(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);

	switch (bbr->mode) {
	case BBR_STARTUP:
		bbr->pacing_gain = bbr_param(sk, startup_pacing_gain);
		bbr->cwnd_gain = bbr_param(sk, startup_cwnd_gain);
		break;
	case BBR_DRAIN:
		bbr->pacing_gain = bbr_param(sk, drain_gain);
		bbr->cwnd_gain = bbr_param(sk, startup_cwnd_gain);
		break;
	case BBR_PROBE_BW:
		bbr->pacing_gain = bbr_pacing_gain[bbr->cycle_idx];
		bbr->cwnd_gain = bbr_param(sk, cwnd_gain);
		if (bbr_param(sk, bw_probe_cwnd_gain) &&
		    bbr->cycle_idx == BBR_BW_PROBE_UP)
			bbr->cwnd_gain +=
				BBR_UNIT * bbr_param(sk, bw_probe_cwnd_gain) / 4;
		break;
	case BBR_PROBE_RTT:
		bbr->pacing_gain = BBR_UNIT;
		bbr->cwnd_gain = BBR_UNIT;
		break;
	default:
		WARN_ONCE(1, "BBR bad mode: %u\n", bbr->mode);
		break;
	}
}

static u32 bbr_sndbuf_expand(struct sock *sk)
{
	return 3;
}

static void bbr_take_max_bw_sample(struct sock *sk, u32 bw)
{
	struct bbr *bbr = inet_csk_ca(sk);
	bbr->bw_hi[1] = max(bw, bbr->bw_hi[1]);
}

static void bbr_advance_max_bw_filter(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);
	if (!bbr->bw_hi[1])
		return;
	bbr->bw_hi[0] = bbr->bw_hi[1];
	bbr->bw_hi[1] = 0;
}

static void bbr_reset_full_bw(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);
	bbr->full_bw = 0;
	bbr->full_bw_cnt = 0;
	bbr->full_bw_now = 0;
}

static u32 bbr_target_inflight(struct sock *sk)
{
	u32 bdp = bbr_inflight(sk, bbr_bw(sk), BBR_UNIT);
	return min(bdp, tcp_snd_cwnd(tcp_sk(sk)));
}

static bool bbr_is_probing_bandwidth(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);
	return (bbr->mode == BBR_STARTUP) ||
		(bbr->mode == BBR_PROBE_BW &&
		 (bbr->cycle_idx == BBR_BW_PROBE_REFILL ||
		  bbr->cycle_idx == BBR_BW_PROBE_UP));
}

static bool bbr_has_elapsed_in_phase(const struct sock *sk, u32 interval_us)
{
	const struct tcp_sock *tp = tcp_sk(sk);
	const struct bbr *bbr = inet_csk_ca(sk);
	return skb_mstamp_us_delta(&tp->delivered_mstamp,
				   &bbr->cycle_mstamp) > interval_us;
}

static void bbr_handle_queue_too_high_in_startup(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);
	u32 bdp;
	bbr->full_bw_reached = 1;
	bdp = bbr_inflight(sk, bbr_max_bw(sk), BBR_UNIT);
	bbr->inflight_hi = max(bdp, bbr->inflight_latest);
}

static void bbr_check_ecn_too_high_in_startup(struct sock *sk, u32 ce_ratio)
{
	struct bbr *bbr = inet_csk_ca(sk);

	if (bbr_full_bw_reached(sk) || !bbr->ecn_eligible ||
	    !bbr_param(sk, full_ecn_cnt) || !bbr_param(sk, ecn_thresh))
		return;

	if (ce_ratio >= bbr_param(sk, ecn_thresh))
		bbr->startup_ecn_rounds++;
	else
		bbr->startup_ecn_rounds = 0;

	if (bbr->startup_ecn_rounds >= bbr_param(sk, full_ecn_cnt))
		bbr_handle_queue_too_high_in_startup(sk);
}

static int bbr_update_ecn_alpha(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	s32 delivered, delivered_ce;
	u64 alpha, ce_ratio;
	u32 gain;

	if (!bbr->ecn_eligible && bbr_can_use_ecn(sk) &&
	    !!bbr_param(sk, ecn_factor) &&
	    (bbr->min_rtt_us <= bbr_ecn_max_rtt_us || !bbr_ecn_max_rtt_us))
		bbr->ecn_eligible = 1;

	if (!bbr->ecn_eligible)
		return -1;

	delivered = tp->delivered - bbr->alpha_last_delivered;
	delivered_ce = tp->delivered_ce - bbr->alpha_last_delivered_ce;

	if (delivered == 0 ||
	    WARN_ON_ONCE(delivered < 0 || delivered_ce < 0))
		return -1;

	ce_ratio = (u64)delivered_ce << BBR_SCALE;
	do_div(ce_ratio, delivered);

	gain = bbr_param(sk, ecn_alpha_gain);
	alpha = ((BBR_UNIT - gain) * bbr->ecn_alpha) >> BBR_SCALE;
	alpha += (gain * ce_ratio) >> BBR_SCALE;
	bbr->ecn_alpha = min_t(u32, alpha, BBR_UNIT);

	bbr->alpha_last_delivered = tp->delivered;
	bbr->alpha_last_delivered_ce = tp->delivered_ce;

	bbr_check_ecn_too_high_in_startup(sk, ce_ratio);
	return (int)ce_ratio;
}

static void bbr_raise_inflight_hi_slope(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	u32 growth_this_round, cnt;

	growth_this_round = 1 << bbr->bw_probe_up_rounds;
	bbr->bw_probe_up_rounds = min(bbr->bw_probe_up_rounds + 1, 30);
	cnt = tcp_snd_cwnd(tp) / growth_this_round;
	cnt = max(cnt, 1U);
	bbr->bw_probe_up_cnt = cnt;
}

static void bbr_probe_inflight_hi_upward(struct sock *sk,
					 const struct rate_sample *rs)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	u32 delta;

	if (!tp->is_cwnd_limited || tcp_snd_cwnd(tp) < bbr->inflight_hi)
		return;

	bbr->bw_probe_up_acks += rs->acked_sacked;
	if (bbr->bw_probe_up_acks >= bbr->bw_probe_up_cnt) {
		delta = bbr->bw_probe_up_acks / bbr->bw_probe_up_cnt;
		bbr->bw_probe_up_acks -= delta * bbr->bw_probe_up_cnt;
		bbr->inflight_hi += delta;
		bbr->try_fast_path = 0;
	}

	if (bbr->round_start)
		bbr_raise_inflight_hi_slope(sk);
}

static bool bbr_is_inflight_too_high(const struct sock *sk,
				     const struct rate_sample *rs)
{
	const struct bbr *bbr = inet_csk_ca(sk);
	u32 loss_thresh, ecn_thresh;
	u32 tx_in_flight = rs->tx_in_flight ? : rs->prior_in_flight;

	if (rs->losses > 0 && tx_in_flight) {
		loss_thresh = (u64)tx_in_flight *
			bbr_param(sk, loss_thresh) >> BBR_SCALE;
		if (rs->losses > loss_thresh)
			return true;
	}

	if (rs->delivered_ce > 0 && rs->delivered > 0 &&
	    bbr->ecn_eligible && !!bbr_param(sk, ecn_thresh)) {
		ecn_thresh = (u64)rs->delivered *
			bbr_param(sk, ecn_thresh) >> BBR_SCALE;
		if (rs->delivered_ce > ecn_thresh)
			return true;
	}

	return false;
}

static u32 bbr_inflight_with_headroom(const struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);
	u32 headroom, headroom_fraction;

	if (bbr->inflight_hi == ~0U)
		return ~0U;

	headroom_fraction = bbr_param(sk, inflight_headroom);
	headroom = ((u64)bbr->inflight_hi * headroom_fraction) >> BBR_SCALE;
	headroom = max(headroom, 1U);
	return max_t(s32, bbr->inflight_hi - headroom,
		     bbr_param(sk, cwnd_min_target));
}

static void bbr_bound_cwnd_for_inflight_model(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	u32 cap;

	if (!bbr->initialized)
		return;

	cap = ~0U;
	if (bbr->mode == BBR_PROBE_BW &&
	    bbr->cycle_idx != BBR_BW_PROBE_CRUISE) {
		cap = bbr->inflight_hi;
	} else {
		if (bbr->mode == BBR_PROBE_RTT ||
		    (bbr->mode == BBR_PROBE_BW &&
		     bbr->cycle_idx == BBR_BW_PROBE_CRUISE))
			cap = bbr_inflight_with_headroom(sk);
	}
	cap = min(cap, bbr->inflight_lo);
	cap = max_t(u32, cap, bbr_param(sk, cwnd_min_target));
	tcp_snd_cwnd_set(tp, min(cap, tcp_snd_cwnd(tp)));
}

static u32 bbr_ecn_cut(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);
	return BBR_UNIT -
		((bbr->ecn_alpha * bbr_param(sk, ecn_factor)) >> BBR_SCALE);
}

static void bbr_init_lower_bounds(struct sock *sk, bool init_bw)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);

	if (init_bw && bbr->bw_lo == ~0U)
		bbr->bw_lo = bbr_max_bw(sk);
	if (bbr->inflight_lo == ~0U)
		bbr->inflight_lo = tcp_snd_cwnd(tp);
}

static void bbr_loss_lower_bounds(struct sock *sk, u32 *bw, u32 *inflight)
{
	struct bbr *bbr = inet_csk_ca(sk);
	u32 loss_cut = BBR_UNIT - bbr_param(sk, beta);

	*bw = max_t(u32, bbr->bw_latest,
		    (u64)bbr->bw_lo * loss_cut >> BBR_SCALE);
	*inflight = max_t(u32, bbr->inflight_latest,
			  (u64)bbr->inflight_lo * loss_cut >> BBR_SCALE);
}

static void bbr_ecn_lower_bounds(struct sock *sk, u32 *inflight)
{
	struct bbr *bbr = inet_csk_ca(sk);
	u32 ecn_cut = bbr_ecn_cut(sk);
	*inflight = (u64)bbr->inflight_lo * ecn_cut >> BBR_SCALE;
}

static void bbr_adapt_lower_bounds(struct sock *sk,
				   const struct rate_sample *rs)
{
	struct bbr *bbr = inet_csk_ca(sk);
	u32 ecn_inflight_lo = ~0U;

	if (bbr_is_probing_bandwidth(sk))
		return;

	if (bbr->ecn_in_round && !!bbr_param(sk, ecn_factor)) {
		bbr_init_lower_bounds(sk, false);
		bbr_ecn_lower_bounds(sk, &ecn_inflight_lo);
	}

	if (bbr->loss_in_round) {
		bbr_init_lower_bounds(sk, true);
		bbr_loss_lower_bounds(sk, &bbr->bw_lo, &bbr->inflight_lo);
	}

	bbr->inflight_lo = min(bbr->inflight_lo, ecn_inflight_lo);
	bbr->bw_lo = max(1U, bbr->bw_lo);
}

static void bbr_reset_lower_bounds(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);
	bbr->bw_lo = ~0U;
	bbr->inflight_lo = ~0U;
}

static void bbr_reset_congestion_signals(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);
	bbr->loss_in_round = 0;
	bbr->ecn_in_round = 0;
	bbr->loss_in_cycle = 0;
	bbr->ecn_in_cycle = 0;
	bbr->bw_latest = 0;
	bbr->inflight_latest = 0;
}

static void bbr_exit_loss_recovery(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	tcp_snd_cwnd_set(tp, max(tcp_snd_cwnd(tp), bbr->prior_cwnd));
	bbr->try_fast_path = 0;
}

static void bbr_update_latest_delivery_signals(
	struct sock *sk, const struct rate_sample *rs, struct bbr_context *ctx)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);

	bbr->loss_round_start = 0;
	if (rs->interval_us <= 0 || !rs->acked_sacked)
		return;

	bbr->bw_latest = max_t(u32, bbr->bw_latest, ctx->sample_bw);
	bbr->inflight_latest = max_t(u32, bbr->inflight_latest, rs->delivered);

	if (!before(rs->prior_delivered, bbr->loss_round_delivered)) {
		bbr->loss_round_delivered = tp->delivered;
		bbr->loss_round_start = 1;
	}
}

static void bbr_advance_latest_delivery_signals(
	struct sock *sk, const struct rate_sample *rs, struct bbr_context *ctx)
{
	struct bbr *bbr = inet_csk_ca(sk);
	if (bbr->loss_round_start) {
		bbr->bw_latest = ctx->sample_bw;
		bbr->inflight_latest = rs->delivered;
	}
}

static void bbr_update_congestion_signals(
	struct sock *sk, const struct rate_sample *rs, struct bbr_context *ctx)
{
	struct bbr *bbr = inet_csk_ca(sk);
	u64 bw;

	if (rs->interval_us <= 0 || !rs->acked_sacked)
		return;
	bw = ctx->sample_bw;

	if (!rs->is_app_limited || bw >= bbr_max_bw(sk))
		bbr_take_max_bw_sample(sk, bw);

	bbr->loss_in_round |= (rs->losses > 0);

	if (!bbr->loss_round_start)
		return;

	bbr_adapt_lower_bounds(sk, rs);
	bbr->loss_in_round = 0;
	bbr->ecn_in_round = 0;
}

/* bbr_note_loss: called from skb_marked_lost callback (not in 4.9).
 * Loss tracking handled via rs->losses in bbr_update_congestion_signals.
 */

static bool bbr_is_reno_coexistence_probe_time(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);
	u32 rounds;
	rounds = min_t(u32, bbr_param(sk, bw_probe_max_rounds),
		       bbr_target_inflight(sk));
	return bbr->rounds_since_probe >= rounds;
}

static void bbr_pick_probe_wait(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);
	bbr->rounds_since_probe =
		prandom_u32_max(bbr_param(sk, bw_probe_rand_rounds));
	bbr->probe_wait_us = bbr_param(sk, bw_probe_base_us) +
			     prandom_u32_max(bbr_param(sk, bw_probe_rand_us));
}

static void bbr_set_cycle_idx(struct sock *sk, int cycle_idx)
{
	struct bbr *bbr = inet_csk_ca(sk);
	bbr->cycle_idx = cycle_idx;
	bbr->try_fast_path = 0;
}

static void bbr_start_bw_probe_refill(struct sock *sk, u32 bw_probe_up_rounds)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);

	bbr_reset_lower_bounds(sk);
	bbr->bw_probe_up_rounds = bw_probe_up_rounds;
	bbr->bw_probe_up_acks = 0;
	bbr->stopped_risky_probe = 0;
	bbr->ack_phase = BBR_ACKS_REFILLING;
	bbr->next_rtt_delivered = tp->delivered;
	bbr_set_cycle_idx(sk, BBR_BW_PROBE_REFILL);
}

static void bbr_start_bw_probe_up(struct sock *sk, struct bbr_context *ctx)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);

	bbr->ack_phase = BBR_ACKS_PROBE_STARTING;
	bbr->next_rtt_delivered = tp->delivered;
	bbr->cycle_mstamp = tp->delivered_mstamp;
	bbr_reset_full_bw(sk);
	bbr->full_bw = ctx->sample_bw;
	bbr_set_cycle_idx(sk, BBR_BW_PROBE_UP);
	bbr_raise_inflight_hi_slope(sk);
}

static void bbr_start_bw_probe_down(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);

	bbr_reset_congestion_signals(sk);
	bbr->bw_probe_up_cnt = ~0U;
	bbr_pick_probe_wait(sk);
	bbr->cycle_mstamp = tp->delivered_mstamp;
	bbr->ack_phase = BBR_ACKS_PROBE_STOPPING;
	bbr->next_rtt_delivered = tp->delivered;
	bbr_set_cycle_idx(sk, BBR_BW_PROBE_DOWN);
}

static void bbr_start_bw_probe_cruise(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);
	if (bbr->inflight_lo != ~0U)
		bbr->inflight_lo = min(bbr->inflight_lo, bbr->inflight_hi);
	bbr_set_cycle_idx(sk, BBR_BW_PROBE_CRUISE);
}

static void bbr_handle_inflight_too_high(struct sock *sk,
					 const struct rate_sample *rs)
{
	struct bbr *bbr = inet_csk_ca(sk);
	const u32 beta = bbr_param(sk, beta);

	bbr->prev_probe_too_high = 1;
	bbr->bw_probe_samples = 0;
	if (!rs->is_app_limited) {
		u32 tx_in_flight = rs->tx_in_flight ? : rs->prior_in_flight;
		bbr->inflight_hi = max_t(u32, tx_in_flight,
					 (u64)bbr_target_inflight(sk) *
					 (BBR_UNIT - beta) >> BBR_SCALE);
	}
	if (bbr->mode == BBR_PROBE_BW && bbr->cycle_idx == BBR_BW_PROBE_UP)
		bbr_start_bw_probe_down(sk);
}

static bool bbr_adapt_upper_bounds(struct sock *sk,
				   const struct rate_sample *rs,
				   struct bbr_context *ctx)
{
	struct bbr *bbr = inet_csk_ca(sk);

	if (bbr->ack_phase == BBR_ACKS_PROBE_STARTING && bbr->round_start)
		bbr->ack_phase = BBR_ACKS_PROBE_FEEDBACK;
	if (bbr->ack_phase == BBR_ACKS_PROBE_STOPPING && bbr->round_start) {
		bbr->bw_probe_samples = 0;
		bbr->ack_phase = BBR_ACKS_INIT;
		if (bbr->mode == BBR_PROBE_BW && !rs->is_app_limited)
			bbr_advance_max_bw_filter(sk);
		if (bbr->mode == BBR_PROBE_BW &&
		    bbr->stopped_risky_probe && !bbr->prev_probe_too_high) {
			bbr_start_bw_probe_refill(sk, 0);
			return true;
		}
	}
	if (bbr_is_inflight_too_high(sk, rs)) {
		if (bbr->bw_probe_samples)
			bbr_handle_inflight_too_high(sk, rs);
	} else {
		u32 tx_in_flight = rs->tx_in_flight ? : rs->prior_in_flight;

		if (bbr->inflight_hi == ~0U)
			return false;
		if (tx_in_flight > bbr->inflight_hi)
			bbr->inflight_hi = tx_in_flight;
		if (bbr->mode == BBR_PROBE_BW &&
		    bbr->cycle_idx == BBR_BW_PROBE_UP)
			bbr_probe_inflight_hi_upward(sk, rs);
	}
	return false;
}

static bool bbr_check_time_to_probe_bw(struct sock *sk,
					const struct rate_sample *rs)
{
	struct bbr *bbr = inet_csk_ca(sk);
	u32 n;

	if (bbr_param(sk, ecn_reprobe_gain) && bbr->ecn_eligible &&
	    bbr->ecn_in_cycle && !bbr->loss_in_cycle &&
	    inet_csk(sk)->icsk_ca_state == TCP_CA_Open) {
		n = ilog2((((u64)bbr->inflight_hi *
			    bbr_param(sk, ecn_reprobe_gain)) >> BBR_SCALE));
		bbr_start_bw_probe_refill(sk, n);
		return true;
	}

	if (bbr_has_elapsed_in_phase(sk, bbr->probe_wait_us) ||
	    bbr_is_reno_coexistence_probe_time(sk)) {
		bbr_start_bw_probe_refill(sk, 0);
		return true;
	}
	return false;
}

static bool bbr_check_time_to_cruise(struct sock *sk, u32 inflight, u32 bw)
{
	if (inflight > bbr_inflight_with_headroom(sk))
		return false;
	return inflight <= bbr_inflight(sk, bw, BBR_UNIT);
}

static void bbr_update_cycle_phase(struct sock *sk,
				   const struct rate_sample *rs,
				   struct bbr_context *ctx)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	bool is_bw_probe_done = false;
	u32 inflight, bw;

	if (!bbr_full_bw_reached(sk))
		return;

	if (bbr_adapt_upper_bounds(sk, rs, ctx))
		return;

	if (bbr->mode != BBR_PROBE_BW)
		return;

	inflight = bbr_packets_in_net_at_edt(sk, rs->prior_in_flight);
	bw = bbr_max_bw(sk);

	switch (bbr->cycle_idx) {
	case BBR_BW_PROBE_CRUISE:
		if (bbr_check_time_to_probe_bw(sk, rs))
			return;
		break;

	case BBR_BW_PROBE_REFILL:
		if (bbr->round_start) {
			bbr->bw_probe_samples = 1;
			bbr_start_bw_probe_up(sk, ctx);
		}
		break;

	case BBR_BW_PROBE_UP:
		if (bbr->prev_probe_too_high &&
		    inflight >= bbr->inflight_hi) {
			bbr->stopped_risky_probe = 1;
			is_bw_probe_done = true;
		} else {
			if (tp->is_cwnd_limited &&
			    tcp_snd_cwnd(tp) >= bbr->inflight_hi) {
				bbr_reset_full_bw(sk);
				bbr->full_bw = ctx->sample_bw;
			} else if (bbr->full_bw_now) {
				is_bw_probe_done = true;
			}
		}
		if (is_bw_probe_done) {
			bbr->prev_probe_too_high = 0;
			bbr_start_bw_probe_down(sk);
		}
		break;

	case BBR_BW_PROBE_DOWN:
		if (bbr_check_time_to_probe_bw(sk, rs))
			return;
		if (bbr_check_time_to_cruise(sk, inflight, bw))
			bbr_start_bw_probe_cruise(sk);
		break;

	default:
		WARN_ONCE(1, "BBR invalid cycle index %u\n", bbr->cycle_idx);
	}
}

static void bbr_exit_probe_rtt(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);

	bbr_reset_lower_bounds(sk);
	if (bbr_full_bw_reached(sk)) {
		bbr->mode = BBR_PROBE_BW;
		bbr_start_bw_probe_down(sk);
		bbr_start_bw_probe_cruise(sk);
	} else {
		bbr->mode = BBR_STARTUP;
	}
}

static void bbr_check_loss_too_high_in_startup(struct sock *sk,
					       const struct rate_sample *rs)
{
	struct bbr *bbr = inet_csk_ca(sk);

	if (bbr_full_bw_reached(sk))
		return;

	if (rs->losses && bbr->loss_events_in_round < 0xf)
		bbr->loss_events_in_round++;
	if (bbr_param(sk, full_loss_cnt) && bbr->loss_round_start &&
	    inet_csk(sk)->icsk_ca_state == TCP_CA_Recovery &&
	    bbr->loss_events_in_round >= bbr_param(sk, full_loss_cnt) &&
	    bbr_is_inflight_too_high(sk, rs)) {
		bbr_handle_queue_too_high_in_startup(sk);
		return;
	}
	if (bbr->loss_round_start)
		bbr->loss_events_in_round = 0;
}

static void bbr_check_full_bw_reached(struct sock *sk,
				       const struct rate_sample *rs,
				       struct bbr_context *ctx)
{
	struct bbr *bbr = inet_csk_ca(sk);
	u32 bw_thresh, full_cnt, thresh;

	if (bbr->full_bw_now || rs->is_app_limited)
		return;

	thresh = bbr_param(sk, full_bw_thresh);
	full_cnt = bbr_param(sk, full_bw_cnt);
	bw_thresh = (u64)bbr->full_bw * thresh >> BBR_SCALE;
	if (ctx->sample_bw >= bw_thresh) {
		bbr_reset_full_bw(sk);
		bbr->full_bw = ctx->sample_bw;
		return;
	}
	if (!bbr->round_start)
		return;
	++bbr->full_bw_cnt;
	bbr->full_bw_now = bbr->full_bw_cnt >= full_cnt;
	bbr->full_bw_reached |= bbr->full_bw_now;
}

static void bbr_check_drain(struct sock *sk, const struct rate_sample *rs,
			    struct bbr_context *ctx)
{
	struct bbr *bbr = inet_csk_ca(sk);

	if (bbr->mode == BBR_STARTUP && bbr_full_bw_reached(sk)) {
		bbr->mode = BBR_DRAIN;
		tcp_sk(sk)->snd_ssthresh =
			bbr_inflight(sk, bbr_max_bw(sk), BBR_UNIT);
		bbr_reset_congestion_signals(sk);
	}
	if (bbr->mode == BBR_DRAIN &&
	    bbr_packets_in_net_at_edt(sk,
		tcp_packets_in_flight(tcp_sk(sk))) <=
	    bbr_inflight(sk, bbr_max_bw(sk), BBR_UNIT)) {
		bbr->mode = BBR_PROBE_BW;
		bbr_start_bw_probe_down(sk);
	}
}

static void bbr_update_model(struct sock *sk, const struct rate_sample *rs,
			     struct bbr_context *ctx)
{
	bbr_update_congestion_signals(sk, rs, ctx);
	bbr_update_ack_aggregation(sk, rs);
	bbr_check_loss_too_high_in_startup(sk, rs);
	bbr_check_full_bw_reached(sk, rs, ctx);
	bbr_check_drain(sk, rs, ctx);
	bbr_update_cycle_phase(sk, rs, ctx);
	bbr_update_min_rtt(sk, rs);
}

static bool bbr_run_fast_path(struct sock *sk, bool *update_model,
			      const struct rate_sample *rs,
			      struct bbr_context *ctx)
{
	struct bbr *bbr = inet_csk_ca(sk);
	u32 prev_min_rtt_us, prev_mode;

	if (bbr_param(sk, fast_path) && bbr->try_fast_path &&
	    rs->is_app_limited && ctx->sample_bw < bbr_max_bw(sk) &&
	    !bbr->loss_in_round && !bbr->ecn_in_round) {
		prev_mode = bbr->mode;
		prev_min_rtt_us = bbr->min_rtt_us;
		bbr_check_drain(sk, rs, ctx);
		bbr_update_cycle_phase(sk, rs, ctx);
		bbr_update_min_rtt(sk, rs);

		if (bbr->mode == prev_mode &&
		    bbr->min_rtt_us == prev_min_rtt_us &&
		    bbr->try_fast_path)
			return true;

		*update_model = false;
	}
	return false;
}

static void bbr_main(struct sock *sk, const struct rate_sample *rs)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	struct bbr_context ctx = { 0 };
	bool update_model = true;
	u32 bw;

	bbr_update_round_start(sk, rs, &ctx);
	if (bbr->round_start) {
		bbr->rounds_since_probe =
			min_t(s32, bbr->rounds_since_probe + 1, 0xFF);
		bbr_update_ecn_alpha(sk);
	}

	bbr->ecn_in_round |= (bbr->ecn_eligible &&
			       (rs->is_ece || rs->delivered_ce > 0));
	bbr_calculate_bw_sample(sk, rs, &ctx);
	bbr_update_latest_delivery_signals(sk, rs, &ctx);

	if (bbr_run_fast_path(sk, &update_model, rs, &ctx))
		goto out;

	if (update_model)
		bbr_update_model(sk, rs, &ctx);

	bbr_update_gains(sk);
	bw = bbr_bw(sk);
	bbr_set_pacing_rate(sk, bw, bbr->pacing_gain);
	bbr_set_cwnd(sk, rs, rs->acked_sacked, bw, bbr->cwnd_gain,
		     tcp_snd_cwnd(tp), &ctx);
	bbr_bound_cwnd_for_inflight_model(sk);

out:
	bbr_advance_latest_delivery_signals(sk, rs, &ctx);
	bbr->prev_ca_state = inet_csk(sk)->icsk_ca_state;
	bbr->loss_in_cycle |= rs->losses > 0;
	bbr->ecn_in_cycle  |= bbr->ecn_in_round;
}

static void bbr_in_ack_event(struct sock *sk, u32 flags)
{
	if (flags & CA_ACK_ECE) {
		struct bbr *bbr = inet_csk_ca(sk);
		bbr->ecn_in_round = 1;
	}
}

static void bbr_init(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);

	bbr->initialized = 1;
	bbr->init_cwnd = min(0x7FU, tcp_snd_cwnd(tp));
	bbr->prior_cwnd = tp->prior_cwnd;
	tp->snd_ssthresh = TCP_INFINITE_SSTHRESH;
	bbr->next_rtt_delivered = tp->delivered;
	bbr->prev_ca_state = TCP_CA_Open;

	bbr->probe_rtt_done_stamp = 0;
	bbr->probe_rtt_round_done = 0;
	bbr->probe_rtt_min_us = tcp_min_rtt(tp);
	bbr->probe_rtt_min_stamp = tcp_jiffies32;
	bbr->min_rtt_us = tcp_min_rtt(tp);
	bbr->min_rtt_stamp = tcp_jiffies32;

	bbr->has_seen_rtt = 0;
	bbr_init_pacing_rate_from_rtt(sk);

	bbr->round_start = 0;
	bbr->idle_restart = 0;
	bbr->full_bw_reached = 0;
	bbr->full_bw = 0;
	bbr->full_bw_cnt = 0;
	bbr->cycle_mstamp.v64 = 0;
	bbr->cycle_idx = 0;
	bbr_reset_startup_mode(sk);

	bbr->ack_epoch_mstamp = tp->delivered_mstamp;
	bbr->ack_epoch_acked = 0;
	bbr->extra_acked_win_rtts = 0;
	bbr->extra_acked_win_idx = 0;
	bbr->extra_acked[0] = 0;
	bbr->extra_acked[1] = 0;

	bbr->ce_state = 0;
	bbr->prior_rcv_nxt = tp->rcv_nxt;
	bbr->try_fast_path = 0;

	cmpxchg(&sk->sk_pacing_status, SK_PACING_NONE, SK_PACING_NEEDED);

	bbr->loss_round_delivered = tp->delivered + 1;
	bbr->loss_round_start = 0;
	bbr->undo_bw_lo = 0;
	bbr->undo_inflight_lo = 0;
	bbr->undo_inflight_hi = 0;
	bbr->loss_events_in_round = 0;
	bbr->startup_ecn_rounds = 0;
	bbr_reset_congestion_signals(sk);
	bbr->bw_lo = ~0U;
	bbr->bw_hi[0] = 0;
	bbr->bw_hi[1] = 0;
	bbr->inflight_lo = ~0U;
	bbr->inflight_hi = ~0U;
	bbr_reset_full_bw(sk);
	bbr->bw_probe_up_cnt = ~0U;
	bbr->bw_probe_up_acks = 0;
	bbr->bw_probe_up_rounds = 0;
	bbr->probe_wait_us = 0;
	bbr->stopped_risky_probe = 0;
	bbr->ack_phase = BBR_ACKS_INIT;
	bbr->rounds_since_probe = 0;
	bbr->bw_probe_samples = 0;
	bbr->prev_probe_too_high = 0;
	bbr->ecn_eligible = 0;
	bbr->ecn_alpha = bbr_param(sk, ecn_alpha_init);
	bbr->alpha_last_delivered = 0;
	bbr->alpha_last_delivered_ce = 0;
}

static u32 bbr_undo_cwnd(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);

	bbr_reset_full_bw(sk);
	bbr->loss_in_round = 0;
	bbr->bw_lo = max(bbr->bw_lo, bbr->undo_bw_lo);
	bbr->inflight_lo = max(bbr->inflight_lo, bbr->undo_inflight_lo);
	bbr->inflight_hi = max(bbr->inflight_hi, bbr->undo_inflight_hi);
	bbr->try_fast_path = 0;
	return bbr->prior_cwnd;
}

static u32 bbr_ssthresh(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);

	bbr_save_cwnd(sk);
	bbr->undo_bw_lo = bbr->bw_lo;
	bbr->undo_inflight_lo = bbr->inflight_lo;
	bbr->undo_inflight_hi = bbr->inflight_hi;
	return tcp_sk(sk)->snd_ssthresh;
}

static enum tcp_bbr_phase bbr_get_phase(struct bbr *bbr)
{
	switch (bbr->mode) {
	case BBR_STARTUP:
		return BBR_PHASE_STARTUP;
	case BBR_DRAIN:
		return BBR_PHASE_DRAIN;
	case BBR_PROBE_BW:
		break;
	case BBR_PROBE_RTT:
		return BBR_PHASE_PROBE_RTT;
	default:
		return BBR_PHASE_INVALID;
	}
	switch (bbr->cycle_idx) {
	case BBR_BW_PROBE_UP:
		return BBR_PHASE_PROBE_BW_UP;
	case BBR_BW_PROBE_DOWN:
		return BBR_PHASE_PROBE_BW_DOWN;
	case BBR_BW_PROBE_CRUISE:
		return BBR_PHASE_PROBE_BW_CRUISE;
	case BBR_BW_PROBE_REFILL:
		return BBR_PHASE_PROBE_BW_REFILL;
	default:
		return BBR_PHASE_INVALID;
	}
}

static size_t bbr_get_info(struct sock *sk, u32 ext, int *attr,
			   union tcp_cc_info *info)
{
	if (ext & (1 << (INET_DIAG_BBRINFO - 1)) ||
	    ext & (1 << (INET_DIAG_VEGASINFO - 1))) {
		struct bbr *bbr = inet_csk_ca(sk);
		u64 bw = bbr_bw_bytes_per_sec(sk, bbr_bw(sk));
		u64 bw_hi = bbr_bw_bytes_per_sec(sk, bbr_max_bw(sk));
		u64 bw_lo = bbr->bw_lo == ~0U ?
			~0ULL : bbr_bw_bytes_per_sec(sk, bbr->bw_lo);
		struct tcp_bbr_info *bbr_info = &info->bbr;

		memset(bbr_info, 0, sizeof(*bbr_info));
		bbr_info->bbr_bw_lo	= (u32)bw;
		bbr_info->bbr_bw_hi	= (u32)(bw >> 32);
		bbr_info->bbr_min_rtt	= bbr->min_rtt_us;
		bbr_info->bbr_pacing_gain = bbr->pacing_gain;
		bbr_info->bbr_cwnd_gain	= bbr->cwnd_gain;
		bbr_info->bbr_bw_hi_lsb = (u32)bw_hi;
		bbr_info->bbr_bw_hi_msb = (u32)(bw_hi >> 32);
		bbr_info->bbr_bw_lo_lsb = (u32)bw_lo;
		bbr_info->bbr_bw_lo_msb = (u32)(bw_lo >> 32);
		bbr_info->bbr_mode	= bbr->mode;
		bbr_info->bbr_phase	= (__u8)bbr_get_phase(bbr);
		bbr_info->bbr_version	= (__u8)BBR_VERSION;
		bbr_info->bbr_inflight_lo = bbr->inflight_lo;
		bbr_info->bbr_inflight_hi = bbr->inflight_hi;
		bbr_info->bbr_extra_acked = bbr_extra_acked(sk);
		*attr = INET_DIAG_BBRINFO;
		return sizeof(*bbr_info);
	}
	return 0;
}

static void bbr_set_state(struct sock *sk, u8 new_state)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);

	if (new_state == TCP_CA_Loss) {
		bbr->prev_ca_state = TCP_CA_Loss;
		bbr_reset_full_bw(sk);
		if (!bbr_is_probing_bandwidth(sk) &&
		    bbr->inflight_lo == ~0U) {
			bbr->inflight_lo =
				max(tcp_snd_cwnd(tp), bbr->prior_cwnd);
		}
	} else if (bbr->prev_ca_state == TCP_CA_Loss &&
		   new_state != TCP_CA_Loss) {
		bbr_exit_loss_recovery(sk);
	}
}

static struct tcp_congestion_ops tcp_bbr_cong_ops __read_mostly = {
	.flags		= TCP_CONG_NON_RESTRICTED,
	.name		= "bbr",
	.owner		= THIS_MODULE,
	.init		= bbr_init,
	.cong_control	= bbr_main,
	.in_ack_event	= bbr_in_ack_event,
	.sndbuf_expand	= bbr_sndbuf_expand,
	.undo_cwnd	= bbr_undo_cwnd,
	.cwnd_event	= bbr_cwnd_event,
	.ssthresh	= bbr_ssthresh,
	.tso_segs_goal	= bbr_tso_segs_goal,
	.get_info	= bbr_get_info,
	.set_state	= bbr_set_state,
};

static int __init bbr_register(void)
{
	BUILD_BUG_ON(sizeof(struct bbr) > ICSK_CA_PRIV_SIZE);
	return tcp_register_congestion_control(&tcp_bbr_cong_ops);
}

static void __exit bbr_unregister(void)
{
	tcp_unregister_congestion_control(&tcp_bbr_cong_ops);
}

module_init(bbr_register);
module_exit(bbr_unregister);

MODULE_AUTHOR("Van Jacobson <vanj@google.com>");
MODULE_AUTHOR("Neal Cardwell <ncardwell@google.com>");
MODULE_AUTHOR("Yuchung Cheng <ycheng@google.com>");
MODULE_AUTHOR("Soheil Hassas Yeganeh <soheil@google.com>");
MODULE_AUTHOR("Mohammad Farid Hendianto (IRedDragonICY) <hendik.suwoto@gmail.com> [backport to 4.9]");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("TCP BBR v3 (Bottleneck Bandwidth and RTT)");
MODULE_VERSION(__stringify(BBR_VERSION));
