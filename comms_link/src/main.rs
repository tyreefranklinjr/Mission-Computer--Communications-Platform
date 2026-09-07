// main.rs
//
// Fault-tolerant communications simulation (bullet 2 of the resume entry).
//
// This is a standalone simulation, not a driver for a real radio or modem.
// Two simulated nodes exchange sequenced messages over two simulated
// links (primary and secondary). All timing in this program is VIRTUAL
// simulated time, tracked by an internal millisecond clock that advances a
// fixed amount each loop iteration; it is not wall-clock time. That lets
// the program simulate a full 30 second communications outage while the
// process itself finishes in well under a second of real time. The actual
// real (wall-clock) run time is measured separately with
// std::time::Instant and printed at the end, so the distinction is
// explicit rather than implied.
//
// Demonstrated end to end here: sequence tracking, ACK/retry, duplicate
// suppression, per-link success-rate scoring, a priority queue of
// outgoing messages, and automatic failover from the primary link to the
// secondary link during a simulated outage, with a measured failover
// latency.

mod link;

use link::Link;
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::collections::HashMap;
use std::collections::HashSet;
use std::fs::File;
use std::io::Write;
use std::time::Instant;

// ---- simulated time constants (all in virtual milliseconds) --------------

const TICK_MS: u64 = 1;
const SEND_INTERVAL_MS: u64 = 10;      // one new outgoing message every 10 ms
const ACK_TIMEOUT_MS: u64 = 20;        // retry if no ack within 20 ms
const ACK_DELAY_MS: u64 = 5;           // simulated round-trip time on a healthy link
const MAX_RETRIES: u32 = 5;
const FAILOVER_SCORE_THRESHOLD: f64 = 0.5;
const MIN_ATTEMPTS_BEFORE_FAILOVER: usize = 3; // don't fail over on a single bad draw

const OUTAGE_START_MS: u64 = 5_000;
const OUTAGE_DURATION_MS: u64 = 30_000; // the "30 s outage" bullet 2 claims
const OUTAGE_END_MS: u64 = OUTAGE_START_MS + OUTAGE_DURATION_MS;
const TOTAL_SIM_MS: u64 = OUTAGE_END_MS + 5_000; // 5 s of recovery after the outage ends

const PRIMARY_BASE_LOSS: f64 = 0.03;
const SECONDARY_BASE_LOSS: f64 = 0.05;
const LATE_ARRIVAL_PROBABILITY: f64 = 0.20; // chance a "lost" send still arrives late as a duplicate

// ---- deterministic RNG -----------------------------------------------

// A small linear congruential generator so every run of this program is
// reproducible, matching the deterministic style used in the C++/C side of
// this project. Not cryptographic.
struct Rng {
    state: u64,
}

impl Rng {
    fn new(seed: u64) -> Rng {
        Rng { state: seed }
    }

    fn next_f64(&mut self) -> f64 {
        self.state = self.state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let bucket = (self.state >> 33) as u32;
        (bucket as f64) / (u32::MAX as f64)
    }
}

// ---- outgoing message / priority queue --------------------------------

#[derive(Clone)]
struct OutgoingMessage {
    seq: u64,
    priority: u8, // 0 = highest priority, matches the CAN message priority convention
    attempts: u32,
}

impl PartialEq for OutgoingMessage {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority && self.seq == other.seq
    }
}

impl Eq for OutgoingMessage {}

impl Ord for OutgoingMessage {
    fn cmp(&self, other: &Self) -> Ordering {
        // BinaryHeap is a max-heap; a lower priority number means more
        // urgent, so reverse the comparison on priority. Ties break by
        // lower sequence number first (older messages go out first).
        match other.priority.cmp(&self.priority) {
            Ordering::Equal => other.seq.cmp(&self.seq),
            ordering => ordering,
        }
    }
}

impl PartialOrd for OutgoingMessage {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// A message that has been sent and is waiting for an ack (or a timeout).
struct InFlight {
    message: OutgoingMessage,
    sent_at_ms: u64,
    link_name: String,
    delivered: bool, // whether this attempt is actually going to arrive
    ack_due_ms: u64,  // when the ack lands, if it is going to arrive
}

// A packet the sender believes was lost (so it will time out and retry)
// but that the network is still going to deliver late, independent of the
// retry. This is what produces a genuine duplicate at the receiver: the
// retried copy arrives normally, and then this straggler shows up too.
struct StrayArrival {
    seq: u64,
    link_name: String,
    due_ms: u64,
}

// ---- report bookkeeping -----------------------------------------------

struct Report {
    messages_sent: u64,
    messages_acked: u64,
    retries: u64,
    duplicates_suppressed: u64,
    messages_dropped: u64,
    failover_event_ms: Option<u64>,
    failover_latency_ms: Option<u64>,
    active_link_at_end: String,
}

fn main() {
    let wall_clock_start = Instant::now();

    // Optional: `comms_link <seed>` runs the same simulation with a
    // different RNG seed, so run_failover_trials.sh can gather several
    // independent failover-latency samples for the dashboard histogram.
    let args: Vec<String> = std::env::args().collect();
    let seed: u64 = if args.len() > 1 {
        args[1].parse().unwrap_or(7)
    } else {
        7
    };

    let mut primary_link = Link::new("primary", PRIMARY_BASE_LOSS);
    let mut secondary_link = Link::new("secondary", SECONDARY_BASE_LOSS);

    let mut rng = Rng::new(seed);

    let mut outgoing_queue: BinaryHeap<OutgoingMessage> = BinaryHeap::new();
    let mut in_flight: HashMap<u64, InFlight> = HashMap::new();
    let mut stray_arrivals: Vec<StrayArrival> = Vec::new();
    let mut seen_by_receiver: HashSet<u64> = HashSet::new();

    let mut next_seq: u64 = 1;
    let mut last_send_at_ms: u64 = 0;
    let mut active_link_is_primary = true;

    let mut report = Report {
        messages_sent: 0,
        messages_acked: 0,
        retries: 0,
        duplicates_suppressed: 0,
        messages_dropped: 0,
        failover_event_ms: None,
        failover_latency_ms: None,
        active_link_at_end: String::from("primary"),
    };

    // The full per-event log is only worth keeping for the default seed's
    // run (that's the one sample committed to samples/); other seeds, run
    // by run_failover_trials.sh to gather more failover-latency samples,
    // just discard their per-event log.
    let log_path: &str = if seed == 7 { "../samples/comms_link_log.csv" } else { "/dev/null" };
    let mut log_file = File::create(log_path).expect("failed to create comms_link log file");
    writeln!(log_file, "virtual_ms,event,link,seq,detail").unwrap();

    println!("comms_link: fault-tolerant communications simulation");
    println!("All times below are VIRTUAL simulated milliseconds, not wall-clock time.");
    println!(
        "Simulated outage: primary link down from t={} ms to t={} ms ({} ms = {:.0} s).",
        OUTAGE_START_MS,
        OUTAGE_END_MS,
        OUTAGE_DURATION_MS,
        OUTAGE_DURATION_MS as f64 / 1000.0
    );
    println!();

    let mut virtual_ms: u64 = 0;
    while virtual_ms <= TOTAL_SIM_MS {
        // Drive the simulated outage on the primary link.
        let outage_active = virtual_ms >= OUTAGE_START_MS && virtual_ms < OUTAGE_END_MS;
        primary_link.forced_down = outage_active;

        // Generate a new outgoing message on schedule.
        if virtual_ms >= last_send_at_ms + SEND_INTERVAL_MS || virtual_ms == 0 {
            last_send_at_ms = virtual_ms;
            let priority_roll = rng.next_f64();
            let priority: u8 = if priority_roll < 0.1 { 0 } else if priority_roll < 0.4 { 1 } else { 2 };
            let message = OutgoingMessage {
                seq: next_seq,
                priority,
                attempts: 0,
            };
            next_seq += 1;
            outgoing_queue.push(message);
        }

        // Send whatever is queued, highest priority first, over whichever
        // link is currently active.
        while let Some(mut message) = outgoing_queue.pop() {
            message.attempts += 1;
            let active_link = if active_link_is_primary { &primary_link } else { &secondary_link };
            send_message(
                &mut message,
                virtual_ms,
                active_link,
                &mut in_flight,
                &mut stray_arrivals,
                &mut rng,
                &mut report,
                &mut log_file,
            );
        }

        // Resolve any in-flight messages whose ack is due now.
        let due_seqs: Vec<u64> = in_flight
            .iter()
            .filter(|(_, pending)| pending.delivered && pending.ack_due_ms <= virtual_ms)
            .map(|(seq, _)| *seq)
            .collect();

        for seq in due_seqs {
            let pending = in_flight.remove(&seq).unwrap();
            resolve_delivery(pending.message.seq, &pending.link_name, &mut seen_by_receiver, &mut report, &mut log_file, virtual_ms);
            record_link_outcome(&pending.link_name, true, &mut primary_link, &mut secondary_link);
        }

        // Any stray (late) arrivals due now: these are duplicates whenever
        // the retried copy of the same message already got through.
        let due_stray_indices: Vec<usize> = stray_arrivals
            .iter()
            .enumerate()
            .filter(|(_, stray)| stray.due_ms <= virtual_ms)
            .map(|(index, _)| index)
            .collect();
        for index in due_stray_indices.into_iter().rev() {
            let stray = stray_arrivals.remove(index);
            resolve_delivery(stray.seq, &stray.link_name, &mut seen_by_receiver, &mut report, &mut log_file, virtual_ms);
        }

        // Check for timeouts: sends that were lost and never arrived at all.
        let timed_out_seqs: Vec<u64> = in_flight
            .iter()
            .filter(|(_, pending)| {
                !pending.delivered && virtual_ms >= pending.sent_at_ms + ACK_TIMEOUT_MS
            })
            .map(|(seq, _)| *seq)
            .collect();

        for seq in timed_out_seqs {
            let pending = in_flight.remove(&seq).unwrap();
            record_link_outcome(&pending.link_name, false, &mut primary_link, &mut secondary_link);
            writeln!(
                log_file,
                "{},timeout,{},{},attempt_{}",
                virtual_ms, pending.link_name, pending.message.seq, pending.message.attempts
            )
            .unwrap();

            if pending.message.attempts < MAX_RETRIES {
                report.retries += 1;
                outgoing_queue.push(pending.message.clone());
            } else {
                report.messages_dropped += 1;
                writeln!(log_file, "{},dropped,{},{},max_retries", virtual_ms, pending.link_name, pending.message.seq).unwrap();
            }
        }

        // Failover decision: only while there's enough history to trust the score.
        if active_link_is_primary
            && primary_link.attempts_recorded() >= MIN_ATTEMPTS_BEFORE_FAILOVER
            && primary_link.score() < FAILOVER_SCORE_THRESHOLD
        {
            active_link_is_primary = false;
            report.active_link_at_end = String::from("secondary");
            if report.failover_event_ms.is_none() {
                report.failover_event_ms = Some(virtual_ms);
                if outage_active {
                    report.failover_latency_ms = Some(virtual_ms - OUTAGE_START_MS);
                }
                writeln!(log_file, "{},failover,primary_to_secondary,-,score={:.2}", virtual_ms, primary_link.score()).unwrap();
                println!(
                    "t={:>6} ms  FAILOVER: primary link score {:.2} < {:.2}, switching to secondary link",
                    virtual_ms,
                    primary_link.score(),
                    FAILOVER_SCORE_THRESHOLD
                );
            }
        } else if !active_link_is_primary && !outage_active && primary_link.score() > 0.9 {
            // Fail back once the primary link has recovered and proven itself again.
            active_link_is_primary = true;
            report.active_link_at_end = String::from("primary");
            writeln!(log_file, "{},failback,secondary_to_primary,-,score={:.2}", virtual_ms, primary_link.score()).unwrap();
        }

        virtual_ms += TICK_MS;
    }

    let wall_clock_elapsed = wall_clock_start.elapsed();

    println!();
    println!("---- comms_link report ----");
    println!("Messages sent (including retries): {}", report.messages_sent);
    println!("Messages acknowledged:              {}", report.messages_acked);
    println!("Retries triggered:                  {}", report.retries);
    println!("Duplicates suppressed:              {}", report.duplicates_suppressed);
    println!("Messages dropped (max retries hit):  {}", report.messages_dropped);
    println!("Final active link:                  {}", report.active_link_at_end);
    println!("Final primary link score:           {:.2}", primary_link.score());
    println!("Final secondary link score:         {:.2}", secondary_link.score());
    match report.failover_latency_ms {
        Some(latency) => println!(
            "Failover latency (simulated time):  {} ms (< 250 ms target: {})",
            latency,
            if latency < 250 { "PASS" } else { "FAIL" }
        ),
        None => println!("Failover latency (simulated time):  no failover was triggered"),
    }
    println!(
        "Real wall-clock time to run this whole {} s simulated scenario: {:.3} s",
        TOTAL_SIM_MS / 1000,
        wall_clock_elapsed.as_secs_f64()
    );

    if seed == 7 {
        println!();
        println!("Full event log written to {}", log_path);
    }
}

fn send_message(
    message: &mut OutgoingMessage,
    virtual_ms: u64,
    active_link: &Link,
    in_flight: &mut HashMap<u64, InFlight>,
    stray_arrivals: &mut Vec<StrayArrival>,
    rng: &mut Rng,
    report: &mut Report,
    log_file: &mut File,
) {
    let link_name = active_link.name.clone();
    report.messages_sent += 1;

    // Whether this attempt will actually reach the receiver is decided at
    // send time using the current per-link fault state.
    let draw = rng.next_f64();
    let delivered = active_link.attempt_delivery(draw);

    writeln!(
        log_file,
        "{},send,{},{},attempt_{}",
        virtual_ms, link_name, message.seq, message.attempts
    )
    .unwrap();

    if !delivered {
        // From the sender's point of view this attempt is lost: it will
        // time out and (if attempts remain) be retried like any other
        // lost message. Separately, the network may still deliver this
        // exact packet late; if it does, the receiver will see the same
        // sequence number twice and must suppress the duplicate.
        let late_draw = rng.next_f64();
        if late_draw < LATE_ARRIVAL_PROBABILITY {
            stray_arrivals.push(StrayArrival {
                seq: message.seq,
                link_name: link_name.clone(),
                due_ms: virtual_ms + ACK_TIMEOUT_MS + ACK_DELAY_MS,
            });
        }
    }

    let pending = InFlight {
        message: message.clone(),
        sent_at_ms: virtual_ms,
        link_name,
        delivered,
        ack_due_ms: virtual_ms + ACK_DELAY_MS,
    };
    in_flight.insert(message.seq, pending);
}

fn resolve_delivery(
    seq: u64,
    link_name: &str,
    seen_by_receiver: &mut HashSet<u64>,
    report: &mut Report,
    log_file: &mut File,
    virtual_ms: u64,
) {
    if seen_by_receiver.contains(&seq) {
        report.duplicates_suppressed += 1;
        writeln!(log_file, "{},duplicate_suppressed,{},{},-", virtual_ms, link_name, seq).unwrap();
        return;
    }

    seen_by_receiver.insert(seq);
    report.messages_acked += 1;
    writeln!(log_file, "{},acked,{},{},-", virtual_ms, link_name, seq).unwrap();
}

fn record_link_outcome(link_name: &str, delivered: bool, primary_link: &mut Link, secondary_link: &mut Link) {
    if link_name == "primary" {
        primary_link.record_outcome(delivered);
    } else {
        secondary_link.record_outcome(delivered);
    }
}
