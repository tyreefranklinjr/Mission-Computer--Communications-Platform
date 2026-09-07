// link.rs
//
// A simulated communications link (a radio link, a cellular modem, a mesh
// hop, whatever the physical layer would be) plus a small link-scoring
// mechanism: each link remembers whether its most recent sends succeeded
// or failed and reports a rolling success rate. That score is what the
// failover logic in main.rs watches to decide when to switch links.

use std::collections::VecDeque;

pub const LINK_SCORE_WINDOW: usize = 6;

pub struct Link {
    pub name: String,
    pub forced_down: bool,
    pub base_loss_probability: f64,
    recent_outcomes: VecDeque<bool>,
}

impl Link {
    pub fn new(name: &str, base_loss_probability: f64) -> Link {
        Link {
            name: name.to_string(),
            forced_down: false,
            base_loss_probability,
            recent_outcomes: VecDeque::new(),
        }
    }

    // Records one send attempt's outcome (true = delivered, false = lost)
    // into the rolling window used for scoring.
    pub fn record_outcome(&mut self, delivered: bool) {
        self.recent_outcomes.push_back(delivered);
        while self.recent_outcomes.len() > LINK_SCORE_WINDOW {
            self.recent_outcomes.pop_front();
        }
    }

    // Recent success rate over the scoring window, in [0.0, 1.0]. A link
    // with no history yet is optimistically assumed healthy (1.0) so it
    // gets a fair chance before being scored down by real failures.
    pub fn attempts_recorded(&self) -> usize {
        self.recent_outcomes.len()
    }

    pub fn score(&self) -> f64 {
        if self.recent_outcomes.is_empty() {
            return 1.0;
        }
        let successes = self.recent_outcomes.iter().filter(|delivered| **delivered).count();
        successes as f64 / self.recent_outcomes.len() as f64
    }

    // Whether one transmission attempt on this link succeeds, given the
    // link's current state and a caller-supplied random draw in [0.0, 1.0).
    pub fn attempt_delivery(&self, random_draw: f64) -> bool {
        if self.forced_down {
            return false;
        }
        random_draw >= self.base_loss_probability
    }
}
