#!/usr/bin/env python3
"""ICBM Hazard Response Daemon — percentage-based speed reduction.

All reductions are percentages of current target speed:
1. Lead vehicle threats (rapid closure, cut-ins)
2. Model confidence (red/yellow/green)
3. Lane visibility (lane line probability)
4. Road edge uncertainty

Writes reduction percentage (0-100) to /data/openpilot/icbm_hazard_reduction.

Run on the Comma device:
  cd /data/openpilot && /usr/local/venv/bin/python3 icbm_hazard_daemon.py &
"""

import time

HAZARD_FILE = "/data/openpilot/icbm_hazard_reduction"
POLL_INTERVAL = 0.1  # 10 Hz
STATUS_INTERVAL = 30

MS_TO_MPH = 2.23694

# --- Lead vehicle threat settings ---
MIN_LEAD_PROB = 0.4
MIN_CLOSING_RATE = 0.5  # m/s

# Threat response table: (max_ttc, max_distance, reduction_pct)
THREAT_TABLE = [
  (3.0, 35, 25),    # Critical: < 3s TTC within 35m → 25% reduction
  (4.5, 50, 20),    # Severe:   < 4.5s TTC within 50m → 20%
  (6.0, 65, 13),    # Moderate: < 6s TTC within 65m → 13%
  (8.0, 90, 7),     # Mild:     < 8s TTC within 90m → 7%
]

# --- Model confidence (red/yellow/green) ---
# Applied instantly — these are serious confidence signals
# confidence enum: red=0, yellow=1, green=2
CONFIDENCE_REDUCTION_PCT = {
  0: 70,   # red:    70% reduction
  1: 40,   # yellow: 40% reduction
  2: 0,    # green:  no reduction
}

# --- Lane visibility ---
# Average probability of the two inner lane lines
LANE_PROB_THRESHOLDS = [
  (0.25, 20),   # Very poor lanes → 20% reduction
  (0.40, 13),   # Poor lanes → 13%
  (0.55, 7),    # Marginal lanes → 7%
]

# --- Road edge uncertainty ---
ROAD_EDGE_STD_THRESHOLD = 3.0
ROAD_EDGE_REDUCTION_PCT = 7

# --- Smoothing ---
LEAD_ONSET_ALPHA = 0.3
LEAD_RESTORE_ALPHA = 0.05
LANE_ALPHA = 0.1
LANE_RESTORE_ALPHA = 0.05
# Confidence: instant onset, moderate restore
CONF_ONSET_ALPHA = 1.0         # instant — user requested
CONF_RESTORE_ALPHA = 0.1


def compute_lead_reduction(v_ego, lead_x, lead_v, lead_prob):
  if lead_prob < MIN_LEAD_PROB or lead_x <= 0:
    return 0.0
  closing_rate = v_ego - lead_v
  if closing_rate < MIN_CLOSING_RATE:
    return 0.0
  ttc = lead_x / closing_rate
  for max_ttc, max_dist, reduction_pct in THREAT_TABLE:
    if ttc < max_ttc and lead_x < max_dist:
      return float(reduction_pct)
  return 0.0


def compute_lane_reduction(lane_probs, road_edge_stds):
  reduction = 0.0
  if len(lane_probs) >= 4:
    inner_prob = (lane_probs[1] + lane_probs[2]) / 2.0
    for threshold, pct in LANE_PROB_THRESHOLDS:
      if inner_prob < threshold:
        reduction = max(reduction, pct)
        break
  if len(road_edge_stds) >= 2:
    avg_edge_std = (road_edge_stds[0] + road_edge_stds[1]) / 2.0
    if avg_edge_std > ROAD_EDGE_STD_THRESHOLD:
      reduction = max(reduction, ROAD_EDGE_REDUCTION_PCT)
  return reduction


def write_reduction(value):
  with open(HAZARD_FILE, "w") as f:
    f.write(str(int(round(value))))


def main():
  import cereal.messaging as messaging

  sm = messaging.SubMaster(["modelV2", "carState"])
  current_lead_pct = 0.0
  current_lane_pct = 0.0
  current_conf_pct = 0.0
  last_status_time = 0
  confidence_baseline_set = False  # only apply confidence reduction after seeing green

  print("ICBM Hazard Response Daemon starting (percentage mode)...")
  write_reduction(0)

  while True:
    sm.update(200)

    if not sm.alive["modelV2"] or not sm.alive["carState"]:
      time.sleep(POLL_INTERVAL)
      continue

    v_ego = sm["carState"].vEgo
    model = sm["modelV2"]
    leads = model.leadsV3

    # --- Lead vehicle threat (percentage) ---
    lead_dist = 0.0
    lead_prob = 0.0
    closing = 0.0
    ttc = 999.0

    desired_lead_pct = 0.0
    if len(leads) > 0:
      lead = leads[0]
      lead_prob = lead.prob
      if lead_prob > MIN_LEAD_PROB and len(lead.x) > 0 and len(lead.v) > 0:
        lead_dist = lead.x[0]
        closing = v_ego - lead.v[0]
        ttc = lead_dist / closing if closing > 0 else 999
        desired_lead_pct = compute_lead_reduction(v_ego, lead_dist, lead.v[0], lead_prob)

    if desired_lead_pct > current_lead_pct:
      current_lead_pct += LEAD_ONSET_ALPHA * (desired_lead_pct - current_lead_pct)
    else:
      current_lead_pct += LEAD_RESTORE_ALPHA * (desired_lead_pct - current_lead_pct)
    if current_lead_pct < 1.0 and desired_lead_pct == 0.0:
      current_lead_pct = 0.0

    # --- Model confidence (instant onset) ---
    # Only apply confidence reductions after we've seen green at least once.
    # On unsupported cars (like the Bronco), confidence may stay red permanently
    # which would make this unusable. Once we see green, we know the model is
    # calibrated and transitions to yellow/red are meaningful.
    conf_raw = model.confidence.raw if model.confidence.raw < 3 else 2
    if conf_raw == 2:  # green
      confidence_baseline_set = True
    desired_conf_pct = float(CONFIDENCE_REDUCTION_PCT.get(conf_raw, 0)) if confidence_baseline_set else 0.0

    if desired_conf_pct > current_conf_pct:
      current_conf_pct += CONF_ONSET_ALPHA * (desired_conf_pct - current_conf_pct)
    else:
      current_conf_pct += CONF_RESTORE_ALPHA * (desired_conf_pct - current_conf_pct)
    if current_conf_pct < 1.0 and desired_conf_pct == 0.0:
      current_conf_pct = 0.0

    # --- Lane visibility + road edge (percentage) ---
    desired_lane_pct = compute_lane_reduction(
      list(model.laneLineProbs), list(model.roadEdgeStds))

    if desired_lane_pct > current_lane_pct:
      current_lane_pct += LANE_ALPHA * (desired_lane_pct - current_lane_pct)
    else:
      current_lane_pct += LANE_RESTORE_ALPHA * (desired_lane_pct - current_lane_pct)
    if current_lane_pct < 1.0 and desired_lane_pct == 0.0:
      current_lane_pct = 0.0

    # --- Total reduction = max of all percentages ---
    total_pct = max(current_lead_pct, current_conf_pct, current_lane_pct)
    write_reduction(total_pct)

    # --- Logging ---
    now = time.monotonic()
    inner_lane = (model.laneLineProbs[1] + model.laneLineProbs[2]) / 2.0 if len(model.laneLineProbs) >= 4 else 0
    conf_name = ["red", "yellow", "green"][conf_raw]

    if total_pct > 0:
      print(f"ACTIVE: {total_pct:.0f}% "
            f"(lead={current_lead_pct:.0f}% conf={current_conf_pct:.0f}% lane={current_lane_pct:.0f}%) "
            f"model={conf_name} lanes={inner_lane:.2f} "
            f"lead={lead_dist:.0f}m ttc={ttc:.1f}s "
            f"v={v_ego*MS_TO_MPH:.0f}mph")
      last_status_time = now
    elif now - last_status_time > STATUS_INTERVAL:
      print(f"STATUS: model={conf_name} lanes={inner_lane:.2f} "
            f"lead={lead_dist:.0f}m v={v_ego*MS_TO_MPH:.0f}mph 0%")
      last_status_time = now

    time.sleep(POLL_INTERVAL)


if __name__ == "__main__":
  main()
