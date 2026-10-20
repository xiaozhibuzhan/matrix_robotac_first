"""Predeclared actual-render acceptance checks for fixed multi-target missions.

Run with ``python -m task3_sim.validate_multi --random-cases 20``. Every sampled
configuration is saved before execution; failures are retained, never rerolled.
The metric positions are read only by this offline referee-side assessment.
"""

from __future__ import annotations

import argparse
import copy
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
import uuid

import numpy as np

from .__main__ import PACKAGE, PROJECT, parse_args, write_json
from .package_update import source_files


def source_hashes():
    """Bind each acceptance report to the exact package Python sources."""
    return {path.relative_to(PACKAGE).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in source_files(PACKAGE) if path.suffix == ".py"}


def tag_only_distractor(identity=42):
    """Test-only visual clutter; a small marker is never a fourth target."""
    return {"id": identity, "tag_only": True, "simulation": {
        "target_center_world_m": [2.7, .95, .71], "target_yaw_rad": 0.0}}


def round_robin_order(ids, ammo):
    """Independent predeclared shot schedule; target identity comes from config."""
    ordered = sorted(ids)
    if not ordered or len(set(ordered)) != len(ordered) or not isinstance(ammo, int) or isinstance(ammo, bool) or ammo < 0:
        raise ValueError("round-robin acceptance requires unique IDs and a nonnegative integer magazine")
    return [ordered[index % len(ordered)] for index in range(ammo)]


def scenarios(base, random_cases=20, seed=20261019):
    """Yield settings and expected outcomes fixed before observing any result."""
    declared_ids = [entry["id"] for entry in base["targets"]]
    repeat = base.get("mission", {}).get("shooting_policy", "single_pass") == "round_robin"
    ids = sorted(declared_ids) if repeat else declared_ids
    if len(ids) != 3 or len(set(ids)) != 3:
        raise ValueError("multi-target acceptance requires exactly three distinct declared targets")
    # A surround mission must discover targets outside the initial camera
    # view and traverse large yaw changes. Retain the old deadline for legacy
    # bounded-search configurations; direction-prior missions still need the
    # full per-target failure deadline in negative cases.
    search_mode = base.get("mission", {}).get("search_mode")
    surround_search = search_mode in ("full_rotation", "fixed_directions")
    mission_limit = (240.0 if repeat else 180.0) if surround_search else (90.0 if repeat else 45.0)

    def expected(status="SUCCESS", shots=None, completed=None, **extra):
        count = (base["simulation"]["ammo_count"] if repeat else len(ids)) if shots is None else shots
        covered = ids if completed is None else completed
        order = round_robin_order(covered, count) if repeat and covered else []
        return {"status": status, "shot_count": count,
                "completed_ids": covered, "shot_order": order if repeat else [identity for identity in ids if identity in covered],
                "min_ring": 9, "max_actual_miss_m": 0.02, **extra}

    nominal = copy.deepcopy(base)
    nominal["distractors"] = []
    nominal["observation"] = {"seed": seed}
    yield "three_targets", nominal, expected(require_bidirectional_search=search_mode == "fixed_directions"), mission_limit, {"kind": "fixed"}

    distractor = copy.deepcopy(base)
    if not distractor.get("distractors"):
        distractor["distractors"] = [tag_only_distractor()]
    distractor["observation"] = {"seed": seed}
    yield "unknown_tag_ignored", distractor, expected(require_unknown_id=42), mission_limit, {
        "kind": "fixed", "distractor_geometry": "small visual-only AprilTag; no green target or scoring surface"}

    calibrated = copy.deepcopy(nominal)
    calibrated["targets"][1]["tag"] = {"size_m": 0.14, "bullseye_offset_tag_m": [-0.06, 0.19, 0.0]}
    calibrated["targets"][2]["tag"] = {"size_m": 0.18, "bullseye_offset_tag_m": [0.05, 0.215, 0.0]}
    yield "per_id_calibration", calibrated, expected(), mission_limit, {"kind": "fixed"}

    missing = copy.deepcopy(nominal)
    if surround_search:
        # A board behind the robot is discoverable during a full turn. Present
        # the middle board's back instead; its mirrored back-face marker is
        # not a valid AprilTag. Acceptance also requires no decoded middle ID.
        initial_yaw = float(missing["targets"][1]["simulation"].get("target_yaw_rad", 0.0))
        missing["targets"][1]["simulation"]["target_yaw_rad"] = initial_yaw + math.pi
    else:
        missing["targets"][1]["simulation"]["target_center_world_m"] = [-2.7, 0.0, 0.5]
    yield "middle_target_missing", missing, expected("NO_AMMO" if repeat else "PARTIAL",
        base["simulation"]["ammo_count"] if repeat else 2, [ids[0], ids[2]],
        require_lost=True, require_unseen_id=ids[1],
        failed_reasons={str(ids[1]): "target_timeout"}), mission_limit, {"kind": "fixed"}

    two = copy.deepcopy(nominal)
    two["simulation"]["ammo_count"] = 2
    yield "two_rounds_only", two, expected("NO_AMMO", 2, ids[:2],
        failed_reasons={str(ids[2]): "no_ammo"}), mission_limit, {"kind": "fixed"}

    empty = copy.deepcopy(nominal)
    empty["simulation"]["ammo_count"] = 0
    yield "empty_magazine", empty, expected("NO_AMMO", 0, [],
        failed_reasons={str(identity): "no_ammo" for identity in ids}), 2.0, {"kind": "fixed"}

    if repeat:
        # Three shots are the competition nominal case. Keep five explicit so
        # repeated-shot accounting remains covered when the default is three.
        for ammo in (1, 5, 6):
            magazine = copy.deepcopy(nominal)
            magazine["simulation"]["ammo_count"] = ammo
            covered = ids[:min(ammo, len(ids))]
            failures = {str(identity): "no_ammo" for identity in ids if identity not in covered}
            yield f"magazine_{ammo}_shots", magazine, expected(
                "SUCCESS" if len(covered) == len(ids) else "NO_AMMO", ammo, covered,
                failed_reasons=failures), mission_limit, {"kind": "fixed", "ammo_count": ammo}
        reordered = copy.deepcopy(nominal)
        reordered["targets"] = list(reversed(reordered["targets"]))
        yield "round_robin_numeric_id_order", reordered, expected(), mission_limit, {
            "kind": "fixed", "declared_order": [entry["id"] for entry in reordered["targets"]]}
        second_round_loss = copy.deepcopy(nominal)
        second_round_loss["simulation"]["ammo_count"] = 6
        second_round_loss["observation"] = {"drop_every_n": 13, "seed": seed}
        yield "second_round_frame_loss", second_round_loss, expected(shots=6,
            require_second_round_drop=True), mission_limit, {"kind": "fixed"}

    delayed = copy.deepcopy(nominal)
    delayed["observation"] = {"delay_s": 0.05, "drop_probability": 0.2, "seed": seed}
    yield "delay_and_drop", delayed, expected(), mission_limit, {"kind": "fixed"}

    periodic = copy.deepcopy(nominal)
    # A complete stable dwell still fits between these deterministic losses.
    periodic["observation"] = {"drop_every_n": 13, "seed": seed}
    yield "periodic_frame_loss", periodic, expected(), mission_limit, {"kind": "fixed"}

    jitter = copy.deepcopy(nominal)
    jitter["observation"] = {"delay_s": 0.03, "jitter_s": 0.04, "seed": seed}
    yield "delivery_jitter", jitter, expected(), mission_limit, {"kind": "fixed"}

    stale = copy.deepcopy(nominal)
    stale["observation"] = {"delay_s": 0.25, "seed": seed}
    # With no fresh startup image the mission never selects an active target;
    # all pending targets fail at the global deadline, without a LOST transition.
    yield "stale_frames_no_fire", stale, expected("TIMEOUT", 0, [],
        require_stale_frame=True, failed_reasons={str(identity): "global_timeout" for identity in ids}), 3.0, {"kind": "fixed"}

    blackout = copy.deepcopy(nominal)
    blackout["observation"] = {"blackout_intervals_s": [[1.0, 3.0]], "seed": seed}
    yield "blackout_then_recover", blackout, expected(require_lost=True), mission_limit, {"kind": "fixed"}

    if search_mode == "fixed_directions":
        # Physical slots stay fixed while decoded identities and declared fire
        # order change independently. No mission field reveals this mapping.
        physical_slots = [copy.deepcopy(entry.get("simulation", {})) for entry in nominal["targets"]]
        for permutation in ((1, 0, 2), (2, 0, 1)):
            swapped = copy.deepcopy(nominal)
            for index, slot in enumerate(permutation):
                swapped["targets"][index]["simulation"] = copy.deepcopy(physical_slots[slot])
            label = "".join(str(value) for value in permutation)
            yield f"identity_directions_{label}", swapped, expected(), mission_limit, {
                "kind": "fixed", "direction_slot_permutation": list(permutation)}
        for order in ((2, 1, 0), (1, 2, 0)):
            reordered = copy.deepcopy(nominal)
            reordered["targets"] = [reordered["targets"][index] for index in order]
            label = "".join(str(value) for value in order)
            yield f"shooting_order_{label}", reordered, expected(), mission_limit, {
                "kind": "fixed", "declared_order": [entry["id"] for entry in reordered["targets"]]}

    rng = np.random.default_rng(seed)
    for index in range(random_cases):
        config = copy.deepcopy(base)
        parameters = {"kind": "seeded_random", "sample_index": index, "seed": seed,
                      "initial_x_m": float(rng.uniform(-0.10, 0.10)),
                      "initial_y_m": float(rng.uniform(-0.10, 0.10)),
                      "initial_pitch_rad": float(rng.uniform(0.0, 0.08)),
                      "initial_yaw_rad": float(rng.uniform(-0.15, 0.15)),
                      "distance_scale": float(rng.uniform(0.9, 1.1)),
                      "exposure": float(rng.uniform(0.75, 1.25)),
                      "noise_std_u8": float(rng.uniform(0.0, 4.0)),
                      "delay_s": float(rng.uniform(0.0, 0.05)),
                      "drop_probability": float(rng.uniform(0.0, 0.15)),
                      "effect_seed": int(rng.integers(0, 2**31)),
                      "transport_seed": int(rng.integers(0, 2**31))}
        config["simulation"]["initial_position_m"][:2] = [parameters["initial_x_m"], parameters["initial_y_m"]]
        config["simulation"]["initial_rpy_rad"][1:] = [parameters["initial_pitch_rad"], parameters["initial_yaw_rad"]]
        for entry in config["targets"] + config.get("distractors", []):
            position = entry["simulation"]["target_center_world_m"]
            entry["simulation"]["target_center_world_m"] = [position[0]*parameters["distance_scale"],
                                                           position[1]*parameters["distance_scale"], position[2]]
        config["appearance"]["camera_effects"].update(exposure=parameters["exposure"],
            noise_std_u8=parameters["noise_std_u8"], seed=parameters["effect_seed"])
        config["observation"] = {"delay_s": parameters["delay_s"],
                                 "drop_probability": parameters["drop_probability"],
                                 "seed": parameters["transport_seed"]}
        yield f"random_{index:03d}", config, expected(), mission_limit, parameters


def direction_route_audit(summary, events, config, expected):
    """Offline proof that direction priors guide search, never target identity."""
    if config.get("mission", {}).get("search_mode") != "fixed_directions":
        return {}, {}
    mission = summary.get("mission", {})
    search = mission.get("search", {})
    ids = {entry["id"] for entry in config["targets"]}
    records = {row["id"]: row for row in mission.get("targets", [])}
    observed = search.get("observed_bearings", {})
    directions = config["mission"].get("search_directions_rad", [0, math.pi / 2, -math.pi / 2])
    initial = search.get("initial_yaw_rad")
    search_events = [event for event in events if event.get("state") == "SEARCHING"]
    rates = [float(event["command"]["yaw_rate_rad_s"]) for event in search_events]
    def wrap(value):
        return math.atan2(math.sin(value), math.cos(value))
    shortest = []
    for event in search_events:
        probes = [probe for probe in records.get(event.get("active_target_id"), {}).get("search_probes", [])
                  if probe["timestamp"] <= event["time_s"] + 1e-9]
        yaw = event.get("feedback", {}).get("rpy_rad", [None, None, None])[2]
        if not probes or yaw is None:
            shortest.append(False)
            continue
        error = wrap(probes[-1]["yaw_rad"] - yaw)
        rate = float(event["command"]["yaw_rate_rad_s"])
        shortest.append(abs(error) <= 1e-9 or rate * error >= -1e-9)
    prior_goals = [probe for record in records.values() for probe in record.get("search_probes", [])
                   if probe.get("source") == "public_direction_prior_identity_unknown"]
    checks = {
        "fixed_direction_search_mode_logged": search.get("mode") == "fixed_directions",
        "bearing_map_contains_only_declared_image_identities": all(
            int(identity) in ids and bearing.get("source") == "image_pose_with_timestamped_robot_feedback"
            for identity, bearing in observed.items()),
        "prior_probe_goals_match_public_directions": all(
            initial is not None and isinstance(probe.get("direction_index"), int)
            and 0 <= probe["direction_index"] < len(directions)
            and abs(wrap(probe["yaw_rad"] - initial - directions[probe["direction_index"]])) < 1e-8
            for probe in prior_goals),
        "search_turns_follow_shortest_angular_error": all(shortest),
        "search_commands_only_yaw": all(all(abs(float(event.get("command", {}).get(key, math.inf))) <= 1e-12
            for key in ("vx_m_s", "vy_m_s", "pitch_rate_rad_s")) for event in search_events),
        "fixed_probe_is_stationary": all(all(abs(float(event.get("command", {}).get(key, math.inf))) <= 1e-12
            for key in ("vx_m_s", "vy_m_s", "yaw_rate_rad_s", "pitch_rate_rad_s"))
            for event in events if event.get("state") == "PROBING"),
        "no_extra_full_rotation_per_selected_target": all(
            0 <= record.get("search_travel_rad", math.inf) < 2 * math.pi for record in records.values()),
    }
    if expected.get("require_bidirectional_search"):
        checks["both_turn_directions_actually_used"] = any(rate > 0 for rate in rates) and any(rate < 0 for rate in rates)
    metrics = {"search_positive_commands": sum(rate > 0 for rate in rates),
               "search_negative_commands": sum(rate < 0 for rate in rates),
               "feedback_rotation_rad_by_target": {str(identity): row.get("search_travel_rad") for identity, row in records.items()},
               "probe_goals_by_target": {str(identity): row.get("search_probes", []) for identity, row in records.items()},
               "observed_bearings": observed}
    return checks, metrics


def _finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _repeat_dwell_evidence(events, config):
    """Require new, stationary image dwell after the preceding physical shot.

    A short missing-delivery poll may pause the clock, exactly as the mission
    does; it earns no dwell time. Invalid feedback, stale/repeated images,
    target changes, motion, and control transitions revoke the whole dwell.
    """
    control = config["control"]
    frame_age = control["max_frame_age_s"]
    feedback_age = control["max_feedback_age_s"]
    stable = control.get("stable_duration_s", .4)
    prior_shot_time = -math.inf
    started = last_poll = last_capture = None
    accumulated, active_id = 0.0, None
    evidence = []
    for event in events:
        now = event.get("time_s")
        identity = event.get("active_target_id")
        feedback = event.get("feedback", {})
        stamp = feedback.get("timestamp")
        stationary = (feedback.get("valid") is True and _finite_number(now) and _finite_number(stamp)
            and 0 <= now-stamp <= feedback_age + 1e-8
            and all(isinstance(feedback.get(key), list) and len(feedback[key]) == 3
                    and all(_finite_number(value) for value in feedback[key])
                    and np.linalg.norm(feedback[key]) <= limit + 1e-10
                    for key, limit in (("gyro_rad_s", control["max_gyro_rad_s"]),
                                       ("velocity_m_s", control["max_speed_m_s"]))))
        capture = event.get("frame_timestamp_s")
        geometry = event.get("geometry") or {}
        detections = [det for det in event.get("detections", []) if det.get("tag_id") == identity]
        aligned = (stationary and event.get("state") in ("SETTLING", "READY")
            and event.get("frame_delivered") is True and _finite_number(capture)
            and capture > prior_shot_time and 0 <= now-capture <= frame_age + 1e-8
            and geometry.get("reachable") is True and _finite_number(geometry.get("error_norm_m"))
            and geometry["error_norm_m"] <= control["aim_tolerance_m"] + 1e-10
            and len(detections) == 1 and detections[0].get("hamming") == 0
            and _finite_number(detections[0].get("timestamp"))
            and abs(detections[0]["timestamp"]-capture) <= 1e-6)
        continuing = (started is not None and identity == active_id and last_capture is not None
            and _finite_number(capture) and last_capture < capture <= last_capture+frame_age+1e-8)
        if aligned:
            if continuing:
                accumulated += max(0.0, now-last_poll)
            else:
                started, accumulated = now, 0.0
            active_id, last_capture, last_poll = identity, capture, now
        elif (stationary and event.get("state") == "WAIT" and not event.get("frame_delivered")
              and started is not None and identity == active_id and last_capture is not None
              and 0 <= now-last_capture <= frame_age+1e-8
              and event.get("reason") == "short camera delivery gap; stable dwell paused"):
            last_poll = now
        else:
            started = last_poll = last_capture = active_id = None
            accumulated = 0.0
        if event.get("shot_record") is not None or event.get("shot") is not None:
            evidence.append({"target_id": identity, "shot_time_s": now,
                "dwell_started_s": started, "observed_stable_s": accumulated,
                "new_image_after_previous_shot": _finite_number(capture) and capture > prior_shot_time,
                "valid": bool(aligned and started is not None and accumulated+1e-7 >= stable)})
            prior_shot_time = now if _finite_number(now) else math.inf
            started = last_poll = last_capture = active_id = None
            accumulated = 0.0
    return evidence


def repeat_ledger_audit(summary, events, config, expected_order=None):
    """Audit every repeat shot rather than trusting the final score dictionary.

    This supplements the existing per-shot physical, identity, freshness, and
    SDK audits. The caller may supply a predeclared negative-case schedule;
    successful native acceptance always uses the full configured schedule.
    """
    if config.get("mission", {}).get("shooting_policy", "single_pass") != "round_robin":
        return {}, {}
    mission = summary.get("mission", {})
    ids = sorted(row["id"] for row in config["targets"])
    ammo = config["simulation"]["ammo_count"]
    order = round_robin_order(ids, ammo) if expected_order is None else list(expected_order)
    shots = mission.get("shots", [])
    physical = summary.get("shots", [])
    targets = mission.get("targets", [])
    records = {row.get("id"): row for row in targets}
    actual_order = [row.get("intended_target_id") for row in shots]
    per_id = {identity: [row for row in shots if row.get("intended_target_id") == identity] for identity in ids}
    hit_counts = {identity: sum(row.get("correct_target_hit") is True for row in rows)
                  for identity, rows in per_id.items()}
    coverage = all(hit_counts.values())
    last_scores = {identity: rows[-1].get("credited_ring") if rows else 0 for identity, rows in per_id.items()}
    attempts = {identity: 0 for identity in ids}
    visit_pairs, attempt_fields = [], []
    for row in shots:
        identity = row.get("intended_target_id")
        if identity in attempts:
            attempts[identity] += 1
        round_index = row.get("round_index")
        visit_pairs.append((round_index, identity))
        attempt_fields.append(row.get("target_attempt") == attempts.get(identity)
            and isinstance(round_index, int) and not isinstance(round_index, bool) and round_index >= 1)
    round_order = [row.get("round_index") for row in shots]
    # A declared ID may retire after a timeout, but a shooting visit can never
    # occur twice in one round, move backward, or skip an available earlier ID.
    expected_rounds = []
    expected_round, previous = 1, None
    for identity in order:
        if previous is not None and identity <= previous:
            expected_round += 1
        expected_rounds.append(expected_round)
        previous = identity
    numeric_scores = all(_finite_number(row.get("credited_ring")) and 0 <= row["credited_ring"] <= 10 for row in shots)
    dwell = _repeat_dwell_evidence(events, config)
    fire_events = [event for event in events if event.get("shot_record") is not None or event.get("shot") is not None]
    visits = [(identity, visit) for identity in ids for visit in records.get(identity, {}).get("visits", [])]
    visit_pairs_all = [(visit.get("round_index"), identity) for identity, visit in visits]
    linked_visits = [(identity, visit) for identity, visit in visits if visit.get("shot_index") is not None]
    visit_order = sorted(visits, key=lambda item: item[1].get("selected_at_s", math.inf))
    visits_valid = all(isinstance(visit.get("round_index"), int) and not isinstance(visit.get("round_index"), bool)
        and visit["round_index"] >= 1 and _finite_number(visit.get("selected_at_s"))
        and (visit.get("finished_at_s") is None or (_finite_number(visit["finished_at_s"])
             and visit["finished_at_s"] >= visit["selected_at_s"])) for _, visit in visits)
    retired_valid = all((row.get("retired") is False and row.get("retired_round") is None and row.get("retired_at_s") is None)
        or (row.get("retired") is True and row.get("visits")
            and row.get("retired_round") == row["visits"][-1].get("round_index")
            and row.get("retired_at_s") == row["visits"][-1].get("finished_at_s")
            and row["visits"][-1].get("shot_index") is None
            and row["visits"][-1].get("state") == "FAILED") for row in targets)
    last_visit_round = max([1] + [visit.get("round_index", 0) for _, visit in visits if isinstance(visit.get("round_index"), int)])
    finished_rounds = 0
    if visits_valid:
        for current_round in range(1, last_visit_round+1):
            available = [identity for identity in ids if not (records.get(identity, {}).get("retired") is True
                         and isinstance(records[identity].get("retired_round"), int)
                         and records[identity]["retired_round"] < current_round)]
            if available and all(any(visit.get("round_index") == current_round
                and visit.get("finished_at_s") is not None for candidate, visit in visits if candidate == identity)
                for identity in available):
                finished_rounds = current_round
            else:
                break
    reason = mission.get("termination_reason")
    if summary.get("ammo_remaining") == 0:
        terminal_reason_valid = reason == "ammo_exhausted"
    elif summary.get("status") in ("PARTIAL", "FAILED"):
        terminal_reason_valid = reason == "all_targets_retired" and all(row.get("retired") is True for row in targets)
    elif summary.get("status") == "TIMEOUT":
        terminal_reason_valid = reason == "global_timeout"
    else:
        terminal_reason_valid = summary.get("status") == "CANCELLED" and isinstance(reason, str)
    checks = {
        "round_robin_policy_and_numeric_order_recorded": mission.get("shooting_policy") == "round_robin"
            and mission.get("firing_order") == ids,
        "round_robin_complete_expected_shot_sequence": actual_order == order and len(physical) == len(order),
        "round_robin_one_shot_per_visit": len(set(visit_pairs)) == len(visit_pairs)
            and all(attempt_fields) and round_order == expected_rounds,
        "round_robin_visits_and_retirement_are_consistent": visits_valid and retired_valid
            and len(set(visit_pairs_all)) == len(visit_pairs_all)
            and [(visit.get("round_index"), identity) for identity, visit in visit_order] == sorted(visit_pairs_all)
            and len(linked_visits) == len(shots)
            and all(sum(identity == row.get("intended_target_id") and visit.get("round_index") == row.get("round_index")
                        and visit.get("shot_index") == row.get("shot_index")
                        and visit.get("finished_at_s") == row.get("timestamp")
                        for identity, visit in linked_visits) == 1 for row in shots),
        "round_robin_summary_round_counters": mission.get("completed_rounds") == finished_rounds
            and ((mission.get("round_index") == last_visit_round) if reason in ("ammo_exhausted", "all_targets_retired")
                 else mission.get("round_index") in (last_visit_round, last_visit_round+1)),
        "round_robin_event_mission_rows_match": len(fire_events) == len(shots)
            and all(event.get("mission_shot_record") == row and event.get("round_index") == row.get("round_index")
                    for event, row in zip(fire_events, shots)),
        "round_robin_per_shot_credit_matches_physical_outcome": len(physical) == len(shots)
            and all(row.get("intended_target_id") == raw.get("intended_target_id")
                and row.get("actual_target_id") == raw.get("actual_target_id")
                and row.get("hit") == raw.get("hit")
                and row.get("correct_target_hit") is (raw.get("hit") is True
                    and raw.get("actual_target_id") == raw.get("intended_target_id"))
                and row.get("credited_ring") == (raw.get("ring") if raw.get("hit") is True
                    and raw.get("actual_target_id") == raw.get("intended_target_id") else 0)
                for row, raw in zip(shots, physical)),
        "round_robin_cumulative_attempts_and_hits": len(targets) == len(ids) and set(records) == set(ids)
            and all(records[identity].get("attempts") == len(per_id[identity])
                    and records[identity].get("successful_hits") == hit_counts[identity] for identity in ids),
        "round_robin_last_score_per_target": numeric_scores
            and all(records.get(identity, {}).get("ring") == last_scores[identity] for identity in ids)
            and mission.get("latest_target_score_total") == sum(last_scores.values()),
        "round_robin_total_score_is_all_shots": numeric_scores
            and mission.get("total_score") == sum(row["credited_ring"] for row in shots),
        "round_robin_target_score_sums": numeric_scores
            and all(records.get(identity, {}).get("score_sum") == sum(row["credited_ring"] for row in per_id[identity])
                    for identity in ids),
        "round_robin_shot_score_sum": numeric_scores
            and mission.get("shot_score_sum") == sum(row["credited_ring"] for row in shots)
            and mission.get("shot_score_sum") == mission.get("total_score"),
        "round_robin_coverage_is_actual_correct_hits": mission.get("coverage_complete") is coverage,
        "round_robin_success_requires_empty_magazine_and_coverage": summary.get("status") != "SUCCESS"
            or (coverage and summary.get("ammo_remaining") == 0 and len(shots) == ammo),
        "round_robin_no_ammo_without_coverage": summary.get("ammo_remaining") != 0
            or summary.get("status") in (("SUCCESS",) if coverage else ("NO_AMMO",)),
        "round_robin_terminal_reason_matches_magazine": terminal_reason_valid,
        "round_robin_every_shot_has_new_independent_dwell": len(dwell) == len(shots)
            and all(row["valid"] and row["new_image_after_previous_shot"] for row in dwell),
    }
    return checks, {"shooting_policy": "round_robin", "expected_shot_order": order,
        "actual_shot_order": actual_order, "round_indices": round_order,
        "attempts_by_target": attempts, "successful_hits_by_target": hit_counts,
        "last_credited_scores": last_scores,
        "cumulative_scores_by_target": {identity: sum(row.get("credited_ring", 0) for row in per_id[identity]) for identity in ids},
        "total_score": mission.get("total_score"), "latest_target_score_total": mission.get("latest_target_score_total"),
        "shot_score_sum": mission.get("shot_score_sum"),
        "per_shot_dwell": dwell}


def assess(summary, events, process_code, config, expected):
    """Check identities, ledger arithmetic and actual per-shot image fire gates."""
    ids = [entry["id"] for entry in config["targets"]]
    repeat = config.get("mission", {}).get("shooting_policy", "single_pass") == "round_robin"
    mission = summary.get("mission", {})
    targets = summary.get("targets", mission.get("targets", []))
    shots = mission.get("shots", [])
    physical_shots = summary.get("shots", [])
    records = {row["id"]: row for row in targets}
    completed = sorted(row["id"] for row in targets if row["state"] == "COMPLETED")
    checks = {
        "status": summary.get("status") == expected["status"],
        "exit_code": process_code == (0 if expected["status"] == "SUCCESS" else 2),
        "declared_id_records": len(targets) == len(ids) and set(records) == set(ids),
        "completed_ids": completed == sorted(expected["completed_ids"]),
        "terminal_target_states": all(row["state"] in ("COMPLETED", "FAILED") for row in targets),
        "shot_count": len(shots) == expected["shot_count"] == summary.get("shot_count"),
        "finite_ammo": summary.get("ammo_initial") == config["simulation"]["ammo_count"]
                       and summary.get("ammo_remaining") == config["simulation"]["ammo_count"]-len(shots),
        "model_unchanged": summary.get("model_sources_unchanged") is True,
        "shutdown_completed": summary.get("shutdown_completed") is True,
        "perception_closed": summary.get("perception_closed") is True,
        "cleanup_errors": summary.get("cleanup_errors") == [],
        "events_present": bool(events),
    }
    if not repeat:
        checks["one_attempt_per_target"] = all(0 <= row.get("attempts", -1) <= 1 for row in targets)
    fire_events = [event for event in events if event.get("shot") is not None]
    checks["one_fire_event_per_ledger_row"] = len(fire_events) == len(shots) == len(physical_shots)
    intended_ids = [row.get("intended_target_id") for row in shots]
    expected_order = expected.get("shot_order", [identity for identity in ids if identity in expected["completed_ids"]])
    checks["shots_follow_declared_order"] = intended_ids == expected_order
    visit_keys = [(row.get("round_index"), row.get("intended_target_id")) for row in shots] if repeat else intended_ids
    checks["no_duplicate_or_unknown_shots"] = len(set(visit_keys)) == len(visit_keys) and set(intended_ids) <= set(ids)
    checks["sequential_shot_indices"] = [row.get("shot_index") for row in shots] == list(range(1, len(shots)+1))
    checks["ammo_ledger"] = all(row.get("ammo_before") == config["simulation"]["ammo_count"]-index
        and row.get("ammo_remaining") == config["simulation"]["ammo_count"]-index-1 for index, row in enumerate(shots))
    checks["same_id_hit_and_credit"] = all(row.get("actual_target_id") == row.get("intended_target_id")
        and row.get("correct_target_hit") is True and row.get("hit") is True
        and row.get("credited_ring", -1) >= expected["min_ring"] for row in shots)
    if repeat:
        checks["completed_targets_have_correct_hit_history"] = all(any(
            shot.get("intended_target_id") == row["id"] and shot.get("correct_target_hit") is True
            for shot in shots) for row in targets if row["state"] == "COMPLETED")
        checks["total_score_matches_ledger"] = mission.get("total_score") == sum(row.get("credited_ring", 0) for row in shots)
    else:
        checks["completed_targets_have_one_matching_shot"] = all(sum(
            shot.get("intended_target_id") == row["id"] and shot.get("correct_target_hit") is True
            and shot.get("credited_ring") == row.get("ring") for shot in shots) == 1
            for row in targets if row["state"] == "COMPLETED")
        checks["total_score_matches_ledger"] = mission.get("total_score") == sum(row.get("credited_ring", 0) for row in shots)
    centers = {entry["id"]: np.asarray(entry.get("simulation", {}).get("target_center_world_m",
               config["simulation"]["target_center_world_m"]), dtype=float) for entry in config["targets"]}
    misses = []
    for row in shots:
        point, target_id = row.get("point_world_m"), row.get("intended_target_id")
        misses.append(float(np.linalg.norm(np.asarray(point)-centers[target_id]))
                      if point is not None and target_id in centers else math.inf)
    checks["actual_miss_limit"] = all(math.isfinite(miss) and miss <= expected["max_actual_miss_m"] for miss in misses)
    limit = config["control"]["max_frame_age_s"]
    checks["fresh_observation_at_every_shot"] = all(event.get("frame_delivered") is True
        and event.get("frame_timestamp_s") is not None and event.get("frame_age_s") is not None
        and -1e-9 <= event["time_s"]-event["frame_timestamp_s"] <= limit+1e-9
        and abs(event["frame_age_s"]-(event["time_s"]-event["frame_timestamp_s"])) < 1e-8
        for event in fire_events)
    checks["shot_events_match_ledger"] = all(event.get("shot_record") == physical
        and event.get("shot_accepted") is True and event.get("active_target_id") == row.get("intended_target_id")
        and abs(event["time_s"]-row.get("timestamp", math.inf)) < 1e-8
        and physical.get("sequence") == row.get("shot_index")
        and physical.get("timestamp_s") == row.get("timestamp")
        and physical.get("ammo_before") == row.get("ammo_before")
        and physical.get("ammo_after") == row.get("ammo_remaining")
        and all(physical.get(key) == row.get(key) for key in ("intended_target_id", "actual_target_id", "hit", "point_world_m"))
        for event, row, physical in zip(fire_events, shots, physical_shots))
    checks["ready_geometry_at_every_shot"] = all(event.get("state") == "READY"
        and event.get("geometry", {}).get("reachable") is True
        and event.get("geometry", {}).get("error_norm_m", math.inf) <= config["control"]["aim_tolerance_m"]
        for event in fire_events)
    checks["matching_decoded_detection_at_every_shot"] = all(
        len([det for det in event.get("detections", []) if det.get("tag_id") == event.get("active_target_id")]) == 1
        for event in fire_events)
    tag_config = {entry["id"]: {**config["tag"], **entry.get("tag", {})} for entry in config["targets"]}
    checks["decoded_detection_quality_at_every_shot"] = all(
        det.get("hamming") == 0
        and det.get("decision_margin", -math.inf) >= tag_config[event["active_target_id"]]["min_decision_margin"]
        and 0 <= det.get("reprojection_error_px", math.inf) <= tag_config[event["active_target_id"]]["max_reprojection_error_px"]
        and abs(det.get("timestamp", math.inf)-event["frame_timestamp_s"]) <= 1e-6
        for event in fire_events for det in event.get("detections", []) if det.get("tag_id") == event.get("active_target_id"))
    checks["fresh_stationary_feedback_at_every_shot"] = all(
        event.get("feedback", {}).get("valid") is True
        and 0 <= event["time_s"]-event["feedback"].get("timestamp", -math.inf) <= config["control"]["max_feedback_age_s"]
        and float(np.linalg.norm(event["feedback"].get("gyro_rad_s", [math.inf]))) <= config["control"]["max_gyro_rad_s"]
        and float(np.linalg.norm(event["feedback"].get("velocity_m_s", [math.inf]))) <= config["control"]["max_speed_m_s"]
        for event in fire_events)
    timestamps = [event["frame_timestamp_s"] for event in events if event.get("frame_delivered")]
    checks["delivered_acquisitions_strictly_increase"] = all(a < b for a, b in zip(timestamps, timestamps[1:]))
    checks["no_future_or_restamped_acquisitions"] = all(
        event.get("frame_timestamp_s") is None or event["frame_timestamp_s"] <= event["capture_timestamp_s"]+1e-9
        for event in events)
    checks["no_shot_in_blackout_without_fresh_preblackout_frame"] = all(
        not any(start <= event["frame_timestamp_s"] < end for start, end in config.get("observation", {}).get("blackout_intervals_s", []))
        for event in fire_events)
    checks["no_motion_without_fresh_observation"] = all(
        all(abs(float(event.get("command", {}).get(key, math.inf))) <= 1e-12
            for key in ("vx_m_s", "vy_m_s", "yaw_rate_rad_s", "pitch_rate_rad_s"))
        for event in events if not event.get("frame_delivered") or event.get("frame_age_s", math.inf) > limit)
    if config.get("mission", {}).get("search_mode") in ("full_rotation", "fixed_directions"):
        checks["search_exhausted_is_stationary"] = all(
            all(abs(float(event.get("command", {}).get(key, math.inf))) <= 1e-12
                for key in ("vx_m_s", "vy_m_s", "yaw_rate_rad_s", "pitch_rate_rad_s"))
            for event in events if event.get("state") == "SEARCH_EXHAUSTED")
    checks["transport_delivery_counts_match_events"] = (
        summary.get("transport", {}).get("pushed") == len(events)
        and summary.get("transport", {}).get("delivered") == len(timestamps))
    for identity, reason in expected.get("failed_reasons", {}).items():
        record = records.get(int(identity), {})
        checks[f"target_{identity}_failure_reason"] = record.get("state") == "FAILED" and record.get("reason") == reason
    if expected.get("require_unknown_id") is not None:
        unknown = expected["require_unknown_id"]
        checks["unknown_tag_actually_detected"] = any(det.get("tag_id") == unknown for event in events for det in event.get("detections", []))
        checks["unknown_tag_never_selected"] = all(event.get("active_target_id") != unknown for event in events)
    if expected.get("require_lost"):
        checks["lost_state_observed"] = any(history["to"] == "LOST" for row in targets for history in row.get("history", []))
    if expected.get("require_unseen_id") is not None:
        checks["missing_id_never_decoded"] = not any(det.get("tag_id") == expected["require_unseen_id"]
            for event in events for det in event.get("detections", []))
    if expected.get("require_stale_frame"):
        checks["stale_frames_actually_delivered"] = any(event.get("frame_age_s") is not None and event["frame_age_s"] > limit for event in events)
    if expected.get("require_second_round_drop"):
        first_round_end = next((shot.get("timestamp") for shot in shots if shot.get("shot_index") == len(ids)), math.inf)
        checks["second_round_delivery_loss_actually_observed"] = any(
            event.get("time_s", -math.inf) > first_round_end and not event.get("frame_delivered") for event in events)
    metrics = {"actual_miss_m": misses, "max_actual_miss_m": max(misses, default=None),
               "shot_times_s": [row["timestamp"] for row in shots],
               "completed_ids": completed, "target_states": {str(row["id"]): row["state"] for row in targets},
               "detected_ids": sorted({det["tag_id"] for event in events for det in event.get("detections", [])}),
               "delivered_frames": len(timestamps), "capture_frames": len(events),
               "transport": summary.get("transport"), "total_score": mission.get("total_score")}
    route_checks, route_metrics = direction_route_audit(summary, events, config, expected)
    checks.update(route_checks)
    if route_metrics:
        metrics["direction_routes"] = route_metrics
    repeat_checks, repeat_metrics = repeat_ledger_audit(summary, events, config, expected_order)
    checks.update(repeat_checks)
    if repeat_metrics:
        metrics["repeat_shooting"] = repeat_metrics
    return checks, metrics


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=PACKAGE/"config/multi_fixed.json")
    parser.add_argument("--model-root", type=Path, default=PROJECT/"src/robot_mujoco/zsibot_robots/xgb")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--random-cases", type=int, default=20)
    parser.add_argument("--seed", type=int, default=20261019)
    args = parser.parse_args(argv)
    if not 0 <= args.random_cases <= 1000 or not 0 <= args.seed < 2**63:
        parser.error("random cases must be in [0,1000]; seed must be in [0,2**63)")
    from .multi import run_multi
    output = (args.output or PACKAGE/"runs"/f"multi_validation_{datetime.now():%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:8]}").resolve()
    if output.exists() and any(output.iterdir()):
        parser.error("output directory must be new or empty")
    output.mkdir(parents=True, exist_ok=True)
    base = json.loads(args.config.read_text(encoding="utf-8-sig"))
    report = {"complete": False, "all_passed": False, "platform": platform.platform(),
              "python": platform.python_version(), "ubuntu_runtime_tested": sys.platform.startswith("linux"),
              "seed": args.seed, "random_cases_requested": args.random_cases,
              "fixed_cases": len(list(scenarios(base, random_cases=0, seed=args.seed))),
              "code_semantics": "Each case calls run_multi in this validator process. process_code is the returned application code; independent CLI termination is a separate check. A native abort terminates validation and leaves pending_case recorded.",
              "source_py_sha256": source_hashes(),
              "scene_layout": {"background": base.get("appearance", {}).get("background", {}).get("layout", "single_wall"),
                  "initial_position_m": base["simulation"]["initial_position_m"],
                  "initial_rpy_rad": base["simulation"]["initial_rpy_rad"],
                  "target_boards": [{"id": entry["id"], "simulation": entry.get("simulation", {})} for entry in base["targets"]]},
              "scope": "Actual MuJoCo RGB, native decoded AprilTags, static boards only; fixed scenarios plus saved seeded samples. Failures are never rerolled. Assumed dimensions; no real-world guarantee.",
              "random_ranges": {"initial_x_m": [-0.10, 0.10], "initial_y_m": [-0.10, 0.10],
                  "initial_pitch_rad": [0, 0.08], "initial_yaw_rad": [-0.15, 0.15], "distance_scale": [0.9, 1.1],
                  "exposure": [0.75, 1.25], "noise_std_u8": [0, 4], "delay_s": [0, 0.05],
                  "drop_probability": [0, 0.15]}, "cases": []}
    for name, config, expected, limit, parameters in scenarios(base, args.random_cases, args.seed):
        config_path = output/f"{name}.json"
        write_json(config_path, config)
        # Persist the pending case before entering native render/detection code.
        report["pending_case"] = {"case": name, "config": str(config_path), "expected": expected, "parameters": parameters}
        write_json(output/"validation.json", report)
        process_code, run_path = run_multi(parse_args(["--headless", "--model-root", str(args.model_root),
            "--config", str(config_path), "--output", str(output/name), "--max-seconds", str(limit)]))
        summary = json.loads((run_path/"summary.json").read_text(encoding="utf-8"))
        events = [json.loads(line) for line in (run_path/"events.jsonl").read_text(encoding="utf-8").splitlines()]
        checks, metrics = assess(summary, events, process_code, config, expected)
        if config.get("appearance", {}).get("background", {}).get("layout") == "surround":
            provenance = json.loads((run_path/"generated/model_provenance.json").read_text(encoding="utf-8"))
            placement = provenance.get("background_placement", {})
            checks["four_background_walls"] = (placement.get("layout") == "surround"
                and len(placement.get("walls", [])) == 4
                and {wall["name"] for wall in placement.get("walls", [])} == {"front", "left", "back", "right"})
            expected_boards = sum(not entry.get("tag_only", False) for entry in config["targets"] + config.get("distractors", []))
            checks["physical_scoring_board_count"] = provenance.get("scoring_board_count") == expected_boards
            checks["visual_only_distractor_tag_count"] = provenance.get("visual_only_distractor_tag_count") == sum(
                entry.get("tag_only", False) for entry in config.get("distractors", []))
        # A fault traceback is significant even if a caller returned success.
        fault_path = run_path/"fault.log"
        checks["native_fault_log_empty"] = fault_path.exists() and fault_path.stat().st_size == 0
        result = {"case": name, "passed": all(checks.values()), "checks": checks,
                  "status": summary.get("status"), "process_code": process_code,
                  "expected": expected, "parameters": parameters, "metrics": metrics,
                  "simulation_time_s": summary.get("simulation_time_s"),
                  "summary": str(run_path/"summary.json"), "config": str(config_path)}
        report["cases"].append(result)
        report.pop("pending_case", None)
        write_json(output/"validation.json", report)
        failed = [name for name, passed in checks.items() if not passed]
        print(f"MULTI_CASE {name}: {'PASS' if result['passed'] else 'FAIL'}"+
              (f" failed={','.join(failed)}" if failed else ""), flush=True)
    report["complete"] = True
    final_hashes = source_hashes()
    report["tested_sources_unchanged"] = report["source_py_sha256"] == final_hashes
    if not report["tested_sources_unchanged"]:
        report["source_py_sha256_after"] = final_hashes
    report["all_passed"] = all(result["passed"] for result in report["cases"]) and report["tested_sources_unchanged"]
    report["random_cases_completed"] = sum(result["parameters"]["kind"] == "seeded_random" for result in report["cases"])
    write_json(output/"validation.json", report)
    print(f"Multi-target validation: {output/'validation.json'}", flush=True)
    return 0 if report["all_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
