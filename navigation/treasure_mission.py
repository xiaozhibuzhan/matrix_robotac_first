"""Persistent, human-confirmed task-two rounds; never writes official scores."""
from contextlib import contextmanager
import copy
import json
import math
import os
from pathlib import Path
import re
import time
import uuid


class MissionError(ValueError):
    pass


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}', value):
        raise MissionError('Use a 1-64 character ID containing letters, digits, _, - or .')
    return value


def point(value):
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise MissionError('Expected two map coordinates')
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in value):
        raise MissionError('Coordinates must be finite numbers')
    return list(map(float, value))


class TreasureMission:
    def __init__(self, directory, *, map_hash, map_frame='map', map_source='',
                 mode='drive', monotonic=time.monotonic, wall_clock=time.time,
                 duration_seconds=1800., duplicate_distance=.25):
        if mode not in ('drive', 'preview'):
            raise MissionError('Invalid mission mode')
        self.directory = Path(directory)
        self.metadata = dict(map_hash=map_hash, map_frame=map_frame,
                             map_source=map_source, mode=mode)
        self.clock, self.wall = monotonic, wall_clock
        self.duration = float(duration_seconds)
        self.duplicate_distance = float(duplicate_distance)
        if not 0 < self.duration <= 1800 or not .05 <= self.duplicate_distance <= 1:
            raise MissionError('Invalid round limits')
        self.data = None
        self.base_elapsed = 0.
        self.anchor = self.clock()

    @property
    def round_id(self):
        return self.data['round_id'] if self.data else None

    @property
    def current_attempt(self):
        if not self.data or self.data['current_index'] is None:
            return None
        return copy.deepcopy(self.data['attempts'][self.data['current_index']])

    def _elapsed(self):
        if self.data is None:
            return 0.
        if self.data['state'] != 'ACTIVE':
            return self.data['elapsed_at_save']
        delta = self.clock() - self.anchor
        if not math.isfinite(delta) or delta < 0:
            raise MissionError('Monotonic clock moved backwards')
        return min(self.duration, self.base_elapsed + delta)

    def _persist(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        self.data['elapsed_at_save'] = self._elapsed()
        self.data['saved_wall'] = self.wall()
        path = self.directory / (self.round_id + '.json')
        temporary = path.with_suffix('.json.tmp')
        with temporary.open('w', encoding='utf-8', newline='\n') as stream:
            json.dump(self.data, stream, ensure_ascii=False, allow_nan=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)

    @contextmanager
    def _change(self):
        previous = copy.deepcopy(self.data)
        try:
            yield
            self._persist()
        except Exception:
            self.data = previous
            raise

    def _active(self):
        self.tick()
        if not self.data or self.data['state'] != 'ACTIVE':
            raise MissionError('No active round; use new-round or resume first')

    def _attempt(self, allowed):
        self._active()
        current = self.current_attempt
        if current is None or current['state'] not in allowed:
            raise MissionError('Action is not allowed for the current attempt state')
        return self.data['attempts'][self.data['current_index']]

    def new_round(self, round_id=None):
        if self.data and self.data['state'] == 'ACTIVE':
            raise MissionError('Finish the current round before starting a new simulation round')
        round_id = identifier(round_id or ('round_' + uuid.uuid4().hex[:16]))
        if (self.directory / (round_id + '.json')).exists():
            raise MissionError('Round ID already exists; use resume without resetting simulation')
        self.anchor, self.base_elapsed = self.clock(), 0.
        with self._change():
            self.data = dict(schema_version=1, round_id=round_id, **self.metadata,
                             duration_seconds=self.duration, state='ACTIVE',
                             started_wall=self.wall(), saved_wall=self.wall(),
                             elapsed_at_save=0., attempts=[], current_index=None)
        return self.snapshot()

    def resume(self, round_id):
        if self.data and self.data['state'] == 'ACTIVE':
            raise MissionError('An active round is already loaded')
        path = self.directory / (identifier(round_id) + '.json')
        try:
            candidate = json.loads(path.read_text(encoding='utf-8'))
            if candidate['schema_version'] != 1 or candidate['round_id'] != round_id:
                raise MissionError('Invalid round schema or identity')
            if any(candidate[key] != value for key, value in self.metadata.items()):
                raise MissionError('Round map/frame/mode/source differs; do not resume another scene')
            if candidate['duration_seconds'] != self.duration or candidate['state'] not in ('ACTIVE', 'FINISHED', 'EXPIRED'):
                raise MissionError('Invalid round duration/state')
            elapsed, saved_wall = candidate['elapsed_at_save'], candidate['saved_wall']
            if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                       for v in (elapsed, saved_wall)) or not 0 <= elapsed <= self.duration:
                raise MissionError('Invalid persisted clock')
            gap = self.wall() - saved_wall
            if not math.isfinite(gap) or gap < 0:
                raise MissionError('Wall clock moved backwards; refuse to grant extra round time')
            attempts = candidate['attempts']
            if not isinstance(attempts, list) or len(attempts) > 10000:
                raise MissionError('Invalid attempt records')
            confirmed = []
            for attempt in attempts:
                identifier(attempt['box_id'])
                point(attempt['center']); point(attempt['approach'])
                radius = attempt['radius']
                if isinstance(radius, bool) or not isinstance(radius, (int, float)) or not .05 <= radius <= .30:
                    raise MissionError('Invalid target radius in record')
                if attempt['state'] not in ('SELECTED', 'APPROACHING', 'READY_CONTACT', 'CONTACTING',
                                             'WAIT_CONFIRM', 'CONFIRMED', 'FAILED', 'CANCELLED', 'SKIPPED'):
                    raise MissionError('Invalid attempt state')
                if attempt['state'] == 'CONFIRMED':
                    if self.metadata['mode'] != 'drive' or any(self._same_target(attempt, old) for old in confirmed):
                        raise MissionError('Invalid or duplicate confirmation records')
                    confirmed.append(attempt)
            index = candidate['current_index']
            if index is not None and (type(index) is not int or not 0 <= index < len(attempts)):
                raise MissionError('Invalid current attempt index')
        except (KeyError, TypeError, json.JSONDecodeError) as exc:
            raise MissionError('Malformed saved round') from exc
        self.base_elapsed = min(self.duration, elapsed + gap) if candidate['state'] == 'ACTIVE' else elapsed
        self.anchor = self.clock()
        with self._change():
            self.data = candidate
            if index is not None and attempts[index]['state'] not in ('CONFIRMED', 'FAILED', 'CANCELLED', 'SKIPPED'):
                attempts[index].update(state='CANCELLED', reason='process_restart; explicitly retry after localization')
        self.tick()
        return self.snapshot()

    def _same_target(self, left, right):
        return (left['box_id'] == right['box_id']
                or math.dist(left['center'], right['center']) <= max(self.duplicate_distance, left['radius'] + right['radius']))

    def select(self, box_id, center, approach, radius):
        self._active()
        current = self.current_attempt
        if current and current['state'] not in ('CONFIRMED', 'FAILED', 'CANCELLED', 'SKIPPED'):
            raise MissionError('Cancel or skip the current target before selecting another')
        if isinstance(radius, bool) or not isinstance(radius, (int, float)) or not math.isfinite(radius) or not .05 <= radius <= .30:
            raise MissionError('Target radius must be 0.05 to 0.30 m')
        target = dict(box_id=identifier(box_id), center=point(center), approach=point(approach),
                      radius=float(radius), state='SELECTED', reason='operator_selected',
                      attempt_id=uuid.uuid4().hex, elapsed_selected=self._elapsed())
        if any(old['state'] == 'CONFIRMED' and self._same_target(target, old) for old in self.data['attempts']):
            raise MissionError('This box ID or overlapping target region was already confirmed')
        with self._change():
            self.data['attempts'].append(target)
            self.data['current_index'] = len(self.data['attempts']) - 1
        return self.snapshot()

    def _transition(self, allowed, state, reason):
        self._attempt(allowed)
        with self._change():
            self.data['attempts'][self.data['current_index']].update(
                state=state, reason=str(reason), elapsed_updated=self._elapsed())
        return self.snapshot()

    def begin_approach(self):
        return self._transition(('SELECTED',), 'APPROACHING', 'ordinary_navigation')

    def approach_arrived(self):
        return self._transition(('APPROACHING',), 'READY_CONTACT', 'arrived; inspect box before contact')

    def begin_contact(self):
        return self._transition(('READY_CONTACT',), 'CONTACTING', 'operator_authorized_contact')

    def contact_stopped(self):
        return self._transition(('READY_CONTACT', 'CONTACTING'), 'WAIT_CONFIRM', 'measured_standstill')

    def confirm(self, *, disappeared=False, stopped=False):
        self._attempt(('WAIT_CONFIRM',))
        if disappeared is not True or stopped is not True:
            raise MissionError('Confirmation requires observed disappearance and measured standstill')
        if self.metadata['mode'] != 'drive':
            raise MissionError('Preview cannot record a real treasure success')
        return self._transition(('WAIT_CONFIRM',), 'CONFIRMED', 'human_observed_official_disappearance')

    def fail(self, reason):
        return self._transition(('SELECTED', 'APPROACHING', 'READY_CONTACT', 'CONTACTING', 'WAIT_CONFIRM'), 'FAILED', reason)

    def cancel(self, reason='operator_cancel'):
        return self._transition(('SELECTED', 'APPROACHING', 'READY_CONTACT', 'CONTACTING', 'WAIT_CONFIRM'), 'CANCELLED', reason)

    def retry(self):
        old = self._attempt(('FAILED', 'CANCELLED', 'SKIPPED'))
        return self.select(old['box_id'], old['center'], old['approach'], old['radius'])

    def skip(self, reason='operator_skip'):
        return self._transition(('SELECTED', 'APPROACHING', 'READY_CONTACT', 'CONTACTING', 'WAIT_CONFIRM', 'FAILED', 'CANCELLED'), 'SKIPPED', reason)

    def finish(self):
        self._active()
        elapsed = self._elapsed()
        with self._change():
            current = self.current_attempt
            if current and current['state'] not in ('CONFIRMED', 'FAILED', 'CANCELLED', 'SKIPPED'):
                self.data['attempts'][self.data['current_index']].update(state='CANCELLED', reason='round_finished')
            self.data.update(state='FINISHED', elapsed_at_save=elapsed)
        return self.snapshot()

    def tick(self):
        if self.data and self.data['state'] == 'ACTIVE' and self._elapsed() >= self.duration:
            with self._change():
                current = self.current_attempt
                if current and current['state'] not in ('CONFIRMED', 'FAILED', 'CANCELLED', 'SKIPPED'):
                    self.data['attempts'][self.data['current_index']].update(state='CANCELLED', reason='round_expired')
                self.data.update(state='EXPIRED', elapsed_at_save=self.duration)
        return self.snapshot()

    def snapshot(self):
        if self.data is None:
            return dict(state='NO_ROUND', round_id=None, current_attempt=None, confirmed_count=0)
        result = copy.deepcopy(self.data)
        elapsed = self._elapsed()
        remaining = max(0., self.duration - elapsed)
        minutes = math.floor(remaining / 60.)
        count = sum(a['state'] == 'CONFIRMED' for a in result['attempts'])
        time_score = round(min(.8 * minutes, 10), 1)
        result.update(current_attempt=self.current_attempt, elapsed_seconds=elapsed,
                      remaining_seconds=remaining, remaining_full_minutes=minutes,
                      confirmed_count=count, ammo_estimate=count,
                      score_estimate=dict(box=min(4 * count, 20), time=time_score,
                                          total=round(min(4 * count, 20) + time_score, 1)),
                      scoring_note='Local estimate only; official timing/eligibility and referee results prevail')
        return result
