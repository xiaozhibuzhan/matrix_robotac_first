"""ROS-free command arbitration between navigation, contact and a saved round."""
from collections import OrderedDict
import hashlib
import json
import math
import time
from .treasure_contact import TreasureContactController, cleared_target_grid
from .treasure_mission import TreasureMission, MissionError, identifier, point


FINAL_ATTEMPTS = ('CONFIRMED', 'FAILED', 'CANCELLED', 'SKIPPED')


class TreasureSession:
    def __init__(self, nav, directory, *, map_hash, map_source='', mode='drive',
                 monotonic=time.monotonic, wall_clock=time.time, map_changed=None,
                 event=None):
        self.nav, self.base_grid = nav, nav.grid
        self.clock = monotonic
        self.mode = mode
        self.mission = TreasureMission(directory, map_hash=map_hash,
                                       map_frame=nav.cfg['frame'], map_source=map_source,
                                       mode=mode, monotonic=monotonic, wall_clock=wall_clock)
        self.contact = TreasureContactController(nav)
        self.map_changed = map_changed
        self.event = event or (lambda kind, detail: None)
        self.selection = None
        self.pending_confirmation = False
        self.confirm_stopped_since = None
        self.acks = OrderedDict()
        self.last_reason = 'Initialize pose, then new-round or resume'

    def snapshot(self):
        selection_step = None
        if self.selection is not None:
            selection_step = 'WAIT_CENTER' if self.selection['center'] is None else 'WAIT_APPROACH'
        return dict(mode=self.mode, round=self.mission.snapshot(),
                    selection=self.selection, selection_step=selection_step,
                    contact=self.contact.snapshot(),
                    confirmation_pending=self.pending_confirmation,
                    reason=self.last_reason,
                    navigation_state=self.nav.state)

    def _active(self):
        self.mission.tick()
        if self.mission.snapshot()['state'] != 'ACTIVE':
            raise MissionError('Start or resume an active round first')

    def _stopped(self, now, healthy=True):
        if healthy and self.nav.healthy(now):
            raise MissionError(self.nav.healthy(now))
        if not self.nav.is_stopped() or any(self.nav.command):
            raise MissionError('Wait for measured standstill before this action')

    def _zero(self, reason):
        self.contact.cancel(reason)
        self.nav.cancel(reason)
        self.nav.command = (0., 0.)

    def _install_grid(self, grid):
        self.nav.cancel('Updating confirmed-disappearance map in memory')
        if self.map_changed is not None:
            self.map_changed(grid)
        else:
            if self.nav.planner is not None:
                # Do not silently leave an asynchronous planner bound to an old map.
                raise RuntimeError('A map_changed callback is required for an asynchronous planner')
            self.nav.grid = grid

    def interrupt(self, reason):
        self._zero(reason)
        self.selection = None
        self.pending_confirmation = False
        self.confirm_stopped_since = None
        current = self.mission.current_attempt
        if self.mission.snapshot()['state'] == 'ACTIVE' and current and current['state'] not in FINAL_ATTEMPTS:
            self.mission.cancel(reason)
        self.last_reason = str(reason)

    def reject_click(self, reason, now=None):
        """Keep an unfinished annotation only when no movement is in progress."""
        now = self.clock() if now is None else now
        round_status = self.mission.tick()
        current = self.mission.current_attempt
        can_retry = (
            round_status['state'] == 'ACTIVE'
            and self.selection is not None
            and (current is None or current['state'] in FINAL_ATTEMPTS)
            and not self.pending_confirmation
            and not self.contact.active
            and self.nav.goal is None
            and not self.nav.healthy(now)
            and self.nav.is_stopped()
            and not any(self.nav.command)
        )
        if not can_retry:
            self.interrupt(reason)
            return False
        self._zero(reason)
        if self.selection['center'] is None:
            next_point = 'box center'
        else:
            next_point = 'reachable approach point 0.55 to 1.00 m from the selected center'
        self.last_reason = f'{reason}; selection kept. Publish Point again on the {next_point}'
        return True

    def _begin_approach(self, now):
        attempt = self.mission.current_attempt
        self.mission.begin_approach()
        self.nav.last_tick = now
        if not self.nav.set_goal(attempt['approach'], now):
            self.mission.fail(self.nav.reason)
            raise MissionError(self.nav.reason)
        self.last_reason = 'Navigating to approach point; contact requires a separate command'

    def click(self, xy, now=None):
        now = self.clock() if now is None else now
        self._active()
        if self.selection is None:
            raise MissionError('Use treasure_cli select BOX_ID --radius R before the two RViz clicks')
        self._stopped(now)
        xy = point(xy)
        grid = self.nav.grid
        x, y = grid.cell(xy)
        if not 0 <= x < grid.width or not 0 <= y < grid.height or grid.occupancy[y, x] == -1:
            raise MissionError('Target/approach point must lie within the observed map')
        if self.selection['center'] is None:
            # Validate the isolated annotation now, before ordinary navigation.
            cleared_target_grid(grid, xy, self.selection['radius'])
            self.selection['center'] = xy
            self.last_reason = 'Box center selected; now Publish Point on a reachable approach position'
        else:
            if not grid.is_free((x, y)):
                raise MissionError('Approach point lacks ordinary robot footprint clearance')
            distance = math.dist(xy, self.selection['center'])
            if not .55 <= distance <= 1.0:
                raise MissionError('Choose an approach point 0.55 to 1.00 m from the box center')
            self.mission.select(self.selection['box_id'], self.selection['center'],
                                xy, self.selection['radius'])
            self.selection = None
            self._begin_approach(now)
        self.event('treasure_click', self.snapshot())
        return self.snapshot()

    def handle(self, request, now=None):
        now = self.clock() if now is None else now
        if not isinstance(request, dict):
            raise MissionError('Command must be a JSON object')
        request_id = request.get('request_id')
        if not isinstance(request_id, str) or not 1 <= len(request_id) <= 80:
            raise MissionError('Missing or invalid request_id')
        signature = hashlib.sha256(json.dumps(request, sort_keys=True, allow_nan=False).encode()).hexdigest()
        if request_id in self.acks:
            old_signature, ack = self.acks[request_id]
            if old_signature != signature:
                return dict(request_id=request_id, success=False, message='request_id reused with different content')
            return ack
        try:
            message = self._command(request, now)
            ack = dict(request_id=request_id, success=True, message=message, status=self.snapshot())
        except (ValueError, TypeError) as exc:
            ack = dict(request_id=request_id, success=False, message=str(exc), status=self.snapshot())
        self.acks[request_id] = (signature, ack)
        while len(self.acks) > 256:
            self.acks.popitem(last=False)
        self.event('treasure_command', dict(request=request, response=ack))
        return ack

    def _command(self, request, now):
        action = request.get('action')
        if action == 'status':
            return self.last_reason
        if action in ('cancel', 'skip', 'finish'):
            self._zero('Operator ' + action)
            self.selection = None
            self.pending_confirmation = False
            self.confirm_stopped_since = None
            current = self.mission.current_attempt
            if action == 'finish':
                self.mission.finish()
            elif current and current['state'] != 'CONFIRMED' and self.mission.snapshot()['state'] == 'ACTIVE':
                if action == 'skip':
                    self.mission.skip()
                elif current['state'] not in FINAL_ATTEMPTS:
                    self.mission.cancel()
            self.last_reason = 'Zero requested; verify measured standstill. ' + action
            return self.last_reason
        if action in ('new-round', 'resume'):
            self._stopped(now)
            self._zero('Loading round; no old movement is restored')
            if action == 'new-round':
                self.mission.new_round()
                updated = self.base_grid
            else:
                self.mission.resume(request.get('round_id'))
                updated = self.base_grid
                for attempt in self.mission.snapshot()['attempts']:
                    if attempt['state'] == 'CONFIRMED':
                        updated = cleared_target_grid(updated, attempt['center'], attempt['radius'])
            self._install_grid(updated)
            self.selection = None
            self.pending_confirmation = False
            self.last_reason = 'Round loaded. Select a box; resumed incomplete attempts require retry'
            return self.last_reason
        self._active()
        if self.pending_confirmation:
            raise MissionError('Confirmation is waiting for measured standstill; use status or cancel')
        if action == 'select':
            self._stopped(now)
            current = self.mission.current_attempt
            if current and current['state'] not in FINAL_ATTEMPTS:
                raise MissionError('Cancel or skip the current target first')
            radius = request.get('radius')
            if isinstance(radius, bool) or not isinstance(radius, (int, float)) or not math.isfinite(radius) or not .05 <= radius <= .30:
                raise MissionError('Explicit target radius must be 0.05 to 0.30 m')
            self.selection = dict(box_id=identifier(request.get('box_id')), radius=float(radius), center=None)
            self.last_reason = 'Publish Point: first the box center, then its reachable approach point'
        elif action == 'retry':
            self._stopped(now)
            self.mission.retry()
            self.selection = None
            self._begin_approach(now)
        elif action == 'contact':
            self._stopped(now)
            attempt = self.mission.current_attempt
            if not attempt or attempt['state'] != 'READY_CONTACT':
                raise MissionError('Contact requires READY_CONTACT after approach ARRIVED')
            if not self.contact.start(attempt['center'], attempt['radius'], now):
                raise MissionError(self.contact.reason)
            self.mission.begin_contact()
            self.last_reason = 'Bounded contact active; confirm only after observing disappearance'
        elif action == 'confirm':
            if request.get('disappeared') is not True:
                raise MissionError('Explicit --disappeared confirmation is required')
            if self.mode != 'drive':
                raise MissionError('Preview cannot record real treasure success')
            attempt = self.mission.current_attempt
            if not attempt or attempt['state'] not in ('READY_CONTACT', 'CONTACTING', 'WAIT_CONFIRM'):
                raise MissionError('Confirm only the current reached/contacted box')
            if self.nav.healthy(now):
                raise MissionError(self.nav.healthy(now))
            self._zero('Operator observed disappearance; waiting for standstill')
            self.pending_confirmation = True
            self.confirm_stopped_since = None
            self.last_reason = 'Disappearance received; confirmation completes only after measured standstill'
        else:
            raise MissionError('Unknown treasure action')
        return self.last_reason

    def tick(self, now=None):
        now = self.clock() if now is None else now
        round_status = self.mission.tick()
        if round_status['state'] != 'ACTIVE':
            if self.nav.goal is not None or self.contact.active or self.pending_confirmation:
                self._zero('Round is not active; no movement or confirmation allowed')
            self.selection = None
            self.pending_confirmation = False
            return (0., 0.)
        attempt = self.mission.current_attempt
        if self.pending_confirmation:
            self.nav.command = (0., 0.)
            error = self.nav.healthy(now)
            if error:
                self.interrupt('Confirmation cancelled: ' + error)
            elif not self.nav.is_stopped():
                self.confirm_stopped_since = None
            elif self.confirm_stopped_since is None:
                self.confirm_stopped_since = now
            elif now - self.confirm_stopped_since >= self.nav.cfg['settle_seconds']:
                updated = cleared_target_grid(self.nav.grid, attempt['center'], attempt['radius'])
                if attempt['state'] != 'WAIT_CONFIRM':
                    self.mission.contact_stopped()
                self.mission.confirm(disappeared=True, stopped=True)
                self._install_grid(updated)
                self.pending_confirmation = False
                self.last_reason = 'CONFIRMED: recorded once; select the next box'
                self.event('treasure_confirmed', self.snapshot())
            return (0., 0.)
        if attempt and attempt['state'] == 'APPROACHING':
            command = self.nav.tick(now)
            if self.nav.state == 'ARRIVED':
                self.mission.approach_arrived()
                self.last_reason = 'READY_CONTACT: inspect the selected box, then contact or confirm disappearance'
            elif self.nav.state in ('STOPPED', 'WAIT_INITIAL_POSE'):
                self.mission.fail(self.nav.reason)
                self.last_reason = self.nav.reason
            return command
        if attempt and attempt['state'] == 'CONTACTING':
            command = self.contact.tick(now)
            self.nav.command = command
            if self.contact.state == 'WAIT_CONFIRM':
                self.mission.contact_stopped()
                self.last_reason = 'WAIT_CONFIRM: check official disappearance; no automatic success'
            elif self.contact.state == 'STOPPED':
                self.mission.fail(self.contact.reason)
                self.last_reason = self.contact.reason
                self.event('treasure_contact_stopped', self.snapshot())
            return command
        self.nav.command = (0., 0.)
        return (0., 0.)
