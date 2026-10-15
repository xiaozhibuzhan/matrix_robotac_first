"""Bounded operator-authorized final approach; never declares treasure success.

Only the explicitly selected compact target circle may be exempted. Unknown
space, borders, unrelated obstacles and the original footprint stay protected.
The caller retains command arbitration and the existing SDK watchdog.
"""
import copy
import math
import numpy as np
from scipy.ndimage import label
from scipy.spatial import cKDTree
from .geometry import wrap
from .grid import GridMap
from .sdk_limits import MIN_FORWARD_SPEED, MIN_YAW_RATE, validate_sdk_velocity


DEFAULT_SETTINGS = {
    'speed': .05, 'max_yaw_rate': .30, 'stop_distance': .25,
    'max_travel': .80, 'timeout': 30., 'progress_timeout': 5.,
    'max_step': .03, 'max_lateral_error': .10, 'isolation_gap': .10,
}


def cleared_target_grid(grid, center, radius, isolation_gap=.10):
    """Return an isolated-target-only copy, never modifying a map file/array."""
    if len(center) != 2 or not all(math.isfinite(v) for v in center):
        raise ValueError('Target center must be finite')
    if isinstance(radius, bool) or not math.isfinite(radius) or not .05 <= radius <= .30:
        raise ValueError('Target radius must be 0.05 to 0.30 m')
    cx, cy = grid.continuous_cell(center)
    extent = radius / grid.resolution
    if cx - extent < 0 or cy - extent < 0 or cx + extent >= grid.width or cy + extent >= grid.height:
        raise ValueError('Target region extends outside the known map')
    yy, xx = np.indices(grid.occupancy.shape)
    distances = np.hypot(xx + .5 - cx, yy + .5 - cy) * grid.resolution
    half_diagonal = grid.resolution / math.sqrt(2.)
    intersects = distances <= radius + half_diagonal
    interior = distances + half_diagonal <= radius
    if np.any((grid.occupancy == -1) & intersects):
        raise ValueError('Target region contains unknown cells; improve map coverage first')
    occupied = grid.occupancy == 100
    if np.any(occupied & intersects & ~interior):
        raise ValueError('Target occupancy crosses the selected region boundary; cannot isolate treasure')
    components, _ = label(occupied, structure=np.ones((3, 3), dtype=np.int8))
    ids = np.unique(components[occupied & interior])
    selected = np.isin(components, ids) if len(ids) else np.zeros_like(occupied)
    if np.any(selected & ~interior):
        raise ValueError('Selected treasure is connected to an obstacle outside its region')
    annulus = (distances > radius) & (distances <= radius + isolation_gap + half_diagonal)
    if np.any((grid.occupancy != 0) & annulus):
        raise ValueError('Target lacks a known-free isolation ring; cannot distinguish nearby obstacles')
    occupancy = grid.occupancy.copy()
    occupancy[selected] = 0
    return GridMap(occupancy, grid.resolution, grid.origin, grid.radius, grid.margin)


class TreasureContactController:
    """start(target_xy, radius, now) -> bool; tick(now) -> (v, w).

    Requires ordinary navigation ARRIVED and measured standstill. WAIT_CONFIRM
    means the bounded approach stopped, not that the official treasure fired.
    Operator selection is not automatic object recognition.
    """

    def __init__(self, nav, settings=None):
        self.nav = nav
        supplied = {} if settings is None else dict(settings)
        if set(supplied) - set(DEFAULT_SETTINGS):
            raise ValueError('Unknown treasure contact setting')
        self.settings = {**DEFAULT_SETTINGS, **supplied}
        for key, value in self.settings.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'Invalid treasure contact {key}')
        limits = {'max_yaw_rate': .30, 'max_travel': .80, 'timeout': 30.,
                  'progress_timeout': 5., 'max_step': .03, 'max_lateral_error': .10}
        if not MIN_FORWARD_SPEED <= self.settings['speed'] <= .08:
            raise ValueError('Contact speed must be 0.05 to 0.08 m/s')
        if any(self.settings[k] > v for k, v in limits.items()):
            raise ValueError('Contact setting exceeds fixed safety bounds')
        if self.settings['max_yaw_rate'] < MIN_YAW_RATE:
            raise ValueError('Contact yaw rate is below the SDK minimum')
        if not .15 <= self.settings['stop_distance'] <= .40:
            raise ValueError('Contact stop_distance must be 0.15 to 0.40 m')
        if self.settings['isolation_gap'] < .10:
            raise ValueError('Contact isolation_gap must be at least 0.10 m')
        if self.settings['speed'] > nav.cfg['max_speed'] or self.settings['max_yaw_rate'] > nav.cfg['max_yaw_rate']:
            raise ValueError('Contact settings exceed navigation/SDK command limits')
        self.state = 'IDLE'
        self.reason = 'Awaiting explicit operator contact request'
        self.command = (0., 0.)
        self.target = self.radius = self.endpoint = self.start_pose = None
        self.started = self.last_tick = self.settle_since = None
        self.travelled = 0.
        self._last_pose = self._grid = self._alignment = None
        self._best_distance = self._best_angle = math.inf
        self._progress_at = None
        self._static_target_cells = 0
        self._initial_live_target_points = 0

    @property
    def active(self):
        return self.state in ('ALIGNING', 'APPROACHING', 'SETTLING')

    @property
    def contact_grid(self):
        """Temporary map; only promote after independently confirmed disappearance."""
        return self._grid

    def cancel(self, reason='Contact cancelled by operator'):
        self.command = (0., 0.)
        self.state = 'STOPPED'
        self.reason = str(reason)
        self.settle_since = None

    def _reject(self, reason):
        self.cancel(reason)
        return False

    def _target_grid(self):
        """Copy isolated target occupancy without altering the source map."""
        grid = self.nav.grid
        result = cleared_target_grid(grid, self.target, self.radius, self.settings['isolation_gap'])
        self._static_target_cells = int(np.count_nonzero(grid.occupancy != result.occupancy))
        return result

    def _filtered_obstacles(self):
        """Filter the annotated map circle, retaining raw odometry coordinates."""
        obstacles = np.asarray(self.nav.obstacles, dtype=float)
        if obstacles.ndim != 2 or obstacles.shape[1] != 2 or not np.isfinite(obstacles).all():
            raise ValueError('Invalid live obstacle data')
        tx, ty, angle = self.nav.alignment
        c, s = math.cos(angle), math.sin(angle)
        mapped = obstacles @ np.array([[c, s], [-s, c]]) + np.array([tx, ty])
        inside = np.linalg.norm(mapped - np.asarray(self.target), axis=1) <= self.radius
        if np.any(inside) and np.any(~inside):
            nearest = cKDTree(mapped[~inside]).query(mapped[inside], k=1)[0]
            if np.any(nearest <= self.settings['isolation_gap']):
                raise ValueError('Live target points join an obstacle outside the selected region')
        return obstacles[~inside], int(inside.sum())

    def _motion_clear(self, v, w):
        filtered, _ = self._filtered_obstacles()
        # Original swept-arc, braking-distance, live/static and footprint guard.
        guard = copy.copy(self.nav)
        guard.grid = self._grid
        guard.obstacles = filtered
        guard.goal = self.endpoint
        guard.path = []
        return guard.motion_clear(v, w)

    def start(self, target_xy, radius, now):
        if self.active:
            return self._reject('An active contact cannot be restarted; stopped first')
        self.command = (0., 0.)
        try:
            if len(target_xy) != 2 or not all(math.isfinite(v) for v in target_xy):
                raise ValueError('Contact target must have two finite coordinates')
            if isinstance(radius, bool) or not math.isfinite(radius) or not .05 <= radius <= .30:
                raise ValueError('Explicit target radius must be 0.05 to 0.30 m')
            if not math.isfinite(now):
                raise ValueError('Invalid contact clock')
            if self.nav.state != 'ARRIVED' or self.nav.goal is not None:
                raise ValueError('First navigate to the approach point and wait for ARRIVED')
            error = self.nav.healthy(now)
            if error:
                raise ValueError(error)
            if not self.nav.is_stopped() or any(self.nav.command):
                raise ValueError('Contact requires measured standstill and zero navigation command')
            self.target = tuple(map(float, target_xy))
            self.radius = float(radius)
            self.start_pose = tuple(self.nav.pose)
            distance = math.dist(self.start_pose[:2], self.target)
            travel = distance - self.settings['stop_distance']
            if travel <= 0:
                raise ValueError('Already within contact stop distance; visually check treasure instead of advancing')
            if travel > self.settings['max_travel'] - .02:
                raise ValueError('Approach point is too far from treasure for bounded contact')
            direction = (np.array(self.target) - np.array(self.start_pose[:2])) / distance
            self.endpoint = tuple(np.array(self.start_pose[:2]) + direction * travel)
            self._alignment = tuple(self.nav.alignment)
            self._source_grid = self.nav.grid
            self._grid = self._target_grid()
            _, live_count = self._filtered_obstacles()
            # An overlap-only UE trigger may produce no blocking LiDAR return.
            # The operator identifies the box; absence of points never proves
            # disappearance and does not waive any surrounding obstacle guard.
            self._initial_live_target_points = live_count
            if not self._grid.segment_free(self.start_pose[:2], self.endpoint):
                raise ValueError('Contact corridor crosses unknown space or an unrelated static obstacle')
            clear, reason = self._motion_clear(0., 0.)
            if not clear:
                raise ValueError(reason)
        except (ValueError, TypeError, OverflowError) as exc:
            return self._reject(str(exc))
        self.state = 'ALIGNING'
        self.reason = 'Explicit isolated target authorized; aligning while stationary'
        self.started = self.last_tick = self._progress_at = float(now)
        self._last_pose = self.start_pose
        self._best_distance = distance
        self._best_angle = math.inf
        self.travelled = 0.
        self.settle_since = None
        return True

    def tick(self, now):
        self.command = (0., 0.)
        if not self.active:
            return self.command
        try:
            if not math.isfinite(now):
                raise ValueError('Invalid contact clock')
            dt = now - self.last_tick
            if dt < 0 or dt > self.nav.cfg['command_timeout']:
                raise ValueError('Contact control loop stalled or clock moved backwards')
            self.last_tick = now
            error = self.nav.healthy(now)
            if error:
                raise ValueError(error)
            if tuple(self.nav.alignment) != self._alignment or self.nav.grid is not self._source_grid:
                raise ValueError('Localization or map changed during contact')
            if self.nav.state != 'ARRIVED' or self.nav.goal is not None:
                raise ValueError('Navigation state/goal changed during contact')
            pose = tuple(self.nav.pose)
            if not all(math.isfinite(v) for v in (*pose, self.nav.linear_speed, self.nav.angular_speed)):
                raise ValueError('Contact pose or measured speed is invalid')
            step = math.dist(pose[:2], self._last_pose[:2])
            if step > max(self.settings['max_step'], .16 * dt):
                raise ValueError('Contact odometry step exceeds bounded motion')
            if abs(wrap(pose[2] - self._last_pose[2])) > max(.15, .60 * dt):
                raise ValueError('Contact heading jumped unexpectedly')
            self.travelled += step
            self._last_pose = pose
            if self.travelled > self.settings['max_travel']:
                raise ValueError('Contact total travel limit exceeded')
            if now - self.started > self.settings['timeout']:
                raise ValueError('Contact time limit exceeded')
            if self.nav.linear_speed > .16 or self.nav.angular_speed > .60:
                raise ValueError('Measured motion exceeds contact safety limits')
            start = np.asarray(self.start_pose[:2])
            delta = np.asarray(self.target) - start
            offset = np.asarray(pose[:2]) - start
            along = float(np.dot(offset, delta) / np.linalg.norm(delta))
            lateral = abs(float(delta[0] * offset[1] - delta[1] * offset[0])) / np.linalg.norm(delta)
            if lateral > self.settings['max_lateral_error'] or along < -.05:
                raise ValueError('Contact departed the authorized straight approach corridor')
            if not self._grid.is_free(self._grid.cell(pose[:2])):
                raise ValueError('Contact pose left footprint-clear space')
            distance = math.dist(pose[:2], self.target)
            if distance < self._best_distance - .01:
                self._best_distance = distance
                self._progress_at = now
            if distance <= self.settings['stop_distance'] or along >= math.dist(start, self.endpoint):
                self.state = 'SETTLING'
            if self.state == 'SETTLING':
                self.reason = 'Zero command; verifying standstill before visual confirmation'
                clear, reason = self._motion_clear(0., 0.)
                if not clear:
                    raise ValueError(reason)
                if not self.nav.is_stopped():
                    self.settle_since = None
                elif self.settle_since is None:
                    self.settle_since = now
                elif now - self.settle_since >= self.nav.cfg['settle_seconds']:
                    self.state = 'WAIT_CONFIRM'
                    self.reason = 'Stopped; operator must confirm official treasure disappearance'
                return self.command
            heading = math.atan2(self.target[1] - pose[1], self.target[0] - pose[0])
            error = wrap(heading - pose[2])
            if self.state == 'ALIGNING' and abs(error) < self._best_angle - .03:
                self._best_angle = abs(error)
                self._progress_at = now
            if now - self._progress_at > self.settings['progress_timeout']:
                raise ValueError('Contact has no measured progress; stopped')
            if self.state == 'ALIGNING':
                v = 0.
                w = float(np.clip(1.5 * error, -self.settings['max_yaw_rate'], self.settings['max_yaw_rate']))
                if abs(error) < .03:
                    w = 0.
                    if self.nav.is_stopped():
                        self.state = 'APPROACHING'
                        self._progress_at = now
            else:
                if abs(error) > .20:
                    raise ValueError('Contact heading deviated; stop and select a new approach')
                v = self.settings['speed']
                w = float(np.clip(1.5 * error, -.15, .15))
            if abs(w) < MIN_YAW_RATE:
                w = 0.
            if v * dt > self.settings['max_step']:
                raise ValueError('Contact command exceeds maximum control step')
            validate_sdk_velocity(v, w)
            clear, reason = self._motion_clear(v, w)
            if not clear:
                raise ValueError(reason)
            self.reason = 'Aligning isolated target' if self.state == 'ALIGNING' else 'Bounded low-speed treasure contact'
            self.command = (float(v), float(w))
        except (ValueError, TypeError, OverflowError) as exc:
            self.cancel(str(exc))
        return self.command

    def snapshot(self):
        return {'state': self.state, 'reason': self.reason, 'command': self.command,
                'target': self.target, 'radius': self.radius, 'endpoint': self.endpoint,
                'travelled_m': self.travelled, 'started': self.started,
                'static_target_cells': self._static_target_cells,
                'initial_live_target_points': self._initial_live_target_points,
                'identification': 'operator_annotation; no automatic treasure recognition',
                'settings': dict(self.settings)}

