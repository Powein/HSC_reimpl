"""Geometric completion rules, independent of the simulator and rendering."""
from dataclasses import dataclass, field


@dataclass
class PassageMonitor:
    """Require all four wheel centres to cross their obstacle lanes and car tail clearance.

    Bounds are taken from current physical geometry, so pushing an obstacle
    forwards cannot masquerade as passing it. The centre obstacle also enters
    the whole-car clearance test, even though it is between the wheel lanes.
    """
    initial_obstacle_max_x: float
    clearance_m: float
    required_clear_frames: int
    crossings: dict = field(default_factory=dict)
    previous_relative_x: dict = field(default_factory=dict)
    consecutive_clear: int = 0
    contact_seen: bool = False

    def update(self, frame, vehicle_min_x, obstacle_max_x, wheels, obstacles,
               mesh_contacts):
        self.contact_seen |= mesh_contacts > 0
        for wheel in wheels:
            side = 0 if wheel['name'] in ('wheel_0', 'wheel_2') else 2
            obstacle = obstacles[side]
            relative = wheel['center'][0] - obstacle['center'][0]
            previous = self.previous_relative_x.get(wheel['name'], relative)
            in_lane = obstacle['min_y'] <= wheel['center'][1] <= obstacle['max_y']
            above = wheel['center'][2] > obstacle['max_z']
            if previous < 0 <= relative and in_lane and above:
                self.crossings[wheel['name']] = {
                    'frame': frame, 'obstacle': obstacle['name'],
                    'wheel_center': wheel['center'],
                    'obstacle_center': obstacle['center'],
                }
            self.previous_relative_x[wheel['name']] = relative
        boundary = max(self.initial_obstacle_max_x, obstacle_max_x)
        clearance = vehicle_min_x - boundary
        eligible = (clearance >= self.clearance_m and len(self.crossings) == 4
                    and self.contact_seen)
        self.consecutive_clear = self.consecutive_clear + 1 if eligible else 0
        return {
            'vehicle_min_x_m': vehicle_min_x,
            'obstacle_max_x_m': obstacle_max_x,
            'clearance_m': clearance,
            'crossed_wheels': sorted(self.crossings),
            'consecutive_clear_frames': self.consecutive_clear,
            'contact_seen': self.contact_seen,
            'passed': self.consecutive_clear >= self.required_clear_frames,
        }
