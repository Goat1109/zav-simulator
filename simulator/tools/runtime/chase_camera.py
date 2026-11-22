"""Viewport-only framing and wall avoidance; never changes flight geometry."""
import math

import numpy as np


class ChaseCameraRig:
    def __init__(self, query, vehicle_path, radius, half_height):
        self.query = query
        self.vehicle_path = str(vehicle_path).rstrip("/")
        self.radius = max(float(radius), .25)
        self.half_height = max(float(half_height), .08)
        self.height_ratio = None
        self.distance = None
        self.last_time = None

    def _clear_distance(self, center, direction, distance):
        """Check a bundle of two-sided rays, including silhouette clearance."""
        right = np.cross(direction, [0., 0., 1.])
        right /= max(float(np.linalg.norm(right)), 1.e-9)
        offsets = [np.zeros(3), right * self.radius * .7,
                   -right * self.radius * .7,
                   np.array([0., 0., self.half_height * .7]),
                   np.array([0., 0., -self.half_height * .7])]
        available = float(distance)

        def hit(report):
            nonlocal available
            # Ray traversal is unordered; retain the closest non-vehicle hit.
            path = str(report.collision)
            body = str(report.rigid_body)
            if any(p == self.vehicle_path or p.startswith(self.vehicle_path + "/")
                   for p in (path, body)):
                return True
            available = min(available, max(0., float(report.distance) - .12))
            return True

        for offset in offsets:
            self.query.raycast_all(tuple(center + offset), tuple(direction),
                                   float(distance), hit, True)
        return available

    def update(self, center, forward, zoom, aspect, sim_time):
        center = np.asarray(center, dtype=float)
        forward = np.asarray(forward, dtype=float)
        # A wide default lens keeps the entire vehicle in view at close range.
        aspect = max(float(aspect), .3)
        half_fov = math.atan(math.tan(math.radians(76.) / 2.) / max(aspect, 1.))
        fit_distance = self.radius / math.sin(half_fov * .8)
        desired = max(math.hypot(1.5, .48) * float(zoom), fit_distance)
        # Keep camera yaw exactly aligned with the aircraft's current heading.
        # Only height, distance and lens may adapt to walls and roofs.
        candidates = []
        for height in (.32, .08):
            direction = np.array([-forward[0], -forward[1], height])
            direction /= np.linalg.norm(direction)
            distance = self._clear_distance(center, direction, desired)
            continuity = 0. if self.height_ratio is None else -abs(height - self.height_ratio)
            score = min(distance / fit_distance, 1.) * 4. + .15 * continuity
            candidates.append((score, height, distance))
            if height == .32 and distance >= fit_distance:
                break
        _, height, available = max(candidates, key=lambda item: item[0])
        dt = .05 if self.last_time is None else max(0., min(float(sim_time) - self.last_time, .1))
        self.last_time = float(sim_time)
        if self.height_ratio is not None:
            height = self.height_ratio + (height - self.height_ratio) * (1. - math.exp(-dt / .16))
        self.height_ratio = height
        # Do not smooth world-space direction: that introduces a yaw lag in turns.
        direction = np.array([-forward[0], -forward[1], height])
        direction /= np.linalg.norm(direction)
        available = self._clear_distance(center, direction, desired)
        if self.distance is None or available < self.distance:
            self.distance = available  # Pull inward immediately at an occluder.
        else:
            self.distance += (available - self.distance) * (1. - math.exp(-dt / .4))
        distance = max(self.distance, .03)
        eye = center + direction * distance
        # Center on the body when squeezed; use only a small lead in open space.
        lead = min(.12, max(0., distance - fit_distance) * .1)
        target = center + forward * lead
        required_half = math.asin(min(.98, self.radius / max(distance, self.radius + .03))) / .8
        lens_half = min(math.radians(65.), max(half_fov, required_half))
        focal_length = min(18., 18. / aspect) / math.tan(lens_half)
        return eye, target, focal_length
