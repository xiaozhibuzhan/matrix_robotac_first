"""Decode all visible IDs and use each declared target's own metric tag size."""

from __future__ import annotations

import cv2
import numpy as np

from .perception import AprilTagPerception, TagDetection, estimate_tag_pose_from_corners


class MultiTagPerception(AprilTagPerception):
    """One managed native detector, calibrated by ID without scene positions."""

    def __init__(self, public_config):
        super().__init__(public_config)
        self.target_tags = {}
        try:
            for target in public_config["targets"]:
                target_id, tag = int(target["id"]), dict(target["tag"])
                size = float(tag["size_m"])
                if target_id in self.target_tags or not np.isfinite(size) or size <= 0:
                    raise ValueError("target IDs must be unique and metric tag sizes positive")
                if tag.get("family", public_config["tag"]["family"]) != public_config["tag"]["family"]:
                    raise ValueError("multi-target detector requires one common tag family")
                self.target_tags[target_id] = tag
        except BaseException:
            self.close()
            raise

    def detect(self, frame):
        if self.detector.tag_detector_ptr is None:
            raise RuntimeError("AprilTag detector is closed")
        rgb = np.asarray(frame.rgb)
        if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
            raise ValueError("Frame.rgb must be an HxWx3 uint8 RGB image")
        results = self.detector.detect(np.ascontiguousarray(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)),
                                       estimate_tag_pose=False)
        detections = []
        for result in results:
            target_id = int(result.tag_id)
            tag = self.target_tags.get(target_id, {})
            corners = np.asarray(result.corners, dtype=float).reshape(4, 2)
            pose = estimate_tag_pose_from_corners(corners, self.camera_params,
                                                  float(tag.get("size_m", self.tag_size_m)))
            limit = float(tag.get("max_reprojection_error_px", self.max_reprojection_error_px))
            if pose is None or pose[2] > limit:
                continue
            rotation, translation, error = pose
            # Unknown IDs remain visible in diagnostics, but have no public
            # mission calibration and can never become shooting objectives.
            detections.append(TagDetection(target_id, rotation, translation,
                float(result.decision_margin), int(result.hamming), corners, error, float(frame.timestamp)))
        return detections
