"""AprilTag3 pixel detection with OpenCV metric pose estimation.

The pinned pupil-apriltags wrapper has an unsafe native teardown order and
does not free the matrices returned by its optional native pose estimator.
Keep the genuine AprilTag detector for decoded IDs/corners, own its lifetime
locally, and estimate pose from those corners without entering that code path.
No bundled library or official source is modified.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import cv2
import numpy as np

from .interfaces import Frame


@dataclass(frozen=True)
class TagDetection:
    tag_id: int
    R_camera_from_tag: np.ndarray
    t_camera_from_tag: np.ndarray
    decision_margin: float
    hamming: int
    corners_px: np.ndarray
    reprojection_error_px: float
    timestamp: float


def reprojection_error(
    rotation: np.ndarray, translation: np.ndarray, corners_px: np.ndarray,
    camera_params: tuple[float, float, float, float], tag_size_m: float,
) -> float:
    """RMS error in pixels; pupil-apriltags corner order is LB,RB,RT,LT."""
    half = tag_size_m / 2.0
    object_corners = np.array([[-half, half, 0], [half, half, 0],
                               [half, -half, 0], [-half, -half, 0]])
    camera_points = (rotation @ object_corners.T).T + translation.reshape(1, 3)
    if np.any(camera_points[:, 2] <= 0):
        return float("inf")
    fx, fy, cx, cy = camera_params
    projected = camera_points[:, :2] / camera_points[:, 2:3]
    projected = projected * [fx, fy] + [cx, cy]
    return float(np.sqrt(np.mean(np.sum((projected - corners_px) ** 2, axis=1))))


def estimate_tag_pose_from_corners(
    corners_px: np.ndarray,
    camera_params: tuple[float, float, float, float],
    tag_size_m: float,
) -> tuple[np.ndarray, np.ndarray, float] | None:
    """Choose a positive-depth square pose by measured reprojection error.

    AprilTag returns LB, RB, RT, LT for an upright marker. Matching those
    corners with +X right, +Y down preserves the existing tag frame and the
    configured positive-Y offset from marker to bullseye. IPPE supplies both
    planar-pose candidates; refine each and select by the observed pixels.
    """
    corners = np.asarray(corners_px, dtype=np.float64).reshape(4, 2)
    if not np.all(np.isfinite(corners)) or abs(cv2.contourArea(corners.astype(np.float32))) < 1e-6:
        return None
    half = tag_size_m / 2.0
    object_corners = np.array([[-half, half, 0], [half, half, 0],
                               [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
    fx, fy, cx, cy = camera_params
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    seeds = []
    # SQPnP also covers IPPE's rotation-vector singularity for a square
    # rotated exactly 180 degrees in the image (OpenCV 4.11).
    for method in (cv2.SOLVEPNP_IPPE_SQUARE, cv2.SOLVEPNP_SQPNP):
        try:
            solved, rotations, translations, _ = cv2.solvePnPGeneric(
                object_corners, corners, camera_matrix, None, flags=method,
            )
        except cv2.error:
            continue
        if solved:
            seeds.extend(zip(rotations, translations))
    candidates = []
    for rotation_vector, translation_vector in seeds:
        if not (np.all(np.isfinite(rotation_vector)) and np.all(np.isfinite(translation_vector))):
            continue
        # A refinement may improve either of the two valid planar solutions.
        # Retain the original as well so refinement cannot worsen the result.
        poses = [(rotation_vector, translation_vector)]
        try:
            poses.append(cv2.solvePnPRefineLM(
                object_corners, corners, camera_matrix, None,
                rotation_vector.copy(), translation_vector.copy(),
            ))
        except cv2.error:
            pass
        for rvec, tvec in poses:
            if not (np.all(np.isfinite(rvec)) and np.all(np.isfinite(tvec))):
                continue
            rotation = cv2.Rodrigues(rvec)[0]
            translation = tvec.reshape(3)
            error = reprojection_error(rotation, translation, corners, camera_params, tag_size_m)
            if np.isfinite(error):
                candidates.append((rotation, translation, error))
    return min(candidates, key=lambda candidate: candidate[2]) if candidates else None


def _release_native_detector(detector: Any) -> None:
    """Release the detector before families, and make repeat cleanup harmless.

    pupil-apriltags 1.0.4.post11 does the reverse in Detector.__del__.
    apriltag_detector_destroy calls apriltag_detector_clear_families, which
    dereferences each family to free its decoder. Freeing a family first is
    therefore a use-after-free (glibc can report it only on a later malloc).
    Nulling the wrapper pointer also disables that vendor finalizer.
    """
    pointer = getattr(detector, "tag_detector_ptr", None)
    if pointer is None:
        return
    detector.tag_detector_ptr = None
    library = detector.libc
    library.apriltag_detector_destroy.restype = None
    library.apriltag_detector_destroy(pointer)
    families = getattr(detector, "tag_families", {})
    for family, family_pointer in families.items():
        destroy = getattr(library, f"{family}_destroy")
        destroy.restype = None
        destroy(family_pointer)
    families.clear()


def create_managed_detector(tag):
    """Share the native detector and its safe lifetime with real-camera input."""
    from pupil_apriltags import Detector

    class ManagedDetector(Detector):
        # Override instead of editing the installed third-party binding.
        # This also protects constructor failures and interpreter cleanup.
        def close(self):
            _release_native_detector(self)

        def __del__(self):
            try:
                self.close()
            except Exception:
                # Native finalizers cannot report exceptions reliably at
                # interpreter shutdown. Explicit close reports failures.
                pass

    return ManagedDetector(
        families=tag.get("family", "tag36h11"),
        nthreads=int(tag.get("detector_threads", 1)),
        quad_decimate=float(tag.get("quad_decimate", 1.0)),
        quad_sigma=float(tag.get("quad_sigma", 0.0)),
        refine_edges=1, decode_sharpening=0.25,
    )


class AprilTagPerception:
    """The detector takes pixels only; simulator ground truth is not accepted."""

    def __init__(self, config: Mapping[str, Any]):

        camera, tag = config["camera"], config["tag"]
        self.camera_params = tuple(float(camera[key]) for key in ("fx", "fy", "cx", "cy"))
        if min(self.camera_params[:2]) <= 0:
            raise ValueError("camera focal lengths must be positive")
        self.tag_size_m = float(tag.get("size_m", 0.16))
        if self.tag_size_m <= 0:
            raise ValueError("tag size must be positive")
        self.max_reprojection_error_px = float(tag.get("max_reprojection_error_px", 2.0))
        if not np.isfinite(self.max_reprojection_error_px) or self.max_reprojection_error_px <= 0:
            raise ValueError("maximum reprojection error must be finite and positive")
        self.detector = create_managed_detector(tag)

    def detect(self, frame: Frame) -> list[TagDetection]:
        if self.detector.tag_detector_ptr is None:
            raise RuntimeError("AprilTag detector is closed")
        rgb = np.asarray(frame.rgb)
        if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
            raise ValueError("Frame.rgb must be an HxWx3 uint8 RGB image")
        gray = np.ascontiguousarray(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
        results = self.detector.detect(gray, estimate_tag_pose=False)
        detections = []
        for result in results:
            corners = np.asarray(result.corners, dtype=float).reshape(4, 2)
            pose = estimate_tag_pose_from_corners(corners, self.camera_params, self.tag_size_m)
            if pose is None or pose[2] > self.max_reprojection_error_px:
                continue
            rotation, translation, error = pose
            detections.append(TagDetection(
                int(result.tag_id), rotation, translation, float(result.decision_margin),
                int(result.hamming), corners,
                error,
                float(frame.timestamp),
            ))
        return detections

    def close(self) -> None:
        """Deterministically release native resources; safe to call twice."""
        detector = getattr(self, "detector", None)
        if detector is not None:
            detector.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
