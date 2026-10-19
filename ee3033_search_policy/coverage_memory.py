"""Persistent searched-area and candidate-visit memory."""

from pathlib import Path

import numpy as np


class CoverageMemory:
    def __init__(self, observable_mask, candidate_count):
        observable = np.asarray(observable_mask, dtype=bool)
        if observable.ndim != 2 or not np.any(observable):
            raise ValueError("observable_mask must be a non-empty 2-D mask")
        if not isinstance(candidate_count, (int, np.integer)) or candidate_count < 1:
            raise ValueError("candidate_count must be positive")
        self.observable_mask = observable.copy()
        self.searched_mask = np.zeros_like(observable)
        self.visit_counts = np.zeros(int(candidate_count), dtype=np.int32)
        self.paused = False

    @property
    def unseen_mask(self):
        return self.observable_mask & ~self.searched_mask

    @property
    def coverage_fraction(self):
        total = int(np.count_nonzero(self.observable_mask))
        return float(np.count_nonzero(self.searched_mask & self.observable_mask) / total)

    def gains(self, visibility_masks):
        visibility = np.asarray(visibility_masks, dtype=bool)
        expected = (self.visit_counts.size,) + self.observable_mask.shape
        if visibility.shape != expected:
            raise ValueError(f"visibility_masks must have shape {expected}")
        return np.count_nonzero(
            visibility & self.unseen_mask[None, :, :], axis=(1, 2),
        ).astype(np.int32)

    def mark_observation_complete(self, candidate_index, visibility_mask):
        """Update memory only after arrival, final heading, and observation."""
        if self.paused:
            return False
        index = int(candidate_index)
        if not 0 <= index < self.visit_counts.size:
            raise IndexError("candidate_index is outside the candidate array")
        visible = np.asarray(visibility_mask, dtype=bool)
        if visible.shape != self.observable_mask.shape:
            raise ValueError("visibility_mask has the wrong shape")
        self.searched_mask |= visible & self.observable_mask
        self.visit_counts[index] += 1
        return True

    def reset(self):
        self.searched_mask.fill(False)
        self.visit_counts.fill(0)
        self.paused = False

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False

    def save(self, path):
        destination = Path(path).expanduser()
        destination.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            destination,
            observable_mask=self.observable_mask.astype(np.uint8),
            searched_mask=self.searched_mask.astype(np.uint8),
            visit_counts=self.visit_counts,
            paused=np.asarray(self.paused, dtype=np.uint8),
        )

    @classmethod
    def load(cls, path, expected_observable_mask=None):
        with np.load(Path(path).expanduser(), allow_pickle=False) as data:
            required = {"observable_mask", "searched_mask", "visit_counts", "paused"}
            if required - set(data.files):
                raise ValueError("Coverage-memory file is incomplete")
            observable = data["observable_mask"].astype(bool)
            searched = data["searched_mask"].astype(bool)
            visits = data["visit_counts"].astype(np.int32)
            paused = bool(data["paused"].item())
        if expected_observable_mask is not None and not np.array_equal(
            observable, np.asarray(expected_observable_mask, dtype=bool),
        ):
            raise ValueError("Saved memory belongs to a different map")
        memory = cls(observable, visits.size)
        if searched.shape != observable.shape or np.any(searched & ~observable):
            raise ValueError("Saved searched mask is invalid")
        if np.any(visits < 0):
            raise ValueError("Saved visit counts are invalid")
        memory.searched_mask = searched
        memory.visit_counts = visits
        memory.paused = paused
        return memory
