"""Safe model selection with the documented immediate-time-aware fallback."""

import numpy as np


def select_with_fallback(policy, features, shortlist_indices, immediate_utility):
    shortlist = np.asarray(shortlist_indices, dtype=np.int32)
    utility = np.asarray(immediate_utility, dtype=np.float64)
    if shortlist.ndim != 1 or shortlist.size == 0:
        raise ValueError("shortlist_indices must be non-empty")
    if utility.ndim != 1 or np.any(shortlist < 0) or np.any(shortlist >= utility.size):
        raise ValueError("immediate_utility does not match shortlist indices")
    try:
        local_index, scores = policy.select(features)
        if not np.all(np.isfinite(scores)):
            raise ValueError("Model returned non-finite scores")
        return int(shortlist[local_index]), scores, "tiny_mlp"
    except (OSError, ValueError, FloatingPointError):
        local_index = int(np.argmax(utility[shortlist]))
        return int(shortlist[local_index]), utility[shortlist].copy(), "fallback"
