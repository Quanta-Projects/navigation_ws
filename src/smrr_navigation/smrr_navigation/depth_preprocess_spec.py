#!/usr/bin/env python3
"""
Canonical depth preprocessing specification for door classifier.

This module defines the EXACT preprocessing used during training.
All inference implementations (Python, C++, etc.) must match this spec bit-for-bit.

Training Preprocessing Steps:
1. Invalid handling: Replace NaN/inf/<=0 with clip_max_m (FAR, not 0!)
2. Clip to [clip_min_m, clip_max_m] where defaults are [0.2, 5.0]
3. Normalize to [0, 1]: (depth - clip_min) / (clip_max - clip_min)
4. Resize to 96x96 using cv2.INTER_AREA (area-based downsampling)
5. Return float32 tensor shaped [1, 1, 96, 96]

CRITICAL:
- Invalid pixels become MAX_DEPTH (far), NOT 0.0 (near)
- Clip minimum is 0.2m, NOT 0.0m
- Resize method is INTER_AREA, NOT INTER_NEAREST
"""

import numpy as np
import cv2


def preprocess_depth_training_spec(
    depth_m: np.ndarray,
    clip_min_m: float = 0.2,
    clip_max_m: float = 5.0,
    out_size: int = 96
) -> np.ndarray:
    """
    Preprocess depth image exactly as done during training.
    
    Args:
        depth_m: Depth image in meters, shape (H, W), dtype float32
        clip_min_m: Minimum valid depth in meters (default: 0.2)
        clip_max_m: Maximum valid depth in meters (default: 5.0)
        out_size: Output size (default: 96 for 96x96)
    
    Returns:
        Preprocessed tensor, shape (1, 1, out_size, out_size), dtype float32, range [0, 1]
    
    Raises:
        ValueError: If input is not 2D or parameters are invalid
    """
    if depth_m.ndim != 2:
        raise ValueError(f"Expected 2D depth array, got shape {depth_m.shape}")
    
    if clip_min_m >= clip_max_m:
        raise ValueError(f"clip_min_m ({clip_min_m}) must be < clip_max_m ({clip_max_m})")
    
    # Ensure float32
    depth_m = depth_m.astype(np.float32)
    
    # Step 1: Replace invalid pixels with clip_max_m (FAR)
    # Invalid = NaN, inf, or <= 0
    valid_mask = np.isfinite(depth_m) & (depth_m > 0)
    depth_m = np.where(valid_mask, depth_m, clip_max_m)
    
    # Step 2: Clip to valid range [clip_min_m, clip_max_m]
    depth_clipped = np.clip(depth_m, clip_min_m, clip_max_m)
    
    # Step 3: Normalize to [0, 1]
    depth_range = clip_max_m - clip_min_m
    depth_normalized = (depth_clipped - clip_min_m) / depth_range
    
    # Step 4: Resize to (out_size, out_size) using area-based downsampling
    # cv2.resize expects (width, height) but numpy arrays are (height, width)
    # INTER_AREA is best for downsampling - averages source pixels
    depth_resized = cv2.resize(
        depth_normalized,
        (out_size, out_size),
        interpolation=cv2.INTER_AREA
    )
    
    # Step 5: Reshape to [1, 1, H, W] tensor format (NCHW)
    tensor = depth_resized.astype(np.float32).reshape(1, 1, out_size, out_size)
    
    # Ensure contiguous memory layout
    tensor = np.ascontiguousarray(tensor)
    
    return tensor


def compute_preprocessing_stats(depth_m: np.ndarray, clip_min_m: float = 0.2, clip_max_m: float = 5.0) -> dict:
    """
    Compute statistics for debugging preprocessing.
    
    Args:
        depth_m: Raw depth image in meters, shape (H, W)
        clip_min_m: Minimum clip value
        clip_max_m: Maximum clip value
    
    Returns:
        Dictionary with preprocessing statistics
    """
    valid_mask = np.isfinite(depth_m) & (depth_m > 0)
    invalid_fraction = 1.0 - valid_mask.mean()
    
    stats = {
        'invalid_fraction': invalid_fraction,
        'valid_pixels': valid_mask.sum(),
        'total_pixels': depth_m.size
    }
    
    if valid_mask.any():
        valid_depths = depth_m[valid_mask]
        stats['valid_min'] = valid_depths.min()
        stats['valid_mean'] = valid_depths.mean()
        stats['valid_max'] = valid_depths.max()
        stats['below_clip_min'] = (valid_depths < clip_min_m).sum()
        stats['above_clip_max'] = (valid_depths > clip_max_m).sum()
    else:
        stats['valid_min'] = np.nan
        stats['valid_mean'] = np.nan
        stats['valid_max'] = np.nan
        stats['below_clip_min'] = 0
        stats['above_clip_max'] = 0
    
    return stats
