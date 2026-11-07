# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import TYPE_CHECKING, TypedDict

import numpy as np

from dimos.types.vector import Vector

if TYPE_CHECKING:
    import open3d as o3d  # type: ignore[import-untyped]


class ObjectData(TypedDict, total=False):
    """Data about an object in the manipulation scene."""

    # Basic detection information
    object_id: int  # Unique ID for the object
    bbox: list[float]  # Bounding box [x1, y1, x2, y2]
    depth: float  # Depth in meters
    confidence: float  # Detection confidence
    class_id: int  # Class ID from the detector
    label: str  # Semantic label (e.g., 'cup', 'table')
    movement_tolerance: float  # (0.0 = immovable, 1.0 = freely movable)
    segmentation_mask: np.ndarray  # type: ignore[type-arg]  # Binary mask of the object's pixels

    # 3D pose and dimensions
    position: dict[str, float] | Vector  # 3D position {x, y, z} or Vector
    rotation: dict[str, float] | Vector  # 3D rotation {roll, pitch, yaw} or Vector
    size: dict[str, float]  # Object dimensions {width, height, depth}

    # Point cloud data
    point_cloud: "o3d.geometry.PointCloud"  # Open3D point cloud object
    color: np.ndarray  # type: ignore[type-arg]  # RGB color for visualization [R, G, B]
