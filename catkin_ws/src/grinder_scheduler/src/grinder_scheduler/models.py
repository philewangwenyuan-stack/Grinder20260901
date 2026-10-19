import enum
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


PLANNING_DIRECTIONS = ("x", "-x", "y", "-y")


def is_planning_direction(value):
    return str(value or "").strip().lower() in PLANNING_DIRECTIONS


def normalize_planning_direction(value, default="x"):
    candidate = str(value or "").strip().lower()
    if candidate in PLANNING_DIRECTIONS:
        return candidate
    fallback = str(default or "x").strip().lower()
    return fallback if fallback in PLANNING_DIRECTIONS else "x"


def planning_direction_axis(value):
    return normalize_planning_direction(value).lstrip("+-")


def explicit_region_start(region):
    """Ignore the bounding-box start synthesized by older map services.

    New edits record provenance so a user can still explicitly choose a corner.
    Legacy records cannot distinguish a chosen corner from the old default;
    treat that exact default as automatic, including for rotated polygons.
    """
    pose = dict(region.get("start_pose", {}) or {})
    if not pose:
        return {}
    if "start_pose_explicit" in region:
        return pose if region["start_pose_explicit"] else {}
    points = region.get("points", []) or []
    if points and (
        abs(float(pose.get("x", 0.0)) - min(float(p["x"]) for p in points)) <= 1e-6
        and abs(float(pose.get("y", 0.0)) - max(float(p["y"]) for p in points)) <= 1e-6
    ):
        return {}
    return pose


class SchedulerState(enum.Enum):
    IDLE = "IDLE"
    READY = "READY"
    PLANNING = "PLANNING"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    COMPLETED = "COMPLETED"
    STOPPED = "STOPPED"
    ERROR = "ERROR"


@dataclass
class TaskConfigModel:
    task_id: str = ""
    map_id: str = ""
    work_regions: List[Dict] = field(default_factory=list)
    obstacle_regions: List[Dict] = field(default_factory=list)
    erase_regions: List[Dict] = field(default_factory=list)
    crop_region: Dict = field(default_factory=dict)
    active_work_region_id: str = ""
    selected_work_region_ids: List[str] = field(default_factory=list)
    region_repeat_config: Dict[str, int] = field(default_factory=dict)
    vehicle_width: float = 0.5
    vehicle_length: float = 0.5
    default_path_spacing: float = 1.17
    global_direction: str = "x"
    planning_angle_deg: Optional[float] = None
    turn_radius: float = 0.8
    overlap_ratio: float = 0.1
    inflation_radius: float = 0.6
    endpoint_margin: float = 2.0
    output_point_spacing: float = 0.3
    aligned_obstacle_inflation: float = 0.7
    aligned_obstacle_max_extent: float = 3.0
    obstacle_corner_angle_deg: float = 45.0
    obstacle_avoidance_distance: float = 0.0
    current_pose: Dict = field(default_factory=dict)
    start_pose: Dict = field(default_factory=dict)
    end_pose: Dict = field(default_factory=dict)
    # Runtime-only checkpoint for a single interrupted region/lap; not protocol config.
    continuation: Dict = field(default_factory=dict)


@dataclass
class PlannerPath:
    task_id: str
    path_version: int
    points: List[Dict]
    nav_path: object
    length_m: float


@dataclass
class VideoStreamState:
    stream_url: str = ""
    codec: str = "h264"
    width: int = 0
    height: int = 0
    online: bool = False
    last_update_utc: int = 0


@dataclass
class MapSnapshot:
    map_version: int
    frame_id: str
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    preview_data: bytes
    preview_format: str
    overlay_json: str
    preview_image: Optional[Any] = None
