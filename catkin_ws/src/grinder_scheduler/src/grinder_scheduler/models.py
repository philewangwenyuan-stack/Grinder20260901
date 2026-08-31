import enum
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


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
    inflation_radius: float = 0.65
    current_pose: Dict = field(default_factory=dict)
    start_pose: Dict = field(default_factory=dict)
    end_pose: Dict = field(default_factory=dict)


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
