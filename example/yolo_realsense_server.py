"""为 vision_pick.cpp 提供 YOLOv8 + RealSense D455 按需识别服务。

TCP 协议每条消息占一行 ASCII 文本：

    C++ -> HELLO VISION_PICK 1
    PY  -> READY 1 REALSENSE
    C++ -> DETECT <请求编号> <类别过滤或-1> <CLOSEST|BEST|CENTER>
                 [法兰X米 法兰Y米 法兰Z米 法兰A弧度 法兰B弧度 法兰C弧度]
    PY  -> TARGET <请求编号> <帧号> <时间戳> <类别> <置信度>
                 <像素u> <像素v> <相机X毫米> <相机Y毫米> <相机Z毫米>
         或 NOT_FOUND <请求编号> <原因>

每次收到 DETECT 后锁定 class 14，连续稳定检测至少 0.5 秒，再返回多帧平均
XYZ，避免机械臂使用旧图像或瞬时误检。
"""

import argparse
from collections import deque
import math
import os
import socket
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple


PROTOCOL_VERSION = 1
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 5010
DEFAULT_IMAGE_SIZE = 640
DEFAULT_CONFIDENCE = 0.50
DEFAULT_FRESH_FRAMES = 3
DEFAULT_TARGET_CLASS = 14
DEFAULT_STABLE_SECONDS = 0.5
DEFAULT_STABLE_MIN_FRAMES = 3
DEFAULT_STABILITY_TIMEOUT_SECONDS = 15.0
SECOND_DETECTION_MAX_MISSING_FRAMES = 2
MAX_STABLE_CENTER_DRIFT_PX = 35.0
MAX_STABLE_DEPTH_DRIFT_MM = 50.0
DEPTH_ROI_RADIUS = 5
MIN_DEPTH_CLUSTER_SAMPLES = 8
DEPTH_CLUSTER_GAP_METERS = 0.030
MIN_DEPTH_METERS = 0.080
MAX_DEPTH_METERS = 2.000
# D455 在 640x480 深度模式下的近距离盲区约到 320 mm；424x240 可把
# 最小可测距离降到约 180 mm。彩色流仍保留 640x480，保证 YOLO 输入清晰度。
DEPTH_STREAM_WIDTH = 424
DEPTH_STREAM_HEIGHT = 240
COLOR_STREAM_WIDTH = 640
COLOR_STREAM_HEIGHT = 480
STREAM_FPS = 30
PREVIEW_WINDOW_NAME = "vision_pick YOLOv8 + D455"
SOCKET_POLL_SECONDS = 0.25
MAX_PROTOCOL_LINE_BYTES = 8192

# 与 vision_pick.cpp 中 hand_in_eye() 当前启用项保持一致，单位为毫米。
HAND_IN_EYE_MATRIX = (
    (0.00882397, 0.99995778, -0.00256451, -124.60064718),
    (-0.99990371, 0.00885091, 0.01068793, -22.19809241),
    (0.01071018, 0.00246995, 0.99993959, 34.82308787),
    (0.0, 0.0, 0.0, 1.0),
)

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent
RUNTIME_DIR = Path(tempfile.gettempdir()) / "rokae_vision_pick"
os.environ.setdefault("YOLO_CONFIG_DIR", str(RUNTIME_DIR))

DEFAULT_MODEL_CANDIDATES = (
    SCRIPT_DIR / "models" / "best.pt",
    Path(r"D:\roake\sdk\librokae-v0.2.8_clean\example\models\best.pt"),
)


@dataclass(frozen=True)
class Candidate:
    """保存一个深度有效的 YOLO 候选目标及其二维、三维和帧信息。"""

    frame_number: int
    timestamp_ms: float
    class_id: int
    confidence: float
    pixel_u: float
    pixel_v: float
    camera_mm: Tuple[float, float, float]
    bbox_xyxy: Tuple[int, int, int, int]
    image_size: Tuple[int, int]
    depth_sample_count: int = 0

    @property
    def distance_mm(self) -> float:
        """计算目标点到相机原点的三维欧氏距离，单位为毫米。"""

        x_mm, y_mm, z_mm = self.camera_mm
        return math.sqrt(x_mm * x_mm + y_mm * y_mm + z_mm * z_mm)

    @property
    def center_distance_sq(self) -> float:
        """计算目标中心到图像中心的像素距离平方，用于 CENTER 选择规则。"""

        width, height = self.image_size
        return ((self.pixel_u - width / 2.0) ** 2 +
                (self.pixel_v - height / 2.0) ** 2)


def choose_candidate(candidates: Iterable[Candidate], class_filter: int,
                     selection: str) -> Optional[Candidate]:
    """按类别过滤后，根据最近距离、最高置信度或最靠近画面中心选择一个目标。"""

    filtered = [
        candidate for candidate in candidates
        if class_filter < 0 or candidate.class_id == class_filter
    ]
    if not filtered:
        return None

    rule = selection.upper()
    if rule == "CLOSEST":
        return min(filtered, key=lambda item: (item.distance_mm, -item.confidence))
    if rule == "BEST":
        return max(filtered, key=lambda item: (item.confidence, -item.distance_mm))
    if rule == "CENTER":
        return min(filtered, key=lambda item: (item.center_distance_sq,
                                                item.distance_mm))
    raise ValueError("UNSUPPORTED_SELECTION_" + rule)


def stability_transition_reason(previous: Candidate, current: Candidate) -> str:
    """诊断相邻三维候选是否发生中心跳变或 Z 深度跳变。"""

    center_drift_px = math.hypot(
        current.pixel_u - previous.pixel_u,
        current.pixel_v - previous.pixel_v,
    )
    depth_drift_mm = abs(current.camera_mm[2] - previous.camera_mm[2])
    if (previous.class_id != current.class_id or
            center_drift_px > MAX_STABLE_CENTER_DRIFT_PX):
        return "CENTER_JUMP"
    if depth_drift_mm > MAX_STABLE_DEPTH_DRIFT_MM:
        return "Z_JUMP"
    return "OK"


def is_same_stable_target(previous: Candidate, current: Candidate) -> bool:
    """判断相邻推理帧是否仍是同一目标，供自测和稳定累计使用。"""

    return stability_transition_reason(previous, current) == "OK"


def may_bridge_missing_frame(stable_sample_count: int, missing_frame_count: int,
                             allowed_missing_frames: int) -> bool:
    """判断当前漏检是否仍可保留已经累计的稳定样本。"""

    return (stable_sample_count > 0 and
            missing_frame_count <= allowed_missing_frames)


def average_candidates(samples: Sequence[Candidate]) -> Candidate:
    """对稳定期间的多帧中心、置信度和 XYZ 取算术平均，降低随机抖动。"""

    if not samples:
        raise ValueError("EMPTY_STABLE_SAMPLE_SET")
    count = float(len(samples))
    last = samples[-1]

    def mean(values: Iterable[float]) -> float:
        """计算当前稳定样本集合中某一字段的算术平均值。"""

        return sum(values) / count

    return Candidate(
        frame_number=last.frame_number,
        timestamp_ms=last.timestamp_ms,
        class_id=last.class_id,
        confidence=mean(sample.confidence for sample in samples),
        pixel_u=mean(sample.pixel_u for sample in samples),
        pixel_v=mean(sample.pixel_v for sample in samples),
        camera_mm=tuple(
            mean(sample.camera_mm[axis] for sample in samples)
            for axis in range(3)
        ),
        bbox_xyxy=tuple(
            int(round(mean(sample.bbox_xyxy[axis] for sample in samples)))
            for axis in range(4)
        ),
        image_size=last.image_size,
        depth_sample_count=int(round(mean(
            sample.depth_sample_count for sample in samples
        ))),
    )


@dataclass(frozen=True)
class FrameObservation:
    """视觉线程发布的一帧轻量结果；不携带 OpenCV/Torch 图像对象。"""

    sequence: int
    candidates: Tuple[Candidate, ...]
    reason: str
    valid_depth_points: int
    current_z_mm: Optional[float]


def sanitize_reason(reason: object) -> str:
    """把异常原因整理成不含换行符的单行协议文字，避免破坏消息边界。"""

    text = str(reason).strip().replace("\r", " ").replace("\n", " ")
    return text or "UNKNOWN"


def multiply_matrices(left: Sequence[Sequence[float]],
                      right: Sequence[Sequence[float]]) -> Tuple[Tuple[float, ...], ...]:
    """计算两个 4x4 齐次矩阵的乘积。"""

    return tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(4))
            for column in range(4)
        )
        for row in range(4)
    )


def flange_pose_to_matrix_mm(flange_pose: Sequence[float]
                             ) -> Tuple[Tuple[float, ...], ...]:
    """把法兰 [X,Y,Z,A,B,C]（米/弧度）转换为毫米制基座到法兰矩阵。"""

    x_m, y_m, z_m, a_rad, b_rad, c_rad = (
        float(value) for value in flange_pose
    )
    ca, sa = math.cos(a_rad), math.sin(a_rad)
    cb, sb = math.cos(b_rad), math.sin(b_rad)
    cc, sc = math.cos(c_rad), math.sin(c_rad)

    # 与 vision_pick.cpp 一致：R = Rz(C) * Ry(B) * Rx(A)。
    return (
        (cc * cb, cc * sb * sa - sc * ca, cc * sb * ca + sc * sa, x_m * 1000.0),
        (sc * cb, sc * sb * sa + cc * ca, sc * sb * ca - cc * sa, y_m * 1000.0),
        (-sb, cb * sa, cb * ca, z_m * 1000.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def camera_target_to_base_mm(camera_mm: Sequence[float],
                             flange_pose: Optional[Sequence[float]]
                             ) -> Optional[Tuple[float, float, float]]:
    """用当前法兰位姿和手眼矩阵，把相机坐标转换为基座坐标。"""

    if flange_pose is None:
        return None
    camera_to_base = multiply_matrices(
        flange_pose_to_matrix_mm(flange_pose), HAND_IN_EYE_MATRIX
    )
    x_mm, y_mm, z_mm = (float(value) for value in camera_mm)
    return (
        camera_to_base[0][0] * x_mm + camera_to_base[0][1] * y_mm +
        camera_to_base[0][2] * z_mm + camera_to_base[0][3],
        camera_to_base[1][0] * x_mm + camera_to_base[1][1] * y_mm +
        camera_to_base[1][2] * z_mm + camera_to_base[1][3],
        camera_to_base[2][0] * x_mm + camera_to_base[2][1] * y_mm +
        camera_to_base[2][2] * z_mm + camera_to_base[2][3],
    )


def parse_detect_request(
        line: str) -> Tuple[int, int, str, Optional[Tuple[float, ...]]]:
    """解析 DETECT，并返回请求编号、类别、选择规则和可选法兰位姿。"""

    fields = line.strip().split()
    if len(fields) not in (4, 10) or fields[0] != "DETECT":
        raise ValueError("EXPECTED_DETECT_REQUEST")
    request_id = int(fields[1])
    class_filter = int(fields[2])
    selection = fields[3].upper()
    if request_id <= 0:
        raise ValueError("INVALID_REQUEST_ID")
    if class_filter < -1:
        raise ValueError("INVALID_CLASS_FILTER")
    if selection not in ("CLOSEST", "BEST", "CENTER"):
        raise ValueError("UNSUPPORTED_SELECTION_" + selection)
    flange_pose = None
    if len(fields) == 10:
        flange_pose = tuple(float(value) for value in fields[4:10])
        if not all(math.isfinite(value) for value in flange_pose):
            raise ValueError("NON_FINITE_FLANGE_POSE")
    return request_id, class_filter, selection, flange_pose


def format_target(request_id: int, target: Candidate) -> str:
    """把选中的目标编码成 C++ 可以直接解析的 TARGET 单行响应。"""

    x_mm, y_mm, z_mm = target.camera_mm
    return (
        "TARGET {request} {frame} {timestamp:.3f} {class_id} {confidence:.6f} "
        "{u:.3f} {v:.3f} {x:.3f} {y:.3f} {z:.3f}\n"
    ).format(
        request=request_id,
        frame=target.frame_number,
        timestamp=target.timestamp_ms,
        class_id=target.class_id,
        confidence=target.confidence,
        u=target.pixel_u,
        v=target.pixel_v,
        x=x_mm,
        y=y_mm,
        z=z_mm,
    )


class MockDetector:
    """提供无需模型和相机的固定坐标，用于验证 TCP 协议；实机模式会拒绝它。"""

    mode = "MOCK"

    def __init__(self, point_mm: Sequence[float]) -> None:
        """保存模拟 XYZ，并初始化严格递增的帧号和时间戳。"""

        self._point_mm = tuple(float(value) for value in point_mm)
        self._frame_number = 0
        self._last_timestamp_ms = 0.0

    def detect(self, class_filter: int, selection: str,
               allowed_missing_frames: int = 0,
               flange_pose: Optional[Sequence[float]] = None
               ) -> Tuple[Optional[Candidate], str]:
        """为每次请求生成一张新模拟帧，再按正式选择规则返回目标。"""

        self._frame_number += 1
        timestamp_ms = max(time.monotonic() * 1000.0,
                           self._last_timestamp_ms + 1.0)
        self._last_timestamp_ms = timestamp_ms
        candidate = Candidate(
            frame_number=self._frame_number,
            timestamp_ms=timestamp_ms,
            class_id=0,
            confidence=0.99,
            pixel_u=320.0,
            pixel_v=240.0,
            camera_mm=self._point_mm,
            bbox_xyxy=(300, 220, 340, 260),
            image_size=(640, 480),
        )
        selected = choose_candidate([candidate], class_filter, selection)
        return selected, "CLASS_FILTER_NO_MATCH" if selected is None else "OK"

    def close(self) -> None:
        """保持与真实检测器相同的关闭接口；模拟模式没有硬件资源需要释放。"""

        return None

    @property
    def stop_requested(self) -> bool:
        """模拟服务没有预览窗口，只能由 --once 或 Ctrl+C 结束。"""

        return False


class RealSenseYoloDetector:
    """管理 D455、深度滤波和 YOLO 模型，按请求生成相机坐标目标。"""

    mode = "REALSENSE"

    def __init__(self, model_path: Path, image_size: int, confidence: float,
                 fresh_frames: int, device: Optional[str], show: bool,
                 target_class: int, stable_seconds: float,
                 stable_min_frames: int, stability_timeout: float) -> None:
        """加载依赖和模型，启动彩色/深度流，并丢弃启动初期不稳定的图像。"""

        RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
        try:
            import cv2
            import numpy as np
            import pyrealsense2 as rs
            from ultralytics import YOLO
        except ImportError as error:
            raise RuntimeError("MISSING_PYTHON_DEPENDENCY_" + error.name) from error

        if not model_path.is_file():
            raise FileNotFoundError("MODEL_NOT_FOUND_" + str(model_path))

        self.cv2 = cv2
        self.np = np
        self.rs = rs
        self.image_size = image_size
        self.confidence = confidence
        self.fresh_frames = max(1, fresh_frames)
        self.device = device
        self.show = show
        self.target_class = target_class
        self.stable_seconds = stable_seconds
        self.stable_min_frames = stable_min_frames
        self.stability_timeout = stability_timeout
        self.model = YOLO(str(model_path))
        model_names = self.model.names
        if isinstance(model_names, dict):
            target_name = model_names.get(self.target_class)
        else:
            target_name = (model_names[self.target_class]
                           if 0 <= self.target_class < len(model_names) else None)
        if target_name is None:
            raise ValueError("TARGET_CLASS_NOT_IN_MODEL_{0}".format(self.target_class))
        print(
            "Target class locked: id={0}, name={1}; stable>={2:.2f}s/{3} frames".format(
                self.target_class, target_name, self.stable_seconds,
                self.stable_min_frames,
            ),
            flush=True,
        )
        self.pipeline = rs.pipeline()
        config = rs.config()
        # 深度采用 D455 的近距离模式；对齐后仍会映射到 640x480 彩色坐标系。
        config.enable_stream(
            rs.stream.depth,
            DEPTH_STREAM_WIDTH,
            DEPTH_STREAM_HEIGHT,
            rs.format.z16,
            STREAM_FPS,
        )
        config.enable_stream(
            rs.stream.color,
            COLOR_STREAM_WIDTH,
            COLOR_STREAM_HEIGHT,
            rs.format.bgr8,
            STREAM_FPS,
        )
        self.align = rs.align(rs.stream.color)
        self.spatial_filter = rs.spatial_filter()
        self.temporal_filter = rs.temporal_filter()
        self.pipeline_started = False
        # 相机与模型只能由一个线程访问；通信线程通过条件变量读取轻量观测结果。
        self._condition = threading.Condition()
        self._observations = deque(maxlen=128)
        self._observation_sequence = 0
        self._stop_event = threading.Event()
        self._worker_error: Optional[BaseException] = None
        self._request_active = False
        self._display_reason = "STARTING"
        self._display_stable_frames = 0
        self._display_stable_elapsed = 0.0
        self._worker: Optional[threading.Thread] = None
        self._window_created = False
        self._stop_reason = ""

        active_profile = self.pipeline.start(config)
        self.pipeline_started = True
        self._print_active_camera_configuration(active_profile)
        for _ in range(10):
            self.pipeline.wait_for_frames(5000)
        self._worker = threading.Thread(
            target=self._vision_loop,
            name="vision-persistent-preview",
            daemon=True,
        )
        self._worker.start()
        print(
            "RealSense started; continuous detection is active; "
            "waiting for DETECT requests.",
            flush=True,
        )

    def _print_active_camera_configuration(self, active_profile) -> None:
        """打印真实设备和最终生效的流配置，避免把请求配置误当成实际配置。"""

        device = active_profile.get_device()

        def read_info(info_key, fallback: str = "N/A") -> str:
            """安全读取设备信息；旧固件不支持的字段返回 N/A。"""

            try:
                if device.supports(info_key):
                    return device.get_info(info_key)
            except RuntimeError:
                pass
            return fallback

        device_name = read_info(self.rs.camera_info.name)
        serial = read_info(self.rs.camera_info.serial_number)
        firmware = read_info(self.rs.camera_info.firmware_version)
        usb_type = read_info(self.rs.camera_info.usb_type_descriptor)
        print(
            "RealSense device: name={0}, serial={1}, firmware={2}, USB={3}".format(
                device_name, serial, firmware, usb_type
            ),
            flush=True,
        )
        if "D455" not in device_name.upper():
            print(
                "Warning: expected D455, but active device is {0}.".format(
                    device_name
                ),
                flush=True,
            )

        for stream_type, label in (
                (self.rs.stream.depth, "depth"),
                (self.rs.stream.color, "color")):
            stream_profile = active_profile.get_stream(stream_type)
            video_profile = stream_profile.as_video_stream_profile()
            print(
                "Active {0} stream: {1}x{2}@{3} format={4}".format(
                    label,
                    video_profile.width(),
                    video_profile.height(),
                    stream_profile.fps(),
                    stream_profile.format(),
                ),
                flush=True,
            )

    def _capture_fresh(self):
        """连续读取若干帧，返回最新的彩色、对齐深度和滤波深度帧。"""

        latest = None
        for _ in range(self.fresh_frames):
            frames = self.pipeline.wait_for_frames(5000)
            aligned = self.align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            filtered = self.spatial_filter.process(depth_frame)
            filtered = self.temporal_filter.process(filtered).as_depth_frame()
            if filtered:
                latest = (depth_frame, color_frame, filtered)
        if latest is None:
            raise RuntimeError("NO_SYNCHRONIZED_REALSENSE_FRAME")
        return latest

    def _nearest_depth_cluster(self, depth_image, depth_scale: float,
                               pixel_u: int,
                               pixel_v: int) -> Tuple[Optional[float], int]:
        """返回框中心邻域最近的有效深度簇中位数及其有效点数。"""

        height, width = depth_image.shape[:2]
        x_min = max(0, pixel_u - DEPTH_ROI_RADIUS)
        x_max = min(width, pixel_u + DEPTH_ROI_RADIUS + 1)
        y_min = max(0, pixel_v - DEPTH_ROI_RADIUS)
        y_max = min(height, pixel_v + DEPTH_ROI_RADIUS + 1)

        depths = (depth_image[y_min:y_max, x_min:x_max].astype(self.np.float32)
                  * depth_scale)
        valid = depths[
            self.np.isfinite(depths) &
            (depths >= MIN_DEPTH_METERS) &
            (depths <= MAX_DEPTH_METERS)
        ]
        if valid.size < MIN_DEPTH_CLUSTER_SAMPLES:
            return None, int(valid.size)

        sorted_depths = self.np.sort(valid)
        split_positions = (
            self.np.where(self.np.diff(sorted_depths) > DEPTH_CLUSTER_GAP_METERS)[0] + 1
        )
        for cluster in self.np.split(sorted_depths, split_positions):
            if cluster.size >= MIN_DEPTH_CLUSTER_SAMPLES:
                return float(self.np.median(cluster)), int(cluster.size)
        return None, int(valid.size)

    def _candidates_from_result(self, result, depth_frame, color_frame,
                                filtered_depth_frame
                                ) -> Tuple[List[Candidate], int, int]:
        """把 YOLO 检测框中心和对齐深度反投影为相机 XYZ 候选目标。"""

        color_image = self.np.asanyarray(color_frame.get_data())
        depth_image = self.np.asanyarray(filtered_depth_frame.get_data())
        height, width = color_image.shape[:2]
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        depth_scale = depth_frame.get_units()
        candidates = []
        invalid_depth_count = 0
        max_depth_sample_count = 0

        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().tolist())
            pixel_u = min(max(int((x1 + x2) / 2), 0), width - 1)
            pixel_v = min(max(int((y1 + y2) / 2), 0), height - 1)
            depth_m, depth_sample_count = self._nearest_depth_cluster(
                depth_image, depth_scale, pixel_u, pixel_v
            )
            max_depth_sample_count = max(max_depth_sample_count,
                                         depth_sample_count)
            if depth_m is None:
                invalid_depth_count += 1
                continue

            camera_m = self.rs.rs2_deproject_pixel_to_point(
                intrinsics, [pixel_u, pixel_v], depth_m
            )
            camera_mm = tuple(float(value) * 1000.0 for value in camera_m)
            candidates.append(Candidate(
                frame_number=int(color_frame.get_frame_number()),
                timestamp_ms=time.monotonic() * 1000.0,
                class_id=int(box.cls[0].cpu()),
                confidence=float(box.conf[0].cpu()),
                pixel_u=float(pixel_u),
                pixel_v=float(pixel_v),
                camera_mm=camera_mm,
                bbox_xyxy=(x1, y1, x2, y2),
                image_size=(width, height),
                depth_sample_count=depth_sample_count,
            ))

        return candidates, invalid_depth_count, max_depth_sample_count

    def _show_result(self, result, candidates: Sequence[Candidate],
                     valid_depth_points: int,
                     current_z_mm: Optional[float]) -> None:
        """刷新常驻预览；关闭窗口或按 Q/Esc 会请求整个视觉服务退出。"""

        if not self.show or self._stop_event.is_set():
            return

        # OpenCV 的 imshow() 会在窗口被关闭后自动重建同名窗口。因此刷新前必须
        # 检查窗口是否仍可见，把右上角 X 转换为整个服务的退出信号。
        if self._window_created:
            try:
                if self.cv2.getWindowProperty(
                        PREVIEW_WINDOW_NAME, self.cv2.WND_PROP_VISIBLE) < 1:
                    self.request_stop("PREVIEW_WINDOW_CLOSED")
                    return
            except self.cv2.error:
                self.request_stop("PREVIEW_WINDOW_CLOSED")
                return
        else:
            self.cv2.namedWindow(PREVIEW_WINDOW_NAME, self.cv2.WINDOW_AUTOSIZE)
            self._window_created = True

        image = result.plot()
        # 无论深度是否有效，都把每个 YOLO 框的几何中心画出来。
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().tolist())
            center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
            self.cv2.circle(image, center, 5, (0, 0, 255), -1)
        for candidate in candidates:
            center = (int(round(candidate.pixel_u)), int(round(candidate.pixel_v)))
            x_mm, y_mm, z_mm = candidate.camera_mm
            self.cv2.putText(
                image,
                "center=({0},{1}) XYZ=({2:.1f},{3:.1f},{4:.1f})mm points={5}".format(
                    center[0], center[1], x_mm, y_mm, z_mm,
                    candidate.depth_sample_count,
                ),
                (max(0, candidate.bbox_xyxy[0]),
                 min(candidate.image_size[1] - 8, candidate.bbox_xyxy[3] + 22)),
                self.cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 255),
                1,
                self.cv2.LINE_AA,
            )

        with self._condition:
            request_active = self._request_active
            display_reason = self._display_reason
            stable_frames = self._display_stable_frames
            stable_elapsed = self._display_stable_elapsed

        mode_text = "REQUEST ACTIVE" if request_active else "MONITORING"
        status = "{0} | {1}".format(mode_text, display_reason)
        depth_text = "depth_points={0}  Z={1}".format(
            valid_depth_points,
            "N/A" if current_z_mm is None else "{0:.1f} mm".format(current_z_mm),
        )
        stable_text = "stable {0}/{1} frames, {2:.2f}/{3:.2f}s".format(
            stable_frames, self.stable_min_frames,
            stable_elapsed, self.stable_seconds,
        )
        status_color = ((0, 255, 0) if display_reason in ("OK", "STABLE")
                        else (0, 165, 255))
        self.cv2.putText(
            image, status, (12, 28), self.cv2.FONT_HERSHEY_SIMPLEX,
            0.60, status_color, 2, self.cv2.LINE_AA,
        )
        self.cv2.putText(
            image, depth_text, (12, 54), self.cv2.FONT_HERSHEY_SIMPLEX,
            0.60, status_color, 2, self.cv2.LINE_AA,
        )
        self.cv2.putText(
            image, stable_text, (12, 80), self.cv2.FONT_HERSHEY_SIMPLEX,
            0.60, status_color, 2, self.cv2.LINE_AA,
        )
        self.cv2.imshow(PREVIEW_WINDOW_NAME, image)
        key = self.cv2.waitKey(1) & 0xFF
        if key in (27, ord("q"), ord("Q")):
            self.request_stop("PREVIEW_EXIT_KEY")

    def _detect_once(self):
        """完成一帧推理和深度定位；只由常驻视觉线程调用。"""

        depth_frame, color_frame, filtered_depth = self._capture_fresh()
        color_image = self.np.asanyarray(color_frame.get_data())
        prediction_args = {
            "source": color_image,
            "imgsz": self.image_size,
            "conf": self.confidence,
            "classes": [self.target_class],
            "verbose": False,
        }
        if self.device:
            prediction_args["device"] = self.device

        result = self.model.predict(**prediction_args)[0]
        box_count = len(result.boxes)
        candidates, invalid_depth_count, max_depth_sample_count = self._candidates_from_result(
            result, depth_frame, color_frame, filtered_depth
        )
        selected = choose_candidate(candidates, self.target_class, "CLOSEST")
        if selected is not None:
            reason = "OK"
        elif box_count == 0:
            reason = "TARGET_LOST"
        elif invalid_depth_count > 0:
            reason = "INVALID_DEPTH"
        else:
            reason = "CLASS_FILTER_NO_MATCH"
        valid_depth_points = (
            selected.depth_sample_count if selected is not None
            else max_depth_sample_count
        )
        current_z_mm = selected.camera_mm[2] if selected is not None else None
        return (reason, result, candidates, valid_depth_points,
                current_z_mm)

    def _vision_loop(self) -> None:
        """持续采集、推理并刷新窗口，同时向通信请求发布轻量帧结果。"""

        previous_monitor_target: Optional[Candidate] = None
        try:
            while not self._stop_event.is_set():
                (reason, result, candidates, valid_depth_points,
                 current_z_mm) = self._detect_once()
                # close() 可能在取帧或模型推理期间发出退出；不要再发布或重建窗口。
                if self._stop_event.is_set():
                    break
                monitor_target = choose_candidate(
                    candidates, self.target_class, "CLOSEST"
                )
                monitor_reason = reason
                if monitor_target is not None:
                    if previous_monitor_target is not None:
                        transition_reason = stability_transition_reason(
                            previous_monitor_target, monitor_target
                        )
                        if transition_reason != "OK":
                            monitor_reason = transition_reason
                    previous_monitor_target = monitor_target
                else:
                    previous_monitor_target = None
                with self._condition:
                    self._observation_sequence += 1
                    observation = FrameObservation(
                        sequence=self._observation_sequence,
                        candidates=tuple(candidates),
                        reason=reason,
                        valid_depth_points=valid_depth_points,
                        current_z_mm=current_z_mm,
                    )
                    self._observations.append(observation)
                    if not self._request_active:
                        self._display_reason = monitor_reason
                        self._display_stable_frames = 0
                        self._display_stable_elapsed = 0.0
                    self._condition.notify_all()
                self._show_result(
                    result, candidates, valid_depth_points, current_z_mm
                )
        except BaseException as error:
            with self._condition:
                self._worker_error = error
                self._condition.notify_all()
            self.request_stop("VISION_WORKER_FAILED")

    def _wait_for_observation(self, after_sequence: int,
                              deadline: float) -> Optional[FrameObservation]:
        """等待请求开始后产生的新推理帧，不向视觉线程发送任何控制指令。"""

        with self._condition:
            while True:
                if self._stop_event.is_set():
                    return None
                if self._worker_error is not None:
                    raise RuntimeError(
                        "VISION_WORKER_FAILED_" + sanitize_reason(self._worker_error)
                    ) from self._worker_error
                for observation in self._observations:
                    if observation.sequence > after_sequence:
                        return observation
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(timeout=min(remaining, 0.5))

    def _set_request_display(self, active: bool, reason: str,
                             stable_frames: int = 0,
                             stable_elapsed: float = 0.0) -> None:
        """更新预览叠加文字；此状态不参与 TCP 响应与坐标计算。"""

        with self._condition:
            self._request_active = active
            self._display_reason = reason
            self._display_stable_frames = stable_frames
            self._display_stable_elapsed = stable_elapsed

    def detect(self, class_filter: int, selection: str,
               allowed_missing_frames: int = 0,
               flange_pose: Optional[Sequence[float]] = None
               ) -> Tuple[Optional[Candidate], str]:
        """使用请求后的新观测；允许稳定窗口内累计指定数量的漏检帧。"""

        # C++ 首次请求使用 -1；视觉端仍强制锁定目标类别，禁止其他类别参与运动。
        if class_filter not in (-1, self.target_class):
            return None, "CLASS_FILTER_NOT_ALLOWED"
        effective_class = self.target_class
        deadline = time.monotonic() + self.stability_timeout
        stable_samples: List[Candidate] = []
        stable_since_ms = 0.0
        last_reason = "TARGET_LOST"
        last_instability_reason = ""
        saw_valid_candidate = False
        missing_frames_in_window = 0
        with self._condition:
            consumed_sequence = self._observation_sequence
        self._set_request_display(True, "WAITING_NEW_FRAME")

        try:
            while time.monotonic() < deadline:
                observation = self._wait_for_observation(
                    consumed_sequence, deadline
                )
                if observation is None:
                    break
                consumed_sequence = observation.sequence
                selected = choose_candidate(
                    observation.candidates, effective_class, selection
                )
                if selected is None:
                    last_reason = observation.reason
                    missing_frames_in_window += 1
                    if may_bridge_missing_frame(
                            len(stable_samples), missing_frames_in_window,
                            allowed_missing_frames):
                        # 二次判定可容忍短暂漏检，但漏检帧本身不计入稳定帧数。
                        stable_elapsed = max(
                            0.0,
                            (time.monotonic() * 1000.0 - stable_since_ms) / 1000.0,
                        )
                        self._set_request_display(
                            True,
                            "{0} GAP {1}/{2}".format(
                                last_reason,
                                missing_frames_in_window,
                                allowed_missing_frames,
                            ),
                            len(stable_samples),
                            stable_elapsed,
                        )
                        continue
                    stable_samples = []
                    stable_since_ms = 0.0
                    missing_frames_in_window = 0
                    self._set_request_display(
                        True,
                        "{0} RESET".format(last_reason)
                        if allowed_missing_frames > 0 else last_reason,
                    )
                    continue

                saw_valid_candidate = True
                transition_reason = "OK"
                if stable_samples:
                    transition_reason = stability_transition_reason(
                        stable_samples[-1], selected
                    )
                if transition_reason == "OK":
                    last_reason = "OK"
                    if not stable_samples:
                        stable_since_ms = selected.timestamp_ms
                        missing_frames_in_window = 0
                    stable_samples.append(selected)
                    display_reason = "STABILIZING"
                else:
                    # 明确记录重置原因，超时时原样返回给 C++，不再统称目标丢失。
                    last_instability_reason = transition_reason
                    last_reason = transition_reason
                    stable_samples = [selected]
                    stable_since_ms = selected.timestamp_ms
                    missing_frames_in_window = 0
                    display_reason = transition_reason

                stable_elapsed = max(
                    0.0, (selected.timestamp_ms - stable_since_ms) / 1000.0
                )
                is_stable = (
                    len(stable_samples) >= self.stable_min_frames and
                    stable_elapsed >= self.stable_seconds
                )
                self._set_request_display(
                    True,
                    "STABLE" if is_stable else display_reason,
                    len(stable_samples),
                    stable_elapsed,
                )
                if not is_stable:
                    continue

                averaged = average_candidates(stable_samples)
                print(
                    "Stable target: class={0}, samples={1}, duration={2:.3f}s, "
                    "mean_camera_mm=({3:.1f},{4:.1f},{5:.1f}), "
                    "depth_points={6}, distance_mm={7:.1f}".format(
                        averaged.class_id,
                        len(stable_samples),
                        stable_elapsed,
                        averaged.camera_mm[0],
                        averaged.camera_mm[1],
                        averaged.camera_mm[2],
                        averaged.depth_sample_count,
                        averaged.distance_mm,
                    ),
                    flush=True,
                )
                target_base_mm = camera_target_to_base_mm(
                    averaged.camera_mm, flange_pose
                )
                if target_base_mm is not None:
                    print(
                        "Target base position after hand-eye/end_to_base (mm): "
                        "X={0:.3f}, Y={1:.3f}, Z={2:.3f}".format(*target_base_mm),
                        flush=True,
                    )
                return averaged, "OK"

            # 当前帧的明确故障优先；否则返回最近一次稳定序列重置原因。
            if last_reason in ("TARGET_LOST", "INVALID_DEPTH"):
                return None, last_reason
            if last_instability_reason:
                return None, last_instability_reason
            return None, "UNSTABLE_TARGET" if saw_valid_candidate else last_reason
        finally:
            self._set_request_display(False, "MONITORING")

    @property
    def stop_requested(self) -> bool:
        """供 TCP 主线程查询预览窗口或视觉线程是否已经请求退出。"""

        return self._stop_event.is_set()

    def request_stop(self, reason: str) -> None:
        """设置统一退出标志，并唤醒正在等待视觉观测的通信线程。"""

        should_log = False
        with self._condition:
            if not self._stop_event.is_set():
                self._stop_reason = reason
                should_log = True
            self._stop_event.set()
            self._condition.notify_all()
        if should_log:
            print(
                "Stopping vision server: {0}".format(sanitize_reason(reason)),
                flush=True,
            )

    def close(self) -> None:
        """停止 RealSense 数据流并关闭本程序创建的 OpenCV 显示窗口。"""

        self.request_stop("PROGRAM_EXIT")
        # 先停止管线，可立即唤醒阻塞在 wait_for_frames() 的视觉线程。
        if self.pipeline_started:
            self.pipeline.stop()
            self.pipeline_started = False
        if self._worker is not None:
            self._worker.join(timeout=6.0)
        if self.show:
            self.cv2.destroyAllWindows()
            self._window_created = False


class VisionProtocolServer:
    """提供单客户端 TCP 服务，把 C++ 识别请求转换为检测器调用和文本响应。"""

    def __init__(self, detector, host: str, port: int, once: bool) -> None:
        """保存检测器、监听地址以及客户端断开后是否立即退出的配置。"""

        self.detector = detector
        self.host = host
        self.port = port
        self.once = once

    def _stop_requested(self) -> bool:
        """统一查询检测器退出标志；MOCK 检测器始终返回 False。"""

        return bool(getattr(self.detector, "stop_requested", False))

    def _receive_line(self, client: socket.socket,
                      pending: bytes) -> Tuple[Optional[str], bytes]:
        """用短超时读取一行协议文本，使窗口关闭后能及时打断 Socket 等待。"""

        while not self._stop_requested():
            newline_index = pending.find(b"\n")
            if newline_index >= 0:
                raw_line = pending[:newline_index]
                remaining = pending[newline_index + 1:]
                return raw_line.rstrip(b"\r").decode("ascii"), remaining
            if len(pending) > MAX_PROTOCOL_LINE_BYTES:
                raise ValueError("PROTOCOL_LINE_TOO_LONG")
            try:
                chunk = client.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                return None, b""
            pending += chunk
        return None, pending

    @staticmethod
    def _send_line(client: socket.socket, line: str) -> None:
        """发送一条完整 ASCII 协议消息。"""

        client.sendall(line.encode("ascii"))

    def _handle_client(self, client: socket.socket, address) -> None:
        """完成握手并逐条处理 DETECT；异常也返回带请求编号的 ERROR 响应。"""

        # 短超时只用于轮询退出标志，不是客户端空闲超时；机械臂仍可长时间保持连接。
        client.settimeout(SOCKET_POLL_SECONDS)
        pending = b""
        try:
            hello, pending = self._receive_line(client, pending)
            if hello is None:
                return
            if hello.strip() != "HELLO VISION_PICK 1":
                self._send_line(client, "ERROR 0 BAD_HANDSHAKE\n")
                return
            self._send_line(
                client,
                "READY {0} {1}\n".format(PROTOCOL_VERSION, self.detector.mode),
            )

            while not self._stop_requested():
                line, pending = self._receive_line(client, pending)
                if line is None:
                    return
                request_id = 0
                try:
                    (request_id, class_filter, selection,
                     flange_pose) = parse_detect_request(line)
                    # 当前单轮流程的 request=2 是二次识别；仅它允许连续漏检两帧。
                    allowed_missing_frames = (
                        SECOND_DETECTION_MAX_MISSING_FRAMES
                        if request_id == 2 else 0
                    )
                    target, reason = self.detector.detect(
                        class_filter, selection, allowed_missing_frames,
                        flange_pose=flange_pose,
                    )
                    # 用户关闭预览时不再发送识别结果，直接结束本次连接和服务。
                    if self._stop_requested():
                        return
                    if target is None:
                        response = "NOT_FOUND {0} {1}\n".format(
                            request_id, sanitize_reason(reason))
                    else:
                        response = format_target(request_id, target)
                    self._send_line(client, response)
                except Exception as error:  # Keep the protocol synchronized.
                    if self._stop_requested():
                        return
                    self._send_line(
                        client,
                        "ERROR {0} {1}\n".format(
                            request_id, sanitize_reason(error)),
                    )
        except (ConnectionError, OSError) as error:
            # C++ 正常退出、超时退出或被终止时只结束本次会话，服务器继续监听。
            if not self._stop_requested():
                print(
                    "vision_pick connection closed: {0}, reason={1}".format(
                        address, sanitize_reason(error)
                    ),
                    flush=True,
                )
        finally:
            print("vision_pick disconnected: {0}".format(address), flush=True)

    def serve_forever(self) -> None:
        """监听 TCP 端口并顺序服务客户端；--once 模式服务一个客户端后退出。"""

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.host, self.port))
            server.listen(1)
            print(
                "Vision server READY on {0}:{1}, mode={2}".format(
                    self.host, self.port, self.detector.mode
                ),
                flush=True,
            )
            # accept() 也采用短轮询，保证没有 C++ 客户端时关闭预览仍能退出。
            server.settimeout(SOCKET_POLL_SECONDS)
            while not self._stop_requested():
                try:
                    client, address = server.accept()
                except socket.timeout:
                    continue
                print("vision_pick connected: {0}".format(address), flush=True)
                with client:
                    self._handle_client(client, address)
                if self.once or self._stop_requested():
                    return


def resolve_model_path(requested: Optional[str]) -> Path:
    """按命令行、环境变量、项目模型、外部旧项目模型的顺序确定权重路径。"""

    if requested:
        return Path(requested).expanduser().resolve()
    environment_path = os.environ.get("VISION_MODEL_PATH")
    if environment_path:
        return Path(environment_path).expanduser().resolve()
    for candidate in DEFAULT_MODEL_CANDIDATES:
        if candidate.is_file():
            return candidate.resolve()
    return DEFAULT_MODEL_CANDIDATES[0].resolve()


def run_self_test() -> None:
    """不加载模型和相机，检查最近目标选择、请求解析和响应格式。"""

    candidates = [
        Candidate(1, 1000.0, 0, 0.95, 320, 240,
                  (0.0, 0.0, 500.0), (300, 220, 340, 260), (640, 480)),
        Candidate(1, 1000.0, 0, 0.70, 200, 200,
                  (20.0, 10.0, 300.0), (180, 180, 220, 220), (640, 480)),
        Candidate(1, 1000.0, 1, 0.99, 310, 230,
                  (0.0, 0.0, 200.0), (290, 210, 330, 250), (640, 480)),
    ]
    closest = choose_candidate(candidates, 0, "CLOSEST")
    assert closest is candidates[1]
    assert choose_candidate(candidates, 0, "BEST") is candidates[0]
    stable_average = average_candidates(candidates[:2])
    assert stable_average.camera_mm == (10.0, 5.0, 400.0)
    assert is_same_stable_target(candidates[0], candidates[1]) is False
    z_jump = Candidate(
        2, 1100.0, 0, 0.90, 320, 240,
        (0.0, 0.0, 560.1), (300, 220, 340, 260), (640, 480), 20,
    )
    center_jump = Candidate(
        2, 1100.0, 0, 0.90, 356, 240,
        (0.0, 0.0, 500.0), (336, 220, 376, 260), (640, 480), 18,
    )
    assert stability_transition_reason(candidates[0], z_jump) == "Z_JUMP"
    assert stability_transition_reason(candidates[0], center_jump) == "CENTER_JUMP"
    assert may_bridge_missing_frame(2, 1, 2)
    assert may_bridge_missing_frame(2, 2, 2)
    assert not may_bridge_missing_frame(2, 3, 2)
    assert not may_bridge_missing_frame(0, 1, 2)
    assert parse_detect_request("DETECT 7 -1 CLOSEST") == (
        7, -1, "CLOSEST", None
    )
    flange_pose = (0.1, 0.2, 0.3, 0.0, 0.0, 0.0)
    assert parse_detect_request(
        "DETECT 7 -1 CLOSEST 0.1 0.2 0.3 0.0 0.0 0.0"
    ) == (7, -1, "CLOSEST", flange_pose)
    transformed = camera_target_to_base_mm((0.0, 0.0, 100.0), flange_pose)
    assert transformed is not None
    assert all(math.isfinite(value) for value in transformed)
    assert format_target(7, closest).startswith(
        "TARGET 7 1 1000.000 0 0.700000"
    )
    print("SELF_TEST_OK")


def build_argument_parser() -> argparse.ArgumentParser:
    """定义视觉服务的监听地址、模型、推理设备、MOCK 和自测等命令行参数。"""

    parser = argparse.ArgumentParser(
        description="YOLOv8 + RealSense request server for ROKAE vision_pick"
    )
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--model", help="Path to YOLO .pt/.onnx model")
    parser.add_argument("--imgsz", type=int, default=DEFAULT_IMAGE_SIZE)
    parser.add_argument("--confidence", type=float, default=DEFAULT_CONFIDENCE)
    parser.add_argument("--fresh-frames", type=int, default=DEFAULT_FRESH_FRAMES)
    parser.add_argument("--target-class", type=int, default=DEFAULT_TARGET_CLASS,
                        help="Only this class may produce robot coordinates")
    parser.add_argument("--stable-seconds", type=float,
                        default=DEFAULT_STABLE_SECONDS)
    parser.add_argument("--stable-min-frames", type=int,
                        default=DEFAULT_STABLE_MIN_FRAMES)
    parser.add_argument("--stability-timeout", type=float,
                        default=DEFAULT_STABILITY_TIMEOUT_SECONDS)
    parser.add_argument("--device", help="Ultralytics device, e.g. cpu or 0")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--once", action="store_true",
                        help="Exit after one client disconnects")
    parser.add_argument("--mock", action="store_true",
                        help="Use deterministic coordinates without camera/model")
    parser.add_argument("--mock-point", type=float, nargs=3,
                        default=(10.0, -5.0, 350.0),
                        metavar=("X_MM", "Y_MM", "Z_MM"))
    parser.add_argument("--self-test", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """解析参数，创建模拟或真实检测器，启动服务并在退出时释放相机资源。"""

    args = build_argument_parser().parse_args(argv)
    if args.self_test:
        run_self_test()
        return 0
    if args.port <= 0 or args.port > 65535:
        raise ValueError("port must be in 1..65535")
    if not 0.0 < args.confidence <= 1.0:
        raise ValueError("confidence must be in (0, 1]")
    if args.target_class < 0:
        raise ValueError("target-class must be >= 0")
    if args.stable_seconds <= 0.0 or args.stable_min_frames < 2:
        raise ValueError("stability requirements must be positive")
    if args.stability_timeout <= args.stable_seconds:
        raise ValueError("stability-timeout must exceed stable-seconds")

    if args.mock:
        detector = MockDetector(args.mock_point)
    else:
        model_path = resolve_model_path(args.model)
        print("Loading model: {0}".format(model_path), flush=True)
        detector = RealSenseYoloDetector(
            model_path=model_path,
            image_size=args.imgsz,
            confidence=args.confidence,
            fresh_frames=args.fresh_frames,
            device=args.device,
            show=args.show,
            target_class=args.target_class,
            stable_seconds=args.stable_seconds,
            stable_min_frames=args.stable_min_frames,
            stability_timeout=args.stability_timeout,
        )

    try:
        VisionProtocolServer(detector, args.host, args.port, args.once).serve_forever()
    except KeyboardInterrupt:
        print("Stopping vision server.", flush=True)
    finally:
        detector.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
