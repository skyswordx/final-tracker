#!/usr/bin/env python3
# 20250220 Wakkk Modified for ROS Noetic
# ROS → AirSim bridge that runs via: python airsim_ros_bridge.py
# Zero arguments, safe RPC loop (no "IOLoop is already running").

import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import numpy as np

import airsim
import rospy
import yaml
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray

# ---------------- Hard-coded defaults (no CLI args) ----------------
DEFAULT_VEHICLE_NAME = ""         # '' = default active vehicle in AirSim
IGNORE_COLLISION = True
AIRSIM_IP = "192.168.1.121"
AIRSIM_PORT = 41451
AIRSIM_TIMEOUT = 10
AIRSIM_CONNECT_ATTEMPTS = 5
AIRSIM_RETRY_DELAY_SEC = 2.0
RPC_RATE_HZ = 20.0                # AirSim RPC update rate
OBJECT_CHUNK_SIZE = 7             # MuJoCo free joint → 7 DoF chunk per object
DEFAULT_QPOS_OFFSET = 0           # 默认消息从额外物体开始
DEFAULT_OBJECT_TOPIC = "/quad/object_poses"  # Float64MultiArray publishing object chunks
DEFAULT_OBJECT_MAP = "airsim_object_map.yaml"
DEFAULT_ODOM_TOPIC = "/diy_odom"
DEFAULT_RGB_TOPIC = "/diy_img/rgb/image_raw"
DEFAULT_DEPTH_TOPIC = "/diy_img/depth/image_raw"
DEFAULT_CAMERA_INFO_TOPIC = "/diy_img/camera_info"
DEFAULT_CAMERA_FRAME_ID = "airsim/camera"
DEFAULT_CAMERA_NAME = "0"
CAMERA_RATE_HZ = 10.0
DEPTH_CLIP_MAX = 100.0

ROTATE_WORLD_Z_DEG = 90.0
ROTATE_WORLD_Z_RAD = math.radians(ROTATE_WORLD_Z_DEG)
ROTATE_WORLD_Z_COS = math.cos(ROTATE_WORLD_Z_RAD)
ROTATE_WORLD_Z_SIN = math.sin(ROTATE_WORLD_Z_RAD)
ROTATE_WORLD_Z_QUAT = (math.cos(ROTATE_WORLD_Z_RAD / 2.0), 0.0, 0.0, math.sin(ROTATE_WORLD_Z_RAD / 2.0))

@dataclass(frozen=True)
class SceneObjectSpec:
    """Mapping between MuJoCo qpos chunk and an Unreal (AirSim) object."""

    name: str
    airsim_name: str
    qpos_index: int  # 0-based index w.r.t objects after the configured offset
    teleport: bool = True



def _rotate_pos_z(x: float, y: float):
    x_new = x * ROTATE_WORLD_Z_COS - y * ROTATE_WORLD_Z_SIN
    y_new = x * ROTATE_WORLD_Z_SIN + y * ROTATE_WORLD_Z_COS
    return x_new, y_new


def _quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)

def ros_quat_to_airsim(q: Quaternion) -> airsim.Quaternionr:
    """ROS geometry_msgs/Quaternion (x,y,z,w) -> AirSim Quaternionr (w,x,y,z)."""
    return airsim.Quaternionr(q.x, -q.y, -q.z, q.w)


def normalize_quaternion(qw: float, qx: float, qy: float, qz: float):
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm < 1e-9:
        return 1.0, 0.0, 0.0, 0.0
    return qw / norm, qx / norm, qy / norm, qz / norm


def chunk_to_pose(chunk: List[float]) -> Optional[airsim.Pose]:
    """Convert MuJoCo free-joint chunk [x,y,z,qw,qx,qy,qz] into an AirSim pose."""
    if len(chunk) != OBJECT_CHUNK_SIZE:
        return None
    x, y, z, qw, qx, qy, qz = chunk
    if abs(ROTATE_WORLD_Z_RAD) > 1e-9:
        x, y = _rotate_pos_z(x, y)
        qw, qx, qy, qz = _quat_multiply(ROTATE_WORLD_Z_QUAT, (qw, qx, qy, qz))
    qw, qx, qy, qz = normalize_quaternion(qw, qx, qy, qz)
    pos = airsim.Vector3r(x, -y, -z)  # MuJoCo world → AirSim NED (flip Y, Z)
    quat_ros = Quaternion(x=qx, y=qy, z=qz, w=qw)
    quat_airsim = ros_quat_to_airsim(quat_ros)
    return airsim.Pose(pos, quat_airsim)



class AirSimRosBridge:
    def __init__(self):
        # --- ROS init ---
        rospy.init_node("airsim_ros_bridge", anonymous=False)

        # --- ROS parameters ---
        self.vehicle_name = rospy.get_param("~vehicle_name", DEFAULT_VEHICLE_NAME)
        self.odom_topic = rospy.get_param("~odom_topic", DEFAULT_ODOM_TOPIC)
        self.rgb_topic = rospy.get_param("~rgb_topic", DEFAULT_RGB_TOPIC)
        self.depth_topic = rospy.get_param("~depth_topic", DEFAULT_DEPTH_TOPIC)
        self.camera_info_topic = rospy.get_param("~camera_info_topic", DEFAULT_CAMERA_INFO_TOPIC)
        self.camera_name = rospy.get_param("~camera_name", DEFAULT_CAMERA_NAME)
        self.camera_frame_id = rospy.get_param("~camera_frame_id", DEFAULT_CAMERA_FRAME_ID)
        self.camera_rate_hz = float(rospy.get_param("~camera_rate_hz", CAMERA_RATE_HZ))
        self.depth_clip = float(rospy.get_param("~depth_clip_max", DEPTH_CLIP_MAX))

        # --- Config for movable objects ---
        map_param = rospy.get_param("~object_map_config", DEFAULT_OBJECT_MAP)
        candidate_path = Path(map_param)
        if not candidate_path.is_absolute():
            candidate_path = Path(__file__).resolve().parent / candidate_path
        self.object_map_path = candidate_path
        self.object_qpos_offset = int(rospy.get_param("~object_qpos_offset", DEFAULT_QPOS_OFFSET))
        if self.object_qpos_offset < 0:
            rospy.logwarn("[AirSimBridge] ~object_qpos_offset 为负值，已重置为 0")
            self.object_qpos_offset = 0
        self.object_state_topic = rospy.get_param("~object_state_topic", DEFAULT_OBJECT_TOPIC)
        self.scene_objects: List[SceneObjectSpec] = self._load_object_mapping(self.object_map_path)

        # --- AirSim client (single instance, thread-unsafe -> we serialize calls) ---
        self.client = self._connect_to_airsim()
        self.client.enableApiControl(True)
        print("Connected to AirSim.")
        print("Client Ver:1 (Min Req: 1), Server Ver:%s (Min Req: %s)" %
              (self.client.getServerVersion(), self.client.getMinRequiredServerVersion()))

        # 初始化后先把速度清零一次，避免残留速度
        try:
            self.client.moveByVelocityAsync(
                0.0, 0.0, 0.0,              # vx, vy, vz
                0.1,                        # duration (s)
                airsim.DrivetrainType.MaxDegreeOfFreedom,
                airsim.YawMode(False, 0.0),
                self.vehicle_name
            ).join()
        except Exception as ex:
            rospy.logwarn("[AirSimBridge] initial moveByVelocityAsync(0,0,0) failed: %s", ex)

        # Latest data caches
        self._latest_odom: Optional[Odometry] = None
        self._latest_object_data: Optional[List[float]] = None
        self._object_remainder_warned = False
        self._camera_info_msg: Optional[CameraInfo] = None

        # Locks
        self._state_lock = threading.Lock()  # protect latest data
        self._rpc_lock = threading.Lock()    # serialize RPC

        # Publishers
        self.rgb_pub = rospy.Publisher(self.rgb_topic, Image, queue_size=1)
        self.depth_pub = rospy.Publisher(self.depth_topic, Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher(self.camera_info_topic, CameraInfo, queue_size=1)

        # Subscribers (callbacks only stash data, no RPC here)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._cb_odom, queue_size=20)
        self.object_sub = None
        if self.scene_objects and self.object_state_topic:
            self.object_sub = rospy.Subscriber(
                self.object_state_topic,
                Float64MultiArray,
                self._cb_object_state,
                queue_size=1
            )
        elif self.scene_objects:
            rospy.logwarn("[AirSimBridge] Scene objects configured but ~object_state_topic is empty.")

        if self.scene_objects:
            names = ", ".join(f"{spec.name}->{spec.airsim_name}" for spec in self.scene_objects)
            rospy.loginfo("[AirSimBridge] Loaded %d object mappings: %s", len(self.scene_objects), names)
        else:
            rospy.loginfo("[AirSimBridge] No movable object mappings loaded.")

        # Timer loop to perform RPC
        self.rpc_timer = rospy.Timer(rospy.Duration(1.0 / RPC_RATE_HZ), self._rpc_tick, oneshot=False)
        self.camera_timer = None
        if self.camera_rate_hz > 0.0:
            self.camera_timer = rospy.Timer(
                rospy.Duration(max(1.0 / self.camera_rate_hz, 1e-3)),
                self._camera_tick,
                oneshot=False,
            )
        rospy.loginfo("[AirSimBridge] Ready. vehicle='%s', ignore_collision=%s, rate=%.1f Hz",
                      self.vehicle_name, str(IGNORE_COLLISION), RPC_RATE_HZ)

    # ---------------- Config helpers ----------------
    def _load_object_mapping(self, config_path: Path) -> List[SceneObjectSpec]:
        if not config_path.exists():
            rospy.logwarn("[AirSimBridge] Object map file not found: %s", str(config_path))
            return []
        try:
            with config_path.open("r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as ex:
            rospy.logwarn("[AirSimBridge] Failed to read object map '%s': %s", str(config_path), ex)
            return []

        objects_cfg = data.get("objects", [])
        specs: List[SceneObjectSpec] = []
        for idx, entry in enumerate(objects_cfg):
            try:
                name = entry["name"]
                airsim_name = entry.get("airsim_name", name)
                qpos_index = int(entry.get("qpos_index", idx))
                teleport = bool(entry.get("teleport", True))
                specs.append(SceneObjectSpec(name=name,
                                              airsim_name=airsim_name,
                                              qpos_index=qpos_index,
                                              teleport=teleport))
            except KeyError as missing:
                rospy.logwarn("[AirSimBridge] Object config missing field %s: %s", missing, entry)
            except Exception as ex:
                rospy.logwarn("[AirSimBridge] Invalid object config entry %s: %s", entry, ex)
        return specs

    def _connect_to_airsim(self) -> airsim.MultirotorClient:
        last_error: Optional[Exception] = None
        for attempt in range(1, AIRSIM_CONNECT_ATTEMPTS + 1):
            client = airsim.MultirotorClient(
                ip=AIRSIM_IP,
                port=AIRSIM_PORT,
                timeout_value=AIRSIM_TIMEOUT,
            )
            try:
                client.confirmConnection()
                rospy.loginfo("[AirSimBridge] Connected to AirSim at %s:%s on attempt %d",
                              AIRSIM_IP, AIRSIM_PORT, attempt)
                return client
            except Exception as ex:
                last_error = ex
                rospy.logwarn("[AirSimBridge] Failed to connect to AirSim (attempt %d/%d): %s",
                              attempt, AIRSIM_CONNECT_ATTEMPTS, ex)
                if attempt < AIRSIM_CONNECT_ATTEMPTS:
                    time.sleep(AIRSIM_RETRY_DELAY_SEC)
        rospy.logfatal("[AirSimBridge] Unable to connect to AirSim at %s:%s after %d attempts.",
                       AIRSIM_IP, AIRSIM_PORT, AIRSIM_CONNECT_ATTEMPTS)
        raise RuntimeError(f"Unable to connect to AirSim at {AIRSIM_IP}:{AIRSIM_PORT}") from last_error

    # ---------------- Callbacks: only cache data ----------------
    def _cb_odom(self, msg: Odometry):
        with self._state_lock:
            self._latest_odom = msg

    def _cb_object_state(self, msg: Float64MultiArray):
        if not msg.data:
            return
        with self._state_lock:
            self._latest_object_data = list(msg.data)

    # ---------------- Timer: safe serialized RPC calls ----------------
    def _rpc_tick(self, _evt):
        # Snapshot latest data
        with self._state_lock:
            odom = self._latest_odom
            object_data = list(self._latest_object_data) if self._latest_object_data is not None else None

        # 1) Pose update to AirSim
        if odom is not None:
            try:
                pos = odom.pose.pose.position
                ori = odom.pose.pose.orientation

                if abs(ROTATE_WORLD_Z_RAD) > 1e-9:
                    x_rot, y_rot = _rotate_pos_z(pos.x, pos.y)
                    rot_w, rot_x, rot_y, rot_z = _quat_multiply(ROTATE_WORLD_Z_QUAT, (ori.w, ori.x, ori.y, ori.z))
                else:
                    x_rot, y_rot = pos.x, pos.y
                    rot_w, rot_x, rot_y, rot_z = ori.w, ori.x, ori.y, ori.z

                airsim_pos = airsim.Vector3r(x_rot, -y_rot, -pos.z - 0.1)         # flip Y,Z for NED
                rotated_ori = Quaternion(x=rot_x, y=rot_y, z=rot_z, w=rot_w)
                airsim_quat = ros_quat_to_airsim(rotated_ori)               # 四元数直接映射 (w,x,y,z)

                pose = airsim.Pose(airsim_pos, airsim_quat)
                with self._rpc_lock:
                    self.client.simSetVehiclePose(pose, IGNORE_COLLISION, self.vehicle_name)
            except Exception as ex:
                rospy.logwarn_throttle(2.0, "[AirSimBridge] simSetVehiclePose failed: %s", ex)
        # 2) Update movable scene objects in UE
        if self.scene_objects and object_data is not None:
            self._sync_scene_objects(object_data)

    def _camera_tick(self, _evt):
        if (self.rgb_pub.get_num_connections() == 0 and
                self.depth_pub.get_num_connections() == 0 and
                self.camera_info_pub.get_num_connections() == 0):
            return

        try:
            with self._rpc_lock:
                responses = self.client.simGetImages(
                    [
                        airsim.ImageRequest(
                            self.camera_name,
                            airsim.ImageType.Scene,
                            pixels_as_float=False,
                            compress=False,
                        ),
                        airsim.ImageRequest(
                            self.camera_name,
                            airsim.ImageType.DepthPerspective,
                            pixels_as_float=True,
                            compress=False,
                        ),
                    ],
                    vehicle_name=self.vehicle_name,
                )
        except Exception as ex:
            rospy.logwarn_throttle(5.0, "[AirSimBridge] simGetImages failed: %s", ex)
            return

        if not responses or len(responses) < 2:
            rospy.logwarn_throttle(5.0,
                                   "[AirSimBridge] simGetImages returned insufficient responses (%d)",
                                   len(responses) if responses else 0)
            return

        stamp = rospy.Time.now()
        scene_response, depth_response = responses[0], responses[1]
        info_width = 0
        info_height = 0

        if scene_response.width > 0 and scene_response.height > 0 and scene_response.image_data_uint8:
            try:
                rgb = np.frombuffer(scene_response.image_data_uint8, dtype=np.uint8)
                rgb = rgb.reshape((scene_response.height, scene_response.width, 3))
            except ValueError as ex:
                rospy.logwarn_throttle(5.0, "[AirSimBridge] reshape RGB image failed: %s", ex)
            else:
                info_width = scene_response.width
                info_height = scene_response.height
                rgb_msg = Image()
                rgb_msg.header.stamp = stamp
                rgb_msg.header.frame_id = self.camera_frame_id
                rgb_msg.height = scene_response.height
                rgb_msg.width = scene_response.width
                rgb_msg.encoding = "rgb8"
                rgb_msg.is_bigendian = False
                rgb_msg.step = scene_response.width * 3
                rgb_msg.data = rgb.tobytes()
                self.rgb_pub.publish(rgb_msg)

        if depth_response.width > 0 and depth_response.height > 0 and depth_response.image_data_float:
            depth_array = np.array(depth_response.image_data_float, dtype=np.float32)
            expected_size = depth_response.width * depth_response.height
            if depth_array.size != expected_size:
                rospy.logwarn_throttle(5.0,
                                       "[AirSimBridge] Depth buffer size mismatch: expected %d, got %d",
                                       expected_size,
                                       depth_array.size)
            else:
                if info_width == 0:
                    info_width = depth_response.width
                    info_height = depth_response.height
                depth_array = depth_array.reshape((depth_response.height, depth_response.width))
                depth_array = np.nan_to_num(depth_array, nan=self.depth_clip, posinf=self.depth_clip, neginf=0.0)
                if self.depth_clip > 0:
                    np.clip(depth_array, 0.0, self.depth_clip, out=depth_array)
                depth_msg = Image()
                depth_msg.header.stamp = stamp
                depth_msg.header.frame_id = self.camera_frame_id
                depth_msg.height = depth_response.height
                depth_msg.width = depth_response.width
                depth_msg.encoding = "32FC1"
                depth_msg.is_bigendian = False
                depth_msg.step = depth_response.width * 4
                depth_msg.data = depth_array.tobytes()
                self.depth_pub.publish(depth_msg)

        if self.camera_info_pub.get_num_connections() > 0 and info_width > 0 and info_height > 0:
            self._ensure_camera_info(info_width, info_height, stamp)

    def _ensure_camera_info(self, width: int, height: int, stamp: rospy.Time):
        if width <= 0 or height <= 0:
            return

        need_refresh = (
            self._camera_info_msg is None
            or self._camera_info_msg.width != width
            or self._camera_info_msg.height != height
        )

        if need_refresh:
            try:
                with self._rpc_lock:
                    cam_info = self.client.simGetCameraInfo(self.camera_name, self.vehicle_name)
            except Exception as ex:
                rospy.logwarn_throttle(10.0, "[AirSimBridge] simGetCameraInfo failed: %s", ex)
                return

            msg = CameraInfo()
            msg.width = width
            msg.height = height
            fov = cam_info.fov if cam_info.fov > 0 else 90.0
            fov_rad = math.radians(fov)
            try:
                fx = (width / 2.0) / math.tan(fov_rad / 2.0)
            except ZeroDivisionError:
                fx = width
            fy = fx
            cx = width / 2.0
            cy = height / 2.0

            msg.K = [fx, 0.0, cx,
                     0.0, fy, cy,
                     0.0, 0.0, 1.0]

            msg.P = [fx, 0.0, cx, 0.0,
                     0.0, fy, cy, 0.0,
                     0.0, 0.0, 1.0, 0.0]

            msg.R = [1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0]

            msg.distortion_model = "plumb_bob"
            msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]

            self._camera_info_msg = msg

        info = self._camera_info_msg
        if info is None:
            return
        info.header.stamp = stamp
        info.header.frame_id = self.camera_frame_id
        self.camera_info_pub.publish(info)

    def _sync_scene_objects(self, object_chunks: List[float]):
        usable = len(object_chunks) - self.object_qpos_offset
        if usable < OBJECT_CHUNK_SIZE:
            rospy.logwarn_throttle(5.0, "[AirSimBridge] object数据长度 %d 不足以构成一个完整物体", len(object_chunks))
            return
        remainder = usable % OBJECT_CHUNK_SIZE
        if remainder and not self._object_remainder_warned:
            rospy.logwarn("[AirSimBridge] object数据长度 %d 与 offset %d 不匹配，忽略末尾 %d 个元素",
                          len(object_chunks), self.object_qpos_offset, remainder)
            self._object_remainder_warned = True
        elif remainder == 0:
            self._object_remainder_warned = False
        count = usable // OBJECT_CHUNK_SIZE
        if count <= 0:
            return
        if len(self.scene_objects) != count:
            rospy.logwarn_throttle(5.0,
                                   "[AirSimBridge] AirSim 映射对象数量 %d 与实际可用物体数量 %d 不一致",
                                   len(self.scene_objects), count)
        for spec in self.scene_objects:
            if spec.qpos_index >= count:
                rospy.logwarn_throttle(5.0,
                                       "[AirSimBridge] 配置中的物体 '%s' 索引 %d 超出当前物体数量 %d",
                                       spec.name, spec.qpos_index, count)
                continue
            start = self.object_qpos_offset + spec.qpos_index * OBJECT_CHUNK_SIZE
            end = start + OBJECT_CHUNK_SIZE
            chunk = object_chunks[start:end]
            pose = chunk_to_pose(chunk)
            if pose is None:
                rospy.logwarn_throttle(5.0, "[AirSimBridge] Invalid chunk for object '%s': %s", spec.name, chunk)
                continue
            try:
                with self._rpc_lock:
                    ok = self.client.simSetObjectPose(spec.airsim_name, pose, spec.teleport)
                if not ok:
                    rospy.logwarn_throttle(2.0,
                                           "[AirSimBridge] simSetObjectPose returned False for '%s' (UE object '%s')",
                                           spec.name, spec.airsim_name)
            except Exception as ex:
                rospy.logwarn_throttle(2.0,
                                       "[AirSimBridge] simSetObjectPose failed for '%s' (%s): %s",
                                       spec.name, spec.airsim_name, ex)


def main():
    bridge = AirSimRosBridge()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
