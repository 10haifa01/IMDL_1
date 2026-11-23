import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from robot_brain.arena_transform import (
    ArenaTransform,
    normalize_angle,
    yaw_to_quaternion,
)


class TaskMissionBase(Node):
    """
    Shared base node for Tasks 1-5:
    - Loads A, B, C and task parameters from task_params.yaml
    - Transforms Arena poses into SLAM 'map' frame
    - Computes costmap-safe standoff poses for pick/place targets
    - Provides clean simulated pick/place hooks for future arm/camera integration
    """

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('point_a', [-0.48, 0.48, 0.0])
        self.declare_parameter('point_b', [0.48, -0.48, -1.5708])
        self.declare_parameter('point_c', [0.00, 0.50, 1.5708])
        self.declare_parameter('task5_color_order', ['red', 'green', 'blue'])
        self.declare_parameter('standoff_distance', 0.32)
        self.declare_parameter('simulated_action_duration', 2.0)
        self.declare_parameter('blind_spot_min_cells', 4)
        self.declare_parameter('blind_spot_standoff', 0.32)

        self.point_a = list(self.get_parameter('point_a').value)
        self.point_b = list(self.get_parameter('point_b').value)
        self.point_c = list(self.get_parameter('point_c').value)
        self.color_order = list(self.get_parameter('task5_color_order').value)
        self.standoff_dist = float(self.get_parameter('standoff_distance').value)
        self.sim_action_sec = float(self.get_parameter('simulated_action_duration').value)
        self.blind_spot_min_cells = int(self.get_parameter('blind_spot_min_cells').value)
        self.blind_spot_standoff = float(self.get_parameter('blind_spot_standoff').value)

        self.transform = ArenaTransform(*self.point_a)

        self.latest_map = None
        self.costmap = None

        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)
        costmap_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._costmap_cb, costmap_qos
        )

        use_sim_time = self.get_parameter('use_sim_time').value
        self.navigator = BasicNavigator()
        self.navigator.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)
        ])

        self.get_logger().info(
            f'Initialized {node_name} | Arena Start A={self.point_a}, '
            f'B={self.point_b}, C={self.point_c}'
        )

    def _map_cb(self, msg: OccupancyGrid):
        self.latest_map = msg

    def _costmap_cb(self, msg: OccupancyGrid):
        self.costmap = msg

    def wait_for_ready(self):
        """Waits until SLAM /map, global costmap, and Nav2 bt_navigator are active."""
        self.get_logger().info('Waiting for /map and /global_costmap/costmap...')
        while rclpy.ok() and (self.latest_map is None or self.costmap is None):
            rclpy.spin_once(self, timeout_sec=0.2)

        self.get_logger().info('Waiting for bt_navigator to become ACTIVE...')
        state_client = self.create_client(GetState, '/bt_navigator/get_state')
        while rclpy.ok() and not state_client.wait_for_service(timeout_sec=0.5):
            rclpy.spin_once(self, timeout_sec=0.1)

        while rclpy.ok():
            future = state_client.call_async(GetState.Request())
            while rclpy.ok() and not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
            if future.result() is not None and (
                future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
            ):
                break
            time.sleep(0.2)

        self.destroy_client(state_client)
        self.get_logger().info('SLAM and Nav2 are active and ready.')

    def arena_pose_to_map(self, arena_xyz_yaw: list[float]) -> PoseStamped:
        pose = self.transform.arena_to_map_pose(
            arena_xyz_yaw[0], arena_xyz_yaw[1], arena_xyz_yaw[2]
        )
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        return pose

    def compute_standoff_pose(
        self,
        target_map_x: float,
        target_map_y: float,
        approach_from_map_x: float,
        approach_from_map_y: float,
        standoff_dist: float | None = None,
    ) -> PoseStamped:
        """
        Computes a collision-free (cost == 0) pose at `standoff_dist` from a target
        center (e.g., object B or drop zone C), oriented directly toward the target.
        """
        dist = self.standoff_dist if standoff_dist is None else standoff_dist
        preferred_angle = math.atan2(
            approach_from_map_y - target_map_y,
            approach_from_map_x - target_map_x,
        )

        # Sample candidate approach angles around the target, prioritizing direct line of approach
        offsets = [0.0]
        for deg in range(15, 185, 15):
            offsets.extend([math.radians(deg), -math.radians(deg)])

        best_xy = None
        for offset in offsets:
            ang = normalize_angle(preferred_angle + offset)
            cx = target_map_x + dist * math.cos(ang)
            cy = target_map_y + dist * math.sin(ang)
            if self._is_costmap_free(cx, cy):
                best_xy = (cx, cy)
                break

        if best_xy is None:
            best_xy = (
                target_map_x + dist * math.cos(preferred_angle),
                target_map_y + dist * math.sin(preferred_angle),
            )

        gx, gy = best_xy
        face_yaw = math.atan2(target_map_y - gy, target_map_x - gx)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = gx
        pose.pose.position.y = gy
        pose.pose.orientation = yaw_to_quaternion(face_yaw)
        return pose

    def _is_costmap_free(self, map_x: float, map_y: float) -> bool:
        if self.costmap is None:
            return True
        cw = self.costmap.info.width
        ch = self.costmap.info.height
        cres = self.costmap.info.resolution
        cox = self.costmap.info.origin.position.x
        coy = self.costmap.info.origin.position.y

        mx = int((map_x - cox) / cres)
        my = int((map_y - coy) / cres)
        if not (0 <= mx < cw and 0 <= my < ch):
            return False
        return self.costmap.data[my * cw + mx] == 0

    def go_to_pose(self, goal: PoseStamped, label: str = 'goal') -> bool:
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        ax, ay = self.transform.map_to_arena_xy(
            goal.pose.position.x, goal.pose.position.y
        )
        self.get_logger().info(
            f'Navigating to {label}: map=({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}) '
            f'| arena=({ax:.2f}, {ay:.2f})'
        )
        self.navigator.goToPose(goal)

        while not self.navigator.isTaskComplete() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'Reached {label}.')
            return True
        self.get_logger().warn(f'Failed to reach {label} (result={result}).')
        return False

    def simulated_pick(self, item_label: str = 'object'):
        """Hook for future camera alignment + arm grasp action."""
        self.get_logger().info(
            f'[SIMULATED PICK] Grasping {item_label} (waiting {self.sim_action_sec:.1f}s)...'
        )
        end_t = time.monotonic() + self.sim_action_sec
        while rclpy.ok() and time.monotonic() < end_t:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'[SIMULATED PICK] {item_label} secured.')

    def simulated_place(self, item_label: str = 'object'):
        """Hook for future arm placement action at C."""
        self.get_logger().info(
            f'[SIMULATED PLACE] Placing {item_label} at C (waiting {self.sim_action_sec:.1f}s)...'
        )
        end_t = time.monotonic() + self.sim_action_sec
        while rclpy.ok() and time.monotonic() < end_t:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'[SIMULATED PLACE] {item_label} placed.')