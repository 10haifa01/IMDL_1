import math
import struct
import time
import rclpy
from rclpy.duration import Duration
from builtin_interfaces.msg import Time
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from explore_lite_msgs.msg import ExploreStatus
from tf2_ros import Buffer, TransformListener, TransformException
from robot_brain.task_mission_base import TaskMissionBase
from robot_brain.blind_spot_detector import find_unclosed_blind_spots


class Task5MultiColorNode(TaskMissionBase):
    def __init__(self):
        super().__init__('task5_node')

        self.cube_memory: dict[str, PoseStamped] = {}
        self.collected_colors: set[str] = set()
        self.active_target_color: str | None = None
        self.clearing_points: list[tuple[float, float, float]] = []
        self.clearing_until: float = 0.0
        self.exploration_complete = False

        self.explore_resume_pub = self.create_publisher(Bool, '/explore/resume', 10)
        self.cube_cloud_pub = self.create_publisher(
            PointCloud2, '/detected_cubes_obstacles', 10
        )

        self.create_subscription(
            ExploreStatus, '/explore/status', self._explore_status_cb, 10
        )

        for color in ('red', 'green', 'blue'):
            self.create_subscription(
                PoseStamped,
                f'/detected_cube/{color}',
                lambda msg, c=color: self._cube_cb(c, msg),
                10,
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.2, self._publish_cube_obstacles)

    def _explore_status_cb(self, msg: ExploreStatus):
        if msg.status == ExploreStatus.EXPLORATION_COMPLETE:
            self.exploration_complete = True

    def _cube_cb(self, color: str, msg: PoseStamped):
        if color in self.collected_colors:
            return

        is_new = color not in self.cube_memory
        self.cube_memory[color] = msg

        if is_new:
            ax, ay = self.transform.map_to_arena_xy(
                msg.pose.position.x, msg.pose.position.y
            )
            turn_num = (
                self.color_order.index(color) + 1 if color in self.color_order else -1
            )
            self.get_logger().info(
                f'[MEMORY] Saved {color.upper()} cube for turn #{turn_num}: '
                f'arena=({ax:.2f}, {ay:.2f}) | '
                f'map=({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
            )

    def _get_robot_xy(self) -> tuple[float, float]:
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except TransformException:
            return 0.0, 0.0

    def _publish_cube_obstacles(self):
        """
        Publishes saved, uncollected non-target cubes as single 3D points (z=0.05m)
        when outside the 0.175m chassis radius so Nav2 steers around them immediately,
        and stamps with Time(0, 0) so local_costmap never drops messages on TF lag.
        """
        rx, ry = self._get_robot_xy()
        points: list[tuple[float, float, float]] = []

        for color, pose in self.cube_memory.items():
            if color in self.collected_colors or color == self.active_target_color:
                continue
            cx = pose.pose.position.x
            cy = pose.pose.position.y
            if math.hypot(cx - rx, cy - ry) >= 0.175:
                points.append((cx, cy, 0.05))

        if time.monotonic() < self.clearing_until:
            points.extend(self.clearing_points)

        msg = PointCloud2()
        msg.header.frame_id = 'map'
        msg.header.stamp = Time(sec=0, nanosec=0)
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True

        buf = bytearray()
        for px, py, pz in points:
            buf.extend(struct.pack('<fff', float(px), float(py), float(pz)))
        msg.data = bytes(buf)
        self.cube_cloud_pub.publish(msg)

    def _clear_cube_from_costmap(self, cx: float, cy: float):
        """Sends a ray past (cx, cy) at z=2.5m so ObstacleLayer clears the target cube."""
        rx, ry = self._get_robot_xy()
        dx = cx - rx
        dy = cy - ry
        dist = math.hypot(dx, dy)
        if dist > 1e-3:
            ux, uy = dx / dist, dy / dist
            end_x = cx + 0.15 * ux
            end_y = cy + 0.15 * uy
        else:
            end_x, end_y = cx + 0.15, cy

        self.clearing_points = [
            (end_x, end_y, 2.5),
            (end_x + 0.03, end_y, 2.5),
            (end_x - 0.03, end_y, 2.5),
            (end_x, end_y + 0.03, 2.5),
            (end_x, end_y - 0.03, 2.5),
        ]
        self.clearing_until = time.monotonic() + 1.0
        for _ in range(4):
            self._publish_cube_obstacles()
            rclpy.spin_once(self, timeout_sec=0.1)

    def _set_exploration_active(self, active: bool):
        msg = Bool()
        msg.data = active
        self.explore_resume_pub.publish(msg)
        if not active:
            self.navigator.cancelTask()

    def _acquire_cube(self, target_color: str) -> bool:
        """
        Uses saved location immediately if already spotted earlier; otherwise explores
        and sweeps unclosed obstacle blind spots until target_color is found.
        """
        if target_color in self.cube_memory:
            pose = self.cube_memory[target_color]
            ax, ay = self.transform.map_to_arena_xy(
                pose.pose.position.x, pose.pose.position.y
            )
            self.get_logger().info(
                f'[MEMORY HIT] Using saved {target_color.upper()} cube at '
                f'arena=({ax:.2f}, {ay:.2f}) — skipping exploration.'
            )
            return True

        self.get_logger().info(f'Searching arena for {target_color.upper()} cube...')
        if not self.exploration_complete:
            self._set_exploration_active(True)
            while (
                rclpy.ok()
                and target_color not in self.cube_memory
                and not self.exploration_complete
            ):
                rclpy.spin_once(self, timeout_sec=0.1)

        if target_color in self.cube_memory:
            return True

        # Sweep unclosed obstacle blind spots if explore_lite finished first
        self._set_exploration_active(False)
        waypoints = find_unclosed_blind_spots(
            self.latest_map,
            self.costmap,
            min_cluster_cells=self.blind_spot_min_cells,
            min_standoff=self.blind_spot_standoff,
        )
        for wp in waypoints:
            if target_color in self.cube_memory or not rclpy.ok():
                break
            wp.header.stamp = self.navigator.get_clock().now().to_msg()
            self.navigator.goToPose(wp)
            while not self.navigator.isTaskComplete() and rclpy.ok():
                if target_color in self.cube_memory:
                    self.navigator.cancelTask()
                    break
                rclpy.spin_once(self, timeout_sec=0.1)

            if target_color not in self.cube_memory:
                for deg in (35.0, -70.0):
                    if target_color in self.cube_memory or not rclpy.ok():
                        break
                    self.navigator.spin(spin_dist=math.radians(deg))
                    while not self.navigator.isTaskComplete() and rclpy.ok():
                        if target_color in self.cube_memory:
                            self.navigator.cancelTask()
                            break
                        rclpy.spin_once(self, timeout_sec=0.1)

        return target_color in self.cube_memory

    def run(self):
        self.wait_for_ready()
        self.get_logger().info(
            f'=== TASK 5 START: Ordered Pick & Place {self.color_order} ==='
        )

        pose_c_center = self.arena_pose_to_map(self.point_c)
        drop_offsets = [(-0.012, 0.0), (0.0, 0.0), (0.012, 0.0)]

        for idx, color in enumerate(self.color_order):
            if not rclpy.ok():
                break

            if not self._acquire_cube(color):
                self.get_logger().error(
                    f'Task 5 aborted: could not locate {color.upper()} cube.'
                )
                return

            self._set_exploration_active(False)
            self.active_target_color = color
            cube_pose = self.cube_memory[color]
            self._clear_cube_from_costmap(
                cube_pose.pose.position.x, cube_pose.pose.position.y
            )

            rx, ry = self._get_robot_xy()
            standoff_candidates = self.compute_standoff_candidates(
                target_map_x=cube_pose.pose.position.x,
                target_map_y=cube_pose.pose.position.y,
                approach_from_map_x=rx,
                approach_from_map_y=ry,
            )

            reached_cube = False
            reached_pose = standoff_candidates[0]
            for attempt, candidate in enumerate(standoff_candidates):
                if self.go_to_pose(
                    candidate,
                    label=f'{color.upper()} Cube Standoff (option {attempt + 1})',
                ):
                    reached_cube = True
                    reached_pose = candidate
                    break

            if not reached_cube:
                self.get_logger().error(
                    f'Task 5 aborted: could not reach {color.upper()} cube.'
                )
                return

            self.simulated_pick(f'{color.upper()} Cube')
            self.collected_colors.add(color)
            self.active_target_color = None

            ox, oy = drop_offsets[idx % len(drop_offsets)]
            target_cx = pose_c_center.pose.position.x + ox
            target_cy = pose_c_center.pose.position.y + oy

            standoff_c = self.compute_standoff_pose(
                target_map_x=target_cx,
                target_map_y=target_cy,
                approach_from_map_x=reached_pose.pose.position.x,
                approach_from_map_y=reached_pose.pose.position.y,
            )
            if not self.go_to_pose(standoff_c, label=f'Drop Standoff C ({color.upper()})'):
                self.get_logger().error(
                    f'Task 5 aborted: could not deliver {color.upper()} cube to C.'
                )
                return

            self.simulated_place(f'{color.upper()} Cube at C (slot {idx + 1})')

        self.get_logger().info('=== TASK 5 COMPLETE ===')


def main(args=None):
    rclpy.init(args=args)
    node = Task5MultiColorNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()