import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
from robot_brain.arena_transform import (
    ArenaTransform,
    normalize_angle,
    quaternion_to_yaw,
    yaw_to_quaternion,
)


class VirtualCubeDetector(Node):
    """
    Simulates front-facing RGB-D cube detections in Gazebo without a camera.
    Checks distance, horizontal FOV cone, and /map raycast line-of-sight so
    obstacles realistically occlude cubes until the robot navigates around them.
    """

    def __init__(self):
        super().__init__('virtual_cube_detector')

        self.declare_parameter('task_mode', 'task4')
        self.declare_parameter('point_a', [-0.48, 0.48, 0.0])
        self.declare_parameter('virtual_camera_max_range', 0.55)
        self.declare_parameter('virtual_camera_min_range', 0.18)
        self.declare_parameter('virtual_camera_fov_deg', 70.0)
        self.declare_parameter('sim_cube_red', [0.50, 0.05])
        self.declare_parameter('sim_cube_green', [-0.15, -0.50])
        self.declare_parameter('sim_cube_blue', [-0.50, -0.02])

        self.task_mode = str(self.get_parameter('task_mode').value)
        point_a = list(self.get_parameter('point_a').value)
        self.max_range = float(self.get_parameter('virtual_camera_max_range').value)
        self.min_range = float(self.get_parameter('virtual_camera_min_range').value)
        self.half_fov = math.radians(
            float(self.get_parameter('virtual_camera_fov_deg').value) / 2.0
        )

        self.transform = ArenaTransform(*point_a)

        all_cubes = {
            'red': list(self.get_parameter('sim_cube_red').value),
            'green': list(self.get_parameter('sim_cube_green').value),
            'blue': list(self.get_parameter('sim_cube_blue').value),
        }

        if self.task_mode == 'task4':
            active_colors = ['red']
        else:
            active_colors = ['red', 'green', 'blue']

        self.cubes_map: dict[str, tuple[float, float]] = {}
        self.cube_pubs = {}
        for color in active_colors:
            ax, ay = all_cubes[color]
            mx, my = self.transform.arena_to_map_xy(ax, ay)
            self.cubes_map[color] = (mx, my)
            self.cube_pubs[color] = self.create_publisher(
                PoseStamped, f'/detected_cube/{color}', 10
            )
            self.get_logger().info(
                f'Virtual cube [{color}] placed at arena=({ax:.2f}, {ay:.2f}) '
                f'-> map=({mx:.2f}, {my:.2f})'
            )

        self.latest_map: OccupancyGrid | None = None
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)
        self.create_subscription(
            PointStamped, '/clicked_point', self._clicked_point_cb, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.announced: set[str] = set()
        self.create_timer(0.1, self._check_camera_cone)

    def _map_cb(self, msg: OccupancyGrid):
        self.latest_map = msg

    def _clicked_point_cb(self, msg: PointStamped):
        """Manual RViz Publish-Point override to inject a detection immediately."""
        for color in self.cube_pubs:
            if color not in self.announced:
                self.cubes_map[color] = (msg.point.x, msg.point.y)
                self._publish_detection(color, msg.point.x, msg.point.y)
                self.get_logger().info(
                    f'[MANUAL RVIZ INJECT] {color} cube at '
                    f'map=({msg.point.x:.2f}, {msg.point.y:.2f})'
                )
                break

    def _has_line_of_sight(
        self, rx: float, ry: float, cx: float, cy: float
    ) -> bool:
        """
        Raycasts through /map with a 0.10 m clearance buffer so pillars block
        detections when a sightline merely grazes the edge of an obstacle.
        """
        if self.latest_map is None:
            return False

        mw = self.latest_map.info.width
        mh = self.latest_map.info.height
        mres = self.latest_map.info.resolution
        mox = self.latest_map.info.origin.position.x
        moy = self.latest_map.info.origin.position.y
        mgrid = np.array(self.latest_map.data, dtype=np.int16).reshape((mh, mw))

        dist = math.hypot(cx - rx, cy - ry)
        steps = max(2, int(dist / (mres * 0.5)))
        pad_cells = max(1, int(round(0.10 / mres)))

        for i in range(1, steps):
            t = i / float(steps)
            px = rx + t * (cx - rx)
            py = ry + t * (cy - ry)
            gx = int((px - mox) / mres)
            gy = int((py - moy) / mres)
            if not (0 <= gx < mw and 0 <= gy < mh):
                return False

            x0 = max(0, gx - pad_cells)
            x1 = min(mw, gx + pad_cells + 1)
            y0 = max(0, gy - pad_cells)
            y1 = min(mh, gy + pad_cells + 1)
            if np.any(mgrid[y0:y1, x0:x1] > 50):
                return False

        return True

    def _publish_detection(self, color: str, cx: float, cy: float):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = cx
        pose.pose.position.y = cy
        pose.pose.position.z = 0.025
        pose.pose.orientation = yaw_to_quaternion(0.0)
        self.cube_pubs[color].publish(pose)

        if color not in self.announced:
            self.announced.add(color)
            self.get_logger().info(
                f'[VIRTUAL CAMERA] Spotted {color.upper()} cube at '
                f'map=({cx:.2f}, {cy:.2f})!'
            )

    def _check_camera_cone(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )
        except TransformException:
            return

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        ryaw = quaternion_to_yaw(tf.transform.rotation)

        for color, (cx, cy) in self.cubes_map.items():
            dx = cx - rx
            dy = cy - ry
            dist = math.hypot(dx, dy)
            if not (self.min_range <= dist <= self.max_range):
                continue

            bearing = normalize_angle(math.atan2(dy, dx) - ryaw)
            if abs(bearing) > self.half_fov:
                continue

            if self._has_line_of_sight(rx, ry, cx, cy):
                self._publish_detection(color, cx, cy)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualCubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()