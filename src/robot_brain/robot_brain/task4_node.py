import math
import rclpy
from rclpy.duration import Duration
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from explore_lite_msgs.msg import ExploreStatus
from tf2_ros import Buffer, TransformListener, TransformException
from robot_brain.task_mission_base import TaskMissionBase
from robot_brain.blind_spot_detector import find_unclosed_blind_spots


class Task4SearchPickNode(TaskMissionBase):
    def __init__(self):
        super().__init__('task4_node')

        self.detected_cube: PoseStamped | None = None
        self.exploration_complete = False

        self.explore_resume_pub = self.create_publisher(Bool, '/explore/resume', 10)
        self.create_subscription(
            ExploreStatus, '/explore/status', self._explore_status_cb, 10
        )
        self.create_subscription(
            PoseStamped, '/detected_cube/red', self._cube_cb, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _explore_status_cb(self, msg: ExploreStatus):
        if msg.status == ExploreStatus.EXPLORATION_COMPLETE:
            self.exploration_complete = True

    def _cube_cb(self, msg: PoseStamped):
        if self.detected_cube is None:
            self.detected_cube = msg
            self.get_logger().info(
                f'Target cube detected at map=({msg.pose.position.x:.2f}, '
                f'{msg.pose.position.y:.2f})! Interrupting search.'
            )

    def _get_robot_xy(self) -> tuple[float, float]:
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except TransformException:
            return 0.0, 0.0

    def _pause_exploration(self):
        msg = Bool()
        msg.data = False
        self.explore_resume_pub.publish(msg)
        self.navigator.cancelTask()

    def run(self):
        self.wait_for_ready()
        self.get_logger().info('=== TASK 4 START: Search -> Pick -> Place at C ===')

        # Phase 1: Wait while explore_lite searches until cube is spotted or frontiers finish
        while rclpy.ok() and self.detected_cube is None and not self.exploration_complete:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Phase 2: If explore_lite finished without seeing cube, sweep unclosed obstacle backs
        if rclpy.ok() and self.detected_cube is None:
            self._pause_exploration()
            self.get_logger().info(
                'Frontiers complete without detection; running blind-spot sweep...'
            )
            waypoints = find_unclosed_blind_spots(
                self.latest_map,
                self.costmap,
                min_cluster_cells=self.blind_spot_min_cells,
                min_standoff=self.blind_spot_standoff,
            )
            for wp in waypoints:
                if self.detected_cube is not None or not rclpy.ok():
                    break
                wp.header.stamp = self.navigator.get_clock().now().to_msg()
                self.navigator.goToPose(wp)
                while not self.navigator.isTaskComplete() and rclpy.ok():
                    if self.detected_cube is not None:
                        self.navigator.cancelTask()
                        break
                    rclpy.spin_once(self, timeout_sec=0.1)

                if self.detected_cube is None:
                    for deg in (35.0, -70.0):
                        if self.detected_cube is not None or not rclpy.ok():
                            break
                        self.navigator.spin(spin_dist=math.radians(deg))
                        while not self.navigator.isTaskComplete() and rclpy.ok():
                            if self.detected_cube is not None:
                                self.navigator.cancelTask()
                                break
                            rclpy.spin_once(self, timeout_sec=0.1)

        if self.detected_cube is None:
            self.get_logger().error('Task 4 failed: target cube was never detected.')
            return

        # Stop explore_lite cleanly and give Nav2 a brief moment to settle cancellation
        self._pause_exploration()
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)

        # Phase 3: Approach detected cube standoff, pick, and deliver to C
        rx, ry = self._get_robot_xy()
        standoff_cube = self.compute_standoff_pose(
            target_map_x=self.detected_cube.pose.position.x,
            target_map_y=self.detected_cube.pose.position.y,
            approach_from_map_x=rx,
            approach_from_map_y=ry,
        )
        if not self.go_to_pose(standoff_cube, label='Detected Cube Standoff'):
            self.get_logger().error('Task 4 aborted: could not reach detected cube.')
            return

        self.simulated_pick('Task 4 Cube')

        pose_c_center = self.arena_pose_to_map(self.point_c)
        standoff_c = self.compute_standoff_pose(
            target_map_x=pose_c_center.pose.position.x,
            target_map_y=pose_c_center.pose.position.y,
            approach_from_map_x=standoff_cube.pose.position.x,
            approach_from_map_y=standoff_cube.pose.position.y,
        )
        if not self.go_to_pose(standoff_c, label='Drop Standoff C'):
            self.get_logger().error('Task 4 aborted: could not reach drop zone C.')
            return

        self.simulated_place('Task 4 Cube at C')
        self.get_logger().info('=== TASK 4 COMPLETE ===')


def main(args=None):
    rclpy.init(args=args)
    node = Task4SearchPickNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()