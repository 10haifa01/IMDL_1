import math
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from explore_lite_msgs.msg import ExploreStatus
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class PostExplorationSweeper(Node):
    def __init__(self):
        super().__init__('post_exploration_sweeper')

        self.latest_map = None
        self.costmap = None
        self.exploration_done = False

        self.create_subscription(
            ExploreStatus,
            '/explore/status',
            self.explore_status_cb,
            10,
        )

        # VOLATILE receives /map from slam_toolbox regardless of publisher durability
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_cb,
            10,
        )

        costmap_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_cb,
            costmap_qos,
        )

        self.get_logger().info('Waiting for explore_lite to finish mapping...')

    def explore_status_cb(self, msg: ExploreStatus):
        if msg.status == ExploreStatus.EXPLORATION_COMPLETE and not self.exploration_done:
            self.get_logger().info('Exploration complete! Scanning map for unclosed obstacle blind spots.')
            self.exploration_done = True

    def map_cb(self, msg: OccupancyGrid):
        self.latest_map = msg

    def costmap_cb(self, msg: OccupancyGrid):
        self.costmap = msg

    def find_unclosed_blind_spots(self, min_cluster_cells: int = 4):
        """
        Finds clusters on /map where free cells (0) directly border unknown cells (-1),
        representing unclosed obstacle backs or unseen corner pockets.
        """
        if self.latest_map is None or self.costmap is None:
            return []

        mw = self.latest_map.info.width
        mh = self.latest_map.info.height
        mres = self.latest_map.info.resolution
        mox = self.latest_map.info.origin.position.x
        moy = self.latest_map.info.origin.position.y

        mgrid = np.array(self.latest_map.data, dtype=np.int16).reshape((mh, mw))

        # Frontier mask: free cells (0) adjacent to at least one unknown cell (-1)
        is_free = (mgrid == 0)
        is_unknown = (mgrid == -1)
        has_unknown_neighbor = np.zeros_like(is_free)
        has_unknown_neighbor[1:, :] |= is_unknown[:-1, :]
        has_unknown_neighbor[:-1, :] |= is_unknown[1:, :]
        has_unknown_neighbor[:, 1:] |= is_unknown[:, :-1]
        has_unknown_neighbor[:, :-1] |= is_unknown[:, 1:]

        frontier_mask = is_free & has_unknown_neighbor
        visited = np.zeros_like(frontier_mask)

        clusters = []
        fy_list, fx_list = np.where(frontier_mask)
        for sy, sx in zip(fy_list, fx_list):
            if visited[sy, sx]:
                continue
            q = deque([(int(sy), int(sx))])
            visited[sy, sx] = True
            cells = []
            while q:
                cy, cx = q.popleft()
                cells.append((cx, cy))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny, nx = cy + dy, cx + dx
                        if 0 <= ny < mh and 0 <= nx < mw:
                            if frontier_mask[ny, nx] and not visited[ny, nx]:
                                visited[ny, nx] = True
                                q.append((ny, nx))
            if len(cells) >= min_cluster_cells:
                clusters.append(cells)

        if not clusters:
            return []

        # Prepare global costmap free cells (cost == 0) for collision-free vantage points
        cw = self.costmap.info.width
        ch = self.costmap.info.height
        cres = self.costmap.info.resolution
        cox = self.costmap.info.origin.position.x
        coy = self.costmap.info.origin.position.y

        cgrid = np.array(self.costmap.data, dtype=np.int16).reshape((ch, cw))
        free_cy, free_cx = np.where(cgrid == 0)
        if len(free_cx) == 0:
            return []

        free_wx = cox + (free_cx + 0.5) * cres
        free_wy = coy + (free_cy + 0.5) * cres

        waypoints = []
        for cells in clusters:
            arr = np.array(cells)
            # Centroid of the unclosed frontier in world coordinates
            ux = mox + (arr[:, 0].mean() + 0.5) * mres
            uy = moy + (arr[:, 1].mean() + 0.5) * mres

            # Choose the nearest collision-free cell that keeps at least 0.32 m standoff
            # so the front camera can rotate cleanly and view the blind spot
            dists = np.hypot(free_wx - ux, free_wy - uy)
            valid_mask = (dists >= 0.32) & (dists <= 1.50)
            if np.any(valid_mask):
                valid_indices = np.where(valid_mask)[0]
                best_idx = int(valid_indices[np.argmin(dists[valid_indices])])
            else:
                best_idx = int(np.argmin(dists))

            gx = float(free_wx[best_idx])
            gy = float(free_wy[best_idx])

            # Aim the robot's front camera (+X) directly at the unclosed blind spot
            yaw = math.atan2(uy - gy, ux - gx)
            qx, qy, qz, qw = yaw_to_quaternion(yaw)

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = gx
            pose.pose.position.y = gy
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            waypoints.append(pose)

        return waypoints


def main(args=None):
    rclpy.init(args=args)
    sweeper = PostExplorationSweeper()

    use_sim_time = sweeper.get_parameter('use_sim_time').get_parameter_value().bool_value
    navigator = BasicNavigator()
    navigator.set_parameters([
        Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)
    ])

    try:
        while rclpy.ok() and not (
            sweeper.exploration_done
            and sweeper.latest_map is not None
            and sweeper.costmap is not None
        ):
            rclpy.spin_once(sweeper, timeout_sec=0.2)

        # Up to 2 passes: visit every unclosed obstacle back, then re-verify if any gaps remain
        for pass_num in range(1, 3):
            if not rclpy.ok():
                break

            # Spin briefly to get the freshest /map and /global_costmap/costmap
            for _ in range(5):
                rclpy.spin_once(sweeper, timeout_sec=0.1)

            waypoints = sweeper.find_unclosed_blind_spots(min_cluster_cells=4)
            if not waypoints:
                sweeper.get_logger().info('All obstacle contours are closed! Sweep complete.')
                break

            sweeper.get_logger().info(
                f'Pass {pass_num}: Found {len(waypoints)} unclosed blind spot(s) to inspect.'
            )

            for i, goal in enumerate(waypoints):
                if not rclpy.ok():
                    break

                goal.header.stamp = navigator.get_clock().now().to_msg()
                sweeper.get_logger().info(
                    f'Inspecting blind spot {i + 1}/{len(waypoints)} at '
                    f'({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})'
                )
                navigator.goToPose(goal)

                while not navigator.isTaskComplete() and rclpy.ok():
                    rclpy.spin_once(sweeper, timeout_sec=0.1)

                if navigator.getResult() == TaskResult.SUCCEEDED:
                    navigator.spin(spin_dist=math.radians(35.0))
                    while not navigator.isTaskComplete() and rclpy.ok():
                        rclpy.spin_once(sweeper, timeout_sec=0.1)

                    if navigator.getResult() == TaskResult.SUCCEEDED:
                        navigator.spin(spin_dist=math.radians(-70.0))
                        while not navigator.isTaskComplete() and rclpy.ok():
                            rclpy.spin_once(sweeper, timeout_sec=0.1)
                else:
                    sweeper.get_logger().warn(
                        f'Could not reach vantage point {i + 1}, skipping.'
                    )

        sweeper.get_logger().info('Post-exploration obstacle closure sweep finished.')

    except KeyboardInterrupt:
        pass
    finally:
        sweeper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()