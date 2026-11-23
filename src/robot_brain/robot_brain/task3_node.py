import rclpy
from robot_brain.task_mission_base import TaskMissionBase


class Task3KnownPickNode(TaskMissionBase):
    def __init__(self):
        super().__init__('task3_node')

    def run(self):
        self.wait_for_ready()

        pose_b_center = self.arena_pose_to_map(self.point_b)
        pose_c_center = self.arena_pose_to_map(self.point_c)

        # Approach B from start A (0, 0 in map frame)
        standoff_b = self.compute_standoff_pose(
            target_map_x=pose_b_center.pose.position.x,
            target_map_y=pose_b_center.pose.position.y,
            approach_from_map_x=0.0,
            approach_from_map_y=0.0,
        )

        self.get_logger().info('=== TASK 3 START: Pick at known B -> Place at C ===')
        if not self.go_to_pose(standoff_b, label='Pre-Grasp Standoff B'):
            self.get_logger().error('Task 3 aborted: failed to reach standoff at B.')
            return

        self.simulated_pick('Task 3 Object (B)')

        # Approach C from current standoff B
        standoff_c = self.compute_standoff_pose(
            target_map_x=pose_c_center.pose.position.x,
            target_map_y=pose_c_center.pose.position.y,
            approach_from_map_x=standoff_b.pose.position.x,
            approach_from_map_y=standoff_b.pose.position.y,
        )

        if not self.go_to_pose(standoff_c, label='Drop Standoff C'):
            self.get_logger().error('Task 3 aborted: failed to reach drop zone C.')
            return

        self.simulated_place('Task 3 Object at C')
        self.get_logger().info('=== TASK 3 COMPLETE ===')


def main(args=None):
    rclpy.init(args=args)
    node = Task3KnownPickNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()