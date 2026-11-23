import rclpy
from robot_brain.task_mission_base import TaskMissionBase


class Task1LocomotionNode(TaskMissionBase):
    def __init__(self):
        super().__init__('task1_node')

    def run(self):
        self.wait_for_ready()

        pose_a = self.arena_pose_to_map(self.point_a)
        pose_b = self.arena_pose_to_map(self.point_b)

        self.get_logger().info('=== TASK 1 START: A -> B -> A ===')
        if not self.go_to_pose(pose_b, label='Point B'):
            self.get_logger().error('Task 1 aborted: failed to reach Point B.')
            return

        self.get_logger().info('Reached Point B. Returning to Point A (forward drive)...')
        if not self.go_to_pose(pose_a, label='Point A'):
            self.get_logger().error('Task 1 aborted: failed to return to Point A.')
            return

        self.get_logger().info('=== TASK 1 COMPLETE ===')


def main(args=None):
    rclpy.init(args=args)
    node = Task1LocomotionNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()