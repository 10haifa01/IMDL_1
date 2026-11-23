import rclpy
from robot_brain.task_mission_base import TaskMissionBase


class Task2ObstaclesNode(TaskMissionBase):
    def __init__(self):
        super().__init__('task2_node')

    def run(self):
        self.wait_for_ready()

        pose_b = self.arena_pose_to_map(self.point_b)
        pose_c = self.arena_pose_to_map(self.point_c)

        self.get_logger().info('=== TASK 2 START: A -> B -> C (Obstacle Avoidance) ===')
        if not self.go_to_pose(pose_b, label='Point B'):
            self.get_logger().error('Task 2 aborted: failed to reach Point B.')
            return

        if not self.go_to_pose(pose_c, label='Point C'):
            self.get_logger().error('Task 2 aborted: failed to reach Point C.')
            return

        self.get_logger().info('=== TASK 2 COMPLETE ===')


def main(args=None):
    rclpy.init(args=args)
    node = Task2ObstaclesNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()