import math
from geometry_msgs.msg import PoseStamped, Quaternion


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ArenaTransform:
    """
    Converts between Arena coordinates (origin at center of arena) and
    SLAM 'map' coordinates (origin at robot start pose A = (x0, y0, alpha)).
    """

    def __init__(self, start_x: float, start_y: float, start_yaw: float):
        self.x0 = float(start_x)
        self.y0 = float(start_y)
        self.alpha = float(start_yaw)
        self._cos_a = math.cos(self.alpha)
        self._sin_a = math.sin(self.alpha)

    def arena_to_map_xy(self, x: float, y: float) -> tuple[float, float]:
        dx = x - self.x0
        dy = y - self.y0
        mx = dx * self._cos_a + dy * self._sin_a
        my = -dx * self._sin_a + dy * self._cos_a
        return mx, my

    def arena_to_map_yaw(self, yaw: float) -> float:
        return normalize_angle(yaw - self.alpha)

    def arena_to_map_pose(
        self, x: float, y: float, yaw: float, frame_id: str = 'map'
    ) -> PoseStamped:
        mx, my = self.arena_to_map_xy(x, y)
        myaw = self.arena_to_map_yaw(yaw)

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = mx
        pose.pose.position.y = my
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(myaw)
        return pose

    def map_to_arena_xy(self, mx: float, my: float) -> tuple[float, float]:
        x = self.x0 + mx * self._cos_a - my * self._sin_a
        y = self.y0 + mx * self._sin_a + my * self._cos_a
        return x, y

    def map_to_arena_yaw(self, myaw: float) -> float:
        return normalize_angle(myaw + self.alpha)