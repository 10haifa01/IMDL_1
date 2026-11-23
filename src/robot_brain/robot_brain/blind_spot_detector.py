import math
from collections import deque
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from robot_brain.arena_transform import yaw_to_quaternion


def find_unclosed_blind_spots(
    latest_map: OccupancyGrid,
    costmap: OccupancyGrid,
    min_cluster_cells: int = 4,
    min_standoff: float = 0.32,
    max_standoff: float = 1.50,
) -> list[PoseStamped]:
    """
    Finds clusters on /map where free cells (0) directly border unknown cells (-1),
    and returns collision-free (cost == 0) vantage poses aimed at each blind spot.
    """
    if latest_map is None or costmap is None:
        return []

    mw = latest_map.info.width
    mh = latest_map.info.height
    mres = latest_map.info.resolution
    mox = latest_map.info.origin.position.x
    moy = latest_map.info.origin.position.y

    mgrid = np.array(latest_map.data, dtype=np.int16).reshape((mh, mw))

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

    cw = costmap.info.width
    ch = costmap.info.height
    cres = costmap.info.resolution
    cox = costmap.info.origin.position.x
    coy = costmap.info.origin.position.y

    cgrid = np.array(costmap.data, dtype=np.int16).reshape((ch, cw))
    free_cy, free_cx = np.where(cgrid == 0)
    if len(free_cx) == 0:
        return []

    free_wx = cox + (free_cx + 0.5) * cres
    free_wy = coy + (free_cy + 0.5) * cres

    waypoints = []
    for cells in clusters:
        arr = np.array(cells)
        ux = mox + (arr[:, 0].mean() + 0.5) * mres
        uy = moy + (arr[:, 1].mean() + 0.5) * mres

        dists = np.hypot(free_wx - ux, free_wy - uy)
        valid_mask = (dists >= min_standoff) & (dists <= max_standoff)
        if np.any(valid_mask):
            valid_indices = np.where(valid_mask)[0]
            best_idx = int(valid_indices[np.argmin(dists[valid_indices])])
        else:
            best_idx = int(np.argmin(dists))

        gx = float(free_wx[best_idx])
        gy = float(free_wy[best_idx])

        yaw = math.atan2(uy - gy, ux - gx)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = gx
        pose.pose.position.y = gy
        pose.pose.orientation = yaw_to_quaternion(yaw)
        waypoints.append(pose)

    return waypoints