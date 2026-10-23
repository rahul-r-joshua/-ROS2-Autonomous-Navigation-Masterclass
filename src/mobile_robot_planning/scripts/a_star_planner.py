#!/usr/bin/env python3
"""
================================================================================
A* Global Path Planner (Standalone Node with Grid Search & Euclidean Heuristic)
--------------------------------------------------------------------------------
Author & Maintainer: Rahul Ramasamy
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
Email: rahul.r.joshua123@gmail.com

Workshop Edition for ROS 2 Mobile Robotics Workshop.
================================================================================
"""

import math
from queue import PriorityQueue
import numpy as np
import scipy.ndimage

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import rclpy.time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose


class GraphNode:
    def __init__(self, x, y, cost=0.0, heuristic=0.0, prev=None):
        self.x = int(x)
        self.y = int(y)
        self.cost = cost
        self.heuristic = heuristic
        self.prev = prev

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))


class AStarPlanner(Node):
    def __init__(self):
        super().__init__(
            "a_star_node",
            parameter_overrides=[
                rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
            ]
        )

        map_qos = QoSProfile(depth=10)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(OccupancyGrid, "/costmap/costmap", self.map_callback, map_qos)
        self.pose_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, "/a_star/path", 10)
        self.plan_pub = self.create_publisher(Path, "/plan", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/a_star/visited_map", 10)

        self.map_ = None
        self.cost_grid = None
        self.visited_map_ = OccupancyGrid()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("A* Planner Node initialized with obstacle inflation & 8-connected search.")

    def map_callback(self, map_msg: OccupancyGrid):
        self.map_ = map_msg
        self.visited_map_.header.frame_id = map_msg.header.frame_id
        self.visited_map_.info = map_msg.info
        self.visited_map_.data = [-1] * (map_msg.info.height * map_msg.info.width)

        h = map_msg.info.height
        w = map_msg.info.width
        res = map_msg.info.resolution
        grid = np.array(map_msg.data, dtype=np.int8).reshape((h, w))

        # Obstacle distance transform: free cells are 0 <= cell < 50
        free_mask = (grid >= 0) & (grid < 50)
        dist = scipy.ndimage.distance_transform_edt(free_mask) * res

        cost_grid = np.zeros_like(dist, dtype=np.int16)
        # Lethal obstacle zone within robot radius (0.24m)
        cost_grid[dist < 0.24] = 254
        # Exponential inflation decay from 0.24m to 0.60m
        inflated = (dist >= 0.24) & (dist < 0.60)
        cost_grid[inflated] = (253.0 * np.exp(-6.0 * (dist[inflated] - 0.24))).astype(np.int16)

        self.cost_grid = cost_grid
        self.get_logger().info(f"Costmap computed: {w}x{h} cells, resolution {res}m, robot clearance 0.24m.")

    def snap_to_free(self, node: GraphNode) -> GraphNode:
        if self.cost_grid is None:
            return node
        h, w = self.cost_grid.shape
        if 0 <= node.x < w and 0 <= node.y < h and self.cost_grid[node.y, node.x] < 254:
            return node

        # Search outward in expanding rings up to 20 cells (~1.0 meter) using minimum Euclidean distance
        best_node = node
        min_dist_sq = float('inf')
        for r in range(1, 21):
            found_in_ring = False
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    nx, ny = node.x + dx, node.y + dy
                    if 0 <= nx < w and 0 <= ny < h and self.cost_grid[ny, nx] < 254:
                        d_sq = dx * dx + dy * dy
                        if d_sq < min_dist_sq:
                            min_dist_sq = d_sq
                            best_node = GraphNode(nx, ny)
                            found_in_ring = True
            if found_in_ring:
                return best_node
        return node

    def get_robot_pose(self) -> Pose:
        """Resolve current robot pose in the global map frame robustly."""
        if self.map_ is None:
            return None
        # 1. Direct tf lookup
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id, "base_footprint", rclpy.time.Time()
            )
            pose = Pose()
            pose.position.x = tf_msg.transform.translation.x
            pose.position.y = tf_msg.transform.translation.y
            pose.orientation = tf_msg.transform.rotation
            return pose
        except Exception:
            pass

        # 2. Planar two-step lookup (bypasses AMCL transform_tolerance time gap)
        try:
            t_odom_base = self.tf_buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
            t_map_odom = self.tf_buffer.lookup_transform(self.map_.header.frame_id, "odom", rclpy.time.Time())

            x_m = t_map_odom.transform.translation.x
            y_m = t_map_odom.transform.translation.y
            qz_m = t_map_odom.transform.rotation.z
            qw_m = t_map_odom.transform.rotation.w
            yaw_m = 2.0 * math.atan2(qz_m, qw_m)

            x_o = t_odom_base.transform.translation.x
            y_o = t_odom_base.transform.translation.y
            qz_o = t_odom_base.transform.rotation.z
            qw_o = t_odom_base.transform.rotation.w
            yaw_o = 2.0 * math.atan2(qz_o, qw_o)

            yaw_total = yaw_m + yaw_o
            pose = Pose()
            pose.position.x = x_m + x_o * math.cos(yaw_m) - y_o * math.sin(yaw_m)
            pose.position.y = y_m + x_o * math.sin(yaw_m) + y_o * math.cos(yaw_m)
            pose.orientation.z = math.sin(yaw_total / 2.0)
            pose.orientation.w = math.cos(yaw_total / 2.0)
            return pose
        except Exception as ex:
            self.get_logger().warn(f"Could not resolve robot pose in map: {ex}")
            return None

    def goal_callback(self, pose: PoseStamped):
        if self.map_ is None or self.cost_grid is None:
            self.get_logger().error("No map received yet!")
            return

        map_frame = self.map_.header.frame_id
        target_pose = pose
        if pose.header.frame_id != map_frame:
            try:
                import tf2_geometry_msgs
                target_pose = self.tf_buffer.transform(
                    pose, map_frame, timeout=Duration(seconds=1.0)
                )
                self.get_logger().info(
                    f"Transformed goal from frame [{pose.header.frame_id}] to [{map_frame}]: "
                    f"({target_pose.pose.position.x:.2f}, {target_pose.pose.position.y:.2f})"
                )
            except Exception as e:
                self.get_logger().warn(
                    f"Could not transform goal from [{pose.header.frame_id}] to [{map_frame}]: {e}. Using raw pose."
                )

        map_to_base_pose = self.get_robot_pose()
        if map_to_base_pose is None:
            return

        self.get_logger().info(
            f"Planning from ({map_to_base_pose.position.x:.2f}, {map_to_base_pose.position.y:.2f}) "
            f"to goal ({target_pose.pose.position.x:.2f}, {target_pose.pose.position.y:.2f})..."
        )
        path = self.plan(map_to_base_pose, target_pose.pose)

        if path.poses:
            self.get_logger().info(f"Shortest path found with {len(path.poses)} poses!")
            self.path_pub.publish(path)
            self.plan_pub.publish(path)
        else:
            self.get_logger().warn("No valid path found to the goal.")

    def plan(self, start: Pose, goal: Pose) -> Path:
        h, w = self.cost_grid.shape
        start_node = self.snap_to_free(self.world_to_grid(start))
        goal_node = self.snap_to_free(self.world_to_grid(goal))

        # 8-connected grid search (orthogonal + diagonal)
        explore_directions = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)
        ]

        pending_nodes = PriorityQueue()
        visited = np.zeros((h, w), dtype=bool)
        cost_so_far = np.full((h, w), np.inf, dtype=np.float32)

        start_node.heuristic = self.euclidean_distance(start_node, goal_node)
        pending_nodes.put(start_node)
        cost_so_far[start_node.y, start_node.x] = 0.0

        active_node = None
        goal_reached = False

        while not pending_nodes.empty() and rclpy.ok():
            active_node = pending_nodes.get()

            if active_node == goal_node:
                goal_reached = True
                break

            if visited[active_node.y, active_node.x]:
                continue
            visited[active_node.y, active_node.x] = True

            for dir_x, dir_y, step_len in explore_directions:
                nx = active_node.x + dir_x
                ny = active_node.y + dir_y

                if 0 <= nx < w and 0 <= ny < h:
                    cell_cost = self.cost_grid[ny, nx]
                    if cell_cost < 254:
                        new_cost = active_node.cost + step_len + float(cell_cost) * 0.05
                        if new_cost < cost_so_far[ny, nx]:
                            cost_so_far[ny, nx] = new_cost
                            new_node = GraphNode(nx, ny, cost=new_cost)
                            new_node.heuristic = self.euclidean_distance(new_node, goal_node)
                            new_node.prev = active_node
                            pending_nodes.put(new_node)

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_.header.frame_id

        if not goal_reached or active_node is None:
            return path

        # Reconstruct path from goal back to start
        raw_poses = []
        curr = active_node
        while curr is not None:
            p = self.grid_to_world(curr)
            raw_poses.append(p)
            curr = curr.prev

        raw_poses.reverse()

        # Compute orientations tangent to the path
        for i in range(len(raw_poses)):
            if i + 1 < len(raw_poses):
                dx = raw_poses[i + 1].position.x - raw_poses[i].position.x
                dy = raw_poses[i + 1].position.y - raw_poses[i].position.y
                yaw = math.atan2(dy, dx)
            elif i > 0:
                dx = raw_poses[i].position.x - raw_poses[i - 1].position.x
                dy = raw_poses[i].position.y - raw_poses[i - 1].position.y
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0

            ps = PoseStamped()
            ps.header.stamp = path.header.stamp
            ps.header.frame_id = self.map_.header.frame_id
            ps.pose.position = raw_poses[i].position
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(ps)

        # Snap path start to exact robot position and end to clicked goal pose
        if path.poses:
            path.poses[0].pose.position = start.position
            orig_gx = int((goal.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
            orig_gy = int((goal.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
            if 0 <= orig_gx < w and 0 <= orig_gy < h and self.cost_grid[orig_gy, orig_gx] < 254:
                path.poses[-1].pose.position = goal.position

        # Preserve desired goal orientation on the final waypoint if given
        if path.poses and (abs(goal.orientation.z) > 0.01 or abs(goal.orientation.w) > 0.01):
            path.poses[-1].pose.orientation = goal.orientation

        # Publish visited map once for RViz visualization
        self.visited_map_.data = [-106 if v else -1 for v in visited.flatten()]
        self.map_pub.publish(self.visited_map_)

        return path

    def world_to_grid(self, pose: Pose) -> GraphNode:
        grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
        grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
        return GraphNode(grid_x, grid_y)

    def grid_to_world(self, node: GraphNode) -> Pose:
        pose = Pose()
        pose.position.x = (node.x + 0.5) * self.map_.info.resolution + self.map_.info.origin.position.x
        pose.position.y = (node.y + 0.5) * self.map_.info.resolution + self.map_.info.origin.position.y
        pose.orientation.w = 1.0
        return pose

    def euclidean_distance(self, node: GraphNode, goal_node: GraphNode) -> float:
        return math.hypot(node.x - goal_node.x, node.y - goal_node.y)


def main():
    rclpy.init()
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
