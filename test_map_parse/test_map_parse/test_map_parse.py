from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
import numpy as np
from collections import deque


class MapParser(Node):
    def __init__(self):
        super().__init__("map_parser")

        #Subscribe to map topic
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )

        #Subscribe to robot's odometry
        self.pose_subscription = self.create_subscription(
            Odometry, "/tb3_0/odom", self.pose_callback, 10
        )

        #Publisher for the planned path
        self.path_publisher = self.create_publisher(Path, "planned_path", 10)

        #Robot's dynamic position
        self.robot_position = None

        #Map data and metadata
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None

        #Prevent multiple explorations
        self.exploration_done = False

        self.get_logger().info("Map parser node started.")

    def pose_callback(self, msg):
        # Extract position from odometry
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(f"Robot position (from /odom): {self.robot_position}")

        #Trigger pathfinding if the map has been received and exploration is not done
        if self.map_data is not None and not self.exploration_done:
            self.explore_maze()

    def map_callback(self, msg):
        self.get_logger().info("Map successfully received!")

        # Extract map metadata
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        self.get_logger().info(
            f"Map info: Resolution={self.map_resolution}, Width={msg.info.width}, Height={msg.info.height}, Origin={self.map_origin.position.x}, {self.map_origin.position.y}"
        )

        # If robot position is already known and exploration is not done, explore the maze
        if self.robot_position is not None and not self.exploration_done:
            self.explore_maze()

    def explore_maze(self):
        if self.robot_position is None or self.map_data is None:
            self.get_logger().error("Missing robot position or map data.")
            return

        # Convert robot position to grid coordinates
        start = self.world_to_grid(self.robot_position, self.map_resolution, self.map_origin)

        self.get_logger().info(f"Start grid position: {start}, Exploring entire maze...")

        # Run BFS to explore the entire maze
        explored_path = self.bfs_explore(self.map_data, start)

        if explored_path:
            self.get_logger().info(f"Explored {len(explored_path)} cells in the maze.")
            self.publish_path(explored_path, self.map_resolution, self.map_origin)
        else:
            self.get_logger().error("Exploration failed or no reachable cells.")

        # Mark exploration as complete
        self.exploration_done = True
        self.get_logger().info("Exploration complete. No further processing will occur.")

    def bfs_explore(self, grid, start):
        self.get_logger().info("Starting BFS exploration...")

        rows, cols = grid.shape
        visited = np.zeros_like(grid, dtype=bool)
        queue = deque([start])  # Queue of cells to visit
        explored_path = []  # List of all visited cells
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up

        steps = 0  # Track the number of steps

        while queue:
            current = queue.popleft()
            x, y = current

            # Log progress every 500 steps
            steps += 1
            if steps % 500 == 0:
                self.get_logger().info(f"BFS step {steps}: Current position: {current}, Queue size: {len(queue)}")

            # Mark the cell as visited
            if visited[y, x]:
                continue
            visited[y, x] = True
            explored_path.append(current)

            # Explore neighbors
            for dx, dy in directions:
                nx, ny = x + dx, y + dy

                # Check if the neighbor is within bounds, not visited, and free
                if (
                    0 <= nx < cols and
                    0 <= ny < rows and
                    not visited[ny, nx] and
                    grid[ny, nx] == 0
                ):
                    queue.append((nx, ny))

        self.get_logger().info("BFS exploration complete.")
        return explored_path

    def publish_path(self, path, resolution, origin):
        # Create a Path message
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Convert grid coordinates to world coordinates
        for (x, y) in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x * resolution + origin.position.x
            pose.pose.position.y = y * resolution + origin.position.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.get_logger().info(f"Publishing path with {len(path)} poses.")
        self.path_publisher.publish(path_msg)
        self.get_logger().info("Exploration path successfully published to /planned_path!")

    def world_to_grid(self, position, resolution, origin):
        # Convert world coordinates to grid coordinates
        grid_x = int((position[0] - origin.position.x) / resolution)
        grid_y = int((position[1] - origin.position.y) / resolution)
        self.get_logger().info(f"Converted world position {position} to grid coordinates: ({grid_x}, {grid_y})")
        return grid_x, grid_y


def main(args=None):
    rclpy.init(args=args)
    node = MapParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Exiting program...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
