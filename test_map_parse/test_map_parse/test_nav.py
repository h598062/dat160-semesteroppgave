import time
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sem_oppg_msgs.srv import MoveSrv
import numpy as np


class MapParser(Node):
    def __init__(self):
        super().__init__("map_parser")
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry, "/tb3_0/odom", self.odom_callback, 10
        )
        self.client = self.create_client(MoveSrv, "move_to")

        self.grid = None
        self.resolution = None
        self.origin = None
        self.robot_position = None
        self.visited = None

        self.path_planned = False

        self.get_logger().info("Map parser started.")

    def map_callback(self, msg):
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin
        self.grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.visited = np.zeros_like(self.grid, dtype=bool)

        self.get_logger().info("Map received. Starting exploration planning.")
        self.plan_path()

    def odom_callback(self, msg):
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info("got odom")
        self.plan_path()

    def plan_path(self):
        if self.grid is None or self.robot_position is None:
            self.get_logger().warn("Map or robot position unavailable.")
            return

        if self.path_planned:
            return
        self.path_planned = True
        # Convert robot position to grid coordinates
        start_x = int(
            (self.robot_position[0] - self.origin.position.x) / self.resolution
        )
        start_y = int(
            (self.robot_position[1] - self.origin.position.y) / self.resolution
        )

        self.get_logger().info(f"Starting DFS from: ({start_x}, {start_y})")

        # Perform DFS to explore the map
        path = self.dfs_iterative(start_x, start_y)

        # Sequentially send goals via the service
        for grid_x, grid_y in path:
            world_x = grid_x * self.resolution + self.origin.position.x
            world_y = grid_y * self.resolution + self.origin.position.y
            self.send_goal(world_x, world_y)

    def dfs(self, x, y, path):
        if x < 0 or y < 0 or x >= self.grid.shape[1] or y >= self.grid.shape[0]:
            return
        if self.visited[y, x] or self.grid[y, x] != 0:
            return

        self.visited[y, x] = True
        path.append((x, y))

        self.dfs(x, y + 1, path)
        self.dfs(x, y - 1, path)
        self.dfs(x - 1, y, path)
        self.dfs(x + 1, y, path)

    def dfs_iterative(self, start_x, start_y):
        stack = [(start_x, start_y)]
        path = []

        while stack:
            x, y = stack.pop()

            # Check for out-of-bounds access
            if x < 0 or y < 0 or x >= self.grid.shape[1] or y >= self.grid.shape[0]:
                continue

            # Skip visited or non-traversable cells
            if self.visited[y, x] or self.grid[y, x] != 0:
                continue

            # Mark as visited and append to path
            self.visited[y, x] = True
            path.append((x, y))

            # Add neighbors to the stack
            directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
            for dx, dy in directions:
                stack.append((x + dx, y + dy))

        return path

    def send_goal(self, x, y):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Move service unavailable.")
            return

        request = MoveSrv.Request()
        request.target_position = Point()
        request.target_position.x = x
        request.target_position.y = y

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().result:
            self.get_logger().info(f"Reached goal: ({x}, {y})")
        else:
            self.get_logger().warn(f"Failed to reach goal: ({x}, {y})")


def main(args=None):
    rclpy.init(args=args)
    controller = MapParser()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("Exiting program...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
