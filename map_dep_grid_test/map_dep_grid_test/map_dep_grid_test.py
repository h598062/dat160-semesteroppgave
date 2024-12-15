from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
import numpy as np
import heapq


def a_star_search(binary_map, start, goal):
    # Initialize variables
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_list:
        current = heapq.heappop(open_list)[1]

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in get_neighbors(current, binary_map):
            tentative_g_score = g_score[current] + 1  # assume cost = 1 per step

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_list, (f_score[neighbor], neighbor))

    return None  # Return None if there is no path


def heuristic(a, b):
    # Using Manhattan distance as heuristic
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(pos, binary_map):
    neighbors = []
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 4-way connectivity
        x2, y2 = pos[0] + dx, pos[1] + dy
        if (
            0 <= x2 < binary_map.shape[0]
            and 0 <= y2 < binary_map.shape[1]
            and binary_map[x2, y2] == 1
        ):
            neighbors.append((x2, y2))
    return neighbors


def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


class MapSubscriber(Node):
    def __init__(self):
        super().__init__("map_subscriber")
        self.subscription = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )
        self.map_data = None
        self.first = True

    def map_callback(self, msg):
        # Extract the map data and metadata
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.get_logger().info("Map data received!")
        if self.first:
            print(self.map_data)
            self.first = False
            # Convert OccupancyGrid data to a binary map for path planning
            binary_map = np.where(
                (self.map_data == 0), 1, 0
            )  # 1 for free, 0 for obstacle
            noe = a_star_search(binary_map, (0, 0), (10, 15))
            print(noe)


def main(args=None):
    rclpy.init(args=args)
    controller = MapSubscriber()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("Exiting program...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
