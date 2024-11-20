from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
import numpy as np

import matplotlib.pyplot as plt


class MapParser(Node):
    def __init__(self):
        super().__init__("map_parser")
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            "map",
            self.map_callback,
            10,  # QoS depth
        )
        self.get_logger().info("Map parser node started.")

    def map_callback(self, msg):
        # Extract metadata
        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        origin = msg.info.origin

        self.get_logger().info(
            f"Got a new map with width: {width}, height: {height}, origin: {origin}, resolution: {resolution}"
        )

        # Convert 1D occupancy data to 2D numpy array
        grid = np.array(msg.data).reshape((height, width))

        # Visualize the map
        self.visualize_map(grid, resolution, origin)

    def visualize_map(self, grid, resolution, origin):
        # Create a colormap for the occupancy grid
        colormap = {
            -1: (0.5, 0.5, 0.5),  # Unknown: Gray
            0: (1.0, 1.0, 1.0),  # Free: White
            100: (0.0, 0.0, 0.0),  # Occupied: Black
        }

        # Convert grid values to RGB for visualization
        rgb_grid = np.zeros((grid.shape[0], grid.shape[1], 3))  # Height x Width x RGB
        for key, color in colormap.items():
            rgb_grid[grid == key] = color

        # Create the plot
        plt.figure(figsize=(8, 8))
        plt.imshow(rgb_grid, origin="lower", interpolation="nearest")
        plt.title("Occupancy Grid Map")
        plt.xlabel("Grid X (cells)")
        plt.ylabel("Grid Y (cells)")
        plt.grid(False)

        # Add map origin as a marker (optional)
        origin_x = int(-origin.position.x / resolution)
        origin_y = int(-origin.position.y / resolution)
        plt.plot(origin_x, origin_y, "ro", label="Map Origin")
        plt.legend()

        plt.show()


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
