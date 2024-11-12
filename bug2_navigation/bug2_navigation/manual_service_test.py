import rclpy
from rclpy.node import Node
from bug2_interfaces.srv import GoToPoint

def main(args=None):
    rclpy.init(args=args)
    node = Node('test_service_node')
    client = node.create_client(GoToPoint, 'go_to_point_service')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for service...')
    
    req = GoToPoint.Request()
    req.move_switch = True
    req.target_position.x = 3.0
    req.target_position.y = 4.0

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info(f'Result: {future.result().success}')
    else:
        node.get_logger().error('Service call failed')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
