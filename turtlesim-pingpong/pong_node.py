import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from turtlesim.srv import Spawn, Kill

class PongNode(Node):
    theta = math.pi / 2
    
    def __init__(self, turtle_name):
        super().__init__('pong_node_' + turtle_name)
        self.spawn_request = Spawn.Request()
        self.turtle_name = turtle_name
        self.spawn_request.name = turtle_name
        self.spawn_client = self.create_client(Spawn, 'spawn')  

    def spawn(self, x=0, y=0, theta=theta):
        if x < 0 or x > 11 or y < 0 or y > 11:
            self.get_logger().error("Invalid spawn point") 
            return 

        # if number of turtle > 2 then don't allow to spawn
        self.spawn_request.x = x 
        self.spawn_request.y = y
        self.spawn_request.theta = theta 

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn service not available')

        self.future = self.spawn_client.call_async(self.spawn_request) 
        self.future.add_done_callback(self.spawn_turtle_callback)

    def spawn_turtle_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Turtle spawned')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    
    # Kill the turtle1 node
    killer_node = Node('killer_node')
    kill_client = killer_node.create_client(Kill, 'kill')
    kill_request = Kill.Request()
    kill_request.name = 'turtle1'

    while not kill_client.wait_for_service(timeout_sec=1.0):
        killer_node.get_logger().info('kill service unable to connect')
    
    future = kill_client.call_async(kill_request)
    rclpy.spin_until_future_complete(killer_node, future)

    # Create and spawn two new turtles
    left_pong_node = PongNode('left_pong_node')
    right_pong_node = PongNode('right_pong_node')

    left_pong_node.spawn(1.0, 5.0)
    right_pong_node.spawn(10.0, 5.0)

    # Use MultiThreadedExecutor to spin both nodes simultaneously
    executor = MultiThreadedExecutor()
    executor.add_node(left_pong_node)
    executor.add_node(right_pong_node)

    try:
        executor.spin()
    finally:
        killer_node.destroy_node()
        left_pong_node.destroy_node()
        right_pong_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
