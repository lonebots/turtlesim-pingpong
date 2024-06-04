import rclpy
import random
import time
import math
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, TeleportAbsolute, Spawn
from geometry_msgs.msg import Twist

# Dictionary to store the direction of the ball
DIRECTION = {
    'STOP': 0,
    'LEFT': 1,
    'LEFT_UP': 2,
    'LEFT_DOWN': 3,
    'RIGHT': 4,
    'RIGHT_UP': 5,
    'RIGHT_DOWN': 6
}

# Class to represent the ball node
class BallNode(Node):
    pose_ = None
    pose_left_ = None
    pose_right_ = None
    direction_ = DIRECTION['STOP']
    absolute_position_ = None

    def __init__(self, name):
        super().__init__(name)
        self.reset()

        self.pose_sub_ = self.create_subscription(Pose, '/ball/pose', self.pose_callback, 10)
        self.left_pose_sub_ = self.create_subscription(Pose, '/left_pong_node/pose', self.left_pose_callback, 10)
        self.right_pose_sub_ = self.create_subscription(Pose, '/right_pong_node/pose', self.right_pose_callback, 10)

        self.cmd_vel_pub_ = self.create_publisher(Twist, '/ball/cmd_vel', 10)

    def update_direction(self):
        # self.get_logger().info('update direction invoked')
        theta = self.pose_.theta
        
        # Different conditions based on theta
        if 0.0 < theta < math.pi/2:
            self.direction_ = DIRECTION['RIGHT_UP']
        elif math.pi/2 <= theta < math.pi:
            self.direction_ = DIRECTION['LEFT_UP']
        elif -math.pi < theta < -math.pi/2:
            self.direction_ = DIRECTION['LEFT_DOWN']
        elif -math.pi/2 <= theta < 0.0:
            self.direction_ = DIRECTION['RIGHT_DOWN']
        else:
            self.direction_ = DIRECTION['RIGHT']

        # self.get_logger().info('direction :' + str(self.direction_))
        
    def check_collision(self):
        # self.get_logger().info('check_collision invoked')
        ball_x = self.pose_.x
        ball_y = self.pose_.y
        ball_theta = self.pose_.theta

        left_x = self.pose_left_.x
        left_y = self.pose_left_.y

        right_x = self.pose_right_.x
        right_y = self.pose_right_.y

        # Paddle dimensions
        delta_x = 0.7
        delta_y = 0.7
        paddle_size = 2.0 * delta_y
        max_bounce_angle = 5 * math.pi / 12

        # Collision with left pong
        if left_x - delta_x <= ball_x <= left_x + delta_x :
            if left_y - delta_y <= ball_y <= left_y + delta_y:
                self.get_logger().info('left collision')
                relative_intersect_y = (left_y + delta_y) - ball_y
                normalized_relative_intersect_y = relative_intersect_y / paddle_size
                bounce_angle = normalized_relative_intersect_y * max_bounce_angle
                # self.get_logger().info('Bounce angle: %f' % bounce_angle)
                self.set_pose_absolute(ball_x, ball_y, bounce_angle)
        
        # Collision with right pong
        if right_x - delta_x <= ball_x <= right_x + delta_x:
            if right_y - delta_y <= ball_y <= right_y + delta_y:
                self.get_logger().info('right collision')
                relative_intersect_y = (right_y + delta_y) - ball_y
                normalized_relative_intersect_y = relative_intersect_y / paddle_size
                bounce_angle = normalized_relative_intersect_y * max_bounce_angle
                # self.get_logger().info('Bounce angle: %f' % bounce_angle)
                self.set_pose_absolute(ball_x, ball_y, bounce_angle)

    def move(self):
        if self.pose_ is None or self.pose_left_ is None or self.pose_right_ is None:
            self.set_pose_absolute(3.0, 3.0, -0.6)
            return
        
        self.set_velocity(2.0, 0.0)

        # Get current position of the ball
        x = self.pose_.x
        y = self.pose_.y
        theta = self.pose_.theta

        self.check_collision()
        self.update_direction()

        # Check if the ball is out of bounds
        new_theta = theta
        # Ball hits the top wall
        if y >= 11.0:
            # self.get_logger().info('hit TOP wall')
            if self.direction_ == DIRECTION['RIGHT_UP'] :
                new_theta = 2 * math.pi - theta
                self.set_pose_absolute(x, y, new_theta) 
            elif self.direction_ == DIRECTION['LEFT_UP'] :
                new_theta = 2 * math.pi - theta
                self.set_pose_absolute(x, y, new_theta)
        
        # Ball hits the bottom wall
        if y <= 0.0 :
            # self.get_logger().info('hit BOTTOM wall')
            if self.direction_ == DIRECTION['RIGHT_DOWN'] :
                new_theta = -theta
                self.set_pose_absolute(x, y, new_theta)
            elif self.direction_ == DIRECTION['LEFT_DOWN'] :
                new_theta = -theta
                self.set_pose_absolute(x, y, new_theta)

        # Ball hits the left wall
        if x <= 0.0:
            # self.get_logger().info('hit LEFT wall')
            if self.direction_ == DIRECTION['LEFT_UP'] :
                new_theta = math.pi - theta
                self.set_pose_absolute(x, y, new_theta)
            elif self.direction_ == DIRECTION['LEFT_DOWN'] :
                new_theta = - (math.pi + theta)
                self.set_pose_absolute(x, y, new_theta)
            
        # Ball hits the right wall
        if x >= 11.0:
            # self.get_logger().info('hit RIGHT wall')
            if self.direction_ == DIRECTION['RIGHT_UP'] :
                new_theta = math.pi - theta
                self.set_pose_absolute(x, y, new_theta)
            elif self.direction_ == DIRECTION['RIGHT_DOWN'] :
                new_theta = - (math.pi + theta)
                self.set_pose_absolute(x, y, new_theta)
    
    def pen_off(self, off):
        setpen = SetPen.Request()
        setpen.off = off 
        setpen_client = self.create_client(SetPen, '/ball/set_pen')
        future = setpen_client.call_async(setpen)
        rclpy.spin_until_future_complete(self, future)

    def set_pose_absolute(self, x, y, theta):
        teleport_request = TeleportAbsolute.Request()
        teleport_request.x = x
        teleport_request.y = y
        teleport_request.theta = theta

        teleport_client = self.create_client(TeleportAbsolute, '/ball/teleport_absolute')
        future = teleport_client.call_async(teleport_request)
        rclpy.spin_until_future_complete(self, future)
        # self.get_logger().info('TELEPORT called')

    def set_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        self.cmd_vel_pub_.publish(twist)
        # self.get_logger().info('Publishing cmd_vel: linear=%f, angular=%f' % (linear, angular))        

    def random_angle(self):
        angle = random.random()
        r = random.random() 

        if r > 0.5:
            return angle * math.pi 
        else:
            return -angle * math.pi

    def reset(self):
        self.set_pose_absolute(3.0, 3.0, 0.0)
        self.pen_off(1)

    def pose_callback(self, msg):
        self.pose_ = msg
        # self.get_logger().info('Ball position: x=%f, y=%f, angle=%f, direction=%f'%(msg.x, msg.y, msg.theta, self.direction_))

    def left_pose_callback(self, msg):
        self.pose_left_ = msg
        # self.get_logger().info('LEFT PONG : x=%f, y=%f, theta=%f' % (self.pose_left_.x, self.pose_left_.y, self.pose_left_.theta))

    def right_pose_callback(self, msg):
        self.pose_right_ = msg
        # self.get_logger().info('Right pong position: x=%f, y=%f' % (msg.x, msg.y))   

def main(args=None):
    rclpy.init(args=args)
    
    node = Node('ball_spawner')
    loop_rate = 0.1  # 10 Hz
    
    ball_spawn_request = Spawn.Request()
    ball_spawn_client = node.create_client(Spawn, '/spawn')
    ball_spawn_request.name = 'ball'
    ball_spawn_request.x = 3.0
    ball_spawn_request.y = 3.0
    ball_spawn_request.theta = 0.0

    if not ball_spawn_client.wait_for_service(timeout_sec=5.0):
        # node.get_logger().error('Service not available')
        return
    
    future = ball_spawn_client.call_async(ball_spawn_request)
    rclpy.spin_until_future_complete(node, future)
    # if future.result() is not None:
    #     # node.get_logger().info('Ball node spawned')
    # else:
    #     # node.get_logger().error('Failed to spawn ball node')
    #     return

    ball_node = BallNode('ball')
    
    try:
        while rclpy.ok():
            ball_node.move()
            rclpy.spin_once(ball_node)
            time.sleep(loop_rate)
    except KeyboardInterrupt:
        pass
    finally:
        ball_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
