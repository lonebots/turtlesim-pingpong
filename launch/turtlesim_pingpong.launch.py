# python launch file 
from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description() :
    ld = LaunchDescription()

    turtlesim_node = Node(
        package = "turtlesim",
        executable = "turtlesim_node"

    )

    pong_node = Node(
        package = "turtlesim_pingpong",
        executable = "pong_node"
    )

    ball_node = Node(
        package = "turtlesim_pingpong",
        executable = "ball_node"
    )

    left_pong_node_controller = Node(
        package = "turtlesim",
        executable = "turtle_teleop_key",
        remappings = [
        ('/turtle1/cmd_vel', '/left_pong_node/cmd_vel'),
        ]
    )

    left_robot_steering = Node(
        package = "rqt_robot_steering",
        executable = "rqt_robot_steering",
        remappings = [
            ('/cmd_vel', 'left_pong_node/cmd_vel')
        ]
    )

    right_robot_steering = Node(
        package = "rqt_robot_steering",
        executable = "rqt_robot_steering",
        remappings = [
            ('/cmd_vel', 'right_pong_node/cmd_vel')
        ]
    )

    # add key telop for left and right pong nodes
    ld.add_action(turtlesim_node)
    ld.add_action(pong_node)
    ld.add_action(ball_node)
    ld.add_action(left_robot_steering)
    ld.add_action(right_robot_steering)

    return ld



