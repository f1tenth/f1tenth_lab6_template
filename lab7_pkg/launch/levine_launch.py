"""What the autograder runs on levine_obs (the straight path and the turn) and
for the obstacle checks:

    ros2 launch lab7_pkg levine_launch.py

Everything your planner needs goes here: one node or several, Python or C++,
and the parameter values you tuned. Start your own nodes only: the simulator
is already running.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

# 'rrt_node.py' is scripts/rrt_node.py, 'rrt_node' is the C++
# src/rrt_node.cpp: name the one you wrote
EXECUTABLE = 'rrt_node.py'

# values for the parameters your node declares
# e.g. {'max_speed': 3.0} or give it a full .yaml config file
PARAMETERS = {}


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab7_pkg',
            executable=EXECUTABLE,
            output='screen',
            parameters=[PARAMETERS],
        ),
    ])
