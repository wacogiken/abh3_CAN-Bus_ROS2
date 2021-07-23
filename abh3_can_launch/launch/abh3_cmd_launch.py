from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  joycnv_node = Node(
    package='abh3_can_topic',
    executable='testCMD',
    name='testCMD',
    parameters=[
      {'gainX':1.0},
      {'gainA':1.0},
    ]
  )
  conv_node = Node(
    package='abh3_can_topic',
    executable='ros2ABH3Cnv',
    name='conv1',
    remappings=[
      ('xa', 'cmd_vel1'),
      ('xa_yx', 'vel_cmd_yx1'),
      ('yx', 'vel_fbk_yx1'),
      ('yx_xa', 'vel_fbk1'),
    ],
    parameters=[
      {'rateNum':1.0},
      {'rateDen':10.0},
      {'wheel':0.1},
      {'width':0.5},
    ]
  )
  cmd_node = Node(
    package='abh3_can_topic',
    executable='ros2ABH3Cmd',
    name='cmd1',
    remappings=[
      ('vel_cmd', 'vel_cmd_yx1'),
      ('vel_fbk_yx', 'vel_fbk_yx1'),
    ],
    parameters=[
      {'rateNum':1.0},
      {'rateDen':10.0},
      {'wheel':0.1},
      {'width':0.5},
    ]
  )

  return LaunchDescription([
    joycnv_node,
    conv_node,
    cmd_node,
  ])