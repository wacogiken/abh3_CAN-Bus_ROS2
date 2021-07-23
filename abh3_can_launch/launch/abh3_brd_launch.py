from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  abh3_node1 = Node(
    package='abh3_can_topic',
    executable='ros2ABH3Brd',
    name='ros2ABH3Brd_id1',
    remappings=[
      ('brd_num', 'brd_num_id1'),
      ('brd0_data', 'brd0_data_id1'),
      ('brd1_data', 'brd1_data_id1'),
      ('brd2_data', 'brd2_data_id1'),
      ('brd3_data', 'brd3_data_id1'),
      ('brd4_data', 'brd4_data_id1'),
      ('brd5_data', 'brd5_data_id1'),
      ('brd6_data', 'brd6_data_id1'),
    ],
    parameters=[
      {'mask':0x0f},
      {'abh3_id':1},
    ]
  )
  abh3_node3 = Node(
    package='abh3_can_topic',
    executable='ros2ABH3Brd',
    name='ros2ABH3Brd_ID3',
    remappings=[
      ('brd_num', 'brd_num_id3'),
      ('brd0_data', 'brd0_data_id3'),
      ('brd1_data', 'brd1_data_id3'),
      ('brd2_data', 'brd2_data_id3'),
      ('brd3_data', 'brd3_data_id3'),
      ('brd4_data', 'brd4_data_id3'),
      ('brd5_data', 'brd5_data_id3'),
      ('brd6_data', 'brd6_data_id3'),
    ],
    parameters=[
      {'mask':0},
      {'abh3_id':3},
      {'device':'can0'},
    ]
  )
  brd_node1 = Node(
    package='abh3_can_topic',
    executable='testBRD',
    name='testBRD_ID1',
    remappings=[
      ('brd_num', 'brd_num_id1'),
    ],
    parameters=[
      {'freq':100},
      {'mask':0x0c},
    ]
  )
  brd_node3 = Node(
    package='abh3_can_topic',
    executable='testBRD',
    name='testBRD_ID3',
    remappings=[
      ('brd_num', 'brd_num_id3'),
    ],
    parameters=[
      {'freq':100},
      {'mask':0x00},
    ]
  )
 
  return LaunchDescription([
    abh3_node1,
    brd_node1,
    # abh3_node3,
    # brd_node3,
  ])