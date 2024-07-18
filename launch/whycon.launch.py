from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Locate parameter file
    PKG = get_package_share_directory('whycon_ros')
    params_file = f'{PKG}/config/params.yaml'

    # Construct Launch Description
    LD = LaunchDescription()

    # Whycon Node
    LD.add_action(Node(package='whycon_ros',
                       executable='whycon_ros',
                       name='whycon_ros',
                       output='screen',
                       arguments=[params_file],
                       remappings=[
                           ('/camera/camera_info', '/depth/camera_info'),
                           ('/camera/image_raw', '/depth/image_raw')
                       ],
                       parameters=[{
                           'useGui': True,
                           'idBits': 5,
                           'idSamples': 360,
                           'hammingDist': 1,
                           'maxMarkers': 100,
                           'fontPath': get_package_share_directory('whycon_ros') + '/etc/font.ttf',
                           'calibDefPath': get_package_share_directory('whycon_ros') + '/etc/default.cal'
                       }]
                       ))

    return LD
