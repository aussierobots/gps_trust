""" Launch gps_trust_node publishing GPS Trust Indicator using GPS Trust API"""
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution, EnvironmentVariable
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
  """Generate launch description for gps_trust_node component."""

  gps_trust_api_url_arg = DeclareLaunchArgument(
    "GPS_TRUST_API_URL", default_value=TextSubstitution(text="https://gtapi.aussierobots.com.au/gps-trust-api")
  )
  gps_trust_device_api_key_arg = DeclareLaunchArgument(
    "GPS_TRUST_DEVICE_API_KEY", default_value=EnvironmentVariable(name="GPS_TRUST_DEVICE_API_KEY", default_value="no_api_key")
  )
  params = [{
    'GPS_TRUST_API_URL': LaunchConfiguration('GPS_TRUST_API_URL'),
    'GPS_TRUST_DEVICE_API_KEY': LaunchConfiguration('GPS_TRUST_DEVICE_API_KEY'),
  }]

  container1 = ComposableNodeContainer(
    name='gps_trust_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
      ComposableNode(
        package='gps_trust_node',
        plugin='gps_trust::GPSTrustNode',
        name='gps_trust',
        parameters=params
      )
    ]
  )

  return launch.LaunchDescription([
    gps_trust_api_url_arg,
    gps_trust_device_api_key_arg,
    container1
  ])