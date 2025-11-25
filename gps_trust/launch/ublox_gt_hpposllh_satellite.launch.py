"""Launch ublox_dgnss_node publishing high precision Lon/Lat messages"""

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description for ublox_dgnss components with launch arguments."""

    device_family = LaunchConfiguration("device_family")

    # Define parameter names and default values
    param_defaults = {
        "DEVICE_FAMILY": "F9P",
        "CFG_USBOUTPROT_NMEA": "false",
        "CFG_UART1OUTPROT_UBX": "false",
        "CFG_UART1OUTPROT_NMEA": "false",
        "CFG_UART1OUTPROT_RTCM3X": "false",
        "CFG_RATE_MEAS": "2000",
        "CFG_RATE_NAV": "1",
        "CFG_SEC_SPOOFDET_SIM_SIG_DIS": "false",
        "CFG_SEC_JAMDET_SENSITIVITY_HI": "true",
        "CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB": "1",
        "CFG_MSGOUT_UBX_NAV_STATUS_USB": "1",
        "CFG_MSGOUT_UBX_NAV_COV_USB": "1",
        "CFG_MSGOUT_UBX_NAV_SAT_USB": "1",
        "CFG_MSGOUT_UBX_NAV_ORB_USB": "1",
        "CFG_MSGOUT_UBX_NAV_SIG_USB": "1",
        "CFG_MSGOUT_UBX_SEC_SIG_USB": "1",
        "CFG_MSGOUT_UBX_SEC_SIGLOG_USB": "1",
        "CFG_MSGOUT_UBX_RXM_COR_USB": "1",
        "CFG_MSGOUT_UBX_RXM_RAWX_USB": "1",
        "CFG_SIGNAL_BDS_ENA": "true",
        "CFG_SIGNAL_GLO_ENA": "true",
        "CFG_SIGNAL_GAL_ENA": "true",
        "CFG_SIGNAL_GPS_ENA": "true",
        "CFG_SIGNAL_SBAS_ENA": "true",
        "CFG_SIGNAL_QZSS_ENA": "true",
        "log_level": "INFO",
    }

    # Declare all arguments with TextSubstitution
    declare_args = [
        DeclareLaunchArgument(name, default_value=TextSubstitution(text=default), description=f"Sets parameter {name}")
        for name, default in param_defaults.items()
    ]

    # Create runtime substitution dictionary for parameters
    runtime_params = [{key: LaunchConfiguration(key)} for key in param_defaults.keys()]

    # Define the composable node container
    container1 = ComposableNodeContainer(
        name="ublox_dgnss_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        composable_node_descriptions=[
            ComposableNode(
                package="ublox_dgnss_node",
                plugin="ublox_dgnss::UbloxDGNSSNode",
                name="ublox_dgnss",
                parameters=runtime_params,
            )
        ],
    )

    return launch.LaunchDescription(declare_args + [container1])
