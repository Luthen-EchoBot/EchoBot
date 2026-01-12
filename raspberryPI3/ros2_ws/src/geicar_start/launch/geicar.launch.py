from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    joystick_node = Node(
        package="joystick",
        executable="joystick_ros2.py",
        emulate_tty=True
    )

    joystick_to_cmd_node = Node(
        package="joystick",
        executable="joystick_to_cmd",
        emulate_tty=True
    )

    can_rx_node = Node(
        package="can",
        executable="can_rx_node",
        emulate_tty=True
    )

    can_tx_node = Node(
        package="can",
        executable="can_tx_node",
        emulate_tty=True
    )

    car_control_node = Node(
        package="car_control",
        executable="car_control_node",
        emulate_tty=True
    )

    tcp_receiver_node = Node(
        package="pkg_tcp_receiver",
        executable="tcp_receiver",
        name="tcp_receiver_node",
        output="screen",
        emulate_tty=True
    )

    gui_data_sender_node = Node(
        package="pkg_gui_data_sender",
        executable="gui_data_sender",
        name="gui_data_sender_node",
        output="screen",
        emulate_tty=True
    )

    state_controller_node = Node(
        package="pkg_state_control",
        executable="state_control",
        name="state_control_node",
        emulate_tty=True
    )

    arduino_reader_node = Node(
        package="pkg_arduino",
        executable="arduino_reader",
        name="arduino_reader_node",
        emulate_tty=True
    )

    config_dir = os.path.join(get_package_share_directory('imu_filter_madgwick'), 'config')

    imu_filter_madgwick_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        parameters=[os.path.join(config_dir, 'imu_filter.yaml')],
        emulate_tty=True
    )

    system_check_node = Node(
        package="system_check",
        executable="system_check_node",
        emulate_tty=True
    )

    ld.add_action(joystick_node)
    ld.add_action(joystick_to_cmd_node)
    ld.add_action(can_rx_node)
    ld.add_action(can_tx_node)
    ld.add_action(car_control_node)
    ld.add_action(tcp_receiver_node)
    ld.add_action(imu_filter_madgwick_node)
    ld.add_action(system_check_node)
    ld.add_action(state_controller_node)
    ld.add_action(arduino_reader_node)
    ld.add_action(gui_data_sender_node)

    return ld
