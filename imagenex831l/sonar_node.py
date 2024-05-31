#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from imagenex831l_ros2.msg import ProcessedRange, RawRange
from imagenex831l.imagenex831l_driver import Imagenex831L
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String

SENSOR_NAME = 'imagenex831l'
SONAR_TOPIC_NAME = 'range'
SONAR_RAW_TOPIC_NAME = 'range_raw'
SONAR_HEALTH_TOPIC_NAME = 'sonar_health'
POLL_FREQUENCY = 10
RESET_TIMEOUT = 1

class SonarNode(Node):
    
    def __init__(self):
        super().__init__('imagenex831l')

        self.declare_parameter("device", "")
        device = self.get_parameter('device').value

        self.range_pub = self.create_publisher(ProcessedRange, f'{SENSOR_NAME}/{SONAR_TOPIC_NAME}', 10)
        self.range_raw_pub = self.create_publisher(RawRange, f'{SENSOR_NAME}/{SONAR_RAW_TOPIC_NAME}', 10)
        self.sonar_health = self.create_publisher(String, f'{SENSOR_NAME}/{SONAR_HEALTH_TOPIC_NAME}', 10)
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_range', rclpy.Parameter.Type.INTEGER),
                ('step_direction', rclpy.Parameter.Type.INTEGER),
                ('start_gain', rclpy.Parameter.Type.INTEGER),
                ('absorption', rclpy.Parameter.Type.INTEGER),
                ('train_angle', rclpy.Parameter.Type.INTEGER),
                ('sector_width', rclpy.Parameter.Type.INTEGER),
                ('step_size', rclpy.Parameter.Type.INTEGER),
                ('pulse', rclpy.Parameter.Type.INTEGER),
                ('min_range', rclpy.Parameter.Type.INTEGER),
                ('pitch_roll_mode', rclpy.Parameter.Type.INTEGER),
                ('profile_mode', rclpy.Parameter.Type.INTEGER),
                ('motor_mode', rclpy.Parameter.Type.INTEGER),
                ('frequency', rclpy.Parameter.Type.INTEGER),
                ('poll_frequency', rclpy.Parameter.Type.INTEGER)
            ])
        
        self.max_range = self.get_parameter('max_range').value
        self.step_direction = self.get_parameter('step_direction').value
        self.start_gain = self.get_parameter('start_gain').value
        self.absorption = self.get_parameter('absorption').value
        self.train_angle = self.get_parameter('train_angle').value
        self.sector_width = self.get_parameter('sector_width').value
        self.step_size = self.get_parameter('step_size').value
        self.pulse = self.get_parameter('pulse').value
        self.min_range = self.get_parameter('min_range').value
        self.pitch_roll_mode = self.get_parameter('pitch_roll_mode').value
        self.profile_mode = self.get_parameter('profile_mode').value
        self.motor_mode = self.get_parameter('motor_mode').value
        self.frequency = self.get_parameter('frequency').value
        self.poll_frequency = self.get_parameter('poll_frequency').value if self.has_parameter('poll_frequency') else POLL_FREQUENCY

        self.sensor = Imagenex831L()
        self.get_logger().info(str(self.get_parameter('frequency').value))
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.first_exception_time = None
        self.sensor.set_parameters(self.step_direction, self.start_gain, self.absorption, self.train_angle, self.sector_width, self.step_size, self.pulse, self.max_range, self.min_range, self.pitch_roll_mode, self.profile_mode, self.motor_mode, self.frequency)

        timer_period = 1.0 / self.poll_frequency  # Timer period in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def parameters_callback(self, params):
        self.sensor.set_parameters(self.step_direction, self.start_gain, self.absorption, self.train_angle, self.sector_width, self.step_size, self.pulse, self.max_range, self.min_range, self.pitch_roll_mode, self.profile_mode, self.motor_mode, self.frequency)
        return SetParametersResult(successful=True)

    def timer_callback(self):
        sonar_msg = ProcessedRange()
        sonar_raw_msg = RawRange()
        current_time = self.get_clock().now()

        try:
            self.sensor.send_request()
            raw_data = self.sensor.read_data()

            sonar_raw_msg.header.stamp = current_time.to_msg()
            sonar_raw_msg.header.frame_id = 'sonar'
            sonar_raw_msg.data = raw_data
            self.get_logger().debug(str(sonar_raw_msg))

            sonar_msg.header.stamp = current_time.to_msg()
            sonar_msg.header.frame_id = 'sonar'
            self.sensor.interpret_data(raw_data, sonar_msg)

            self.range_raw_pub.publish(sonar_raw_msg)
            self.range_pub.publish(sonar_msg)
            msg = String()
            msg.data = "Active"
            self.sonar_health.publish(msg)

            if self.first_exception_time:
                self.first_exception_time = None

        except Exception as e:
            msg = String()
            msg.data = "Not Active"
            self.sonar_health.publish(msg)
            if self.first_exception_time is None:
                self.get_logger().error(f'Exception when reading sonar data: {e}')
                self.first_exception_time = current_time
            else:
                if current_time - self.first_exception_time > rclpy.duration.Duration(seconds=RESET_TIMEOUT):
                    self.get_logger().error('Sonar sensor not ready')
                    self.sensor.close_connection()
                    self.timer.cancel()

    def destroy_node(self):
        self.sensor.close_connection()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    sonar_node = SonarNode()
    rclpy.spin(sonar_node)
    sonar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
