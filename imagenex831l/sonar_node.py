#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from imagenex831l_ros2.msg import ProcessedRange, RawRange
from imagenex831l.imagenex831l_driver import Imagenex831L
from rcl_interfaces.msg import SetParametersResult

SENSOR_NAME = 'imagenex831l'
SONAR_TOPIC_NAME = 'range'
SONAR_RAW_TOPIC_NAME = 'range_raw'
POLL_FREQUENCY = 1000
RESET_TIMEOUT = 1

class SonarNode(Node):
    
    def __init__(self):
        super().__init__('imagenex831l')
        self.range_pub = self.create_publisher(ProcessedRange, f'{SENSOR_NAME}/{SONAR_TOPIC_NAME}', 10)
        self.range_raw_pub = self.create_publisher(RawRange, f'{SENSOR_NAME}/{SONAR_RAW_TOPIC_NAME}', 10)
        
        self.frequency = self.get_parameter('poll_frequency').value if self.has_parameter('poll_frequency') else POLL_FREQUENCY
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
            ('frequency', rclpy.Parameter.Type.INTEGER)
            ])

        self.add_on_set_parameters_callback(self.parameters_callback)
        self.first_exception_time = None
        self.sensor = Imagenex831L()

    def parameters_callback(self, params):
        self.sensor.set_parameters(params)
        return SetParametersResult(successful=True)


    def spin(self):
        node = self.create_rate(self.frequency)
        while rclpy.ok():
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

                if self.first_exception_time:
                    self.first_exception_time = None

            except Exception as e:
                if self.first_exception_time is None:
                    self.get_logger().error(f'Exception when reading sonar data: {e}')
                    self.first_exception_time = current_time
                else:
                    if current_time - self.first_exception_time > rclpy.Duration(RESET_TIMEOUT):
                        self.get_logger().error('Sonar sensor not ready')
                        self.sensor.close_connection()
                        break

            node.sleep()
        self.sensor.close_connection()

def main(args=None):
    rclpy.init(args=args)
    sonar_node = SonarNode()
    sonar_node.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
