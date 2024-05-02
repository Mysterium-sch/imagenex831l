#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from imagenex831l.msg import ProcessedRange, RawRange
from imagenex831l.imagenex831l_driver import Imagenex831L
from rcl_interfaces.msg import SetParametersResult

SENSOR_NAME = 'imagenex831l'
SONAR_TOPIC_NAME = 'range'
SONAR_RAW_TOPIC_NAME = 'range_raw'
POLL_FREQUENCY = 1000
RESET_TIMEOUT = 1

class SonarNode(Node):
    
    # Parameters


    def __init__(self):
        super().__init__('imagenex831l')
        self.range_pub = self.create_publisher(ProcessedRange, f'{SENSOR_NAME}/{SONAR_TOPIC_NAME}', 10)
        self.range_raw_pub = self.create_publisher(RawRange, f'{SENSOR_NAME}/{SONAR_RAW_TOPIC_NAME}', 10)
        
        #self.frequency = self.get_parameter('poll_frequency').value if self.has_parameter('poll_frequency') else POLL_FREQUENCY
        self.declare_parameters(
            namespace='',
            parameters=[
            ('max_range', -1),
            ('step_direction', -1),
            ('start_gain', -1),
            ('absorption', -1),
            ('train_angle', -1),
            ('sector_width', -1),
            ('step_size', -1),
            ('pulse', -1),
            ('min_range', -1),
            ('pitch_roll_mode', -1),
            ('profile_mode', -1),
            ('motor_mode', -1),
            ('frequency', -1)
            ])

        self.add_on_set_parameters_callback(self.parameters_callback)
        #self.parameter_server = Server(self, Imagenex831LConfig, self.parameters_callback)
        self.first_exception_time = None
        #self.sensor = Imagenex831L()

    def parameters_callback(self, params):
        for param in params:
            print(vars(param))
        return SetParametersResult(successful=True)


    def spin(self):
        #node = self.create_rate(self.frequency)
        self.get_logger().info(str(self.get_parameter('pulse')))
        while rclpy.ok():
            sonar_msg = ProcessedRange()
            sonar_raw_msg = RawRange()
            current_time = self.get_clock().now()

            try:
                #self.sensor.send_request()
                #raw_data = self.sensor.read_data()

                sonar_raw_msg.header.stamp = current_time.to_msg()
                sonar_raw_msg.header.frame_id = 'sonar'
                #sonar_raw_msg.data = raw_data
                self.get_logger().debug(str(sonar_raw_msg))

                sonar_msg.header.stamp = current_time.to_msg()
                sonar_msg.header.frame_id = 'sonar'
                #self.sensor.interpret_data(raw_data, sonar_msg)

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
                        #self.sensor.close_connection()
                        break

            #node.sleep()
        #self.sensor.close_connection()

def main(args=None):
    rclpy.init(args=args)
    sonar_node = SonarNode()
    sonar_node.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
