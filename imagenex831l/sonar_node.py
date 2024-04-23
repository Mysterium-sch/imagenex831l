#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from imagenex831l.msg import ProcessedRange, RawRange
from imagenex831l.imagenex831l_driver import Imagenex831L
from imagenex831l.srv import SonarParameters


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
        self.sensor = Imagenex831L()
        self.set_sonar_parameters()
        #self.parameter_server = Server(self, Imagenex831LConfig, self.parameters_callback)
        self.first_exception_time = None

    #def parameters_callback(self, config, level):
    #    config = self.sensor.set_parameters(config)
    #    return config

    def set_sonar_parameters(self):
        client = self.create_client(SonarParameters, 'set_sonar_parameters')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        request = SonarParameters.Request()
        request.max_range = 60
        request.step_direction = 0
        request.start_gain = 0
        request.absorption = 0
        request.train_angle = 0
        request.sector_width = 0
        request.step_size = 0
        request.pulse_length = 0
        request.min_range = 0
        request.pitch_roll_mode = 0
        request.profile_mode = 0
        request.motor_mode = 0
        request.frequency = 2150

        future = client.call_async(request)

        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                if future.result() is not None:
                    response = future.result()
                    if response.success:
                        self.get_logger().info("Sonar parameters set successfully!")
                    else:
                        self.get_logger().error("Failed to set sonar parameters.")
                break

    def spin(self):
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

            self.get_logger().info('Publishing data...')
            self.spin_once()

        self.sensor.close_connection()

def main(args=None):
    rclpy.init(args=args)
    sonar_node = SonarNode()
    sonar_node.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
