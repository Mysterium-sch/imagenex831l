#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from imagenex831l_ros2.imagenex831l_driver import Imagenex831L

class SonarNode(Node):
    def __init__(self):
        super().__init__('imagenex831l')
        self.sensor = Imagenex831L()

    def spin(self):
        self.sensor.absorption = 171
        self.sensor.start_gain = 21
        self.sensor.pulse = 10
        self.sensor.range = 1
        self.sensor.current_sector_width = 0
        self.sensor.current_train_angle = 0

        try:
                # Send request and read data
            self.sensor.send_request()
            data = self.sensor.read_data()

                # Interpret data
            self.sensor.interpret_data(data)

        except Exception as e:
            print(f"Error while communicating with sonar: {e}")

        finally:
                # Close connection
            self.sensor.close_connection()


def main(args=None):
    rclpy.init(args=args)
    sonar_node = SonarNode()
    sonar_node.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
