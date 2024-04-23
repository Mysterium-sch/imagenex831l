#!/usr/bin/env python3

import rclpy
from imagenex831l.imagenex831l_driver import Imagenex831L

def main(args=None):
    rclpy.init(args=args)

    # Initialize Imagenex831L
    sonar = Imagenex831L()

    # Set parameters
    sonar.absorption = 171
    sonar.start_gain = 21
    sonar.pulse = 10
    sonar.range = 1
    sonar.current_sector_width = 0
    sonar.current_train_angle = 0

    try:
        # Send request and read data
        sonar.send_request()
        data = sonar.read_data()

        # Interpret data
        sonar.interpret_data(data)

    except Exception as e:
        print(f"Error while communicating with sonar: {e}")

    finally:
        # Close connection
        sonar.close_connection()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
