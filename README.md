# imagenex831l

ROS2 driver for the [IMAGENEX831L](https://imagenex.com/products/831l-pipe-profiling).

## Getting started
This ROS2 package connects to the sonar and publishes both `raw_range` and `processed_range` custom messages. Originally created by Alberto Quattrini Li for ROS1 Melodic in Python 2, the old package can be found [here](https://github.com/quattrinili/imagenex831l).

This package has been converted to ROS2. The parameters for the node can be found in `/cfg/sonar.yaml`. These parameters are automatically loaded from this file when the launch file is called. Additionally, a `sonar_health` topic was created to publish the connection status of the sonar (active if the sonar is properly connected and not active if there is no connection). The node publishes the sonar's range at a rate of 1/frequency, which can be set in the YAML file.

### Prerequisites

Set the IP address of the computer connected to the sensor.

### Running
```
ros2 launch imagenex831l sonar.launch
```

## Authors
* Lilian Lamb - [Autonomous Field Robotics Lab](https://afrl.cse.sc.edu), University of South Carolina
  
#### Source of orginal imagenex831l driver
* [Alberto Quattrini Li](https://sites.google.com/view/albertoq) - [Reality and Robotics Lab](https://rlab.cs.dartmouth.edu), Dartmouth College
* Sharmin Rahman - [Autonomous Field Robotics Lab](https://afrl.cse.sc.edu), University of South Carolina

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
