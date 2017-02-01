# LIDAR-Lite v3 Arduino Library

* [Product Page](https://buy.garmin.com/en-US/US/oem/sensors-and-boards/lidar-lite-v3/prod557294.html)
* [Operating Manual and Technical Specifications](http://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf)

This library provides quick access to all the basic functions of LIDAR-Lite
via the National Instruments RoboRio I2c interface.

For detailed specifications, pinout, and connection diagrams, see the manual linked above.

**Note:**This library was forked from the Arudiono library available here. [https://github.com/garmin/LIDARLite_v3_Arduino_Library](https://github.com/garmin/LIDARLite_v3_Arduino_Library)

## Example Code
### GetDistanceI2c
This demonstration shows how to read distance using the I2C interface and choose preset configurations.

### ShortRangeHighSpeed
This example shows a method to run LIDAR-Lite at high speed for short range applications. It combines a variety of settings to trade off range and accuracy for very fast measurements.

## Version History
* [v1.0.2](https://github.com/garmin/LIDARLite_v3_Arduino_Library/tree/v1.0.2) - Library Manager Update
* [v1.0.1](https://github.com/garmin/LIDARLite_v3_Arduino_Library/tree/v1.0.1) - Release to Library Manager
* [1.0.0](https://github.com/garmin/LIDARLite_v3_Arduino_Library/tree/1.0.0) - Initial release

## License
Copyright (c) 2016 Garmin Ltd. or its subsidiaries. Distributed under the Apache 2.0 License.
See [LICENSE](LICENSE) for further details.
