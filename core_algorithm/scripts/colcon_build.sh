#!/bin/bash

# skip sensor dependend packages automatically.
# type package name(s) behind to build specified package(s)

set -e

SKIP_PKGS="phidgets_drivers libphidget22 phidgets_accelerometer phidgets_analog_inputs phidgets_analog_outputs phidgets_api \
 phidgets_digital_inputs phidgets_digital_outputs phidgets_gyroscope phidgets_high_speed_encoder phidgets_ik phidgets_magnetometer \
 phidgets_motors phidgets_msgs phidgets_spatial phidgets_stepper phidgets_temperature ydlidar_ros2_driver ydlidar_sdk obstacle_detector"

cd ../../..

if [ $# -gt 0 ]; then
    echo "🛠 Building selected packages"
    colcon build --packages-select "$@"
else
    echo "🛠 Building all except skipped packages"
    colcon build --packages-skip $SKIP_PKGS
fi
