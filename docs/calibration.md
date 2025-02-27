# Calibration


We use camera-calibration ros package

Example on how to calibration for side_right camera
```bash
ros2 run camera_calibration cameracalibrator -c side_right --square 0.0435 --size 5x8 -p chessboard -c side_right --ros-args -r image:=/camera/side_right/image_raw
```
