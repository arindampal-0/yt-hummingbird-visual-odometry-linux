# Monocular Visual Odometry
### Visual Odometry Series by [Hummingbird](https://www.youtube.com/@hummingbird19)
- [Visual Odometry Series - Part 1 (Concept and Math)](https://www.youtube.com/watch?v=H_1OtbMD-sE)
- [Visual Odometry Series - Part 2 (C++ Code for Monocular Visual Odometry)](https://www.youtube.com/watch?v=TuJLOUxZKMQ)

### Download Kitty dataset
https://www.cvlibs.net/datasets/kitti/eval_odometry.php

## Dependencies
Install OpenCV
```shell
sudo apt install libopencv-dev
```

## Setup
```shell
# compile
cmake -B build
cmake --build build

# run
build/vo path_to_kitty_dataset
```
