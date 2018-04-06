# Balloon Bundle Adjuster
Detects and estimated the position of red and blue balloons using GPS and Bundle Adjustment.

## Requirements
* [Eigen](https://github.com/eigenteam/eigen-git-mirror)
* [OpenCV](https://github.com/opencv/opencv)
* [Google Ceres](http://ceres-solver.org/installation.html)

## Build and Run
First, copy balloon images and their corresponding poses into the images directory. Then build and run:

```
mkdir build-release
cd build-release
cmake ..
make -j4
./BalloonBundleAdjuster
```


