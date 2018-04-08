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

## Reference Frames
### Camera frame
* Origin - Center of image
* Z - Along boresight
* Y - Down
* X - Right

### Quad
* Origin - Center of primary GPS antenna
* Z - Up
* Y - Left
* X - Forward

### Inertial (ENU)
* Origin - On top of the chimney
* Z - Up
* Y - North
* X - East

## Notation
### Rotation Matrix vs Direction Cosine Matrix
To dispell ambiguity, I make a distinction between Rotation Matrices and Direction Cosine Matrices.

Rotation matrices describe how frame B is rotated away from frame A as
viewed from frame A. The columns of the rotation matrix R can be
interpreted as the unit vectors of B as viewed in frame A. A rotation of
frame B away from frame A is denoted by 'RBA'.

Direction cosine matrices are operators that take a vector expressed in frame A and express it in frame B. Direction cosine matrices are denoted by 'DA2B'.
