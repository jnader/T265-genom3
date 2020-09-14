# How to build calibration tools

## Setup Vicon DataStream SDK

If this SDK is not already installed, get and build from source:

```
$ cd $DRONE_WS
$ git clone https://gitlab.inria.fr/lagadic/ViconDataStreamSDK_1.8.0_105615h.git
$ cd ViconDataStreamSDK_1.8.0_105615h/18.04/Linux64
$ make -j4 CONFIG=Release VERBOSE=ON
```

This makes available `$DRONE_WS/ViconDataStreamSDK_1.8.0_105615h/18.04/Linux64/bin/Release/libViconDataStreamSDK_CPP.so`

## Build project

```
$ cd $DRONE_WS
$ git clone https://gitlab.inria.fr/lagadic/uavvo-genom3.git
$ cd T265/calibration
$ mkdir build; cd build
$ cmake ../ -DViconDataStreamSDK_HOME=$DRONE_WS/ViconDataStreamSDK_1.8.0_105615h/18.04/Linux64 \
            -DVISP_DIR=$VISP_WS/visp-build
```

## Usage <a name="usage"></a>

The calibration procedure is similar to the one explained in this [tutorial](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-calibration-extrinsic.html)

To improve calibration accuracy, you should follow the following steps:
1) Maximize the rotation angle for interstation rotations.
2) Minimize the distance between the camera lens (T265) center and the calibration block (initial pose reference frame). In our case, don't move the drone too much between 2 acquisitions.
3) Between 2 different acquisitions, don't move the drone too much.


### Acquire drone poses from Vicon and from T265
- Launch calibration script:
```
$ ./mk-acquire-calib-data
```

- A white image will appear waiting for you left click to acquire a pose.

- Before the first acquisition, move the drone randomly with your hand so that tracking confidence increases.

- Move the drone while following the [steps](#usage) and `left` click every time on the white image to acquire new data.

- Acquire around 9 set of data to have a good accuracy.

### Estimate end-effector to camera transformation

Follow these instructions: <https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-calibration-extrinsic.html#calib_ext_tsai>

```
$ $VISP_WS/visp-build/tutorial/calibration/tutorial-hand-eye-calibration --ndata <number of images or robot poses>
```
