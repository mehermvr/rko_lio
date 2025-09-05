# RKO_LIO - LiDAR-Inertial Odometry Without Sensor-Specific Modelling

RKO_LIO is a LiDAR-inertial odometry system that is simple to deploy on different sensor configurations and robotic platforms with as minimal a change in configuration as necessary.

We have tested our system on data from multiple different sensors and platforms.
For example, we've used data from different versions of LiDARs from Velodyne, Ouster, Hesai, Livox, Robosense, all without any change in how we process the incoming LiDAR data, i.e., you don't specify which LiDAR you're using.
For the IMU, we only require the accelerometer and gyroscope readings, the bare minimum.

You don't need to look up manufacturer spec sheets when you want to use our approach on your data.
Nor do you need to calibrate measurement noise, bias specifications, or other sensor-specific parameters of your IMU or LiDAR.
We do not ask these values as parameters because we don't use them.

All you need to provide is the extrinsic transformation between the IMU and LiDAR and you can start using our system for your LiDAR-inertial odometry needs!


<p align="center">
  <img src="docs/example_multiple_platforms.png" alt="Visualization of odometry system running on data from four different platforms in four different environments" />
  <br />
  <em>Four different platforms, four different environments, one odometry system</em>
</p>

In case you already have a rosbag which contains a TF tree, using our system is as simple as calling

```bash
rko_lio /path/to/rosbag
```

For further details, please refer to the [Python readme](python/README.md).

## Setup

### ROS2

> We are working on getting the odometry package into the ROS index, so you can install it using system package managers instead of building from source.

We currently support ROS2 Jazzy, and plan to additionally support Humble, Kilted and Rolling.

Clone the repository into your ROS workspace and then

```bash
colcon build --packages-select rko_lio # --symlink-install --event-handlers console_direct+ 
```

Note that we have some [default build configuration options](https://colcon.readthedocs.io/en/released/user/configuration.html) which should automatically get picked up by colcon.

We have a few dependencies, but as long as the defaults apply, the package should build without any further consideration.
If you encounter any issues, please check [BUILD.md](BUILD.md) for further details or open an issue afterwards.


Please refer to the [ROS readme](ros/README.md) for further ROS-specific details.

## Python

> We are working on providing the python interface via the Python Package Index, so you will be able to install the system using pip (or some other frontend) instead of building from source.

We provide a python interface to our system which can be convenient to investigate recorded data offline as you don't need to setup a ROS environment first.

As before, clone the repository somewhere and then

```bash
cd python && pip install .
```

There's a few optional dependencies that will need to be pulled in based on what part of the interface you use.
For example, inspecting rosbag data will require the `rosbags` package, and enabling visualization will require the `rerun-sdk` package; you will be prompted when a dependency is missing. 
In case you don't mind pulling in a few additional dependencies and want everything available, instead run

```bash
cd python && pip install ".[all]"
# or from the repository root
make python
```

**Please note:** the ROS version is the intended way to use our odometry system on a robot.
The python version is slower than the ROS version, not on the odometry itself, but on how we read incoming data, i.e. data-loading.
Without getting into details, if you can, you should prefer using the ROS version.
We also provide a way to directly inspect and run our odometry on recorded rosbags (see [ROS usage](ros/README.md) which still has the same performance benefit over the python version.
The python interface is merely meant to be a convenience.

With that out of the way, a special mention goes out to [Rerun](https://rerun.io/) for providing an extremely easy-to-use but highly performative visualization system.
Without this, I probably would not have made a python interface at all.

## A note on transformations

Please refer to the [ROS](ros/README.md) or [Python](python/README.md) readmes for specific use instructions, but it bears mentioning here our convention for specifying sensor extrinsics, the one parameter we do require you to provide.

Throughout this package, we refer to transformations using `transform_<from-frame>_to_<to-frame>` or `transform_<from-frame>2<to-frame>`.
By this, we mean a transformation that converts a vector expressed in the `<from-frame>` coordinate system to the `<to-frame>` coordinate system.

Mathematically, this translates to:

$$
\mathbf{v}^{\text{to}} = {}^{\text{to}} \mathbf{T}_{\text{from}} \, \mathbf{v}^{\text{from}}
$$

The superscript on the vector indicates the frame in which the vector is expressed.

## License

This project is free software made available under the MIT license. For details, see the [LICENSE](LICENSE) file.

## RA-L Submission

You can check out the branch `ral_submission` for the version of the code used for submission to RA-L.
Please note that that branch is meant to be an as-is reproduction of the code used during submission and is not supported.
The `master` and release versions are vastly improved and are the recommended way to use this system.

## Acknowledgements

This package is inspired by and would not be possible without the work of [KISS-ICP](https://github.com/PRBonn/kiss-icp) and [Kinematic-ICP](https://github.com/PRBonn/kinematic-icp).
Additionally, we use and rely heavily on, either in the package itself or during development, [Bonxai](https://github.com/facontidavide/Bonxai), [PlotJuggler](https://github.com/facontidavide/PlotJuggler), [Rerun](https://github.com/rerun-io/rerun) and of course ROS itself.
