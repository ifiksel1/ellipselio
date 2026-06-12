<div align="center">
    <h1>EllipseLIO</h1>
    <a href="https://github.com/v4rl-ucy/ellipselio"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/v4rl-ucy/ellipselio"><img src="https://img.shields.io/badge/ROS2-blue" /></a>
    <a href="https://github.com/v4rl-ucy/ellipselio"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://github.com/v4rl-ucy/ellipselio/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-MIT-green.svg" alt="MIT License" /></a>
    <br />
    <br />
    <a href="https://youtu.be/eIZ8CK4TAuA">Video</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/v4rl-ucy/ellipselio/blob/main/README.md">Install</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="http://arxiv.org/abs/2605.21150">Paper</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/v4rl-ucy/ellipselio/issues">Report Issues</a>
  <br />
  <br />
  <p align="center"><img src=ellipselio.gif alt="animated" /></p>

  [EllipseLIO][arXivlink] is an **Adaptive LiDAR Inertial Odometry Approach with an Ellipsoid Representation**
</div>

[arXivlink]: http://arxiv.org/abs/2605.21150

## ROS2 Humble and Jazzy

### Build

```sh
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone git@github.com:v4rl-ucy/ellipselio.git
cd ..
colcon build --packages-select ellipselio --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source ~/colcon_ws/install/setup.bash
```

### Run standalone with a bag file

```sh
ros2 launch ellipselio ellipselio_standalone.launch.py config_file:=<config_file_name>
ros2 bag play --clock <imu_rate> <bag_folder> --topics <lidar_topic> <imu_topic>
```

### Included dataset configs

| Config file | Dataset |
| --- | --- |
| [`config/os128_ncd.yaml`](config/os128_ncd.yaml) | [`Newer College Multi-Cam`](https://ori-drs.github.io/newer-college-dataset/multi-cam/) |
| [`config/os64_ncd.yaml`](config/os64_ncd.yaml) | [`Newer College Stereo-Cam`](https://ori-drs.github.io/newer-college-dataset/stereo-cam/) |
| [`config/qt64_spires.yaml`](config/qt64_spires.yaml) | [`Oxford Spires`](https://dynamic.robots.ox.ac.uk/datasets/oxford-spires/) |
| [`config/vlp16_bot.yaml`](config/vlp16_bot.yaml) | [`BotanicGarden`](https://github.com/robot-pesg/BotanicGarden) |
| [`config/vlp16_geode.yaml`](config/vlp16_geode.yaml) | [`GEODE Alpha`](https://thisparticle.github.io/geode/) |
| [`config/os64_geode.yaml`](config/os64_geode.yaml) | [`GEODE Beta`](https://thisparticle.github.io/geode/) |
| [`config/vlp16_graco.yaml`](config/vlp16_graco.yaml) | [`GRACO`](https://github.com/SYSU-RoboticsLab/GrAco) |

### Run standalone with live data

```sh
ros2 launch ellipselio ellipselio_standalone.launch.py config_file:=<config_file_name> use_sim_time:=false
```

## Photometric fusion (COIN-LIO intensity residual)

EllipseLIO can optionally fuse a **LiDAR-intensity direct-photometric residual**, ported from
[COIN-LIO][coinliolink], into the same IKFoM update, so geometry-degenerate viewpoints (blank
walls, smooth surfaces) are additionally constrained by reflectivity texture. Both frameworks
share the FAST-LIO2 state manifold, so the photometric residual is stacked alongside the geometric
one in `TensorRegistration` with no change to the state.

Enable it with a `photometric:` block in the config (see [`config/os1_64_ouster.yaml`](config/os1_64_ouster.yaml)):

```yaml
photometric:
    enable: true
    cloud_topic: "/ouster/points"   # full-resolution LiDAR cloud (NOT the downsampled internal one)
    photo_scale: 1.0e-9             # photometric row weight (see note below)
    num_features: 65
    patch_size: 5
```

| Parameter | Description |
| --- | --- |
| `enable` | Turn the photometric channel on/off (geometric-only when `false`) |
| `cloud_topic` | Full-resolution LiDAR cloud used to build the intensity image (the raw driver cloud, not the downsampled internal one) |
| `photo_scale` | Constant photometric row weight. The photometric Jacobian is ~O(10³) vs the geometric O(1), so in the information-form update it must be ~`1e-9` to balance — larger values let it dominate and diverge. Write it as `1.0e-9` (`1e-9` parses as a string) |
| `num_features` / `patch_size` | Tracked features per scan and patch size |

The intensity image is built from the **full-resolution** `/ouster/points` (every valid return),
not EllipseLIO's downsampled internal cloud, which is too sparse for a usable image. Either an
organized or an unorganized driver cloud works — points are projected individually and empty/NaN
returns are skipped. Beam altitude angles default to the OS1-64 values; override with
`photometric.beam_altitude_angles` for other sensors.

[coinliolink]: https://github.com/ethz-asl/coin-lio

## :pencil: Citation

If you use EllipseLIO please cite our preprint on [arXiv][arXivLink]
```
@article{border2026ellipselio,
   author = {Border, Rowan and Chli, Margarita},
   journal = {arXiv},
   title = {{EllipseLIO}: Adaptive {LiDAR} Inertial Odometry with an Ellipsoid Representation},
   url = {http://arxiv.org/abs/2605.21150},
   year = {2026}
}

```

## :pray: Acknowledgements

Many thanks to the authors of [FAST-LIO2][fastliolink], [IKFoM][ikfomlink], [i-Octree][ioctreelink], and [COIN-LIO][coinliolink] for open-sourcing their work, which made the development of EllipseLIO possible. 

[fastliolink]: https://github.com/hku-mars/FAST_LIO
[ikfomlink]: https://github.com/hku-mars/IKFoM
[ioctreelink]: https://github.com/zhujun3753/i-octree

## :mailbox: Contact information

If you have any questions, please do not hesitate to contact
* [Rowan Border][rblink] :envelope: rborder `dot` robots `at` gmail `dot` com

[rblink]: https://github.com/rowanborder
