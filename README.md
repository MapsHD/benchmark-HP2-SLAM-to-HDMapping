## Hint

Please change branch to [Bunker-DVI-Dataset-reg-1](https://github.com/MapsHD/benchmark-HP2-SLAM-to-HDMapping/tree/Bunker-DVI-Dataset-reg-1) for quick experiment.

## Example Dataset:

Download the dataset from [Bunker DVI Dataset](https://charleshamesse.github.io/bunker-dvi-dataset/)

# benchmark-HP2-SLAM-to-HDMapping

Runs [HP2-SLAM](https://github.com/cogniboticslab/hp2slam) on a rosbag and
converts the output to an [HDMapping](https://github.com/MapsHD/HDMapping)
session.

HP2-SLAM (N. Tran et al., "HP2-SLAM: Adaptive Hybrid ICP for Robust and
Efficient LiDAR SLAM", arXiv 2608.14996) is a LiDAR-only SLAM built on
KISS-SLAM. Its odometry, hybrid-icp, replaces KISS-ICP's point-to-point
registration with a hybrid one: every correspondence is classified by the
planarity of its map neighbourhood and contributes a point-to-plane or a
point-to-point residual, weighted by the planar fraction of the scan. Local
maps, MapClosures loop detection and a g2o pose graph come from KISS-SLAM.

## Prerequisites

- Docker
- A ROS 1 bag file or a ROS 2 bag directory with a `sensor_msgs/PointCloud2`
  topic. A per-point time field (`t`, `timestamp` or `time`) lets HP2-SLAM
  deskew the scans; without it they are used as they are.

HP2-SLAM needs no ROS: it reads the bag through the
[rosbags](https://pypi.org/project/rosbags/) library.

## Step 1 — Clone with submodules

```bash
git clone https://github.com/MapsHD/benchmark-HP2-SLAM-to-HDMapping.git --recursive
cd benchmark-HP2-SLAM-to-HDMapping
```

## Step 2 — Build the Docker image

```bash
docker build -t hp2-slam_standalone .
```

This installs:
- Ubuntu 22.04, Python 3.10, Eigen, SuiteSparse, TBB
- hybrid-icp (vendored in HP2-SLAM) and HP2-SLAM as pip packages; g2o and
  SuiteSparse for the pose graph are built by HP2-SLAM's own CMake
- rosbags, laspy and the HDMapping converter

## Step 3 — Run the pipeline

```bash
chmod +x docker_session_run-hp2-slam.sh
./docker_session_run-hp2-slam.sh /path/to/input.bag /path/to/output/dir
```

The input is a ROS 1 `.bag` file or a ROS 2 bag directory. With no arguments, a
GUI file selector is used (requires `zenity`).

| Variable | Default | Meaning |
|----------|---------|---------|
| `CONFIG_FILE` | none (author's defaults) | HP2-SLAM parameter file, see below |
| `TOPIC` | `/livox/pointcloud` | `PointCloud2` topic in the bag |
| `VIS` | `1` | HP2-SLAM's Open3D live viewer (off without a `DISPLAY`) |
| `POINT_SKIP` | `1` | the converter keeps every Nth point |
| `CONVERT_RESULTS_DIR` | none | an existing results directory in the container, e.g. `/output/hp2-slam-results/<timestamp>`: skip SLAM, only convert |

**Parameters.** Without `CONFIG_FILE`, HP2-SLAM runs with its defaults
(`odometry.preprocessing.max_range` 100 m, `local_mapper.splitting_distance`
100 m, ...). To change them, write the defaults to a file with HP2-SLAM's own
tool and edit it:

```bash
docker run --rm -u 1000:1000 -v "$PWD":/work -w /work hp2-slam_standalone hp2_slam_dump_config
CONFIG_FILE=hp2_slam.yaml ./docker_session_run-hp2-slam.sh /path/to/input.bag /path/to/output/dir
```

HP2-SLAM's README suggests for indoor data `max_range` 50 (which sets the voxel
size to 0.5 m) and a smaller `splitting_distance`.

Before anything starts, the script checks that the topic is in the bag as
`PointCloud2` with `x`, `y`, `z` fields.

**What happens:**

HP2-SLAM runs offline: it reads the bag as fast as it processes the scans and
exits when the bag is done. There is no playback and no recording. With `VIS=1`
its Open3D viewer shows the local map and the trajectory while it runs
(`SPACE` pauses, `F` toggles the follow/top-down view). Do not close the viewer
or press `ESC`: that exits HP2-SLAM without writing results.

When HP2-SLAM has finished, the same container converts its output into the
HDMapping session format.

## Step 4 — Open in HDMapping

Output files appear in `<output_dir>/output_hdmapping-hp2-slam/`:

```
lio_initial_poses.reg
poses.reg
scan_lio_0.laz
...
session.json
trajectory_lio_0.csv
...
```

Open `session.json` with the
[multi_view_tls_registration_step_2](https://github.com/MapsHD/HDMapping)
application.

HP2-SLAM's own results stay in `<output_dir>/hp2-slam-results/<timestamp>/`
(`latest` links to the newest run): `<sequence>_poses_tum.txt`,
`trajectory.g2o`, `trajectory.png` with the loop closures, the local maps and
`config.yml`. The log is in `<output_dir>/logs/hp2-slam.log`.

## Notes on HP2-SLAM

**Output.** HP2-SLAM writes one pose per scan, in scan order, in TUM format
(`timestamp tx ty tz qx qy qz qw`), stamped with the bag time of the scan. The
poses are the final ones after loop closing and the pose-graph optimisation.
The converter places every scan of the bag with its pose (scan `i` with pose
`i`, checked by time; the nearest pose in time otherwise) and writes the
chunked session. Points closer than 0.5 m to the sensor are left out, as in
the other converters.

**Timestamps:** the benchmark does not change HP2-SLAM's clock. The
trajectory times are the bag times of the scans.

**Patch.** HP2-SLAM contains three debugging leftovers that stall or flood an
unattended run:
- the ICP loop clears the terminal and prints a status bar on every iteration;
- after convergence, when the planar fraction exceeds 0.95, it writes a PLY of
  the correspondences and blocks until `ENTER` is pressed;
- the viewer saves a full-HD screenshot of every frame into `./renders`.

The Docker build switches the first two off (behind `kDebugOutput = false`)
and removes the screenshots with
[overlay/patches/hp2slam_no_debug_leftovers.patch](overlay/patches/hp2slam_no_debug_leftovers.patch);
the estimate is unchanged. The build also redirects a few `kiss_icp` imports
left in the vendored dataloaders to the vendored `hybrid_icp` package.

**Optional outputs.** The run script calls HP2-SLAM's pipeline stage by stage:
SLAM and the pose file are required, while the closure plot, the local maps,
the g2o graph and the logs are each optional, so a failure in one of them is
reported and does not cost the run.

## Contact

januszbedkowski@gmail.com
