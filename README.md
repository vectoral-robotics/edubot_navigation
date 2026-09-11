# edubot_navigation

Nav2 and SLAM configuration for the [EduBot](https://github.com/vectoral-robotics) robot — by Vectoral.

## What it is

`edubot_navigation` holds the launch files, parameter sets and maps that run the
[Nav2](https://navigation.ros.org) navigation stack and SLAM on EduBot. It does
not contain robot drivers — it expects the robot to be running (odometry and TF
provided by [`edubot_bringup`](https://github.com/vectoral-robotics/edubot_bringup))
and layers autonomous navigation on top.

- `launch/` — `edubot_navigation.launch.py` (Nav2), `edubot_slam.launch.py` (mapping)
- `param/` — tuned Nav2 and SLAM parameter files
- `maps/` — saved maps

## Installation

Requires ROS 2 Humble and Nav2.

```bash
cd ~/ros2_ws/src
git clone https://github.com/vectoral-robotics/edubot_navigation.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y   # installs nav2_bringup
colcon build --packages-select edubot_navigation
source install/setup.bash
```

## Usage

With the robot already up (`ros2 launch edubot_bringup bringup.launch.py`):

```bash
# Build a map with SLAM
ros2 launch edubot_navigation edubot_slam.launch.py

# Run navigation against an existing map
ros2 launch edubot_navigation edubot_navigation.launch.py
```

Pair with [`edubot_viz`](https://github.com/vectoral-robotics/edubot_viz)
(`navigation_view.launch.py`) to set goals and visualize costmaps in RViz.

## Contributing

- Work on a short-lived feature branch and open a pull request against `main`
  (which is protected); changes land via PR review.
- Commit messages follow [Conventional Commits](https://www.conventionalcommits.org)
  (`feat:`, `fix:`, `docs:`, …). See `CLAUDE.md` for repo conventions.

## License

PolyForm Perimeter 1.0.0 (source-available) — see [LICENSE](LICENSE).

## Dashboard mapping and persistent navigation

The meta-repo starts `ros2 run edubot_navigation navigation_manager` alongside
ROS Core. Enable `features.navigation` in the dashboard robot profile and build
both the ROS and dashboard images from the updated sources (`make dev` in the
meta-repo). Fleet deployments need a new ROS image **and** dashboard image.

1. Open **Navigation**, choose **New map**, then hold the mapping controls
   to drive around. Release to stop; lost browser updates expire after 0.4 s.
2. Enter a unique map name and choose **Save map**. This snapshots the current
   occupancy map; SLAM continues. Existing maps are never overwritten.
3. Select the saved map and choose **Use map**. This stops SLAM and
   starts AMCL + Nav2 with the saved map. The selection survives restarts.
4. Choose **Set position** and drag on the map to specify the actual robot position and
   heading. Once localization, sensors and Nav2 are ready, choose **Set destination** and
   drag to send a destination and final heading. **Stop robot** cancels the real
   NavigateToPose action and immediately closes the navigation motor gate.
5. **End session** stops SLAM/Nav2 without deleting the selected map. **Open
   active map** resumes it; the same map also loads automatically on reboot.
   Loading a map never automatically starts a goal. Set the initial pose again
   after reboot or map activation; no previous physical position is assumed.

The manager stores maps as `maps/<name>/map.yaml` + `map.pgm`, and the active
selection as `active-map.json`, under `/state/edubot/navigation`. This is the
existing persistent `edubot_state` Docker volume. A map is activated locally on
the robot; no second file upload is needed. `EDUBOT_NAVIGATION_STATE` can change
the manager directory; if changed, also set `DASHBOARD_NAVIGATION_SOCKET` to its
`control.sock` path in the dashboard container. Both containers must see the
same directory. The control socket has no TCP listener.

`session.log` contains the latest SLAM/Nav2 launch output. The API reports a
process failure rather than silently falling back to another map. Saving uses
a staging directory and publishes only complete map pairs. A lifetime lock
prevents a second manager from starting competing navigation processes.

### Motion and sensor configuration

`dashboard_navigation.launch.py` uses `param/edubot_differential.yaml` and routes
all controller and recovery motion through `/navigation/cmd_vel`. The manager
forwards only longitudinal velocity and yaw to `/cmd_vel`, with limits of
0.20 m/s forward, 0.15 m/s reverse and 0.8 rad/s. Manual mapping uses at most
0.15 m/s and 0.6 rad/s. The original omni configuration remains available through
the standalone navigation launch.

The existing hardware kinematics remain Mecanum. A current `/scan`, `/odom` and
TF chain `map -> odom -> base_link -> laser` are required. The manager stops
navigation output on sensor/localization/controller timeout. Nav2 handles
obstacle avoidance while navigating; mapping controls are manual teleoperation.
Use only one command source at a time: other existing teleop/Blockly/Vibe tools
still publish directly to `/cmd_vel` and are not arbitrated by this manager.

Verify the configured 0.35 m square footprint, LiDAR mounting transform, encoder
scale and IMU/EKF alignment on the actual robot before the first navigation run.
The dashboard is fixed to the map frame once a map arrives, including the laser
mount transform. The hardware simulator uses wall time; `use_sim_time` stays
false unless a separate simulator supplies `/clock`.

### Validation

```bash
python3 -m unittest discover -s tests -v
uvx ruff@0.12.0 check .
uvx ruff@0.12.0 format --check .
```

After building the package in a **hardware-free, isolated ROS 2 Humble
environment**, `python3 tests/integration_ros.py` publishes synthetic room scans
and odometry, creates/saves/activates a map, sends and cancels a real Nav2 goal,
checks zero lateral output, and restarts the manager to verify map persistence.
It does not model wheel dynamics or replace a real-robot navigation test.
