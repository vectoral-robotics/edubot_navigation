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
