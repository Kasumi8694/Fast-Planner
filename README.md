# Fast-Planner

**Fast-Planner** is developed aiming to enable quadrotor fast flight in complex unknown environments. It contains a rich set of carefully designed planning algorithms.

---

## Quick Start

```bash
# Setup environment
source /opt/ros/noetic/setup.bash
source ~/Fast-Planner/hector_ws/devel/setup.bash --extend
source ~/Fast-Planner/devel/setup.bash --extend

# Launch
roslaunch plan_manage hector_fast_planner.launch
```

**After launching:**
- Use "2D Nav Goal" in RViz to set target positions
- Watch autonomous obstacle avoidance in action!

---

## Features

- Autonomous obstacle avoidance
- Dynamic replanning
- Kinodynamic path searching
- B-spline trajectory optimization
- ESDF-based environment mapping
- Hector Quadrotor + Gazebo integration

---

## Documentation

| Document | Description |
|----------|-------------|
| [PROJECT_STRUCTURE.md](PROJECT_STRUCTURE.md) | Code organization |
| [PROJECT_STRUCTURE_zh_TW.md](PROJECT_STRUCTURE_zh_TW.md) | 專案結構（繁體中文） |
| [INSTALL_zh_TW.md](INSTALL_zh_TW.md) | 安裝指南（繁體中文） |
| [PROJECT_REPORT.md](PROJECT_REPORT.md) | 專案報告（繁體中文） |
| [CLAUDE.md](CLAUDE.md) | Development guide |

---

## Installation

Tested on Ubuntu 18.04 (ROS Melodic) and 20.04 (ROS Noetic).

### 1. Install Dependencies

```bash
# NLopt v2.7.1
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt && mkdir build && cd build
cmake .. && make && sudo make install

# Armadillo
sudo apt-get install libarmadillo-dev
```

### 2. Build Fast-Planner

```bash
cd ~/Fast-Planner
catkin_make
```

### 3. Build Hector Workspace (for Gazebo simulation)

```bash
cd ~/Fast-Planner/hector_ws
catkin_make
```

---

## Algorithms

The project contains these planning algorithms:

- **Kinodynamic A\***: 6D state space search respecting quadrotor dynamics
- **B-spline Optimization**: Gradient-based trajectory smoothing with obstacle avoidance
- **Topological Path Searching**: Multiple distinct paths in 3D environments
- **ESDF Mapping**: Euclidean signed distance field for collision checking

### Module Structure

```
fast_planner/
├── plan_env/        # ESDF mapping
├── path_searching/  # Kinodynamic A*, Topo PRM
├── bspline/         # B-spline representation
├── bspline_opt/     # Trajectory optimization
└── plan_manage/     # Planning FSM, launch files
```

---

## Papers

Please cite our papers if you use this project:

- [Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight](https://ieeexplore.ieee.org/document/8758904), IEEE RA-L 2019
- [Robust Real-time UAV Replanning Using Guided Gradient-based Optimization and Topological Paths](https://arxiv.org/abs/1912.12644), IEEE ICRA 2020
- [RAPTOR: Robust and Perception-aware Trajectory Replanning for Quadrotor Fast Flight](https://arxiv.org/abs/2007.03465), IEEE T-RO

---

## Related Projects

- [ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner) - Extended for multi-agent
- [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL) - Fast autonomous exploration
- [RACER](https://github.com/SYSU-STAR/RACER) - Autonomous racing

---

## Authors

[Boyu Zhou](http://sysu-star.com), [Fei Gao](http://zju-fast.com/fei-gao/), and [Shaojie Shen](http://uav.ust.hk/group/) from [HKUST Aerial Robotics Group](http://uav.ust.hk/).

---

## License

GPLv3 - See [LICENSE](http://www.gnu.org/licenses/) for details.

## Disclaimer

Research code provided without warranty.
