# Fast-Planner (Hector Integration)

四旋翼運動規劃系統，整合 [HKUST Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) 與 Hector Quadrotor + Gazebo，可在複雜未知環境中即時自主避障飛行。

---

## Features

- Autonomous obstacle avoidance with kinodynamic A* + B-spline optimization
- ESDF-based volumetric mapping from depth camera
- Hector Quadrotor + Gazebo simulation integration
- Real-time replanning (FSM-driven)

---

## Installation

Tested on Ubuntu 20.04 / ROS Noetic. See [INSTALL_zh_TW.md](INSTALL_zh_TW.md) for full instructions.

```bash
# Build main workspace
source /opt/ros/noetic/setup.bash
catkin_make

# Build Hector workspace (separate)
cd hector_ws && catkin_make && cd ..

# Launch the integrated simulation
source devel/setup.bash
source hector_ws/devel/setup.bash
roslaunch plan_manage hector_fast_planner.launch
```

Use the **2D Nav Goal** tool in RViz to send goal positions.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Gazebo simulation                       │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐      │
│  │    world    │    │ Hector UAV  │    │   Kinect    │      │
│  └─────────────┘    └──────┬──────┘    │    depth    │      │
│                            │           └──────┬──────┘      │
└────────────────────────────┼──────────────────┼─────────────┘
                  /ground_truth/state    /camera/depth
                             │                  │
                             ▼                  ▼
                    ┌───────────────────────────────┐
                    │         Fast-Planner          │
                    │  ┌──────────┐  ┌───────────┐  │
                    │  │ SDFMap   │→ │ Kino A*   │  │
                    │  │ (ESDF)   │  │           │  │
                    │  └──────────┘  └─────┬─────┘  │
                    │                      ▼        │
                    │              ┌───────────┐    │
                    │              │ B-spline  │    │
                    │              │ Optimizer │    │
                    │              └─────┬─────┘    │
                    └────────────────────┼──────────┘
                                /position_cmd
                                         │
                                         ▼
                    ┌────────────────────────────────┐
                    │      Hector Command Bridge     │
                    │   (pos→vel + PD control)       │
                    └────────────────┬───────────────┘
                                /cmd_vel
                                         ▼
                         ┌───────────────────┐
                         │ Hector Controller │
                         └───────────────────┘
```

### Module structure

```
fast_planner/
├── plan_env/        # ESDF mapping (SDFMap)
├── path_searching/  # Kinodynamic A*, Topo PRM
├── bspline/         # B-spline representation
├── bspline_opt/     # Trajectory optimization (NLopt)
└── plan_manage/     # Planning FSM, traj_server, hector_cmd_bridge
```

---

## Customizations for Hector

Beyond the upstream Fast-Planner, this fork adds:

- **`hector_cmd_bridge`** ([plan_manage/src/hector_cmd_bridge.cpp](fast_planner/plan_manage/src/hector_cmd_bridge.cpp)) — converts Fast-Planner's `PositionCommand` to Hector's `/cmd_vel`, using a PD controller (position error + velocity feedforward) and world-to-body frame rotation.
- **`DEPTH_ODOM_INDEP` mode** ([plan_env/src/sdf_map.cpp](fast_planner/plan_env/src/sdf_map.cpp)) — `pose_type=3` lets depth image and odometry be subscribed independently, avoiding the timestamp sync issues caused by Hector's data pipeline.
- **Wider depth camera** ([hector_ws/.../kinect_camera.urdf.xacro](hector_ws/src/hector_models/hector_sensors_description/urdf/kinect_camera.urdf.xacro)) — upgraded Hector's default Kinect from 60°/3m to 90°/8m so the depth range exceeds the A* planning horizon (Fast-Planner's original assumption).
- **Hector-tuned launch & params** ([plan_manage/launch/hector_fast_planner.launch](fast_planner/plan_manage/launch/hector_fast_planner.launch), [kino_algorithm_hector.xml](fast_planner/plan_manage/launch/kino_algorithm_hector.xml)) — conservative dynamic limits (max_vel 0.8 m/s) matched to Hector's controller response.
- **Test worlds** ([plan_manage/worlds/](fast_planner/plan_manage/worlds/)) — Gazebo scenarios for progressively harder obstacle layouts (sparse, dense forest, long walls of 4/8/12/16 m, dense gaps, ramps, mazes).

---

## Launch Arguments

```bash
roslaunch plan_manage hector_fast_planner.launch \
    world:=long_wall_bypass \   # see plan_manage/worlds/*.world
    max_vel:=0.8 \
    max_acc:=0.8 \
    ceil_height:=2.5 \          # virtual ceiling, use 4.8 for ramp_slope
    flight_type:=1              # 1=2D Nav Goal, 2=predefined waypoints
```

---

## Algorithms

- **Kinodynamic A\*** — 6D state space (position + velocity) search with motion primitives, respecting quadrotor dynamics.
- **B-spline Optimization** — gradient-based smoothness + clearance + feasibility cost via NLopt.
- **ESDF Mapping** — probabilistic occupancy with Euclidean signed distance field for collision queries.
- **Topological Path Searching** — multiple distinct paths in 3D environments (available, off by default in Hector launch).

---

## Papers

- *Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight*, IEEE RA-L 2019 — [link](https://ieeexplore.ieee.org/document/8758904)
- *Robust Real-time UAV Replanning Using Guided Gradient-based Optimization and Topological Paths*, IEEE ICRA 2020 — [arXiv](https://arxiv.org/abs/1912.12644)
- *RAPTOR: Robust and Perception-aware Trajectory Replanning for Quadrotor Fast Flight*, IEEE T-RO — [arXiv](https://arxiv.org/abs/2007.03465)

---

## Related Projects

- [ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner) — multi-agent extension
- [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL) — autonomous exploration
- [RACER](https://github.com/SYSU-STAR/RACER) — autonomous racing

---

## Credits

Upstream: [Boyu Zhou](http://sysu-star.com), [Fei Gao](http://zju-fast.com/fei-gao/), [Shaojie Shen](http://uav.ust.hk/group/) — [HKUST Aerial Robotics Group](http://uav.ust.hk/).

Hector Quadrotor: TU Darmstadt — [tu-darmstadt-ros-pkg](https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor).

## License

GPLv3 — see [LICENSE](LICENSE).
