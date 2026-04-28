# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Fast-Planner is a quadrotor motion planning system for fast flight in complex unknown environments, integrated with Hector Quadrotor + Gazebo for simulation. It implements kinodynamic A* path searching, B-spline trajectory optimization, and ESDF-based environment mapping.

**Tech Stack:** ROS Noetic (Ubuntu 20.04), C++14, CMake, Eigen3, PCL, NLopt, Armadillo, Gazebo

## Workspace Structure

This is a catkin workspace. The `src/` directory contains symlinks to the actual package directories:
- `src/fast_planner` -> `fast_planner/` (planning algorithms)
- `src/uav_simulator` -> `uav_simulator/` (simulator utilities, waypoint generator)

A separate `hector_ws/` contains the Hector Quadrotor stack (built independently).

## Build Commands

```bash
# Full build (from workspace root /home/etho/Fast-Planner)
source /opt/ros/noetic/setup.bash
catkin_make

# Build specific package
catkin_make --pkg plan_manage

# Source workspace after build
source devel/setup.bash

# Initial workspace setup (creates symlinks + initializes catkin)
./setup_workspace.sh

# Hector workspace is separate
cd hector_ws && catkin_make && source devel/setup.bash
```

No test suite or linter is configured.

## Running the Hector Simulation

Single launch file runs everything (Gazebo + Hector + Fast-Planner + RViz):
```bash
source devel/setup.bash
# Also need hector workspace overlay:
source hector_ws/devel/setup.bash
roslaunch plan_manage hector_fast_planner.launch
```

Useful launch arguments:
- `world:=dense_forest` - Select world (options: `fast_planner_obstacles`, `dense_forest`, `long_wall`, `long_wall_test`, `ramp_slope`, `wall_with_hole`, `grid_maze`, `cage_trap`, `obstacle_course`)
- `max_vel:=0.8` / `max_acc:=0.8` - Dynamic limits (conservative for Hector)
- `flight_type:=1` - 1=RViz 2D Nav Goal, 2=predefined waypoints
- `ceil_height:=2.5` - Virtual ceiling (use 4.8 for ramp_slope)
- `point0_x:=5.0 point0_y:=0.0 point0_z:=1.0` - Waypoints when flight_type=2

Use the "2D Nav Goal" tool in RViz to send goal positions.

## Architecture

### Planning Pipeline
```
Depth Image + Odometry
  -> SDFMap (raycasting + ESDF)
  -> KinodynamicAstar (6D state space: position + velocity)
  -> NonUniformBspline (fit path to B-spline)
  -> BsplineOptimizer (gradient-based: smoothness + clearance + feasibility)
  -> Trajectory execution via traj_server -> hector_cmd_bridge
```

### Key Packages

| Package | Role |
|---------|------|
| `plan_env` | SDFMap: probabilistic volumetric mapping + ESDF computation |
| `path_searching` | KinodynamicAstar, TopologyPRM, geometric Astar |
| `bspline` | NonUniformBspline: De Boor evaluation, feasibility checking |
| `bspline_opt` | BsplineOptimizer: NLopt-based multi-objective optimization |
| `plan_manage` | KinoReplanFSM, FastPlannerManager, traj_server, hector_cmd_bridge |
| `fast_planner_bridge` | Gazebo control bridge, odom-to-TF utilities |

### FSM States (KinoReplanFSM)
`INIT -> WAIT_TARGET -> GEN_NEW_TRAJ -> EXEC_TRAJ -> REPLAN_TRAJ`

Replanning triggers: collision detected on current trajectory, or new goal received.

### Hector Integration

`hector_cmd_bridge` converts Fast-Planner's `PositionCommand` to Hector's `/cmd_vel` velocity commands using a PD controller (position gain + velocity feedforward). The `fast_planner_node` starts with a 5-second delay (`launch-prefix="bash -c 'sleep 5; ...'"`) to wait for Gazebo.

SDFMap `pose_type=3` (DEPTH_ODOM_INDEP) is used for Hector to avoid timestamp sync issues between depth and odometry topics.

## Configuration

Algorithm parameters are in `fast_planner/plan_manage/launch/kino_algorithm_hector.xml`. Key tuning areas:

- **Mapping:** `sdf_map/resolution` (0.1m), `depth_filter_maxdist` (2.9m sensing range), `virtual_floor_height` (0.5m, prevents A* routing under obstacles)
- **Planner:** `manager/max_vel`, `manager/max_acc`, `manager/local_segment_length` (replanning horizon)
- **Search:** `search/horizon` (7.0m), `search/lambda_heu` (A* heuristic weight), `search/allocate_num` (node pool)
- **Optimization:** `lambda1` (smoothness), `lambda2` (distance/clearance), `dist0` (safe distance threshold)

## Development Patterns

### Adding a Planning Algorithm
1. Implement in `fast_planner/path_searching/`
2. Register in `FastPlannerManager::initPlanModules()` (`plan_manage/src/planner_manager.cpp`)
3. Add selection logic in the FSM
4. Create launch/algorithm XML in `plan_manage/launch/`

### Modifying Optimization Costs
Edit cost functions in `bspline_opt/src/bspline_optimizer.cpp`: `calcSmoothnessCost()`, `calcDistanceCost()`, `calcFeasibilityCost()`. Weights are the `lambda` params in algorithm XML.

### Custom Sensor Integration
Replace topic remappings in launch files and update camera intrinsics (`cx`, `cy`, `fx`, `fy`) and `k_depth_scaling_factor`.

## Common Issues

- **NLopt build errors:** Requires NLopt v2.7.1 installed to `/usr/local/lib`
- **Path not found:** Increase `search/horizon` or check `sdf_map/obstacles_inflation`
- **Dynamic limit violations:** Decrease `max_vel`/`max_acc` or increase `bspline/limit_ratio`
- **ESDF not updating:** Check `pose_type` matches your odometry source; verify depth topic is publishing

## ROS Topics

**Key subscriptions:** `/ground_truth/state` (odom), `/camera/depth/image_raw`, `/camera/depth/points`
**Key publications:** `/position_cmd` (trajectory commands), `/planning/bspline`, `/cmd_vel` (Hector velocity)
