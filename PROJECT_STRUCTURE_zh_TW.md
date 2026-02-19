# Fast-Planner 專案結構

本文件說明 Fast-Planner 程式碼庫與 Hector Quadrotor + Gazebo 整合後的組織架構。

## 根目錄

```
Fast-Planner/
├── README.md                    # 主要專案文件
├── CLAUDE.md                    # AI 助手開發指南
├── PROJECT_STRUCTURE.md         # 專案結構（英文）
├── PROJECT_STRUCTURE_zh_TW.md   # 專案結構（繁體中文）
├── PROJECT_REPORT.md            # 專案報告
├── INSTALL_zh_TW.md             # 安裝指南（繁體中文）
├── .gitignore                   # Git 忽略規則
├── fast_planner/                # Fast-Planner 核心模組
├── uav_simulator/Utils/         # 訊息類型與工具套件
├── hector_ws/                   # Hector Quadrotor 工作空間
└── src/                         # ROS 工作空間來源連結
```

## 核心模組

### fast_planner/
Fast-Planner 主要演算法與規劃系統。

```
fast_planner/
├── plan_env/              # 環境感知（ESDF、佔據地圖）
├── path_searching/        # 前端路徑規劃（Kinodynamic A*、Topo PRM）
├── bspline/              # B-spline 軌跡表示
├── bspline_opt/          # 後端軌跡優化
├── plan_manage/          # 高階規劃狀態機與協調
│   ├── launch/           # 啟動檔案
│   │   ├── hector_fast_planner.launch    # Hector+Gazebo 主啟動檔
│   │   ├── kino_algorithm_hector.xml     # Hector 規劃參數
│   │   └── rviz.launch                   # 視覺化
│   ├── src/              # 原始碼
│   │   ├── hector_cmd_bridge.cpp         # Fast-Planner → Hector 速度橋接
│   │   └── traj_server.cpp               # 軌跡執行伺服器
│   ├── scripts/          # 輔助腳本
│   │   └── odom_to_tf.py                 # RViz 用 TF 廣播器
│   └── worlds/           # Gazebo 世界檔案
│       └── fast_planner_obstacles.world  # 障礙物環境
├── traj_utils/           # 軌跡工具
└── poly_traj/            # 多項式軌跡表示
```

### hector_ws/
Hector Quadrotor 模擬工作空間，用於 Gazebo 整合。

```
hector_ws/
└── src/
    ├── hector_quadrotor/          # 四旋翼模擬與控制
    │   ├── hector_quadrotor_gazebo/       # Gazebo 生成啟動檔
    │   ├── hector_quadrotor_description/  # URDF 模型
    │   └── hector_quadrotor_controllers/  # 飛行控制器
    └── hector_models/
        └── hector_sensors_description/    # Kinect 相機 URDF
```

### uav_simulator/Utils/
訊息類型定義與工具套件。

```
uav_simulator/Utils/
├── quadrotor_msgs/        # 四旋翼訊息類型（PositionCommand 等）
├── waypoint_generator/    # 將 RViz 目標轉換為航點
├── odom_visualization/    # 里程計視覺化
└── uav_utils/             # 通用工具函式
```

## 啟動系統

**啟動指令：**
```bash
source /opt/ros/noetic/setup.bash
source ~/Fast-Planner/hector_ws/devel/setup.bash --extend
source ~/Fast-Planner/devel/setup.bash --extend
roslaunch plan_manage hector_fast_planner.launch
```

**特色：**
- 完整 Gazebo 3D 視覺化
- 真實 Kinect 深度相機
- Hector Quadrotor 物理模擬
- 障礙物世界環境

## 設定檔案

| 檔案 | 用途 |
|------|------|
| `kino_algorithm_hector.xml` | Hector 規劃參數（ESDF、優化） |
| `hector_fast_planner.launch` | Hector 系統主啟動檔 |
| `kino.rviz` | RViz 視覺化設定 |
| `fast_planner_obstacles.world` | Gazebo 障礙物世界 |

### 關鍵參數（kino_algorithm_hector.xml）

| 參數 | 值 | 說明 |
|------|-----|------|
| `sdf_map/pose_type` | 3 | DEPTH_ODOM_INDEP 模式（無需時間戳同步） |
| `sdf_map/obstacles_inflation` | 0.3 | 障礙物安全邊距 |
| `optimization/dist0` | 0.8 | 安全距離閾值 |
| `optimization/lambda2` | 30.0 | 距離代價權重 |
| `manager/max_vel` | 1.0 | 最大速度（m/s） |

## 資料流程（Hector 系統）

```
Gazebo 世界（障礙物）
  → Kinect 相機（深度影像）
  → SDFMap（ESDF 建圖，pose_type=3）
  → Fast-Planner（Kinodynamic A* + B-spline）
  → 軌跡伺服器
  → Hector 指令橋接（/cmd_vel）
  → Hector Quadrotor 控制器
  → Gazebo 物理模擬
  → 里程計回饋 → （循環）
```

### ROS 話題

**輸入：**
- `/move_base_simple/goal` - RViz 2D 導航目標
- `/ground_truth/state` - Hector 里程計
- `/camera/depth/image_raw` - Kinect 深度影像
- `/camera/depth/points` - Kinect 點雲

**輸出：**
- `/cmd_vel` - Hector 速度指令
- `/planning/bspline` - 規劃軌跡
- `/sdf_map/occupancy` - ESDF 佔據地圖

## 編譯系統

```bash
# 先編譯 Hector 工作空間
cd ~/Fast-Planner/hector_ws
catkin_make

# 編譯 Fast-Planner
cd ~/Fast-Planner
catkin_make

# 編譯特定套件
catkin_make --pkg plan_manage
```

## Hector 整合關鍵元件

1. **`hector_cmd_bridge.cpp`** - 將 Fast-Planner 位置指令轉換為 Hector 速度指令
2. **`pose_type=3`（DEPTH_ODOM_INDEP）** - 深度/里程計獨立訂閱，避免時間戳同步問題
3. **相機座標系轉換** - 將 Kinect 光學座標系轉換為世界座標系
4. **地圖邊界檢查** - 防止無人機飛出地圖邊界時崩潰

## 使用流程

1. 啟動系統：`roslaunch plan_manage hector_fast_planner.launch`
2. 等待 Gazebo 和 RViz 初始化
3. 在 RViz 中選擇「2D Nav Goal」工具
4. 點擊目標位置（保持 z ≈ 1.0m）
5. 觀看自主避障飛行！

## 開發筆記

- **規劃演算法**：Kinodynamic A*（前端）+ B-spline 優化（後端）
- **控制器**：透過 `/cmd_vel` 的 Hector 速度控制
- **感知**：基於 Kinect 深度的 ESDF 障礙物表示
- **模擬**：Gazebo + Hector Quadrotor 物理模擬
- **座標系 ID**：`world`（全域）、`base_link`（四旋翼）

## 外部相依套件

- ROS Noetic
- Gazebo 11
- Eigen3
- PCL（點雲庫）
- NLopt（v2.7.1）
- Armadillo
- Hector Quadrotor 套件

安裝說明請參閱 [INSTALL_zh_TW.md](INSTALL_zh_TW.md)。
