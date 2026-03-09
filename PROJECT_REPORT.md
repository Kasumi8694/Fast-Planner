# Fast-Planner 專案報告

## 一、專案概述

本專案基於 HKUST Aerial Robotics Group 開發的 Fast-Planner 四旋翼運動規劃系統，整合 Hector Quadrotor 與 Gazebo 模擬環境，實現無人機在複雜環境中的自主避障飛行。

---

## 二、使用的外部專案與函式庫

### 2.1 核心演算法來源

| 專案 | 來源 | 授權 |
|------|------|------|
| **Fast-Planner** | [HKUST-Aerial-Robotics/Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) | GPLv3 |

**原始作者：** Boyu Zhou, Fei Gao, Shaojie Shen (香港科技大學)

**相關論文：**
- "Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight" - IEEE RA-L 2019
- "Robust Real-time UAV Replanning Using Guided Gradient-based Optimization and Topological Paths" - IEEE ICRA 2020

### 2.2 無人機模擬平台

| 專案 | 來源 | 版本 | 用途 |
|------|------|------|------|
| **Hector Quadrotor** | [tu-darmstadt-ros-pkg/hector_quadrotor](https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor) | v0.3.5 | 四旋翼 URDF 模型與控制器 |
| **Hector Gazebo** | [tu-darmstadt-ros-pkg/hector_gazebo](https://github.com/tu-darmstadt-ros-pkg/hector_gazebo) | v0.3.5+ | Gazebo 物理模擬插件 |
| **Hector Localization** | [tu-darmstadt-ros-pkg/hector_localization](https://github.com/tu-darmstadt-ros-pkg/hector_localization) | v0.3.5+ | 位姿估計 |
| **Hector Models** | [tu-darmstadt-ros-pkg/hector_models](https://github.com/tu-darmstadt-ros-pkg/hector_models) | v0.5.2+ | 感測器 URDF 描述 |

**維護者：** Johannes Meyer 等人，達姆施塔特工業大學 (TU Darmstadt)
**授權：** BSD 3-Clause

### 2.3 第三方函式庫

| 函式庫 | 版本 | 用途 |
|--------|------|------|
| **NLopt** | v2.7.1 | 非線性軌跡優化 |
| **Armadillo** | 9.0+ | C++ 線性代數運算 |
| **Eigen3** | 3.3+ | 矩陣運算 |
| **PCL** | 1.10+ | 點雲處理 |
| **OpenCV** | 4.x | 深度圖像處理 |
| **Boost (odeint)** | - | 常微分方程求解 |

### 2.4 ROS 生態系統

```
核心套件: roscpp, rospy, std_msgs, geometry_msgs, nav_msgs,
         sensor_msgs, visualization_msgs, tf2_ros, cv_bridge
```

---

## 三、自行開發的內容

### 3.1 新增節點：hector_cmd_bridge（核心橋接）

**路徑：** `fast_planner/plan_manage/src/hector_cmd_bridge.cpp`

**功能：** 橋接 Fast-Planner 與 Hector Quadrotor 控制系統，將 Fast-Planner 的 `PositionCommand` 轉換為 Hector 的 `/cmd_vel` 速度控制。

**技術特點：**
- 位置誤差 + 速度前饋的混合控制策略
- 世界座標系到機體座標系的速度轉換
- Yaw 角度歸一化與 P 控制
- 馬達自動啟用與超時懸停保護

```cpp
// 控制律核心：期望速度 = Kp * 位置誤差 + Kv * 前饋速度
double vx_world = kp_ * ex + kv_ * cmd->velocity.x;
double vy_world = kp_ * ey + kv_ * cmd->velocity.y;
double vz_world = kp_ * ez + kv_ * cmd->velocity.z;
```

### 3.2 修改 Fast-Planner 核心：sdf_map.cpp

**路徑：** `fast_planner/plan_env/src/sdf_map.cpp` / `sdf_map.h`

原始 Fast-Planner 不支援 Hector Quadrotor 的資料格式，對核心環境感知模組進行了以下修改：

| 修改項目 | 說明 |
|----------|------|
| DEPTH_ODOM_INDEP 模式（pose_type=3） | 新增深度圖與里程計獨立訂閱模式，解決 Hector 時間戳不同步問題 |
| 相機光學座標系轉換 | 加入 Kinect 光學座標系到無人機機體座標系的旋轉矩陣轉換 |
| 地圖邊界檢查 | 在 `depthIndepCallback` 中加入邊界保護，防止無人機飛出地圖時崩潰 |

### 3.3 修改 Fast-Planner 核心：traj_server.cpp

**路徑：** `fast_planner/plan_manage/src/traj_server.cpp`

調整軌跡伺服器以配合 Hector 控制介面，輸出適合橋接節點使用的命令格式。

### 3.4 配置檔案與啟動檔

| 檔案 | 說明 |
|------|------|
| `hector_fast_planner.launch` | Hector + Gazebo 整合啟動檔 |
| `kino_algorithm_hector.xml` | Hector 專用規劃參數配置 |
| `fast_planner_obstacles.world` | Gazebo 障礙物世界檔 |

### 3.5 技術文件

| 文件 | 語言 | 內容 |
|------|------|------|
| `PROJECT_STRUCTURE.md` | 英文 | 專案架構說明 |
| `PROJECT_STRUCTURE_zh_TW.md` | 繁體中文 | 專案架構說明 |
| `INSTALL_zh_TW.md` | 繁體中文 | 完整安裝指南 |
| `CLAUDE.md` | 英文 | AI 輔助開發指南 |

### 3.6 程式碼統計

| 類型 | 內容 | 估計行數 |
|------|------|----------|
| 新增 C++ | `hector_cmd_bridge.cpp` | ~190 行 |
| 修改 C++ | `sdf_map.cpp` / `.h` | ~80 行 |
| 修改 C++ | `traj_server.cpp` | ~50 行 |
| Launch/XML 配置 | 啟動檔與參數 | ~200 行 |
| 技術文件 | Markdown 文件 | ~900 行 |
| **總計** | | **~1420 行** |

---

## 四、遇到的困難與解決方式

### 4.1 時間戳同步問題

**困難描述：**
Hector Quadrotor 的深度圖像與里程計資料時間戳不同步，導致 Fast-Planner 原本使用的 `message_filters` 同步機制無法正常工作，ESDF 地圖無法正確建立。

**解決方式：**
修改 `pose_type` 參數為 `3`（DEPTH_ODOM_INDEP 模式），讓深度圖和里程計使用獨立的訂閱者，避免時間戳同步問題。

```xml
<!-- kino_algorithm_hector.xml -->
<param name="sdf_map/pose_type" value="3"/>
```

### 4.2 座標系轉換問題

**困難描述：**
Kinect 相機使用光學座標系（Z 前、X 右、Y 下），而 Fast-Planner 使用 NED/ENU 座標系（X 前、Y 左、Z 上），直接使用會導致地圖方向錯誤。

**解決方式：**
在 `hector_cmd_bridge` 中實作世界座標系到機體座標系的速度轉換：

```cpp
void worldToBody(double vx_world, double vy_world, double yaw,
                 double& vx_body, double& vy_body) {
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);
    vx_body =  cos_yaw * vx_world + sin_yaw * vy_world;
    vy_body = -sin_yaw * vx_world + cos_yaw * vy_world;
}
```

### 4.3 控制介面不相容

**困難描述：**
Fast-Planner 輸出 `quadrotor_msgs::PositionCommand`（位置 + 速度 + 加速度命令），但 Hector Quadrotor 只接受 `geometry_msgs::Twist`（速度命令），兩者控制介面完全不同。

**解決方式：**
開發 `hector_cmd_bridge` 節點，實作 PD 控制器將位置命令轉換為速度命令：

```
期望速度 = Kp × 位置誤差 + Kv × 前饋速度
```

並加入速度限幅、超時保護等安全機制。

### 4.4 地圖邊界崩潰問題

**困難描述：**
當無人機飛出預設地圖邊界時，ESDF 查詢會返回無效值，導致規劃器崩潰。

**解決方式：**
在 SDFMap 中加入邊界檢查，當查詢點超出地圖範圍時返回安全的預設值而非崩潰。

### 4.5 避障參數調整

**困難描述：**
原始 Fast-Planner 參數是針對高速飛行優化的，在 Gazebo 模擬中控制響應較慢，導致無人機來不及避開障礙物。

**解決方式：**
調整優化參數，增加安全距離與距離代價權重：

```xml
<param name="sdf_map/obstacles_inflation" value="0.2"/>  <!-- 原 0.099 -->
<param name="optimization/dist0" value="0.5"/>           <!-- 原 0.4 -->
<param name="optimization/lambda2" value="30.0"/>        <!-- 原 5.0 -->
<param name="manager/max_vel" value="1.0"/>              <!-- 原 3.0 -->
```


---

## 五、系統架構圖

```
┌─────────────────────────────────────────────────────────────┐
│                     Gazebo simulation                       │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐      │
│  │    world    │    │ Hector UAV  │    │   Kinect    │      │
│  └─────────────┘    └──────┬──────┘    │    depth    │      │
│                            │           └──────┬──────┘      │
└────────────────────────────┼──────────────────┼─────────────┘
                             │                  │
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
                                         │
                              /position_cmd (軌跡命令)
                                         │
                                         ▼
                    ┌────────────────────────────────┐
                    │      Hector Command Bridge     │
                    │   (pos→速度命令轉換 + PD控制)     │
                    └────────────────┬───────────────┘
                                     │
                                /cmd_vel (速度命令)
                                     │
                                     ▼
                         ┌───────────────────┐
                         │ Hector Controller │
                         └───────────────────┘
```

---

## 六、結論

本專案成功將學術研究等級的 Fast-Planner 運動規劃系統，與工業級的 Hector Quadrotor + Gazebo 模擬環境整合。透過開發自訂的橋接模組，解決了控制介面不相容、座標系統轉換、時間戳同步等技術挑戰，最終實現了無人機在模擬環境中的即時避障飛行功能。

**專案貢獻：**
1. 完整的 Hector Quadrotor 整合方案
2. 可重用的控制橋接模組
3. 詳細的中英文技術文件
4. 針對 Gazebo 模擬優化的參數配置

---

## 七、參考資料

1. [Fast-Planner GitHub](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)
2. [Hector Quadrotor ROS Wiki](http://wiki.ros.org/hector_quadrotor)
3. [NLopt Documentation](https://nlopt.readthedocs.io/)
4. Zhou, B., Gao, F., et al. "Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight" IEEE RA-L 2019
