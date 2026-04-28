# Fast-Planner Debug Reference Document

本文件是結構化的除錯參考文件，供 Claude Code 在 context 超出限制後快速回復上下文使用。

---

## 1. 專案架構摘要

### 核心 Pipeline
```
Depth Image + Odometry
  → SDFMap (raycasting + occupancy → ESDF)
  → KinodynamicAstar (6D state space: pos + vel)
  → NonUniformBspline (fit B-spline to path)
  → BsplineOptimizer (gradient-based refinement)
  → traj_server → hector_cmd_bridge → /cmd_vel
```

### 關鍵源碼位置
| 功能 | 檔案 | 重要行號 |
|------|------|----------|
| SDF Map 初始化 & 參數 | `plan_env/src/sdf_map.cpp` | L31-195 |
| 深度投影 | `plan_env/src/sdf_map.cpp` | L440-490 (projectDepthImage) |
| Raycasting | `plan_env/src/sdf_map.cpp` | L499-604 (raycastProcess) |
| 膨脹 + 虛擬天花板/地板 | `plan_env/src/sdf_map.cpp` | L672-801 (clearAndInflateLocalMap) |
| ESDF 計算 | `plan_env/src/sdf_map.cpp` | L260-357 (updateESDF3d) |
| A* 搜索主循環 | `path_searching/src/kinodynamic_astar.cpp` | L41-300 |
| Shot trajectory 驗證 | `path_searching/src/kinodynamic_astar.cpp` | L407-480 |
| FSM 狀態機 | `plan_manage/src/kino_replan_fsm.cpp` | L125-247 |
| Waypoint 回調 (z=1.0 硬編碼) | `plan_manage/src/kino_replan_fsm.cpp` | L68-93 |
| 碰撞檢測 | `plan_manage/src/planner_manager.cpp` | L95-135 |
| kinodynamicReplan | `plan_manage/src/planner_manager.cpp` | L141-248 |
| Hector 命令橋接 | `plan_manage/src/hector_cmd_bridge.cpp` | L1-50 |
| Launch 參數 | `plan_manage/launch/kino_algorithm_hector.xml` | 全部 |
| Launch 入口 | `plan_manage/launch/hector_fast_planner.launch` | 全部 |

### 重要參數值 (kino_algorithm_hector.xml)
| 參數 | 值 | 用途 |
|------|-----|------|
| resolution | 0.1m | 體素大小 |
| obstacles_inflation | 0.2m | 膨脹半徑 → inf_step = ceil(0.2/0.1) = 2 voxels |
| depth_filter_maxdist | 2.9m | 最大感測距離 |
| max_ray_length | 2.9m | 最大 raycasting 長度 |
| virtual_ceil_height | 2.5m (default) | 虛擬天花板 |
| virtual_floor_height | 0.5m | 虛擬地板 |
| ground_height | -0.1m | 地圖 z 原點 |
| max_vel | 0.8 m/s | 最大速度 |
| max_acc | 0.8 m/s² | 最大加速度 |
| search/horizon | 7.0m | A* 搜索地平線 |
| search/allocate_num | 100000 | A* 節點池大小 |
| clearance_threshold | 0.2m | 最小障礙距離 |
| collision check threshold | 0.18m | planner_manager.cpp:117 |
| dist0 (optimization) | 0.4m | 優化安全距離閾值 |
| map_size | 80×40×5 m | 地圖範圍 |

---

## 2. 測試世界分析 & 作弊向量

### 2.1 cage_trap.world — 方形牢籠
- **佈局**: 6m×6m 方形牆 (4.5m 高, 0.3m 厚), 無天花板, drone spawn (0,0,1.0)
- **目的**: 測試 planner 在無路可逃時是否 graceful fail
- **作弊風險**: **低** — 牆 4.5m + virtual_ceil 2.5m → 無法飛越。四面封閉。
- **驗證重點**: FSM 是否正確在 MAX_REPLAN_ATTEMPTS(30) 次失敗後放棄

### 2.2 dense_forest.world — 密集障礙物
- **佈局**: 3 區段 (稀疏→密集→中等), 目標 (16, 0, 1.5)
- **Section B 間距分析**:
  - 中央 (b2-b6): 間距 1.5m - 2×0.6 = 0.3m → **完全封閉**
  - 外側 (b1-b2, b6-b7): 間距 2.0m - 0.5 - 0.6 = 0.9m → 膨脹後 0.5m → **borderline passable**
- **作弊風險**: **中** — drone 可在 |y| > 5.5 完全繞過 Section B, 不需穿越任何縫隙。Comment 說 "only outer corridors at |y|>5" 但實際上 |y|>5.5 完全空曠。柱子高 4.8m > virtual_ceil 2.5m, 無法飛越。
- **建議**: Section B 的最外側柱子 (b1, b7) 應該往外移或加寬, 封住 |y|>5.5 的繞路

### 2.3 long_wall.world — 超長牆壁 (飛越測試)
- **佈局**: 牆 at x=8, 36m 寬 (y=-18 to y=18), **實際高度只有 2.0m** (box size 0.3×36×2.0, center z=1.0 → z: 0~2.0)
- **作弊風險**: **高 — 存在明顯問題**
  1. **Comment 與實際不符**: 註解說 "height 2.5m" 但 box size 是 2.0m
  2. **飛越可行性計算**: 牆頂 z=2.0 + inflation(2 voxels=0.2m) ≈ z=2.2, virtual_ceil=2.5. 間隙 0.3m (3 voxels). A* 的 getInflateOccupancy 檢查會看到這些 voxel 是 free (0), 所以 **A* 可能找到穿越路徑**。但 ESDF 在此間隙的值 ≈ 0.15m < dist0(0.4m), B-spline 優化可能將軌跡推出此間隙導致失敗。
  3. **繞路**: 牆端在 y=±18, map 到 y=±20, 有 2m 間隙。但繞路需 36m+, 超出 search/horizon(7m)。
- **建議**: 將牆高度改為 ≥ 2.5m 或改 box size 為 `0.3 36 2.5` (center z=1.25)

### 2.4 long_wall_test.world — 不可飛越的高牆 (繞路測試)
- **佈局**: 牆 at x=6, 8m 長 (y=-4 to y=4), 5.0m 高 (z=0~5.0)
- **作弊風險**: **低** — 牆高 5.0m 填滿地圖 z 軸, 無法飛越。兩端 (|y|>4) 有明確通道。
- **潛在問題**: 因為 depth_filter_maxdist=2.9m, drone 在 x=3 才能看到 x=6 的牆。search/horizon=7.0m 但需要橫向繞 4m+, 可能需要多次 replan。

### 2.5 grid_maze.world — 網格形障礙物
- **佈局**: 5×5 格柵, 0.3m×0.3m 柱, 1.8m 間距, **高 2.5m** (box size 0.3×0.3×2.5, center z=0)
- **作弊風險**: **嚴重 — 柱子可被飛越!**
  - 柱子 center z=0, size 2.5 → 跨度 z=-1.25 到 z=1.25
  - drone 在 z=1.0 時在柱子範圍內
  - **但 z=1.5 已完全在柱子上方!** virtual_ceil=2.5, 所以 z ∈ [1.25+inflation, 2.5] 完全暢通
  - A* 可以直接規劃 z > 1.5 的路徑飛越所有柱子, 完全無視 grid maze 設計
- **建議**: 將柱子高度改為 ≥ 5.0m (填滿 z 軸), 或 center z=2.5, size 5.0

### 2.6 ramp_slope.world — 大斜坡
- **佈局**: 30° 斜坡 center (8,0,1.6), 10×14×0.5m, 側牆 at y=±8 (4.5m 高)
- **作弊風險**: **中 — 斜坡側邊有縫隙**
  - 斜坡 y 跨度: -7 to +7 (14m wide)
  - 側牆 at y=±8 (0.3m thick)
  - 斜坡邊緣到側牆間距: 8 - 7 = 1.0m, 扣除半牆厚 0.15m → 0.85m
  - 膨脹後: 0.85 - 2×0.2 = 0.45m → **可以擠過去!**
  - drone 可能不爬坡, 而是從斜坡與側牆之間的縫隙繞過
- **建議**: 將斜坡寬度增加到 ≥ 15.3m (填滿側牆內部), 或將側牆往內移

### 2.7 wall_with_hole.world — 有洞的牆
- **佈局**: 牆 at x=8, 16m 寬, 3.5m 高, 中間有 2m×2m 洞 (y:-1~1, z:0.6~2.6)
- **有效洞口**: 寬 2.0-2×0.2=1.6m, 高 2.0-2×0.2=1.6m → **可通過**
- **作弊風險**: **低**
  - 飛越: 牆高 3.5m > virtual_ceil 2.5m → 不可能
  - 繞路: 牆端 y=±8, 需 16m+ 繞路, 不如穿洞
  - 洞口設計合理, 中心 z=1.6 接近 drone 飛行高度 z=1.0~1.5
- **測試良好**, 但需確認 ESDF 正確計算洞口區域的距離場

### 2.8 obstacle_course.world — 綜合障礙賽
- **佈局**: 5 個區域, 目標 (35, 0, 1.5) (但 marker_goal 在 x=39)
- **Zone 1 問題**: Comment 說 "2m 窄縫" 但實際間隙 = 4m (y=-2 to y=2), **不算窄**
- **Zone 2**: 8 根圓柱, 間距 1.5-2.0m, 合理
- **Zone 3 (Low Bridge)**: 橋底 z=1.8, 上方封死到 z=5.0, 側面封死 → **必須降高穿越**. 設計良好.
  - 可通行高度: z=0.5(virtual_floor) 到 z=1.8(橋底) - inflation ≈ z=0.7~1.6 = 0.9m gap
- **Zone 4**: 兩面錯位牆, 高 2.5m, 通道在 y<-1 和 y<1. 合理.
- **作弊風險**: **Zone 1 中等** (間隙太寬不算測試), 其他低

### 2.9 dense_limit.world — 漸進密度極限測試 (NEW)
- **佈局**: 走廊 (側牆 y=±4.15, x=[2,20], 高 6m) 含 3 個漸進密度區段
  - Phase A (x=4,6): gap=1.2m, 預期輕鬆通過
  - Phase B (x=10,12): gap=1.0m, 預期可通過
  - Phase C (x=16,18): gap=0.8m, **理論極限邊界** (inflated gap=0.4m, ESDF center=0.4m=dist0)
- **起點/終點**: (0,0,1.0) → (22,0,1.5)
- **圓柱**: R=0.3m, 高 6m (z=[-3,3]), 交錯排列迫使 S 型飛行
- **作弊風險**: **低** — 側牆 + virtual_ceil 封閉上方和左右, 邊緣柱子封住與牆的縫隙
- **用途**: 確定 planner 能通過的最小間距閾值
  - Phase A+B 通過, Phase C 失敗 → 極限介於 0.8-1.0m
  - Phase C 通過 → 極限 ≤ 0.8m, 可設計更嚴苛測試
  - Phase A 失敗 → 參數分析錯誤, 需調查

### 2.10 long_wall_bypass.world — 超視距長牆繞行 (NEW)
- **佈局**: 單面牆 at x=8, 長 16m (y=[-8,8]), 高 5.0m, 厚 0.3m
- **起點/終點**: (0,0,1.0) → (16,0,1.5)
- **作弊風險**: **低** — 牆高 5m >> virtual_ceil 2.5m, 無法飛越
- **設計核心**: 從偵測點 (5,0) 到牆端 (8,±8) = 8.5m > A* horizon 7.0m
  - 首次規劃時 A* **看不到牆端** → 必須 incremental replan
  - 預期 2-3 次 replan 才能找到牆端繞過
- **用途**: 測試 FSM 在 A* horizon 不足時的 incremental exploration 能力
  - 如果成功: 證明 horizon-bounded A* + replan 可處理大型障礙
  - 如果失敗 (卡在原地反覆 replan): horizon 太小或 heuristic 引導不足

---

## 3. 程式碼層面關鍵問題

### 3.1 目標高度硬編碼為 z=1.0
- **位置**: `kino_replan_fsm.cpp:75`
- **代碼**: `end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;`
- **影響**: 使用 2D Nav Goal 時, 無論 RViz 設定什麼高度, 目標 z 永遠是 1.0
- **後果**: long_wall (飛越), ramp_slope (爬坡), obstacle_course Zone 3 (低橋) 等需要目標在不同高度的測試, planner 永遠以 z=1.0 為終點。planner 必須靠 kinodynamic search 自行發現需要改變高度, 但因為 end_pt z=1.0, A* 的 heuristic 會傾向保持低高度。
- **嚴重度**: 中 — 不完全阻止飛越 (A* 可以在 horizon 內爬升), 但增加了失敗機率

### 3.2 未知空間視為障礙的保守策略
- **位置**: `kinodynamic_astar.cpp:231-232` (search), `kinodynamic_astar.cpp:471` (shot traj)
- **代碼**:
  ```cpp
  if (pos(2) > start_pt_(2) + 0.3 && edt_environment_->sdf_map_->isUnknown(pos))
  ```
- **影響**: drone 起飛高度 +0.3m 以上的未知空間被視為障礙
- **後果**: 對於需要爬升的場景 (long_wall, ramp_slope), 在牆頂/坡頂尚未被觀察到之前, A* 無法規劃穿越路徑。drone 必須先接近到感測距離 (2.9m) 內, 觀察到上方空間是 free, 才能規劃上升路徑。這是設計上的保守選擇, 但可能導致反覆 replan 和接近障礙物的危險行為。

### 3.3 Shot Trajectory 動態約束檢查被禁用
- **位置**: `kinodynamic_astar.cpp:450-454`
- **代碼**: velocity/acceleration check 被 comment out (`// return false;`)
- **影響**: shot trajectory (A* 到目標的直線連接) 可能超出 max_vel/max_acc 限制
- **後果**: 最終軌跡可能包含動態不可行段, 導致 Hector 追蹤失敗

### 3.4 碰撞檢測閾值不一致
- `obstacles_inflation`: 0.2m (sdf_map)
- `clearance_threshold`: 0.2m (planner_manager config)
- `collision check`: 0.18m (planner_manager.cpp:117 的 `dist < 0.18`)
- `dist0`: 0.4m (optimizer 安全距離)
- **問題**: 0.18 < 0.2, 意味著碰撞檢測比膨脹半徑更寬鬆, 理論上不應觸發

### 3.5 getInflateOccupancy 對地圖外返回 -1
- **位置**: `sdf_map.h:436-443`
- A* 和 shot traj 使用 `!= 0` 來判斷障礙: 地圖外 (-1) 也被視為障礙 ✓ 正確
- 但 `getDistance()` 使用 `boundIndex()` 會 clamp 到邊界, 可能返回邊界 voxel 的值而非安全值

### 3.6 ESDF 更新只在 local_bound 內
- `updateESDF3d()` 只更新 `local_bound_min_` 到 `local_bound_max_` 範圍
- 邊界由 `raycastProcess()` 根據感測範圍設定
- **後果**: drone 遠離某區域後, 該區域的 ESDF 不再更新, 使用的是過時的距離值

---

## 4. FSM 行為

### 狀態轉換
```
INIT → (有 odom + trigger) → WAIT_TARGET
WAIT_TARGET → (有 target) → GEN_NEW_TRAJ
GEN_NEW_TRAJ → (成功) → EXEC_TRAJ
             → (失敗 < 30 次) → GEN_NEW_TRAJ (retry)
             → (失敗 ≥ 30 次) → WAIT_TARGET (放棄)
EXEC_TRAJ → (到達終點) → WAIT_TARGET
          → (需重規劃) → REPLAN_TRAJ
          → (碰撞偵測) → REPLAN_TRAJ
REPLAN_TRAJ → (成功) → EXEC_TRAJ
            → (失敗 < 30 次) → GEN_NEW_TRAJ
            → (失敗 ≥ 30 次) → WAIT_TARGET (放棄)
```

### Replan 觸發條件
1. 當前位置距 trajectory start > replan_thresh (1.5m)
2. 當前位置距目標 > no_replan_thresh (2.0m)
3. safety_timer 檢測到碰撞 (ESDF dist < 0.18m)
4. 目標在障礙物內 → 搜索附近替代目標

---

## 5. Hector 整合注意事項

- `pose_type=3` (DEPTH_ODOM_INDEP): 深度圖和 odometry 分開訂閱, 避免時間戳同步問題
- `fast_planner_node` 延遲 5 秒啟動 (等待 Gazebo)
- `hector_cmd_bridge`: PD controller (Kp=1.8, Kv=0.8), 速度上限 2.0 m/s
- Hector Kinect: 60° FOV, 640×480, fx=fy=554.26

---

## 6. 測試世界作弊問題總結 (需修復)

| 優先級 | 世界 | 問題 | 建議修復 |
|--------|------|------|----------|
| **P0** | grid_maze | 柱子只到 z=1.25, 可直接飛越 | 改柱子 size 為 `0.3 0.3 5.0`, center z=2.5 |
| **P1** | long_wall | 牆高 2.0m (非註解的 2.5m), 飛越間隙 borderline | 改 box 為 `0.3 36 2.5` center z=1.25 |
| **P1** | ramp_slope | 斜坡與側牆間 0.85m 縫隙可繞過 | 增大斜坡寬度或將側牆內移 |
| **P2** | dense_forest | |y|>5.5 完全空曠可繞過 Section B | 增加外側柱子或延長 Section B |
| **P2** | obstacle_course Zone 1 | 間隙 4m 非 2m, 不算窄縫測試 | 調整牆位置使間隙為 2m |

---

## 7. 更新日誌

- **2026-04-12**: 初始版本 — 完成專案結構分析、測試世界審查、程式碼問題識別
- **2026-04-16**: 新增 dense_limit.world (漸進密度) 與 long_wall_bypass.world (16m 長牆) 兩個左右飛行極限測試世界
