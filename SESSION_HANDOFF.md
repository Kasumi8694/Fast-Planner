# Session Handoff — 2026-04-12

下一個 Claude Code session 請先閱讀此文件 + `DEBUG_REFERENCE.md` + `CLAUDE.md` 來恢復上下文。

---

## 當前任務：Lateral Navigation Limit Tests

用戶要測試 Fast-Planner 左右飛行避障的極限，具體兩個場景：

### Task 1: 密集障礙物極限測試
**問題**: 障礙物間距多窄時 planner 會失敗？
**結論**: 理論最小可通過間距 ≈ **0.8m**（inflated gap = 0.4m = 4 voxels, center ESDF = 0.4m = dist0）

### Task 2: 長牆繞行測試
**問題**: 牆超出深度相機 FOV（2.9m range, 60° → 3.35m 水平覆蓋）時，planner 能否透過 incremental replanning 繞過去？

**重要約束**: 用戶明確要求禁止無人機從上空飛越（virtual_ceil=2.5m），只測試左右飛行。

---

## 已批准的計劃（需要實作）

完整計劃在 `/home/etho/.claude/plans/swirling-wandering-lightning.md`

### 需建立的文件

#### 1. `fast_planner/plan_manage/worlds/dense_limit.world`
走廊式漸進密度測試：
- 走廊側牆 y=±4.0（height 6m，防繞路）
- Phase A (x=4,6): gap=1.2m — 應輕鬆通過
- Phase B (x=10,12): gap=1.0m — 應可通過
- Phase C (x=16,18): gap=0.8m — 邊界值，這是測試重點
- 所有圓柱 R=0.3m, height=6.0m（超過 virtual_ceil）
- Start (0,0,1.0) → Goal (22,0,1.5)

每 phase 用 staggered rows（交錯排列），迫使 drone 走 S 型。具體圓柱位置見計劃文件。

#### 2. `fast_planner/plan_manage/worlds/long_wall_bypass.world`
16m 長牆繞行測試：
- 牆 at x=8, height=5.0m, thickness=0.3m, length=16m (y=[-8,8])
- 牆端距離計算：從偵測點 (5,0) 到牆端 (8,±8) = 8.5m > A* horizon 7.0m
- 需 2-3 次 replan 才能找到牆端繞過去
- Start (0,0,1.0) → Goal (16,0,1.5)
- 加 visual markers（綠=起點區，紅=終點區）

#### 3. 更新文件
- `hector_fast_planner.launch`: 在 world 選項 comment 加入新世界名
- `DEBUG_REFERENCE.md`: 加入新測試世界分析

---

## 關鍵參數速查（設計地圖必知）

| 參數 | 值 | 影響 |
|------|-----|------|
| obstacles_inflation | 0.2m | 間距縮小 2×0.2=0.4m |
| dist0 (optimizer) | 0.4m | 軟約束，< 0.4m 被 penalty |
| search/margin | 0.2m | **已被 comment out，不使用** |
| collision threshold | 0.18m | runtime replan 觸發 |
| resolution | 0.1m | 1 voxel = 0.1m |
| virtual_ceil_height | 2.5m | 填充 inflate buffer 阻止飛越 |
| virtual_floor_height | 0.5m | 填充 inflate buffer 阻止低飛 |
| 深度相機 | 60° FOV, 2.9m max | 水平覆蓋 ~3.35m at max range |
| A* horizon | 7.0m | 單次搜索最大距離 |
| max_vel / max_acc | 0.8 / 0.8 | 轉彎半徑 ≈ v²/a = 0.8m |
| Flyable zone | z ∈ [0.5, 2.5] | 只有 2.0m 垂直空間 |

### 最小間距計算
- A* hard limit: gap > 0.4m (兩側各 0.2m inflation)
- Optimizer soft limit: gap ≥ 0.8m (兩側各 0.4m dist0)
- 實際極限: **~0.8m**（A* 能過，optimizer 在邊界，runtime check 0.18m < 0.4m center ESDF OK）

### Gazebo SDF 座標注意
- Box `<pose>x y z</pose>` + `<size>sx sy sz</size>` → 佔 [x±sx/2, y±sy/2, z±sz/2]
- Cylinder `<pose>x y z</pose>` + `<radius>r</radius><length>L</length>` → z 佔 [z-L/2, z+L/2]
- 所有障礙物高度必須 ≥ 2.5m（virtual_ceil），否則可被飛越

---

## 尚未處理的已知 Bug（來自 DEBUG_REFERENCE.md §3 和 §6）

這些在當前 task 之後需要處理：
1. **P0** grid_maze.world 柱子太矮可飛越
2. **P1** long_wall.world 牆高 2.0m 非 2.5m
3. **P1** ramp_slope.world 斜坡邊有 0.85m 縫隙
4. **P2** dense_forest / obstacle_course 作弊路徑
5. z=1.0 目標高度硬編碼 (kino_replan_fsm.cpp:75)
6. Shot trajectory 動態約束被 comment out (kinodynamic_astar.cpp:450-454)
7. 碰撞檢測閾值不一致 (0.18 vs 0.2 vs 0.4)

---

## 用戶偏好

- 使用繁體中文溝通
- 偏好一步一步系統化除錯
- 當前 focus: 先建立測試場景確認極限，之後再修 bug
- 用戶說「關於上下的我們會在之後處理」— 本輪只做左右飛行測試
