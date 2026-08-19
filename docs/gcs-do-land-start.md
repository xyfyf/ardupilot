# 地面站开发说明：`DO_LAND_START` 与 `LAND`（任务备降）

> 适用范围：ArduCopter（多旋翼）任务编辑、上传、触发与 UI。  
> 依据：本仓库 `libraries/AP_Mission/AP_Mission.cpp`、`ArduCopter/mode_auto.cpp`、`ArduCopter/GCS_MAVLink_Copter.cpp`、MAVLink `common.xml`。  
> **与 RALLY（集合点）无关**：RALLY 不在任务航点命令列表中，走普通 RTL。

---

## 1. 结论（给产品/UI 一句话）

| 命令 | MAVLink ID | 角色 |
|------|------------|------|
| `MAV_CMD_DO_LAND_START` | **189** | **标记**：某段备降/降落航段的起点，本身不飞 |
| `MAV_CMD_NAV_LAND`（界面常显示 `LAND`） | **21** | **导航动作**：飞到指定点并降落 |

任务备降用法：任务中写入一组或多组 `DO_LAND_START` + 降落段；地面站触发 **AUTO RTL（模式 27）** 或下发 **命令 189** 时，飞控跳到**地理距离最近**的 `DO_LAND_START`，再顺序执行后面的进近/`LAND`。

普通 RTL（模式 6）**不会**跳任务里的 `DO_LAND_START`。

---

## 2. 命令字段（`MISSION_ITEM_INT` / `MISSION_ITEM`）

### 2.1 `DO_LAND_START`（189）

| 字段 | 含义 | GCS 建议 |
|------|------|----------|
| param1 ~ param4 | 空 | 填 `0` |
| x / y（纬度 / 经度） | 可选；用于选最近备降段 | **建议填写备降场参考坐标**（可与下一条进近或 `LAND` 同区域） |
| z（高度，m） | 可选 | 可填相对高度；选最近点时参与 3D 距离 |
| `frame` | 高度坐标系 | 与任务一致，常用相对高度 |

说明：

- 飞控不解析 param1~4。
- 若本条 lat/lon 无效，选点时会尝试用**下一条有效导航命令**的位置算距离。
- 任务顺序执行到该条时：Copter **立即完成（空操作）**，进入下一条。不要把它当成必须飞达的航点。

### 2.2 `LAND` / `NAV_LAND`（21）

| 字段 | 含义 | GCS 建议 |
|------|------|----------|
| param1 | Abort Alt（m） | 多旋翼可 `0` |
| param2 | 精确降落模式 | `0` = 普通降落 |
| param3 | 空 | `0` |
| param4 | 期望偏航（deg）；NaN = 跟系统偏航策略 | 可 `0` 或 NaN |
| x / y | 降落点纬经度 | **应填有效坐标**（为 0 则原地降落） |
| z | 着陆高度（当前高度系下的“地面”） | 相对高度时通常 `0` |
| `frame` | 高度坐标系 | 与任务一致 |

**Copter 执行逻辑：**

1. lat/lon 非 0：先以**当前高度**水平飞到该点，再进入下降降落。  
2. lat/lon 为 0：原地进入降落。

---

## 3. 任务结构（GCS 如何组航点）

### 3.1 单备降场

```text
… 正常作业航点 …

DO_LAND_START     ← 标记；建议带 lat/lon（备降参考点）
WAYPOINT / …      ← 可选进近
LAND              ← 真实落点（lat/lon 有效，alt 常为 0）
```

### 3.2 多备降场（才能“选最近”）

```text
… 作业航点 …

DO_LAND_START     # 备降 A（坐标靠近 A）
LAND              # 落点 A

DO_LAND_START     # 备降 B（坐标靠近 B）
LAND              # 落点 B
```

### 3.3 选点算法（飞控）

`AP_Mission::get_landing_sequence_start()`：

1. 遍历任务中所有 `DO_LAND_START`；  
2. 取该条位置（无效则取后续导航点位置）；  
3. 与**当前飞机位置**算距离，取最近；  
4. `set_current_cmd` 跳到该 index，从该标记往后继续执行。

成功时 GCS 文本常见：`Landing sequence start`。  
失败常见：`Unable to start landing sequence`。

### 3.4 规划注意

- 若希望飞机先飞到某高度点再降，在 `DO_LAND_START` 与 `LAND` 之间加 `WAYPOINT`，**不要**指望飞机会“飞停”在 `DO_LAND_START` 那一行。  
- 任务仅含 `DO_LAND_START`+`LAND`、无起飞作业段时：上传合法，但通常需触发 AUTO RTL / 命令 189 才会跳入执行；产品文案需写清楚。  
- 可选：`DO_RETURN_PATH_START`（188）标记回程段；切模式 27 时本仓库会**优先**尝试接回程，失败再跳 `DO_LAND_START`。

---

## 4. 触发方式（GCS 必须支持）

| 方式 | 实现 | 说明 |
|------|------|------|
| 切模式 | `SET_MODE` → **AUTO_RTL = 27** | 非普通 AUTO(3)，也非 RTL(6) |
| 发命令 | `COMMAND_INT` / `COMMAND_LONG`：`MAV_CMD_DO_LAND_START`(189) | 立即选最近备降段并跳转 |
| 失控保护 | 如 `FS_THR_ENABLE=6`、`FS_GCS_ENABLE=6` | 内部走 AUTO DO_LAND_START，失败则 RTL |
| 遥控辅助 | 通道映射 `AUTO_RTL` | 与切模式 27 同类 |

### 4.1 两种“189”勿混用

| 场景 | 行为 |
|------|------|
| 写入任务的 **mission item** | 仅标记；顺序执行到时立即跳过 |
| 即时 **COMMAND**（非任务项） | **立即**选最近 `DO_LAND_START` 并跳转 |

地面站「一键任务备降」应发 **COMMAND 189**，或切 **mode 27**。

### 4.2 与普通 RTL / RALLY

| 功能 | 数据存在哪 | 触发 | 行为 |
|------|------------|------|------|
| 任务备降 | 任务航点里的 `DO_LAND_START` | AUTO RTL / 命令 189 / FS=6 | 跳任务降落段 |
| Rally | 独立 Rally 存储（非航点命令） | 普通 RTL | 飞最近 Rally 或 Home 后降落 |
| 普通回家 | Home | 普通 RTL | 回 Home |

UI **不要**在航点命令下拉里找 “RALLY” 字样。

---

## 5. GCS 实现建议

### 5.1 任务编辑

- 命令列表提供 `DO_LAND_START`、`LAND`。  
- `DO_LAND_START` 标注为「备降段起点（标记）」，地图上勿画成必经飞达点。  
- 支持多组备降段；地图分色/分组显示。

### 5.2 上传前校验

- 每个 `DO_LAND_START` 之后、下一个 `DO_LAND_START` 之前，应能到达 `LAND` / `NAV_VTOL_LAND`（或产品明确允许的降落类命令）。  
- `LAND` 的 lat/lon 一般不应为 0（除非故意原地降）。  
- 若产品依赖失控动作 6，提示检查对应 FS 参数。

### 5.3 运行时按钮与状态

- 「任务备降 / AUTO RTL」→ `SET_MODE(27)` 或 `COMMAND(189)`。  
- 「普通返航」→ RTL(6)，文案标明**不使用**任务备降段。  
- 根据 `COMMAND_ACK`、STATUSTEXT、当前模式是否进入 AUTO RTL，提示成功/失败。  
- 可选：根据当前任务 index 是否处于某 `DO_LAND_START` 至 `LAND` 之间，显示「正在执行任务降落段」。

### 5.4 模式号参考（Copter）

| 模式 | 值 | 与备降关系 |
|------|-----|------------|
| AUTO | 3 | 正常跑任务；撞到 `DO_LAND_START` 只是跳过标记 |
| RTL | 6 | 不跳 `DO_LAND_START` |
| AUTO_RTL | **27** | 跳最近降落段（或先回程段） |
| LAND | 9 | 当前点降落模式，与任务备降段无关 |

---

## 6. 最小联调清单

1. 上传一组 `DO_LAND_START` + `LAND`，读回字段一致。  
2. Auto 执行其它航点时发 `COMMAND 189` 或切 mode 27 → 当前航点跳到该 `DO_LAND_START`，随后执行 `LAND`。  
3. 两组备降：飞机靠近 B 触发 → 应进入 B 段。  
4. 切普通 RTL → 不进入任务备降段。  
5. 任务无 `DO_LAND_START` 时发 189 / mode 27 → 失败 ACK / 文本提示，UI 不得显示成功。

---

## 7. 示例任务片段（示意）

| seq | command | lat | lon | alt | 说明 |
|-----|---------|-----|-----|-----|------|
| 1 | `DO_LAND_START` (189) | 备降参考点 | 备降参考点 | 100（相对） | 标记；选点用 |
| 2 | `LAND` (21) | 落点 | 落点 | 0（相对） | 水平飞到后降落 |

若 seq1 与 seq2 坐标不同：触发后飞控从标记瞬间切到下一条，再按 `LAND` 飞向落点（先水平到点，再下降）。中间需要进近高度/航点时，在两条之间插入 `WAYPOINT`。

---

## 8. 相关源码索引

| 内容 | 路径 |
|------|------|
| 最近 `DO_LAND_START` 选择 | `libraries/AP_Mission/AP_Mission.cpp` → `get_landing_sequence_start` / `jump_to_landing_sequence` |
| 任务中执行标记（空操作） | `ArduCopter/mode_auto.cpp` → `MAV_CMD_DO_LAND_START` |
| `LAND` 飞到点再降 | `ArduCopter/mode_auto.cpp` → `do_land` |
| GCS 发 189 触发跳转 | `ArduCopter/GCS_MAVLink_Copter.cpp` → `MAV_CMD_DO_LAND_START` |
| 模式 27 入口 | `ArduCopter/mode.cpp` → `AUTO_RTL` → `return_path_or_jump_to_landing_sequence_auto_RTL` |
| 协议定义 | `modules/mavlink/message_definitions/v1.0/common.xml` → `MAV_CMD_DO_LAND_START` / `MAV_CMD_NAV_LAND` |
