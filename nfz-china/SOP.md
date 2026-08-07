# 禁飞区离线识别 — 操作 SOP 与 RID 二次开发说明

面向：飞控测试、产线部署、RID 工程师联调。

---

## 1. 目标与边界

### 1.1 要做什么

- 飞控本地读取 SD 卡禁飞区数据，判断当前是否在禁飞区内 / 是否即将进入。
- 地面在区内：**禁止解锁**，提示「禁飞区内禁止解锁」。
- 飞行中靠近边界（含提前量）：提示「禁止进入禁飞区，飞机即将返航」，默认切 **RTL**。
- 支持后期由 **RID 模块 / 地面站** 增删改动态圆形禁飞区（参数接口已就绪；MAVLink 协议待 RID 定稿）。

### 1.2 不做 / 限制

- **不是**民航局 UOM 官方矢量；当前基座来自大疆 GEO（禁飞 level 2/4）。合规仍以 UOM 为准。
- Lua 堆内存有限：**禁止**把整份 GeoJSON 读进脚本。
- 不能 100% 保证物理上「绝不进入」（GPS 误差、速度、切模式延迟）；靠 `NFZ_MARGIN` + 刹车距离尽量提前。
- 本仓库 ROMFS 的 `mavlink_msgs` **无** `COMMAND_LONG` 模块，脚本暂用 **参数 `NFZ_*`** 做增删改，避免启动报错。

---

## 2. 目录结构（整理后）

```
nfz-china/
├── README.md                 # 入口索引
├── SOP.md                    # 本文
├── sdcard/
│   └── EFT_nfz/              # ★ 上传到飞控 SD：EFT/nfz/
│       ├── index.csv         # 总索引（窗口筛选）
│       ├── circles.csv       # 圆形区
│       ├── polys.csv         # 多边形区（机场等）
│       ├── overlay.csv       # 动态增改（测试圆等）
│       └── deleted.csv       # 已删除 id 屏蔽表
├── raw/
│   ├── dji_cn_restricted_raw.json   # 拉取的原始 GEO
│   └── dji_cn_restricted.geojson    # 可视化预览（不要给 Lua 读）
└── tools/
    └── rebuild_pack.py       # 从 raw 再生 sdcard 包
```

飞控脚本路径（二选一）：

- 编进固件：`ROMFS_custom/scripts/nfz_zone_checker.lua`
- 或 SD：`EFT/scripts/nfz_zone_checker.lua`

默认 CSV 也可编进固件：`ROMFS_custom/nfz/*` → `@ROMFS/nfz/`。开机脚本检查 SD `EFT/nfz/`，**缺哪个写哪个，已有不覆盖**（动态 `overlay`/`deleted` 同理，固件里是空表头）。更新基座后需同步拷到 `ROMFS_custom/nfz/` 再编固件，或手动覆盖 SD。

---

## 3. 开发思路（为什么这样设计）

```
大疆 GEO API
    ↓ 网格拉取（禁飞）
raw JSON（含机场 sub_areas 多边形）
    ↓ tools/rebuild_pack.py
紧凑 CSV（圆 + 多边形 + 索引）≈ 二百 KB 级
    ↓ 拷贝到 EFT/nfz/
Lua 分帧加载「附近窗口」+ overlay
    ↓ 点落区 / 航迹外推
PreArm 禁止解锁 / RTL 返航
```

### 3.1 关键决策

| 问题 | 决策 |
|------|------|
| GeoJSON 太大 | 转 CSV；Lua 只读 CSV |
| 机场不是圆 | 优先 `sub_areas` 多边形；无多边形才用圆 |
| 全国一次扫爆 time limit | **分帧**读文件（每帧约 30 行） |
| 内存 | 只缓存附近窗口（`NFZ_RADIUS` km）+ 硬顶条数 |
| 动态改 | 基座只读；改动落 `overlay.csv` / `deleted.csv` |
| 与电子围栏 fence.stg | 独立；不占用全国 fence 容量 |

### 3.2 运行时判定

1. `index.csv` 筛出「圆心距飞机 < 窗口+包围半径」的 id。  
2. 加载对应 `circles` / `polys` 子集。  
3. 叠加 `overlay`，跳过 `deleted`。  
4. 圆：距离 ≤ 半径；多边形：射线法。  
5. **提前量**：`NFZ_MARGIN`（默认 50 m）+ 按地速估算的刹车距离；沿速度方向外推，预测将进入则触发。

---

## 4. 部署 SOP（测试 / 产线）

### 4.1 首次部署

1. 电脑打开 Mission Planner → MAVLink FTP。  
2. 进入 SD 根下 **`EFT`**，新建文件夹 **`nfz`**（若无）。  
3. 上传 `nfz-china/sdcard/EFT_nfz/` 内全部文件到 **`EFT/nfz/`**。  
4. 上传 `nfz_zone_checker.lua` 到 **`EFT/scripts/`**（或刷含该脚本的固件）。  
5. 确认 `SCR_ENABLE=1`。  
6. 推荐参数：

| 参数 | 推荐值 | 含义 |
|------|--------|------|
| `NFZ_ENABLE` | 0（默认关；要用时改 1） | 总开关 |
| `NFZ_ARMBLK` | 1 | 区内禁止解锁 |
| `NFZ_ACTION` | 2 | 靠近/进入 → RTL |
| `NFZ_MARGIN` | 50 | 边界外提前量（米） |
| `NFZ_RADIUS` | 40 | 附近缓存窗口（km） |
| `NFZ_BASE` | 1 | 加载基座全国包 |

7. 重启飞控。有定位后若干秒完成分帧加载（不再刷 time limit）。

### 4.2 公司现场测试圆（可选）

`overlay.csv` 示例（合肥公司测试，半径 100 m）：

```csv
id,kind,lat,lng,r
900001,0,31.7786729,117.2722463,100
```

或地面站写参数后执行：

- `NFZ_ID / LAT / LNG / RAD` 填好 → `NFZ_CMD=1` 写入  
- `NFZ_CMD=2` 按 ID 删除  
- `NFZ_CMD=4` 重载附近缓存  
- `NFZ_CMD=5` GCS 打印条数（调试用）

### 4.3 验收清单

| 步骤 | 期望 |
|------|------|
| 人在测试圆/禁飞区内，尝试解锁 | PreArm：`禁飞区内禁止解锁` |
| 人到区外 | 可正常解锁（其它 PreArm 除外） |
| 飞行靠近边界（留足 margin） | 提示：`禁止进入禁飞区，飞机即将返航`，切 RTL |
| GCS 不再出现 `exceeded time limit` | 通过 |
| `NFZ_BASE=1` 且 SD 有完整 csv | 附近机场等多边形参与判定 |

### 4.4 更新全国基座

1. 更新 `raw/dji_cn_restricted_raw.json`（重新网格拉取）。  
2. 执行：

```bash
python nfz-china/tools/rebuild_pack.py
```

3. 重新上传 `sdcard/EFT_nfz/` 中 `index/circles/polys`（可保留现场 `overlay.csv`）。  
4. 飞控 `NFZ_CMD=4` 或重启。

---

## 5. 提示文案（当前产品口径）

| 场景 | 文案 |
|------|------|
| 地面、已在区内 | `禁飞区内禁止解锁` |
| 飞行中靠近或进入（ACTION=2） | `禁止进入禁飞区，飞机即将返航` |
| 告警重复间隔 | 约 30 s（刚触发会立刻报一次） |

---

## 6. 与 RID 工程师二次开发约定

### 6.1 职责划分

| 角色 | 职责 |
|------|------|
| 飞控 / Lua | 本地几何判定、PreArm、RTL/Loiter、落盘 overlay/deleted |
| RID | 接收云端/UOM/厂家禁飞策略，转成「增删改」指令发给飞控；上报状态 |
| 地面站 | 调试参数、FTP 传包、联调可视化 |

### 6.2 当前已具备的飞控侧接口（参数）

RID / GCS 可通过 **PARAM_SET** 驱动（无需 COMMAND_LONG）：

| 参数 | 作用 |
|------|------|
| `NFZ_CMD=1` | UPSERT 圆：配合 `NFZ_ID, NFZ_LAT, NFZ_LNG, NFZ_RAD` |
| `NFZ_CMD=2` | DELETE：`NFZ_ID` |
| `NFZ_CMD=3` | 清空动态层 + 删除表 |
| `NFZ_CMD=4` | 重载附近基座缓存 |
| `NFZ_CMD=5` | 调试：打印 act/ov 等 |

写成功后脚本会把 `NFZ_CMD` 清 0，并更新 SD：`EFT/nfz/overlay.csv`、`deleted.csv`。

**建议 RID 侧流程：**

1. 收到云端禁飞变更 → 映射为圆（或先圆后多边形）。  
2. 写 `NFZ_ID/LAT/LNG/RAD` → `NFZ_CMD=1`。  
3. 轮询或等 STATUSTEXT `NFZ: CMD ok`。  
4. 删除同理 `NFZ_CMD=2`。  
5. 大批量更新：可改由 RID 写 SD 文件后发 `NFZ_CMD=4`（需约定 FTP/路径权限）。

### 6.3 推荐后续：自定义 MAVLink（RID 主导定消息）

因本机暂无通用 `COMMAND_LONG`，建议 RID 与飞控共定 **专用消息**（写入你们已有的 `ROMFS_custom/scripts/modules/MAVLink/`），例如：

**方案 A — 单圆指令（与现参数对齐）**

```
NFZ_ZONE_CMD
  op: uint8   # 1=upsert 2=delete 3=clear 4=reload
  zone_id: uint32
  lat: int32  # degE7
  lng: int32  # degE7
  radius_m: uint16
```

**方案 B — 多边形分片（机场级）**

```
NFZ_POLY_HDR   zone_id, n_vert, flags
NFZ_POLY_VERT  zone_id, seq, lonE7, latE7   # 多包拼完再生效
```

飞控 Lua：`mavlink:register_rx_msgid` + 你们现有 `mavlink_msgs` 扩展；**不要**依赖缺失的 `mavlink_msg_COMMAND_LONG.lua`。

### 6.4 RID 联调检查表

| 项 | 说明 |
|----|------|
| 路径 | 数据必须在 `EFT/nfz/`，不是 `APM/` |
| ID 段 | 建议动态区用 `900000+`，避免与基座 GEO id 冲突 |
| 幂等 | 同一 `zone_id` 重复 UPSERT = 修改 |
| 删除基座 | DELETE 写入 `deleted.csv` 屏蔽，不改 polys 原文 |
| 状态回传 | 可选：RID 订 STATUSTEXT，或后续加 `NFZ_STATUS`（区内/外、最近距离） |
| 安全 | 飞行中改表要评估；建议降落/上锁后再大批量更新 |

### 6.5 与现有 RID 心跳脚本共存

- `rid_heartbeat_check.lua` 与 `nfz_zone_checker.lua` 各占 **独立** `arming:get_aux_auth_id()`。  
- 任一失败都会 PreArm；RID 丢线和禁飞区内会同时拦解锁（符合预期）。  
- 参数表 key：NFZ 使用 `98`，勿与 RIDHB/UOM 等冲突。

---

## 7. 常见问题

| 现象 | 处理 |
|------|------|
| `exceeded time limit` | 使用分帧版脚本；勿一次扫完 polys；`NFZ_BASE=0` 可仅测 overlay |
| `Waiting for auxiliary authorisation` | 脚本曾崩溃占鉴权槽；修好脚本后重启；或区内本就会拦解锁 |
| 只有测试圆、没有附近机场 | `NFZ_BASE=1` + SD 上有完整 `index/polys/circles` |
| 改了 MARGIN 默认仍是旧值 | 参数已存在时需手动改 `NFZ_MARGIN` |
| 想用 UOM 官方数据 | 需官方/厂家对接；当前包不可替代 UOM |

---

## 8. 变更记录（摘要）

- 数据：大疆禁飞 → 多边形优先 + 圆兜底 → `EFT/nfz` CSV。  
- 脚本：窗口缓存、分帧加载、overlay CRUD（参数）、margin 提前、ACTION 默认 RTL。  
- 文案：地面「禁飞区内禁止解锁」；在飞「禁止进入禁飞区，飞机即将返航」。  
- MAVLink COMMAND_LONG：暂禁用，留给 RID 自定义消息二次开发。
