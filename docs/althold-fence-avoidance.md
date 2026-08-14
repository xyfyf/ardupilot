# 定高模式电子围栏限速 — 开发说明

> 状态：待开发（代码已回滚，本文档保留完整思路与补丁内容）

## 1. 需求

飞机在 **AltHold（定高）** 模式下，飞手推杆朝围栏外飞时，应在 **FENCE_MARGIN 缓冲带内** 减速停住，**不能飞出电子围栏**。

## 2. 问题根因

### 2.1 AltHold 与 Loiter 对围栏的处理完全不同

| 模式 | 围栏处理方式 | 效果 |
|------|-------------|------|
| **Loiter** | `AC_Loiter::calc_desired_velocity()` 每周期调用 `AC_Avoid::adjust_velocity()`，连续限速 | 贴边减速，不易越界 |
| **AltHold** | 仅调用 `AC_Avoid::adjust_roll_pitch_rad()`，**只处理 proximity 雷达**，不处理围栏 | 杆量直接改姿态，无水平限速 |

关键代码位置：

- `ArduCopter/mode_althold.cpp` — 定高主循环
- `libraries/AC_Avoidance/AC_Avoid.cpp` — `adjust_roll_pitch_rad()` 入口即判断 `proximity_avoidance_enabled()`，与围栏无关
- `libraries/AC_WPNav/AC_Loiter.cpp` — Loiter 围栏限速参考实现

### 2.2 围栏库 margin 只报警、不拦飞

`AC_Fence::check_fence_circle()` 在 `dist >= radius - margin` 时只 `record_margin_breach()`，**不会**在 AltHold 里自动减速或切模式。

现有 `ArduCopter/fence.cpp` 已在 **真正 breach 后** 统一切 Loiter，属于事后兜底，无法阻止「先飞出去再切」。

### 2.3 Lua 方案的局限（当前临时方案）

文件：`ROMFS_custom/scripts/fence_althold_to_loiter.lua`

做法：5Hz 轮询，距边界 + 径向速度 + 刹车距离估算，提前 `set_mode(Loiter)`。

**无法完美满足需求的原因：**

1. 离散切模式，非连续限速（100Hz 控制环 vs 5Hz 脚本）
2. `vehicle:set_mode()` 有延迟；Loiter 默认 `LOIT_BRK_DELAY=1s`，切过去后仍惯性冲出 0.8~2m+
3. Lua 无法在 AltHold 下直接限制杆量/速度（`set_target_velocity_NED` 仅 Guided 有效）
4. 速度 > 0.5m/s 实测仍会冲出一段才切 Loiter

**结论：Lua 可作兜底，根治需改固件。**

---

## 3. 推荐方案（方案 A）：AltHold 接入围栏限速

### 3.1 思路

在 `mode_althold.cpp` 的 100Hz 控制环中，复用 Loiter 同一套 `AC_Avoid::adjust_velocity()` 围栏逻辑：

```
飞手杆量 → lean 角
    ↓
换算期望水平速度（当前速度 + 杆量意图 + lean 加速度）
    ↓
AC_Avoid::adjust_velocity() 围栏限速
    ↓
若 limits_active()：将限速后速度反算为 roll/pitch，覆盖杆量
    ↓
姿态控制
```

与 Lua「切 Loiter」相比：**不换模式，在定高内连续拦截**。

### 3.2 调用顺序（与雷达避障的关系）

```
1. 读取飞手 lean（pilot_roll / pilot_pitch）
2. adjust_roll_pitch_rad()        — 雷达 proximity 避障
3. apply_fence_avoidance_to_lean_rad() — 围栏限速（新增，触发时覆盖 lean）
```

围栏为硬边界，限速触发时应覆盖 proximity 叠加后的 lean。

### 3.3 需修改的文件

| 文件 | 改动 |
|------|------|
| `ArduCopter/mode.h` | 声明 `apply_fence_avoidance_to_lean_rad()` |
| `ArduCopter/mode.cpp` | 实现上述函数（约 70 行） |
| `ArduCopter/mode_althold.cpp` | Flying 分支调用 |
| `ROMFS_custom/scripts/fence_althold_to_loiter.lua` | 固件完成后 **可删除** |

`ArduCopter/fence.cpp` 的 breach → Loiter 兜底 **保留**，不必改。

---

## 4. 具体代码

### 4.1 `ArduCopter/mode.h`

在 `get_avoidance_adjusted_climbrate_ms()` 声明后增加：

```cpp
    // limit pilot lean in GPS modes without a NE position controller (e.g. AltHold)
    // uses the same fence velocity logic as Loiter; only modifies roll/pitch when limiting is active
    void apply_fence_avoidance_to_lean_rad(float pilot_roll_rad, float pilot_pitch_rad, float &roll_rad, float &pitch_rad) const;
```

### 4.2 `ArduCopter/mode.cpp`

在 `get_avoidance_adjusted_climbrate_ms()` 实现后增加：

```cpp
// apply_fence_avoidance_to_lean_rad - limit horizontal lean near geofence in non-GPS-velocity modes
void Mode::apply_fence_avoidance_to_lean_rad(float pilot_roll_rad, float pilot_pitch_rad, float &roll_rad, float &pitch_rad) const
{
#if AP_AVOIDANCE_ENABLED
    AC_Avoid *avoid = AP::ac_avoid();
    if (avoid == nullptr || !avoid->enabled()) {
        return;
    }

    const float dt_s = attitude_control->get_dt_s();
    if (!is_positive(dt_s)) {
        return;
    }

    // current NE horizontal velocity (m/s)
    const Vector2f vel_ne_ms = pos_control->get_vel_estimate_NEU_ms().xy();

    // pilot stick intent as a velocity (m/s)
    float gnd_speed_limit_ms = wp_nav->get_default_speed_NE_ms();
    float ekf_gnd_spd_limit_ms, ahrs_control_scale_xy;
    AP::ahrs().getControlLimits(ekf_gnd_spd_limit_ms, ahrs_control_scale_xy);
    gnd_speed_limit_ms = MIN(gnd_speed_limit_ms, ekf_gnd_spd_limit_ms);

    const Vector2f pilot_vel_ms = get_pilot_desired_velocity(gnd_speed_limit_ms);

    // velocity vector for fence stopping-distance prediction
    Vector2f desired_vel_ms = vel_ne_ms;
    if (!pilot_vel_ms.is_zero()) {
        if (desired_vel_ms.is_zero() || pilot_vel_ms.length() > desired_vel_ms.length()) {
            desired_vel_ms = pilot_vel_ms;
        }
    }

    // include pilot lean acceleration so low-speed stick input still triggers early limiting
    const Vector3f pilot_euler_rad {pilot_roll_rad, pilot_pitch_rad, ahrs.get_yaw_rad()};
    const Vector3f pilot_accel_neu_mss = pos_control->lean_angles_rad_to_accel_NEU_mss(pilot_euler_rad);
    desired_vel_ms += pilot_accel_neu_mss.xy() * dt_s;

    Vector3f desired_vel_cms {desired_vel_ms.x * 100.0f, desired_vel_ms.y * 100.0f, 0.0f};
    avoid->adjust_velocity(
        desired_vel_cms,
        pos_control->get_pos_NE_p().kP(),
        pos_control->get_max_accel_NE_mss() * 100.0f,
        pos_control->get_pos_U_p().kP(),
        pos_control->get_max_accel_U_mss() * 100.0f,
        dt_s);

    if (!avoid->limits_active()) {
        return;
    }

    const Vector2f limited_vel_ms = desired_vel_cms.xy() * 0.01f;

    // map fence-limited velocity back to lean angles
    const float vel_ne_kP = pos_control->get_pos_NE_p().kP();
    float accel_n_mss = (limited_vel_ms.x - vel_ne_ms.x) * vel_ne_kP;
    float accel_e_mss = (limited_vel_ms.y - vel_ne_ms.y) * vel_ne_kP;

    const float accel_max_mss = angle_rad_to_accel_mss(attitude_control->lean_angle_max_rad());
    Vector2f accel_ne_mss {accel_n_mss, accel_e_mss};
    const float accel_len = accel_ne_mss.length();
    if (is_positive(accel_len) && accel_len > accel_max_mss) {
        accel_ne_mss *= accel_max_mss / accel_len;
    }

    pos_control->accel_NE_mss_to_lean_angles_rad(accel_ne_mss.x, accel_ne_mss.y, roll_rad, pitch_rad);
#endif
}
```

### 4.3 `ArduCopter/mode_althold.cpp`

将 `#if AP_AVOIDANCE_ENABLED` 块由：

```cpp
#if AP_AVOIDANCE_ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad());
#endif
```

改为：

```cpp
#if AP_AVOIDANCE_ENABLED
        // proximity sensor avoidance (radar etc.)
        const float pilot_roll_rad = target_roll_rad;
        const float pilot_pitch_rad = target_pitch_rad;
        copter.avoid.adjust_roll_pitch_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad());
        // geofence velocity limiting (same logic as Loiter; overrides lean when active)
        apply_fence_avoidance_to_lean_rad(pilot_roll_rad, pilot_pitch_rad, target_roll_rad, target_pitch_rad);
#endif
```

---

## 5. 备选方案（方案 B，改动更小）

仅改 `ArduCopter/fence.cpp`：在 `get_margin_breaches()` 非零且当前为 AltHold 时，提前切 Loiter（不必等真正 breach）。

- 比 Lua 快（25Hz vs 5Hz）
- 仍是离散切模式，高速下可能仍冲出 1~2m
- 建议配合 `LOIT_BRK_DELAY=0`

**方案 A 优于 B**；若时间紧可先做 B 过渡。

---

## 6. 参数与前提

| 参数 | 要求 |
|------|------|
| `FENCE_ENABLE` | 1 |
| `AVOID_ENABLE` | 含 bit0 UseFence（默认已开） |
| GPS | 必需 |

建议联调参数：

- `FENCE_MARGIN` — 缓冲距离（默认 2m，可按机型加大）
- `AVOID_BEHAVE` — 1（Stop，遇围栏直接停；0 为 Slide 贴边滑过）

---

## 7. 测试计划（联测 2~3 天）

1. **圆形围栏 + 定高**：不同速度（0.5 / 2 / 5 m/s）朝围栏外推杆，记录是否越界、冲出距离
2. **贴边切向飞**：沿围栏切向飞行，不应误触发
3. **朝内飞 / 松杆**：不应异常拉回
4. **弱 GPS / 无 GPS**：行为可接受（不限速或 fallback）
5. **与雷达避障同时开**：近围栏 + 近障碍物
6. **对比 Loiter**：同场景 Loiter 应表现接近
7. **真越界兜底**：确认 `fence.cpp` breach → Loiter 仍生效

测试注意：底层飞控改动，先在安全场地、备好转 Loiter/手动接管。

---

## 8. 完成后清理

固件方案 A 验证通过后：

- 删除 `ROMFS_custom/scripts/fence_althold_to_loiter.lua`
- 无需 Lua + 固件双轨（避免重复切模式）

---

## 9. 参考代码路径

| 用途 | 路径 |
|------|------|
| Loiter 围栏限速 | `libraries/AC_WPNav/AC_Loiter.cpp` → `calc_desired_velocity()` |
| 圆形围栏限速算法 | `libraries/AC_Avoidance/AC_Avoid.cpp` → `adjust_velocity_circle_fence()` |
| lean ↔ 加速度换算 | `libraries/AC_AttitudeControl/AC_PosControl.cpp` → `lean_angles_rad_to_accel_NEU_mss()` / `accel_NE_mss_to_lean_angles_rad()` |
| 定高垂直围栏（已有） | `ArduCopter/mode.cpp` → `get_avoidance_adjusted_climbrate_ms()` |
| breach 兜底 | `ArduCopter/fence.cpp` |
