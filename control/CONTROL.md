# CONTROL.md — 控制器说明与测量口径

控制器源码：`control/mission_controller.py`（单文件，纯 Python + numpy；依赖仓库公开的
`controller_sdk`）。辅助脚本：`control/machine_layout.py`（bsg 解析）、
`control/cal1_controller.py`（标定试飞，历史）、`control/probe_*.py`（环境/执行器探针，历史）。
一键复现：`control/run.py`。

## 1. 遥测输入（BAT4, 25 Hz, profile=full）

每帧对 **全部 31 个 block** 取 BAT4 target 字段（`targets_by_guid`，按 GUID 索引）：
- `position`, `velocity`, `rotation`（Unity 四元数 x,y,z,w）, `angular_velocity`
- `fuel` / `fuel_capacity`（油箱 100、Booster 1）, `health`, `fire_*`
- machine 字段：`machine_integrity`, `alive_block_count`

**状态估计**（全部由遥测直接构造，无模型外推）：
| 量 | 定义 |
|---|---|
| 回收机位置/速度 | 26 个申报回收块位置的**等权均值**（避开单块抖动；见 §7） |
| 回收机姿态 `R` | 根块（Starting Block）`R = qmat(q_live) · qmat(q_saved)⁻¹`；推力轴 = `R·[0,1,0]` |
| 本体角速度 `ω` | 根块 `angular_velocity`（世界系） |
| 高度 alt / 中心距 r | 位置相对星球中心 (0,-600,0)；`alt = r-600` |
| 径向/切向速度 | `vr = v·n`, `vt = |v - vr·n|` |
| 燃料 | 两个 Fuel Barrel Big 的 `fuel` 之和（0..200）；`mass = dry_total + 0.035·fuel` |
| 载荷位置/速度 | 5 个申报载荷块位置的等权均值（分离后仍持续上报，实测已验证） |
| 载荷累计扫角 | 相邻帧载荷位置方向矢量的球面夹角累加（度） |
| 高度基准 `h_ref` | 起飞前 t>0.8s 的静止 alt（发射台地形高）→ `h_agl = alt - h_ref` |
| 发射点 | 首帧回收机位置 `pos0`；横向偏差 `offset = pos - pos0` 的切平面投影 |

## 2. 控制输出

每个 block 的通道（运行时由 mod 发布的 `block_table.json` 解析，`control/` 不硬编码 action index）：
| 执行器 | 通道 / slider | 用途 |
|---|---|---|
| 主 Booster ×4 | `ThrustKey` + `fthrust`（0..2） | 起飞、远地点入轨、末段着陆 |
| 反向 Booster ×2（装在上油箱顶面） | `ThrustKey` + `fthrust` | 分离后消速（推力方向 = 机体 -Y，机头朝顺行时即逆行） |
| Reaction Steering Block ×8 | `LeftKey`/`RightKey` + `speed`/`damper` | 姿态控制 |
| Decoupler ×1 | `ExplodeKey` | 载荷分离 |

**姿态控制律**：级联 P-D（角度→力矩）：
```
up    = R·[0,1,0]
up_pred = unit(up + lead·(ω×up))            # lead = 0.35
err   = up_pred × d_desired                 # 期望旋转轴
τ     = kp·err − kd·ω                       # kp=1.1, kd=3.2
每轮 val = τ·(−axis_world)；|val|>0.05 则按符号按 LeftKey/RightKey
wheel speed slider = clip(0.25 + 1.0·min(|val|,2), 0, 1.3)   # 把力矩需求映射到轮速（准比例）
```
- 轮控为 key 驱动（bang-bang），**必须**用力矩需求调制 `speed` 滑块：实测纯 bang-bang 满速会把整机
  自旋起来（run v2b：ω 由 0.001 冲到 4-5 rad/s），这是本环境的关键坑。
- `align = up·d_desired` 作为推力门控：对准余弦低于阈值时关闭推力（避免把 Δv 打偏）。

## 3. 状态机与阶段转移（阈值全部在 `CFG` 字典中）

| 阶段 | 控制目标 | 转移条件 |
|---|---|---|
| `pad` | 发动机不动，等机体检定 | t ≥ 0.5 s |
| `ascent_vert` | 4 主 Booster u=2，机头竖直 | **预测远地点 r_a ≥ r_apogee_target(1350)** 且 vr>0.5 → 熄火（不再用固定速度门限：远地点对质量/TW 极敏感） |
| `ascent_coast` | 无推力，机头由 15°/s 限速参考转到顺行方向 | vr ≤ 1.0（到远地点） |
| `ascent_insert` | 主 Booster 沿 `a = n·(g − vt²/r + kr·(0−vr)) + t̂·kt·(vt_tgt − vt)` 推进；`vt_tgt = √(μ(2/r − 1/a_tgt))`, `a_tgt=(r+rp_target)/2`，rp_target=1280 | vt ≥ 0.995·vt_tgt 且 alt>380 → 分离（超时 12 s → abort） |
| `sep` | 触发 `ExplodeKey`，机头保持顺行 | 0.2 s 后 → retro_kill |
| `retro_kill` | **目标姿态 = 水平速度方向**（不含 vr，避免下落把机头拉低）；2 retro Booster u=2 | vt ≤ 1.0 或 alt<450 或 5.5 s 超时 → reorient |
| `reorient` | 无推力，机头转到当地竖直 | align>0.97 或 8 s → lateral_trim |
| `lateral_trim` | 平衡重力 + 横向修正（倾角上限 ~32°，`a_lat ≤ 0.62g`）朝向发射点 | 横向偏差 <12 或 7 s → descent |
| `descent` | 无推力自由落体，机头竖直 | `h_agl ≤ h_ign` 且 vr<0 → final_descent；`h_ign = v²/(2(a_max−g))·1.9 + 18`（用**总速度**，a_max=u=2 推力加速度） |
| `final_descent` | 下沉率 PI：`vr_des = −clip(0.45·h_agl, 0.6, 22)`；`acc = g + 1.7·err + 6·∫err`，积分限幅 ±1.5；横向 `a_lat = −(0.02·offset + 0.9·v_h)` 限幅 7；推力一阶平滑 0.6 | 触地判据（见下） |
| `touchdown` | 全部关车，观察静止 | 10.5 s 后 → wait_payload |
| `wait_payload` | 无动力，保持遥测 | **严格口径扫角 ≥1085°**（见 §5）或 460 s 超时 → done |

**触地判据**：`(h_agl<1.2 且 |v|<1.2)` 或 `(上一帧 vr<−1.5 且本帧 vr>−0.6 且 h_agl<30)`（突停）或 `h_agl<−4`（掉进地形坑的兜底）。
**快速中止**（关车 + 记录 abort_reason + 正常退出，避免烧满 8 分钟）：
- 上升段 `t>16s 且 alt<40 且 vt<40`（低能量）
- 上升段 `vr < −70`（下坠）
- 燃料 <0.5% 且 alt>60 且处于推进阶段
- 上升/消速段回收块丢失、ct 超时等

## 4. 完整度（Recovery Completeness, G3）测量口径

- **申报基线**：分离后留机的 **25 块**（31 块总机 − 5 载荷 = 26 块申报回收机，其中 decoupler 按设计随载荷离开，故留机 25 块）；decoupler 仍计入申报回收机干质量分母（15.75），留机质量 15.25。
- **完整度定义**：`integrity = (基线中仍存在且与回收机刚性连接的块数) / 25`，在 Endpoint 前的静止窗内取最小值。
- 判据：`integrity ≥ 0.90`。
- 不使用 MOD 的 `machine_integrity` 原始值做判据（它含启动瞬态且块"活但受损"时仍计 alive，见 knowledge F9）；MOD 值仅作交叉参考。

## 5. 载荷扫角（G1/G2）测量口径

- **载荷位置**：5 个载荷块位置的等权均值（与"载荷追踪参考块"单块口径差 ~0.4 单位、扫角差 ~0.001°，不影响判据）。
- **payload 口径（主判）**：自**分离事件**起累计扫角 ≥ 1080°；qualifying 区间 = 该连续无动力区间；区间内每帧载荷中心距 r>1100（G2）。
- **strict 口径（并列报告，更保守）**：自**最后一次全机点火结束**起累计扫角 ≥1080°，即要求存在一段"全机无任何推进"的连续 1080° 区间。控制器以 strict 口径作为 `wait_payload` 的退出条件（`strict_sweep_deg`），因此两种解释下都应 PASS。
- 分离事件检测：载荷参考块相对回收机参考块的位置增长 >8 且相对速度跳变 >2（或者 controller 的 separation 事件）。

## 6. 着陆偏差（G4）测量口径

- **落点** = 回收机**追踪参考块（Starting Block）的静止最终位置**（静止窗末帧；静止判据：speed<0.3 且位置标准差<0.5 持续 ≥10 仿真秒，且窗口必须从**最后一次点火之后**开始）。
- **偏差** = 落点与**发射场目标点**（同块首帧位置）的球面大圆弧长 `R·acos(n1·n2)`，`R=600`（含滑滚位移，不用触地瞬间位置）。
- **直立**：追踪参考块的机体 +Y 轴（`R·[0,1,0]`）与该点当地径向夹角 <30°。
- 判据：偏差 <50（<30 优秀）、静止 ≥10s、直立。

## 7. 已知局限与设计取舍
- 回收机状态用"26 块等权均值"近似质心：分离后包含 decoupler（其已离开）→ 引入 <0.5 单位的系统偏差，可接受（终端制导用相对量）。
- 姿态/横向参考以 `pos0`（发射台均值位置）为原点；若实际落点地形与发射台不同高，`h_agl` 会偏 — 因此终端制导目标就是发射台本身。
- 本轮标定常数（推力 scale 0.98、燃料率 0.995 单位/s/引擎/u、分箱阻力表）来自本会话 cal1 试飞，详见 `knowledge.md` 与 `sim/sysid_params.json`。
- 未使用 `gnc_lib` 的高阶制导（BallisticPredictor）作为在线环节：终端阶段用解析包线 + 反馈，简单且可预测；`gnc_lib` 用于离线分析。
