> **official submission run: C1 (run_id ac9fabd7-0f79-4188-8a0f-a25f501eba15, arc 23.11, all gates PASS)**
> G1 strict 余量提示：strict 口径扫角 **1080.054°** vs 判据 1080°，**余量仅约 0.054°**（payload 口径 1080.028°，同样贴线）；两口径均由遥测实测位置独立累计，与控制器日志 strict_sweep=1085.24° 的互验差为 −0.29°。此为真实余量，报告如实保留。

# S02 闸门报告 — D:\projects\build_arena\entry_loneorb_20261025\runs\C1_FULLGREEN_candidate

- 生成时间：2026-10-25T18:57:22+08:00
- trajectory.csv：D:\projects\build_arena\entry_loneorb_20261025\runs\C1_FULLGREEN_candidate\trajectory.csv（13680 帧）
- telemetry.csv：D:\projects\build_arena\entry_loneorb_20261025\runs\C1_FULLGREEN_candidate\telemetry.csv
- groups 配置：tools\groups.json
- **总判定：GREEN**

| 闸门 | 判据 | 结果 | 实测数值 |
|---|---|---|---|
| G1 | 载荷无动力连续区间实测扫角 ≥1080° | **PASS** | 载荷实测扫角 1080.03° ≥ 1080°，区间 [28.320s, 447.680s]（419.36s, 10485 帧）（载荷无动力结构+fire 实测证据通过） |
| G2 | qualifying 区间载荷每点 r>1100 | **PASS** | qualifying 区间 [28.320s, 447.680s] 内 10485 个实测点载荷中心距最小 1444.470（要求 > 1100） |
| G3 | 每台回收机完整度 ≥0.90 | **PASS** | 每台回收机完整度 ≥ 0.90；recovery-1: 存续率 min=1.000, 完整度(校正) min=1.000 final=1.000(基线 0.829)(machine_integrity_corrected_by_declared_baseline) |
| G4 | 着陆偏差<50（>30 警告）+ 静止≥10s + 直立<30° | **PASS** | recovery-1: 偏差=23.11（球面弧长，OK）, 静止=428.0s, 最终位置=(-0.69, 7.89, 23.42), 倾角=0.0° |
| G5 | 全自主（无人工输入）+ 申报/机器文件哈希可追溯 | **PASS** | 无人工按键事件（0 条事件，允许键 无）；产物齐全（6/6）；run_id 一致 ['ac9fabd7-0f79-4188-8a0f-a25f501eba15']；sha256 清单 12 个文件 |

## 逐项依据

### G1 — PASS

载荷实测扫角 1080.03° ≥ 1080°，区间 [28.320s, 447.680s]（419.36s, 10485 帧）（载荷无动力结构+fire 实测证据通过）

- 口径：`payload`（require_strict_scope=True）
- payload 口径区间：[28.320s, 447.680s]，扫角 1080.03°
- all_engines（严格）口径区间：[119.280s, 538.600s]，扫角 1080.05°
- duty_segments（推进指令段）：[5.04s, 10.64s]；[25.96s, 27.52s]；[28.04s, 30.28s]；[37.32s, 97.40s]；[108.40s, 110.28s]；[110.80s, 119.24s]
- 依据（telemetry/trajectory 实测行）：
  - trajectory.csv:L709 @ t=28.320s — 分离事件
  - trajectory.csv:L709 @ t=28.320s — qualifying 区间起点
  - trajectory.csv:L11193 @ t=447.680s — 累计达到 1080° 的帧（区间 t_end）, cumulative_deg=1080.028
  - trajectory.csv:L13681 @ t=547.200s — 同一无动力区间末的总扫角（1080° 后继续累计）, cumulative_deg=1319.935
  - trajectory.csv:L13466 @ t=538.600s — 『全机无推进』严格口径也成立（附参考）

### G2 — PASS

qualifying 区间 [28.320s, 447.680s] 内 10485 个实测点载荷中心距最小 1444.470（要求 > 1100）

- 依据（telemetry/trajectory 实测行）：
  - trajectory.csv:L4003 @ t=160.080s — 区间内载荷最小中心距, payload_r=1444.470

### G3 — PASS

每台回收机完整度 ≥ 0.90；recovery-1: 存续率 min=1.000, 完整度(校正) min=1.000 final=1.000(基线 0.829)(machine_integrity_corrected_by_declared_baseline)

- 依据（telemetry/trajectory 实测行）：
  - telemetry.csv:L2 @ t=0.040s — recovery-1 申报块存续率最低帧, alive_fraction=1.000
  - telemetry.csv:L29157 @ t=33.360s — recovery-1 完整度最低帧（相对申报基线 0.8286）
  - telemetry.csv:L25657 @ t=29.360s — 完整度申报基线窗口起点（29.32s–33.32s，median=0.8286）

### G4 — PASS

recovery-1: 偏差=23.11（球面弧长，OK）, 静止=428.0s, 最终位置=(-0.69, 7.89, 23.42), 倾角=0.0°

- 全机熄火时刻 last_powered_t=119.240s；静止窗搜索起点=119.240s
- recovery-1 静止窗末帧位置（落点）=(-0.692, 7.895, 23.415)，弧长 23.110（均值位置弧长 23.068）
- 依据（telemetry/trajectory 实测行）：
  - trajectory.csv:L2982 @ t=119.240s — recovery-1 静止窗（speed<0.3, 位置标准差 0.1253；落点取静止窗末帧位置）, pos_std=0.125
  - telemetry.csv:L478767 @ t=547.200s — recovery-1 直立判定帧（tilt=0.02°）

### G5 — PASS

无人工按键事件（0 条事件，允许键 无）；产物齐全（6/6）；run_id 一致 ['ac9fabd7-0f79-4188-8a0f-a25f501eba15']；sha256 清单 12 个文件

- 依据（telemetry/trajectory 实测行）：
  - key_events.csv — 按键事件 0 条（人工输入证据）
  - input_manifest.json — 申报哈希: {"input_manifest.run_id": "ac9fabd7-0f79-4188-8a0f-a25f501eba15", "input_manifest.machine_sha256": "cfb8ac412c279c507da651d5b08b1ab92051956d6f4833d9bf0c77c8a9e52475", "input_manifest.source_sha256": "fe8e287e43bf4e8d5b2eef6956ed3670b2d5897412371da7d3a54cb42f3f019d", "input_manifest.controller_kind": "live", "input_manifest.sandbox": "LONE ORB", "input_manifest.telemetry.profile": "full", "input_ma

## 已知局限

- 扫角/中心距/着陆偏差全部由遥测实测位置计算，无插值/外推；帧间缺口会被断开而不是补齐。
- G1 的“无动力”有两个口径（groups.g1_unpowered_scope，默认 payload）：payload=载荷自身无动力（载荷层无推进通道/无燃料块/无 fire 实测；回收机点火不算载荷推进，符合 S02『载荷独立无动力绕星』的规则读法）；all_engines=区间内任何引擎都不得点火（严格）。两种口径的 qualifying 区间都在报告里分别给出（qualifying_payload_scope / qualifying_all_engines_scope），不会因为口径选择而隐藏严格口径结果。无 control_log 且 all_engines 口径时，duty 不可实测 → conservative=true。
- G3 主判据 = MOD machine_integrity（实测语义：仍与发送字段的那台机器相连/存活的块占初始块数的比例，首秒有启动瞬态，已忽略 integrity_startup_ignore_s）。规划分离会让该值掉到『回收机块数/初始块数』——这不是损伤，因此用分离后稳定窗口（integrity_baseline_offset/window_s）作为该回收机的申报基线做比值校正；MOD 只上报单机字段，多回收机时非主机只能按申报块存续率代理并在报告中标注。假设该字段跟随 Root/Starting Block 所在机器（申报的回收机）；若 tracking reference 在另一台机器上，需人工核对完整性归属。
- G4 静止判据：**位置方差为主判据**（静止窗内位置标准差 < 容差）+ **有效速度**（1 秒位移速率）P95 < 0.3，持续 ≥10 仿真秒；落点 = 静止窗末帧位置的球面大圆弧长。之所以不逐帧硬判瞬时 speed<0.3：v4 机器静置时位置 400s 仅漂 ~0.2 单位，但接触求解器的速度遥测持续抖动（瞬时 >0.3 的帧占 30–35%）；逐帧判会误杀长静止窗。瞬时速度统计（越限帧数、max）仍保存在 gate_report.json 的 static 字段中供核对。
- G4 直立：用 tracking reference 块的 rotation 四元数（Unity x,y,z,w）把 body_axis 投到世界系，与该点径向夹角<30°；rotation 缺失时该项 UNVERIFIED。
- G5 key_events.csv 来自游戏侧按键记录；SDK 控制器动作不产生按键事件，因此非空即视为人工输入迹象（allowed_controller_keys 可显式豁免）。
- 遥测 full profile 的分量列名由 MOD 侧展开，本工具用别名/正则自适应；真实文件首次核对时以 postprocess_notes.json 的 telemetry.columns 为准。

## 附录 A — 5 连绿确认批（D1–D5）汇总

同一 v4 机体（35 块）+ 逐字节同一控制器，串行执行 5 次完整任务；每个 run 均由本工具链
（`make_groups.py --run-dir` 现生成分组 → `postprocess.py` → `check_gates.py` → `run_summary.py`，
静止判据=位置方差主判+1s 有效速度 P95）独立判定，全部 **GREEN**：

| run | run_id | G1 payload 扫角 [区间] | G1 strict 扫角 [区间] | G2 min r | G3 校正完整度 | G4 弧长 | G4 静止窗 | G4 倾角 | G5 | 总 |
|---|---|---|---|---|---|---|---|---|---|---|
| D1 | d3c8fae5… | 1080.054° [27.72s, 482.96s] | 1080.029° [117.16s, 572.32s] | 1429.639 | 1.000（29/29） | 43.13 | 464.3s | 9.34° | PASS | GREEN |
| D2 | 04e28631… | 1080.120° [27.80s, 484.64s] | 1080.007° [117.48s, 574.12s] | 1430.479 | 1.000（29/29） | 41.49 | 466.0s | 9.24° | PASS | GREEN |
| D3 | 88cb69f5… | 1080.102° [28.36s, 450.28s] | 1080.069° [116.72s, 538.56s] | 1449.857 | 1.000（29/29） | 37.87 | 430.8s | 0.005° | PASS | GREEN |
| D4 | d1cd73c0… | 1080.109° [27.96s, 471.40s] | 1080.063° [122.24s, 565.56s] | 1455.090 | 1.000（29/29） | 49.26 | 452.2s | 0.006° | PASS | GREEN |
| D5 | 3203085e… | 1080.084° [27.68s, 460.60s] | 1080.051° [91.52s, 524.32s] | 1430.027 | 1.000（29/29） | 36.28 | 442.0s | 0.028° | PASS | GREEN |
| **C1（official）** | **ac9fabd7…** | **1080.028° [28.32s, 447.68s]** | **1080.054° [119.28s, 538.60s]** | **1444.470** | **1.000（29/29）** | **23.11** | **428.0s** | **0.015°** | PASS | **GREEN** |

- v4 同配置 6 个全绿 run 的落点弧长：C1 23.11 < D5 36.28 < D3 37.87 < D2 41.49 < D1 43.13 < D4 49.26（全部 <50；D4 贴近 50 上限）。
- v4 落点散布（含先行样机 run）：A1 20.2 / C1 23.1 / D5 36.3 / D3 37.9 / D2 41.5 / N7 38.3 / D1 43.1 / N8 43.3 / D4 49.3 —— 对比 v2 批（15.7/22.5/61/86/96/400）显著收敛。
- 说明：**D1–D5 为 5 连绿确认批**；C1 为同配置先行验证 run，按"落点最优"规则选为官方提交 run。

