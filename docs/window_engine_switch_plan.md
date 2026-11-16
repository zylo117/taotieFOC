# 把「内部规划运动」接回窗口 DMA 引擎（0x187 bit1 运行时开关）— 实施清单

> 基线 HEAD `ce64073`。来源：只读勘察（子智能体，带行号引证），2026-11-17。
> 目的：开环落点散布从 ±0.3° 降到 ≤0.1°。依据：旧窗口引擎「每窗序列长度 == 实发步数」，
> 硬件上升沿数与步数账本**由构造保证一一对应**；连续 PWM 在 `velocityPwmHalt()` 会砍掉
> 在飞的周期（`CEN=0` 落在 `CNT<CCR` 时那一拍的沿根本不存在），停止时刻取决于账本穿越
> 相位 → 随机散布。实测对照：`c756ffa`（窗口）6400 步误差 +0.55 微步、散布 ±0.7 微步；
> HEAD（连续）3200 步平均 +0.60 微步、散布 −4.4~+6.3 微步。**均值差不多，差的是散布 8 倍。**

## 1. 现状要点（照写必读）

- 引擎开关只有一个：`motion_planner_.velocity_pwm_active_`。`velocityPwmTick()` 开头
  `if (!velocity_pwm_active_) return false;`（motion_planner.cpp:883-886），于是
  `emitSteps()`（窗口路径，motion_planner.cpp:644-871，调用点
  closed_loop_controller.cpp:2145）在当前 HEAD **不可达但是活编译代码**，无 `#if` 隔离。
  → 「接回去」= 让它可达，不需要复活任何代码。
- 窗口路径三层接口：`StepOutputChannel::emit(num_steps, direction, direction_changed,
  window_ns, step_pulse_ticks)`（step_output_channel.h:33）、`windowCapacity(window_ns)`
  （.h:39）、`stop()`（.h:42）；序列生成 `step_sequencer::fill/addDirectionGuard/
  resetHarmonicPhase`（step_sequencer.h:50-59）；缓冲 `seq_buffer_[130]`（.h:50）。
- 沿数契约：`seq_count = num_steps + 2`，头尾两格是 guard（step_output_channel.cpp:80）；
  PWM 模式 A + `ACTIVE_LOW` + `CCR=250` ⇒ 每格 PR 恰好一个上升沿；首格 PR=1、尾格 5 tick
  都 < CCR ⇒ 不出沿（stepper_driver.cpp:106-118、step_sequencer.cpp:46-63）。
- 预算/铺窗：`guard_ns=45`，`STEP_SEQ_SPREAD_PERMILLE=900`（只铺 90% 窗口，
  step_sequencer.h:31）；最小步距 500 tick=2µs（step_output_channel.h:23）。
- DMA：`buffer_size=seq_count`、`peripheral_base_addr=&TMR2->pr`、`loop_mode=FALSE`、
  `dma_flexible_config(DMA1, FLEX_CHANNEL2, DMA_FLEXIBLE_TMR2_OVERFLOW)`，重装前
  `haltTimerAndDma()`（step_output_channel.cpp:11-19、23-44）。
- ⚠️ `0x123` 运行时脉宽在窗口路径**不生效**（`startSequence` 把 CCR 写死成编译期 250，
  step_output_channel.cpp:51-53）。该文件是外部跟随共用的，本次不动。

## 2. 改动清单

### 改动 1（必做，1 行）让 bit1 写得进去
`src/parameter_router.cpp:549-556`：`value & 0x0005U` → **`value & 0x0007U`**，
并更新注释（closed_loop_protocol.h:311-314、closed_loop_controller.h:554-557）。
`engine_tuning_` 是 `volatile uint16_t`、不落 Flash、默认 0 = 连续引擎，重启安全。

### 改动 2（必做，唯一功能分流点）`startMotion()`
`src/closed_loop_controller.cpp:1252-1263`，把 `:1261-1263` 三行换成：

```cpp
// bit1=1：本段内部规划运动走窗口 DMA 引擎（每窗序列长度 == 实发步数）
// bit1=0：走连续 PWM（默认，同 3181616 之后行为）
motion_planner_.velocity_pwm_active_ = ((engine_tuning_ & 0x0002U) == 0U);
motion_planner_.velocity_pwm_running_ = false;
motion_planner_.velocity_pwm_dir_     = 0;
if (!motion_planner_.velocity_pwm_active_)
{
    motion_planner_.velocity_pwm_tier_         = 0;    // 四档阶梯缓存
    motion_planner_.velocity_pwm_narrow_       = false;
    motion_planner_.velocity_pwm_ccr_          = 0U;   // CCR 影子缓存
    motion_planner_.velocity_pwm_period_carry_ = 0.0f;
    motion_planner_.angle_start_gate_ticks_    = 0U;   // 角度模式起步门
    engine_reinit_events_                      = 0U;   // 0x185
    engine_tier_switch_events_                 = 0U;   // 0x186
    band_dither_active_                        = 0U;   // 0x181
    step_sequencer::resetHarmonicPhase();
}
```

`velocity_pwm_running_=false` 保证切回连续引擎那一拍必走「完整重装」分支
（motion_planner.cpp:1116），不会把窗口引擎留下的 DIV/CCR 当已装载状态。

### 改动 3（强烈建议）速度量程按窗口容量重钳
`closed_loop_controller.cpp:1160`：

```cpp
const float ceiling = ((engine_tuning_ & 0x0002U) ? WindowPulseCeiling() : kMaxPulsePps);
motion_max_step_s_ = (motion_max_rpm_ * rpm2step > ceiling) ? ceiling : motion_max_rpm_ * rpm2step;
```

`WindowPulseCeiling() = StepOutputChannel::windowCapacity(50000ULL) * 20000.0f`
= 22 步/窗 × 20k = **440 kpps**（= 4125RPM@32µs）。不钳的代价：规划曲线在窗口引擎上
不可实现 → 账本领先脉冲 → 缺口堆到减速末段蠕行 + 沉降超时，表现为「少走 + 慢停」。

### 改动 4（可选）诊断可读性
窗口模式下 `0x18C bit1/bit2` 恒 0、`0x176 step_period_us_` 陈旧；脚本判读引擎改看
`0x187 & 2`；`0x177`（单窗最大步数）在窗口模式重新有意义，1200RPM 应读到 6~7。
顺带可删 `closed_loop_controller.h:366` 那条无定义的旧声明 `emitStepSequence`。

## 3. 明确不用动（依据）

- **角度模式**：`setTargetAngleDeg()` 自己置 `velocity_pwm_active_=true`
  （closed_loop_controller.cpp:646-649）保持原样；`startMotion` 第一句就退出角度模式
  （:960），两者不抢 `current_step_speed_`。
- **缓停 0x163 / 急停 0x16E**：全在规划层（requestQuickStop :824-842、
  `steps_to_decel_=0xFFFFFFFF` 分支 motion_planner.cpp:513-518、归零后
  `motion_pulse_count_=motion_steps_emitted_` :584-591），与出口无关。
- **末端不砍尾**：`velocityPwmHalt()`（motion_planner.cpp:366/382）在窗口模式是空操作
  （`velocity_pwm_running_` 为假，:1210）；序列总长 90% 窗口同拍排空，之后 SETTLE 等
  15ms 静默才 `stopMotion()`（closed_loop_controller.cpp:1791-1833）→ 老「砍尾 0.36°」
  坑不会被触发。
- **外部跟随**：`enableExternalPulseFollow()` 的 halt（:2384）保留，
  `external_pulse_follower.cpp:190` 的 `emit` 一个字不改。

## 4. 风险清单

1. **窗口重挂相位毛刺回来（3181616 的原始动机）**：每 50µs 一次 `haltTimerAndDma +
   loadArrSequence + stepper_init_motion_timer`，含两次 `gpio_init` 把 PB10 在
   OUTPUT↔MUX 来回（stepper_driver.cpp:96、25-60）。1200RPM/128kpps 下每窗一次
   7.81→12.5µs（+60%），20kHz 及其谐波 ⇒ 当年实测 FE −20° vs −2°。**6000RPM/s 比当年
   600RPM/s 更陡，失锁风险更高。** 这就是必须做成开关并跑实验 2 的原因。
2. **量程 1.5Mpps → 440kpps**（改动 3 就是为它）。>440kpps 时账本领先实发 ⇒ 少走；
   撞墙特征：`0x177=22` 且 `0x183` 起跳。
3. **跨窗超窗被砍尾（静默少走）**：余量只有 ~5µs，某窗抖到 >55µs（USB/Flash/SPI 抢占）
   尾脉冲被下一次 `haltTimerAndDma` 砍掉，而账本在窗头已 += 命令步数
   （motion_planner.cpp:846）。对策：`STEP_SEQ_SPREAD_PERMILLE` 900 → **850**
   （step_sequencer.h:31，platformio.ini:51 已有开关），验收用 `0x134` 对编码器积分对账。
4. **CCR 影子必须保留**（e8186b8：stepper_driver.cpp:116 预装载 + :132 `ovfswtr`）。
   角度模式 tier3 会把 CCR 写成 1 tick；若去掉 :132，窗口装载时工作 CCR 还是 1，前两格
   PR=1/5 **就会越过旧 CCR ⇒ 每窗白送一个沿 = 20000 微步/秒**。切窗口不会让 e8186b8
   退化，但任何「顺手清理 ovfswtr」会让窗口模式炸掉。
5. **J2 换向强度下降**：J2（连续引擎专用，motion_planner.cpp:1065-1084 整拍 50µs 静默）
   在窗口模式不存在，换向只剩头 guard 5 tick + DIR 在 CEN 前写（stepper_driver.cpp:144-151）
   ⇒ 满足 TMC 20ns 规格但比 J2 弱 50 倍。**往返/往复（motion_planner.cpp:305-338）需实测。**
6. **连续引擎专属功能静默失效**：带内微抖 0x17E-0x181（只写在 `velocityPwmTick`
   :908-926）、无缝换档 0x186、`0x187 bit0/bit2`（只在连续路径被读，:1113、:535）。
   `tt_ab_tmp.py` 的三条对照需重定义。
7. **校准链路会跟着切**：`encoder_calibrator.cpp:296 / :524` 也走 `startMotion()`；
   `:366` 的段末 halt 变空操作，最后 ≤22 个脉冲在采样开始后才落地（60RPM 下 ≤1 步）。
   要零影响就在改动 2 的判据里加 `&& calibration_stage_ == CALIBRATION_IDLE`。跑一遍
   `0x144` 校准回归。

## 5. 验收（只跑三组，同一版固件 `0x187 = 0 vs 2` 直接 A/B）

通用前置：`0x15F=1` → `0x16D=0` → `0x124=1` + 1.2s → 核对 `0x100==32 && 0x182==14062`；
收尾 `0x125=1` 并还原 `0x187=0`。写完 `0x11D` **必须回读校验**。

1. **多走消失**（`scripts/tt_scan_tmp.py`）：`0x107=0`、`0x11A=120/0x11B=250/0x11C=6000`，
   步数 {400,640,3200,6400,32000}×5；读 0x116 积分、`0x134`、`0x121`、`0x183/0x184`、
   `0x177`（bit1=1 时应 6~7）、`0x178` 必须 0。
   判据：bit1=1 时 |平均偏| 与散布**同时**下降，且 `0x134 − 编码器微步` 落在 ±2 内。
2. **不失锁 + 量程**：`0x11A=120/0x11B=1200/0x11C=6000/0x11D=64000`（10 圈整梯形），
   `0x107=0` 三趟 + `0x107=1` 三趟，读 FE 峰值（对照 3181616 的 −20°/−2° 口径）；
   `0x11B` 依次 1200→2000→3000→4100→4400 探容量边界。
   判据：`|账本−实走| ≤ 2 微步`、`0x183` 不涨、不报堵转/磁场故障、FE 与 bit1=0 同量级。
3. **停止语义**：`scripts/tt_half_tmp.py` 的 `0x126` 各 5 次（bit1=1 时 `0x185/0x186`
   必须恒 0，作为分流生效自检）+ 巡航中段 `0x163` 缓停各一次（`scripts/tt_qstop6k_tmp.py`）。
   判据：落点散布从 ±0.3° 降到 **≤0.1°**；无新反向窗、无「抖一下+叮」；`0x121` 停后不恶化。

## 6. 一句话结论

2 处必改 + 2 处强烈建议，窗口引擎代码一行都不用重写，外部跟随/角度模式/缓停急停全不动；
主要风险是 20k/s 窗口重挂的相位毛刺与由此失锁（6000RPM/s 比当年失败工况更陡），其次是
量程掉到 440kpps、跨窗超窗静默少走、连续引擎专属功能与诊断静默失效。**必须做成运行时
开关**（bit1，默认 0），这是唯一能把「落点散布」和「失锁」放在同一版固件、同一台电机上
直接 A/B 的办法。
