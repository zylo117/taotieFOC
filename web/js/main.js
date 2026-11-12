// Entry: remaining helpers + bindControls + initialization (module 11, loaded last).
// Everything here runs after all other modules are defined, so it may safely
// reference any global function/const declared by earlier modules.

        const logArea = document.getElementById('logArea');
        const connBadge = document.getElementById('connBadge');
        const protoBadge = document.getElementById('protoBadge');
        const faultBadge = document.getElementById('faultBadge');


        const OFFICIAL_REGS = {
            GCONF: 0x00,
            GSTAT: 0x01,
            IOUT: 0x06,
            DRV_CONF: 0x0A,
            IHOLD_IRUN: 0x10,
            TPOWERDOWN: 0x11,
            TSTEP: 0x12,
            TPWMTHRS: 0x13,
            TCOOLTHRS: 0x14,
            THIGH: 0x15,
            MSCNT: 0x6A,
            MSCURACT: 0x6B,
            CHOPCONF: 0x6C,
            COOLCONF: 0x6D,
            DCCTRL: 0x6E,
            DRV_STATUS: 0x6F,
            PWMCONF: 0x70,
            PWMSCALE: 0x71,
            ENCM_CTRL: 0x72
        };


        const REGISTER_DESCRIPTIONS = new Map([
            [OFFICIAL_REGS.GCONF, ['0x0000', 'GCONF', 'TMC 全局配置 / 单线 UART 选择']],
            [OFFICIAL_REGS.GSTAT, ['0x0001', 'GSTAT', 'TMC 状态清除寄存器']],
            [OFFICIAL_REGS.DRV_CONF, ['0x000A', 'DRV_CONF', '驱动器模式配置 / 静音模式']],
            [OFFICIAL_REGS.IHOLD_IRUN, ['0x0010', 'IHOLD_IRUN', '保持电流与运行电流']],
            [OFFICIAL_REGS.TSTEP, ['0x0012', 'TSTEP', '步进周期']],
            [OFFICIAL_REGS.CHOPCONF, ['0x006C', 'CHOPCONF', '斩波与微步配置']],
            [OFFICIAL_REGS.DRV_STATUS, ['0x006F', 'DRV_STATUS', '驱动状态']],
            [CUSTOM_REGS.MICROSTEPS, ['0x0100', 'MICROSTEPS', '微步数']],
            [CUSTOM_REGS.INTERPOLATION, ['0x0101', 'INTERPOLATION', '插值微步数']],
            [CUSTOM_REGS.RUN_CURRENT_A, ['0x0102', 'RUN_CURRENT_A', '运行电流，单位 A × 1000']],
            [CUSTOM_REGS.HOLD_CURRENT_A, ['0x0103', 'HOLD_CURRENT_A', '保持电流，单位 A × 1000']],
            [CUSTOM_REGS.CLOSED_LOOP_ENABLE, ['0x0107', 'CLOSED_LOOP_ENABLE', '闭环使能']],
            [CUSTOM_REGS.FAULT_STATUS, ['0x0108', 'FAULT_STATUS', '故障位图：bit0 编码器，bit1 磁场，bit2 停输出，bit3 堵转']],
            [CUSTOM_REGS.ENCODER_FAULT, ['0x0109', 'ENCODER_FAULT', '编码器故障']],
            [CUSTOM_REGS.MAGNETIC_FAULT, ['0x010A', 'MAGNETIC_FAULT', '磁场报警']],
            [CUSTOM_REGS.OUTPUT_STOP, ['0x010B', 'OUTPUT_STOP', '输出已停止']],
            [CUSTOM_REGS.PHASE_A_CURRENT_MA, ['0x010C', 'PHASE_A_CURRENT', 'A 相电流，单位 mA']],
            [CUSTOM_REGS.PHASE_B_CURRENT_MA, ['0x010D', 'PHASE_B_CURRENT', 'B 相电流，单位 mA']],
            [CUSTOM_REGS.POSITION_LOOP_HZ, ['0x010E', 'POSITION_LOOP_HZ', '位置环频率，单位 Hz']],
            [CUSTOM_REGS.VELOCITY_LOOP_HZ, ['0x010F', 'VELOCITY_LOOP_HZ', '速度环频率，单位 Hz']],
            [CUSTOM_REGS.CURRENT_LOOP_HZ, ['0x0110', 'CURRENT_LOOP_HZ', '电流环频率，单位 Hz']],
            [CUSTOM_REGS.LAST_FAULT, ['0x0111', 'LAST_FAULT', '最近一次故障位图']],
            [CUSTOM_REGS.TARGET_POSITION, ['0x0112', 'TARGET_POSITION', '目标位置，单位闭环步数']],
            [CUSTOM_REGS.ACTUAL_POSITION, ['0x0113', 'ACTUAL_POSITION', '当前位置，单位闭环步数']],
            [CUSTOM_REGS.FOLLOW_ERROR, ['0x0114', 'FOLLOW_ERROR', '跟随误差，单位闭环步数']]
            , [CUSTOM_REGS.ENCODER_RAW, ['0x0115', 'ENCODER_RAW', 'KTH7823 原始角度，0 至 65535']]
            , [CUSTOM_REGS.ENCODER_ANGLE_MDEG, ['0x0116', 'ENCODER_ANGLE', '磁编码器角度，单位 0.001°']]
            , [CUSTOM_REGS.MAGNETIC_HIGH, ['0x0117', 'MAGNETIC_HIGH', 'MGH 磁场过高输入']]
            , [CUSTOM_REGS.MAGNETIC_LOW, ['0x0118', 'MAGNETIC_LOW', 'MGL 磁场过低输入']]
            , [CUSTOM_REGS.ENCODER_ZERO, ['0x0119', 'ENCODER_ZERO', '编码器启动零点，原始角度值']]
            , [CUSTOM_REGS.TARGET_ANGLE_DEG, ['0x0128', 'TARGET_ANGLE_DEG', '目标角度，单位 0.001°']]
            , [CUSTOM_REGS.POSITION_KP, ['0x0129', 'POSITION_KP', '位置环 Kp，单位 0.001']]
            , [CUSTOM_REGS.POSITION_LEAD_ALPHA, ['0x015C', 'POSITION_LEAD_ALPHA', '位置环超前系数 α，单位 0.001（1000=不启用）']]
            , [CUSTOM_REGS.POSITION_KI, ['0x012A', 'POSITION_KI', '位置环 Ki，单位 0.001']]
            , [CUSTOM_REGS.POSITION_KD, ['0x012B', 'POSITION_KD', '位置环 Kd，单位 0.001']]
            , [CUSTOM_REGS.VELOCITY_KP, ['0x012C', 'VELOCITY_KP', '速度环 Kp，单位 0.001']]
            , [CUSTOM_REGS.VELOCITY_KI, ['0x012D', 'VELOCITY_KI', '速度环 Ki，单位 0.001']]
            , [CUSTOM_REGS.VELOCITY_KD, ['0x012E', 'VELOCITY_KD', '速度环 Kd，单位 0.001']]
            , [CUSTOM_REGS.ANGLE_MAX_RPM, ['0x012F', 'ANGLE_MAX_RPM', '闭环整定最大速度，单位 RPM']]
            , [CUSTOM_REGS.ANGLE_ACCEL_RPM_S, ['0x0130', 'ANGLE_ACCEL_RPM_S', '闭环整定加速度，单位 RPM/s']]
            , [CUSTOM_REGS.PID_TUNE_CONTROL, ['0x0131', 'PID_TUNE_CONTROL', '固件 PID 整定评分采集控制']]
            , [CUSTOM_REGS.PID_TUNE_SCORE_MDEG, ['0x0132', 'PID_TUNE_SCORE', '固件计算的跟随误差评分，单位 0.001°']]
            , [CUSTOM_REGS.PID_TUNE_SAMPLE_COUNT, ['0x0133', 'PID_TUNE_SAMPLES', '固件参与评分的误差样本数']]
            , [CUSTOM_REGS.MOTION_SCHEDULED_STEPS, ['0x0134', 'MOTION_SCHEDULED_STEPS', '当前动作已提交的 STEP 数']]
            , [CUSTOM_REGS.MOTION_COMMAND_SPEED_MRPM, ['0x0135', 'MOTION_COMMAND_SPEED', '当前命令速度，单位 0.001 RPM']]
            , [CUSTOM_REGS.CPU_USAGE_CENTIPERCENT, ['0x0137', 'CPU_USAGE', 'MCU 总 CPU 占用率，单位 0.01%']]
            , [CUSTOM_REGS.CONTROL_TASK_USAGE_CENTIPERCENT, ['0x0138', 'CONTROL_TASK_USAGE', 'Control_task CPU 占用率，单位 0.01%']]
            , [CUSTOM_REGS.LAG_ANGLE_UPLOAD_COUNT, ['0x013C', 'LAG_ANGLE_UPLOAD_COUNT', '上位机待上传拟合点数']]
            , [CUSTOM_REGS.LAG_ANGLE_POINT_INDEX, ['0x013D', 'LAG_ANGLE_POINT_INDEX', '待写入的拟合点索引']]
            , [CUSTOM_REGS.LAG_ANGLE_POINT_SPEED_RPM, ['0x013E', 'LAG_ANGLE_POINT_SPEED_RPM', '拟合点速度，单位 RPM']]
            , [CUSTOM_REGS.LAG_ANGLE_POINT_MDEG, ['0x013F', 'LAG_ANGLE_POINT_MDEG', '拟合点滞后角，单位 0.001°']]
            , [CUSTOM_REGS.LAG_ANGLE_POINT_COMMIT, ['0x0140', 'LAG_ANGLE_POINT_COMMIT', '提交当前拟合点']]
            , [CUSTOM_REGS.LAG_ANGLE_FIT, ['0x0141', 'LAG_ANGLE_FIT', '固件执行拟合并保存']]
            , [CUSTOM_REGS.LAG_ANGLE_QUERY_SPEED_RPM, ['0x0142', 'LAG_ANGLE_QUERY_SPEED_RPM', '设置待查询速度，单位 RPM']]
            , [CUSTOM_REGS.LAG_ANGLE_QUERY_MDEG, ['0x0143', 'LAG_ANGLE_QUERY_MDEG', '固件拟合查询值，单位 0.001°']]
            , [CUSTOM_REGS.LAG_ANGLE_POINT_COUNT, ['0x0144', 'LAG_ANGLE_POINT_COUNT', '已保存拟合点数']]
            , [CUSTOM_REGS.LAG_ANGLE_FIT_SLOPE, ['0x0145', 'LAG_ANGLE_POWER_COEFFICIENT', '幂律系数 c，单位 °/RPM^a × 1000000']]
            , [CUSTOM_REGS.LAG_ANGLE_FIT_INTERCEPT, ['0x0146', 'LAG_ANGLE_FIT_INTERCEPT', '固件拟合截距，单位 0.001°']]
            , [CUSTOM_REGS.LAG_ANGLE_FIT_EXPONENT, ['0x0147', 'LAG_ANGLE_FIT_EXPONENT', '幂律指数 a，单位 0.000001']]
            , [CUSTOM_REGS.STALL_DETECTED, ['0x015D', 'STALL_DETECTED', '堵转锁存：1=已触发（停机+失能），写 0x15F 清除']]
            , [CUSTOM_REGS.STALL_CLEAR, ['0x015F', 'STALL_CLEAR', '写 1 清除堵转锁存']]
            , [CUSTOM_REGS.RESONANCE_LO_RPM, ['0x016F', 'RESONANCE_LO_RPM', '共振带下限 RPM，存 Flash']]
            , [CUSTOM_REGS.RESONANCE_HI_RPM, ['0x0170', 'RESONANCE_HI_RPM', '共振带上限 RPM，存 Flash（0=关闭）']]
            , [CUSTOM_REGS.RESONANCE_BYPASS, ['0x0171', 'RESONANCE_BYPASS', '历史遗留 no-op：共振带 2026-11-12 起不再钳位运动，写值只记录']]
            , [CUSTOM_REGS.STALL_QUIET_MS, ['0x0174', 'STALL_QUIET_MS', '堵转判定静止时长 ms，默认 200，存 Flash']]
            , [CUSTOM_REGS.STALL_MOVE_LSB, ['0x0175', 'STALL_MOVE_LSB', '堵转判定净位移阈值 LSB，默认 100，存 Flash']]
            , [CUSTOM_REGS.ENCODER_INVERT, ['0x0176', 'ENCODER_INVERT', '磁钢极性：1=校准鉴别出装反，原始角度已反转，存 Flash（改动后须重新校准）']]
             , [CUSTOM_REGS.STEP_MAX_WINDOW_STEPS, ['0x0177', 'STEP_MAX_WINDOW_STEPS', '脉冲质量：上次运动起单窗索要最大步数（450RPM/64细分正常≈5~6，几十=任务停顿过，固件已截断防爆发）']]
             , [CUSTOM_REGS.STEP_TASK_STALL_EVENTS, ['0x0178', 'STEP_TASK_STALL_EVENTS', '脉冲质量：控制任务停顿(窗间隔>150µs)事件累计，持续增长=有CPU抢占源']]
            , [0x11A, ['0x011A', 'START_RPM', '测速启动转速，单位 RPM']]
            , [0x11B, ['0x011B', 'MAX_RPM', '测速最高转速，单位 RPM']]
            , [0x11C, ['0x011C', 'ACCEL_RPM_S', '测速加速度，单位 RPM/s']]
            , [0x11D, ['0x011D', 'PULSE_COUNT', '测速脉冲总数']]
            , [0x11E, ['0x011E', 'MOTION_MODE', '运动模式：0正转 1反转 2速度正转 3速度反转 4正转归零 5反转归零 6正向起步往返 7反向起步往返']]
            , [0x11F, ['0x011F', 'MOTION_CMD', '运动命令：0停止 1启动']]
            , [0x120, ['0x0120', 'SPEED_RPM', '当前测速转速，单位 RPM']]
            , [0x121, ['0x0121', 'FOLLOW_ERROR_DEG', '位置跟随误差：理论累计行程 - 编码器累计行程，单位 0.001°']]
            , [0x122, ['0x0122', 'WAVE_WINDOW_MS', '波形窗口长度，单位 ms']]
            , [0x123, ['0x0123', 'STEP_PULSE_WIDTH_NS', 'STEP 高电平脉宽，单位 ns']]
            , [0x124, ['0x0124', 'MOTOR_ENABLE', '电机使能，EN 输出低电平']]
            , [0x125, ['0x0125', 'MOTOR_DISABLE', '电机失能，EN 输出高电平']]
            , [0x126, ['0x0126', 'SINGLE_FORWARD_STEP', 'DIR 高电平并输出单个 STEP']]
            , [0x136, ['0x0136', 'CALIBRATE_ENCODER', '磁编码器校准：写 1 触发一次校准']]
            , [CUSTOM_REGS.ENCODER_FILTER_WINDOW, ['0x0148', 'ENCODER_FILTER_WINDOW', '磁编码器滑动平均窗口，单位样本（0=关闭滤波），1 样本 = 2µs，上限 256']]
            , [CUSTOM_REGS.EXT_FOLLOW_ENABLE, ['0x0149', 'EXT_FOLLOW_ENABLE', '外部 PUL/DIR 脉冲跟随使能：0 关 / 1 开（1 脉冲 = 1 微步）']]
            , [CUSTOM_REGS.EXT_FOLLOW_MAX_STEP_S, ['0x014A', 'EXT_FOLLOW_MAX_STEP_S', '跟随限速，单位步/s；0 = 按角度模式 RPM 上限自动换算']]
            , [CUSTOM_REGS.EXT_FOLLOW_CMD_STEPS, ['0x014B', 'EXT_FOLLOW_CMD_STEPS', '外部累计命令步数（有符号）']]
            , [CUSTOM_REGS.EXT_FOLLOW_EMITTED_STEPS, ['0x014C', 'EXT_FOLLOW_EMITTED_STEPS', '已 1:1 转发步数（有符号）']]
            , [CUSTOM_REGS.EXT_FOLLOW_BACKLOG, ['0x014D', 'EXT_FOLLOW_BACKLOG', '当前积压步数（命令 - 已转发）']]
            , [CUSTOM_REGS.EXT_FOLLOW_DROPPED, ['0x014E', 'EXT_FOLLOW_DROPPED', '累计丢步：超过限速而长期积压的部分']]
            , [CUSTOM_REGS.EXT_FOLLOW_RESET, ['0x014F', 'EXT_FOLLOW_RESET', '写 1 清零命令/已转发/积压/丢步计数']]
            , [CUSTOM_REGS.EXT_FOLLOW_WINDOW_REVERSALS, ['0x0150', 'EXT_FOLLOW_WINDOW_REVERSALS', '窗内发生过换向的窗口数：只统计不改输出，持续增长说明需要段队列']]
        ]);



        async function writeRegisterAndWaitAck(reg, value, timeoutMs = 1200) {
            const previousWaiter = state.registerWriteAckWaiters.get(reg);
            if (previousWaiter) {
                throw new Error(`WRITE 0x${reg.toString(16).padStart(4, '0')} 仍有未完成的 ACK 等待`);
            }
            const expectedValue = value >>> 0;
            let waiter;
            const ackPromise = new Promise((resolve, reject) => {
                waiter = {
                    expectedValue,
                    resolve,
                    reject,
                    timeout: window.setTimeout(() => {
                        if (state.registerWriteAckWaiters.get(reg) === waiter) {
                            state.registerWriteAckWaiters.delete(reg);
                        }
                        const error = new Error(`阶段 1/3 失败：WRITE 已提交给串口，但 ${timeoutMs}ms 内未收到 0x82 ACK`);
                        error.lookupTransactionSent = true;
                        reject(error);
                    }, timeoutMs)
                };
                state.registerWriteAckWaiters.set(reg, waiter);
            });
            try {
                await writeRegister(reg, expectedValue);
                await ackPromise;
                window.clearTimeout(waiter.timeout);
            } catch (error) {
                window.clearTimeout(waiter.timeout);
                if (state.registerWriteAckWaiters.get(reg) === waiter) {
                    state.registerWriteAckWaiters.delete(reg);
                }
                if (!error.lookupTransactionSent) error.lookupTransactionSent = true;
                throw error;
            }
        }






        // 连接串口时只把运动参数（速度/加速度）刷一遍，【绝对不碰闭环使能】。
        // 之前这里有一行 writeRegister(CLOSED_LOOP_ENABLE, 1) + checked = true，
        // 导致每次一连上硬件就自动勾上闭环补偿。在外部脉冲跟随模式下这会让电机
        // 在 1:1 转发之外再叠一层 PID 修正，即使 backlog=0 也会超速（实测电流 1.4A）。
        // 闭环只应该在“真正用到角度模式”时由 ensureMotionDefaultsForAngleControl 打开。


        function buildRpmOptions(selectId, minValue, maxValue, stepValue, offset) {
            const select = document.getElementById(selectId);
            const options = [];
            for (let value = minValue; value <= maxValue; value += stepValue) {
                if (value <= 1000) {
                    if (value < 25) continue;
                    options.push(value);
                } else if (value === 1000 || value > 1000) {
                    options.push(value);
                }
            }
            if (offset !== undefined) {
                const fallback = options.includes(offset) ? offset : options[0];
                select.innerHTML = options.map((value) => `<option value="${value}">${value}</option>`).join('');
                select.value = String(fallback);
            } else {
                select.innerHTML = options.map((value) => `<option value="${value}">${value}</option>`).join('');
            }
        }

        function buildMotionControlOptions() {
            const startSelect = document.getElementById('startRpmSelect');
            const maxSelect = document.getElementById('maxRpmSelect');
            const accelSelect = document.getElementById('accelRpmSelect');

            const startValues = [];
            for (let v = 0; v <= 600; v += 60) {
                startValues.push(v);
            }
            startSelect.innerHTML = startValues.map((v) => `<option value="${v}">${v}</option>`).join('');
            startSelect.value = '120';

            const maxValues = [];
            for (let v = 25; v <= 1000; v += 75) {
                maxValues.push(v);
            }
            for (let v = 1000; v <= 10000; v += 1000) {
                maxValues.push(v);
            }
            const maxUnique = [...new Set(maxValues.filter((v) => v >= 25 && v <= 10000))];
            maxSelect.innerHTML = maxUnique.map((v) => `<option value="${v}">${v}</option>`).join('');
            maxSelect.value = '1000';

            // 加速度档位：0~300 段加密（低速定位/大惯量/防失步常用低加速度，原来最低就
            // 300 太粗），300 以上保留 300 步长到 2400。固件 0x11C 接受 1~12000。
            const accelValues = [25, 50, 75, 100, 150, 200, 250];
            for (let v = 300; v <= 2400; v += 300) {
                accelValues.push(v);
            }
            accelSelect.innerHTML = accelValues.map((v) => `<option value="${v}">${v}</option>`).join('');
            accelSelect.value = '600';
        }

        function getStepsPerRevolution() {
            // 一圈 STEP 数 = 200 步/圈 x 驱动器实际微步数，必须跟 “微步” 输入框保持一致，
            // 不能再用写死的 51200（那是 200x256 的老口径，与实际下发脉冲不符）。
            const microsteps = Number(document.getElementById('microsteps').value || 64);
            return 200 * Math.max(1, microsteps);
        }

        function getEffectivePulseCount() {
            return Number(document.getElementById('pulseCountInput').value || getStepsPerRevolution());
        }









        // 编码器滤波窗口的输入框读写：0 = 关闭滤波，上限 256（固件 EncoderFilterConfig::kMaxWindow）
        function readEncoderFilterWindowInput() {
            const input = document.getElementById('encoderFilterWindowInput');
            const value = clamp(Math.round(Number(input.value) || 0), 0, 256);
            input.value = String(value);
            return value;
        }

        async function startMotionTest() {
            if (state.autoTuneActive) {
                appendTuneLog('整定期间不能启动开环运动');
                return;
            }
            if (state.resonanceScanActive) {
                log('[WARN] 共振自动检测进行中，已屏蔽启动运动（先中止扫描）');
                return;
            }
            if (!state.connected) {
                log('[WARN] 尚未连接串口，无法启动测速运动');
                return;
            }
            const startRpm = Number(document.getElementById('startRpmSelect').value || 120);
            const maxRpm = Number(document.getElementById('maxRpmSelect').value || 1000);
            const accelRpm = Number(document.getElementById('accelRpmSelect').value || 600);
            const motionMode = Number(document.getElementById('motionModeSelect').value || 0);
            const pulseCount = Math.max(2, Math.round(getEffectivePulseCount()));
            const windowMs = clamp(Number(document.getElementById('waveWindowMsInput').value || 10000), 10000, 10000);
            const stepPulseWidthNs = clamp(Number(document.getElementById('stepPulseWidthNsInput').value || 200), 200, 20000);
            const encoderFilterWindow = readEncoderFilterWindowInput();
            document.getElementById('waveWindowMsInput').value = String(windowMs);
            document.getElementById('stepPulseWidthNsInput').value = String(stepPulseWidthNs);
            document.getElementById('encoderFilterWindowInput').value = String(encoderFilterWindow);

            // 【安全锁】用户点过「失能」后固件会拒绝所有 0x11F=1（日志 [SAFETY]）。
            // 点「启动」本身就是明确要让电机转的意图，所以先显式使能一次。
            await writeRegisterAndWaitAck(0x124, 1);   // MOTOR_ENABLE

            // 【按界面勾选框下发补偿开关】不能不管：
            // 角度模式（setTargetAngleDeg）会把 closed_loop_compensation_enabled_ 置 true，
            // 规划运动如果不下发 0x107，就会继承这个 true -> 开环规划运动带着 PID 修正跑 -> 抖动。
            // 也不能像以前那样强制发 0：那会让勾选了闭环的用户每次启动都被偷偷关掉。
            // 正确做法：以界面勾选框为准，每次启动都显式下发。
            await writeRegisterAndWaitAck(CUSTOM_REGS.CLOSED_LOOP_ENABLE,
                document.getElementById('closedLoopEnableInput').checked ? 1 : 0);

            // 【必须等 ACK】这里原来全是 writeRegister（只等写进 USB 缓冲，不等固件确认），
            // 紧接着就发 0x11F 启动。看校准表之后 USB 缓冲被 dump 流量占着，
            // 写请求会丢——脉冲数没设上时，固件用上一次的值跑，
            // 表现为“脉冲少了 / 低速也丢步”，而且重启硬件无用（固件侧根本没收到）。
            // 换成 writeRegisterAndWaitAck：每条都等固件的 0x82 应答，
            // 不仅保证送达，还保证 0x11F 启动一定在参数全部生效之后才发出。
            await writeRegisterAndWaitAck(0x11A, startRpm);
            await writeRegisterAndWaitAck(0x11B, maxRpm);
            await writeRegisterAndWaitAck(0x11C, accelRpm);
            await writeRegisterAndWaitAck(0x11D, pulseCount);
            await writeRegisterAndWaitAck(0x11E, motionMode);
            await writeRegisterAndWaitAck(0x123, stepPulseWidthNs);
            await writeRegisterAndWaitAck(CUSTOM_REGS.ENCODER_FILTER_WINDOW, encoderFilterWindow);
            await writeRegisterAndWaitAck(0x122, windowMs);
            await writePidParameters();
            await writeRegisterAndWaitAck(0x11F, 1);
            startWaveCapture();
            log(`[INFO] 脉冲运动：start=${startRpm} RPM max=${maxRpm} RPM accel=${accelRpm} RPM/s pulse=${pulseCount} mode=${motionMode} window=${windowMs}ms step=${stepPulseWidthNs}ns encFilter=${encoderFilterWindow}`);
        }

        async function sendManualMotionCommand(reg, message) {
            if (state.autoTuneActive) {
                if (reg === 0x125) {
                    state.autoTuneCancelled = true;
                } else {
                    appendTuneLog('整定期间已屏蔽其他手动运动命令');
                    return;
                }
            }
            // 共振扫描进行中：除"失能/急停"外的手动运动会搅乱测点，屏蔽掉。
            if (state.resonanceScanActive && reg !== 0x125 && reg !== 0x16E) {
                log('[WARN] 共振自动检测进行中，已屏蔽该手动运动（如需干预请先中止扫描）');
                return;
            }
            if (!state.connected) {
                log('[WARN] 尚未连接串口，无法发送电机测试命令');
                return;
            }
            if (reg === 0x124 || reg === 0x125) {
                stopWaveCapture();
            }
            await writeRegister(reg, 1);
            if (reg === 0x126) startWaveCapture();
            log('[INFO] ' + message);
        }

        function toggleWavePause() {
            if (!state.wavePaused) {
                state.wavePaused = true;
                state.wavePausedAt = performance.now();
            } else {
                const resumeAt = performance.now();
                const elapsed = resumeAt - state.wavePausedAt;
                state.waveHistory = state.waveHistory.map((entry) => ({...entry, t: entry.t + elapsed}));
                state.wavePaused = false;
                state.wavePausedAt = 0;
            }
            const btn = document.getElementById('pauseWaveBtn');
            btn.textContent = state.wavePaused ? '继续查看' : '暂停查看';
            log(`[INFO] 波形${state.wavePaused ? '已暂停' : '已恢复'}`);
        }

        function bindControls() {
            document.getElementById('connectBtn').addEventListener('click', connectSerial);
            const disconnectBtn = document.getElementById('disconnectBtn');
            if (disconnectBtn) disconnectBtn.addEventListener('click', () => disconnectSerial('手动断开'));
                document.getElementById('tabMotionBtn').addEventListener('click', () => switchWorkTab('motion'));
                document.getElementById('tabExtFollowBtn').addEventListener('click', () => switchWorkTab('extfollow'));
            document.getElementById('readAllBtn').addEventListener('click', readAllConfig);
            document.getElementById('lagAngleCalibrationBtn').addEventListener('click', openLagAngleCalibration);
            // 滞后角模型参数：改完立即下发（存 Flash 靠「写入参数到 Flash」按钮）
            for (const [id, reg] of [['lagAccelCoef', CUSTOM_REGS.LAG_ACCEL_COEF],
                                     ['lagDeadbandMdeg', CUSTOM_REGS.LAG_DEADBAND_MDEG]]) {
                document.getElementById(id).addEventListener('change', async (event) => {
                    if (!state.connected) { log('[WARN] 未连接串口'); return; }
                    const value = Math.max(0, Math.round(Number(event.target.value) || 0));
                    try {
                        await writeRegisterAndWaitAck(reg, value);
                        log(`[INFO] 0x${reg.toString(16)} = ${value}`);
                    } catch (error) {
                        log(`[ERROR] 写 0x${reg.toString(16)} 失败：${error.message}`);
                    }
                });
            }
            document.getElementById('enableModeSelect').addEventListener('change', async (event) => {                const mode = Number(event.target.value) | 0;
                if (!state.connected) {
                    log('[WARN] 未连接串口，使能来源未下发');
                    return;
                }
                try {
                    await writeRegisterAndWaitAck(CUSTOM_REGS.ENABLE_MODE, mode);
                    log(`[INFO] 使能来源 = ${mode === 1 ? '外部 EN 引脚（PC13 低有效）' : '上位机控制'}`);
                    await refreshEnableState();
                } catch (error) {
                    log('[ERROR] 设置使能来源失败：' + error.message);
                    // 写失败就把下拉框拨回固件里的真实值
                    try {
                        const real = (await readRegisterValue(CUSTOM_REGS.ENABLE_MODE)) | 0;
                        event.target.value = String(real & 1);
                    } catch (e) { /* ignore */ }
                }
            });
            document.getElementById('clearStallBtn').addEventListener('click', async () => {
                if (!state.connected) return;
                try {
                    await writeRegisterAndWaitAck(CUSTOM_REGS.STALL_CLEAR, 1);
                    appendTuneLog('已清除堵转锁存（0x15F=1）');
                    await refreshStallStatus();
                } catch (error) {
                    appendTuneLog(`清除堵转失败：${error.message}`);
                }
            });
            document.getElementById('closeLagAngleDialogBtn').addEventListener('click', () => {
                document.getElementById('lagAngleDialog').close();
            });
            document.getElementById('lagAngleDialog').addEventListener('cancel', (event) => {
                if (state.lagAngleCalibrationRunning) event.preventDefault();
            });
            document.getElementById('startLagAngleCalibrationBtn').addEventListener('click', startLagAngleCalibration);
            document.getElementById('cancelLagAngleCalibrationBtn').addEventListener('click', async () => {
                if (!state.lagAngleCalibrationRunning) return;
                state.lagAngleCalibrationCancelled = true;
                document.getElementById('lagAngleStatus').textContent = '正在停止当前运动...';
                await stopLagAngleMotion();
            });
            document.getElementById('refreshLagAngleCalibrationBtn').addEventListener('click', async () => {
                if (state.lagAngleCalibrationRunning) return;
                try {
                    await refreshLagAngleCalibration();
                    document.getElementById('lagAngleStatus').textContent = '数据来自固件校准表与拟合结果';
                } catch (error) {
                    document.getElementById('lagAngleStatus').textContent = `读取失败：${error.message}`;
                }
            });
            document.getElementById('viewCalibrationTableBtn').addEventListener('click', openCalibrationTableViewer);
            document.getElementById('closeCalibrationTableBtn').addEventListener('click', closeCalibrationTableViewer);
            document.getElementById('calibrationTableDialog').addEventListener('cancel', () => {
                void closeCalibrationTableViewer();
            });
            document.getElementById('calibrationTableSelect').addEventListener('change', async () => {
                const kind = document.getElementById('calibrationTableSelect').value;
                if (kind === 'fast') {
                    const pageInput = document.getElementById('calibrationTablePageInput');
                    const page = clamp(Math.max(1, Number(pageInput.value || 1)), 1, 256);
                    pageInput.value = String(page);
                    await requestCalibrationTableDump('fast', page - 1);
                    return;
                }
                renderCalibrationTablePage();
            });
            document.getElementById('calibrationTablePageInput').addEventListener('change', async () => {
                const kind = document.getElementById('calibrationTableSelect').value;
                if (kind === 'fast') {
                    const page = clamp(Math.max(1, Number(document.getElementById('calibrationTablePageInput').value || 1)), 1, 256);
                    document.getElementById('calibrationTablePageInput').value = String(page);
                    await requestCalibrationTableDump('fast', page - 1);
                    return;
                }
                renderCalibrationTablePage();
            });
            document.getElementById('calibrationTablePrevBtn').addEventListener('click', async () => {
                const input = document.getElementById('calibrationTablePageInput');
                input.value = String(Math.max(1, Number(input.value || 1) - 1));
                const kind = document.getElementById('calibrationTableSelect').value;
                if (kind === 'fast') {
                    await requestCalibrationTableDump('fast', Number(input.value) - 1);
                    return;
                }
                renderCalibrationTablePage();
            });
            document.getElementById('calibrationTableNextBtn').addEventListener('click', async () => {
                const input = document.getElementById('calibrationTablePageInput');
                input.value = String(Number(input.value || 1) + 1);
                const kind = document.getElementById('calibrationTableSelect').value;
                if (kind === 'fast') {
                    await requestCalibrationTableDump('fast', Number(input.value) - 1);
                    return;
                }
                renderCalibrationTablePage();
            });
            document.getElementById('calibrationRawLookupBtn').addEventListener('click', () => void runCalibrationLookup('forward'));
            document.getElementById('calibrationCorrectedLookupBtn').addEventListener('click', () => void runCalibrationLookup('reverse'));
            document.getElementById('calibrationRawInput').addEventListener('change', () => {
                if (state.calibrationLookupInFlight) return;
                void runCalibrationLookup('forward');
            });
            document.getElementById('calibrationCorrectedInput').addEventListener('change', () => {
                if (state.calibrationLookupInFlight) return;
                void runCalibrationLookup('reverse');
            });
            document.getElementById('calibrationRawInput').addEventListener('input', () => {
                if (state.calibrationLookupInFlight) return;
                updateCalibrationLookupDisplays();
            });
            document.getElementById('calibrationCorrectedInput').addEventListener('input', () => {
                if (state.calibrationLookupInFlight) return;
                updateCalibrationLookupDisplays();
            });
            document.getElementById('exportCalibrationTableBtn').addEventListener('click', exportCalibrationTable);
            document.getElementById('writeAllBtn').addEventListener('click', applyAllConfig);
            document.getElementById('sendTargetBtn').addEventListener('click', sendTargetPosition);
            document.getElementById('zeroBtn').addEventListener('click', () => {
                ensureMotionTabForPositionControl();
                zeroEncoder();
            });
            // stopBtn（归零隔壁的缓停）已删除：全局唯一「缓停」入口挪到规划卡
            // （stopMotionBtn → stopMotor/0x163）。这里不能再绑已不存在的元素，
            // 否则 addEventListener(null) 抛异常，bindControls 后半段全部失效。
            const emergencyStopBtn = document.getElementById('emergencyStopBtn');
            if (emergencyStopBtn) emergencyStopBtn.addEventListener('click', emergencyStop);
            document.getElementById('startMotionBtn').addEventListener('click', startMotionTest);
            document.getElementById('stopMotionBtn').addEventListener('click', stopMotor);
            document.getElementById('motorEnableBtn').addEventListener('click', async () => {
                await sendManualMotionCommand(0x124, '电机使能：EN=0');
                await refreshEnableState();
            });
            document.getElementById('motorDisableBtn').addEventListener('click', async () => {
                await sendManualMotionCommand(0x125, '电机失能：EN=1');
                await refreshEnableState();
            });
            document.getElementById('singleStepBtn').addEventListener('click', async () => {
                // 从页面下拉框读取参数
                const startRpm = Number(document.getElementById('startRpmSelect').value);
                const maxRpm = Number(document.getElementById('maxRpmSelect').value);
                const accelRpm = Number(document.getElementById('accelRpmSelect').value);
                const motionMode = Number(document.getElementById('motionModeSelect').value);

                // 下发运动相关参数寄存器
                await writeRegister(0x11A, startRpm);
                await writeRegister(0x11B, maxRpm);
                await writeRegister(0x11C, accelRpm);
                await writeRegister(0x11E, motionMode);
                const stepPulseWidthNs = clamp(Number(document.getElementById('stepPulseWidthNsInput').value || 1000), 1000, 20000);
                await writeRegister(0x123, stepPulseWidthNs);

                // 再触发单步正向脉冲命令寄存器
                await sendManualMotionCommand(0x126, '半圈正向脉冲：DIR=1（已同步速度/加速度参数）');
            });
            document.getElementById('pauseWaveBtn').addEventListener('click', toggleWavePause);
            document.getElementById('popWaveBtn').addEventListener('click', () => toggleWavePopup());
            document.getElementById('maximizeWavePopupBtn').addEventListener('click', toggleWavePopupMaximize);
            document.getElementById('closeWavePopupBtn').addEventListener('click', () => toggleWavePopup(false));
            ['startRpmSelect', 'maxRpmSelect', 'accelRpmSelect', 'pulseCountInput', 'microsteps'].forEach((id) => {
                document.getElementById(id).addEventListener('change', updateMotionDurationEstimate);
            });
            document.getElementById('calibrateEncoderBtn').addEventListener('click', async () => {
                if (!state.connected) {
                    log('[WARN] 尚未连接串口，无法执行磁编码器校准');
                    return;
                }
                // 校准要"回零 + 逐段扫描"（内部运动）：外部跟随模式下先自动接管通道，
                // 否则就是实机反馈的"校准只能归零无法运动"（固件侧 0x136 也有同款兜底）。
                if (!(await ensureInternalMotionSource('磁编码器校准'))) return;
                await writeRegister(CUSTOM_REGS.CALIBRATE_ENCODER, 1);
                log('[INFO] 已发送磁编码器校准触发命令（先自动鉴别磁钢极性：开环向前 1/4 圈，'
                    + '装反会反转读数再归零；看串口 [CALIBRATION] polarity 行）');
            });
            document.getElementById('tuneBtn').addEventListener('click', startAutoTune);
            document.getElementById('startTuneBtn').addEventListener('click', startAutoTune);
            ['tuneMaxSpeed', 'tuneAccel', 'tuneTestTurns'].forEach((id) => {
                document.getElementById(id).addEventListener('change', () => {
                    if (!state.autoTuneActive && !state.tuneActionBusy) renderTuneActionRows(true);
                });
            });
            document.getElementById('waveWindowMsInput').addEventListener('change', function () {
                const value = clamp(Number(this.value || 10000), 10000, 10000);
                this.value = String(value);
                if (state.connected) {
                    writeRegister(0x122, value);
                }
            });

            document.getElementById('encoderFilterWindowInput').addEventListener('change', async function () {
                const value = readEncoderFilterWindowInput();
                if (!state.connected) return;
                await writeRegister(CUSTOM_REGS.ENCODER_FILTER_WINDOW, value);
                log(`[INFO] 编码器滤波窗口=${value} 个样本（约 ${(value * 2.05).toFixed(2)}µs 内的喷发采样平均，约 1 秒后生效）`);
            });

            document.getElementById('targetPosInput').addEventListener('change', function () {
                state.targetAngle = normalize360Angle(Number(this.value || 0));
                updateDialVisual();
                queueTargetAngleWrite();
            });

            document.getElementById('targetPosInput').addEventListener('input', function () {
                state.targetAngle = normalize360Angle(Number(this.value || 0));
                updateDialVisual();
                queueTargetAngleWrite();
            });

            document.getElementById('angleMaxRpmInput').addEventListener('change', async function () {
                const value = clamp(Math.round(Number(this.value) || 1), 1, 12000);
                this.value = String(value);
                state.angleMaxRpm = value;
                if (state.connected) await writeRegister(CUSTOM_REGS.ANGLE_MAX_RPM, value);
            });

            document.getElementById('angleAccelRpmSInput').addEventListener('change', async function () {
                const value = clamp(Math.round(Number(this.value) || 1), 1, 12000);
                this.value = String(value);
                state.angleAccelRpmS = value;
                if (state.connected) await writeRegister(CUSTOM_REGS.ANGLE_ACCEL_RPM_S, value);
            });

            document.getElementById('closedLoopEnableInput').addEventListener('change', async function () {
                if (state.connected) {
                    await writeRegister(CUSTOM_REGS.CLOSED_LOOP_ENABLE, this.checked ? 1 : 0);
                }
            });

            // 波形图“误差不计滞后角”勾选框：勾选/取消时立即重绘，不用等下一次采样刷新
            const excludeLagAngleInput = document.getElementById('waveExcludeLagAngleInput');
            if (excludeLagAngleInput) {
                excludeLagAngleInput.addEventListener('change', () => {
                    if (state.waveCaptureActive || state.waveHistory.length > 0) {
                        drawWaveform();
                        if (document.getElementById('wavePopup').classList.contains('visible')) {
                            drawWaveform('motionWavePopupCanvas');
                        }
                    }
                });
            }

            // 滞后角前馈信任度：改动时下发 0x152（百分数）。
            // 滞后角表按某个标定负载拟合，实际负载变化时真实滞后角会偏离表值；
            // 降到 50~70 让前馈更保守（偏差交给 PID），0 = 完全关闭前馈。
            const ffRatioInput = document.getElementById('lagAngleFfRatioInput');
            if (ffRatioInput) {
                ffRatioInput.addEventListener('change', async function () {
                    const percent = clamp(Math.round(Number(this.value) || 0), 0, 100);
                    this.value = String(percent);
                    if (!state.connected) return;
                    try {
                        await writeRegister(CUSTOM_REGS.LAG_ANGLE_FF_RATIO, percent);
                        log(`[INFO] 滞后角前馈信任度 = ${percent}%`);
                    } catch (error) {
                        log(`[ERROR] 设置前馈信任度失败：${error.message}`);
                    }
                });
            }

            // 编码器滤波窗口快捷对比按钮
            const filterShortcuts = [
                ['filterWin0Btn', 0],
                ['filterWin2Btn', 2],
                ['filterWin4Btn', 4],
                ['filterWin16Btn', 16],
                ['filterWin32Btn', 32]
            ];
            filterShortcuts.forEach(([id, value]) => {
                const btn = document.getElementById(id);
                if (btn) btn.addEventListener('click', () => setEncoderFilterWindowShortcut(value, id));
            });

            document.getElementById('extFollowEnableInput').addEventListener('change', async function () {
                if (!state.connected) {
                    log('[WARN] 尚未连接串口，无法切换外部脉冲跟随');
                    return;
                }
                await writeRegister(CUSTOM_REGS.EXT_FOLLOW_ENABLE, this.checked ? 1 : 0);
                // 0x149 是 0x16D 的别名：勾选框动了，UI 门控状态跟着走
                state.motionSource = this.checked ? 1 : 0;
                applyMotionSourceGate();
                log(this.checked
                    ? '[INFO] 外部脉冲跟随已开启：PB5/PB4 的脉冲将 1:1 驱动电机（内部运动已被固件互斥拒绝）'
                    : '[INFO] 外部脉冲跟随已关闭，通道交还内部规划');
            });

            // ===== 共振速度区间（0x16F/0x170，存 Flash）=====
            for (const [inputId, register, label] of [
                ['resonanceLoRpmInput', CUSTOM_REGS.RESONANCE_LO_RPM, '共振带下限'],
                ['resonanceHiRpmInput', CUSTOM_REGS.RESONANCE_HI_RPM, '共振带上限']]) {
                const el = document.getElementById(inputId);
                if (!el) continue;
                el.addEventListener('change', async function () {
                    const value = clamp(Math.round(Number(this.value) || 0), 0, 6000);
                    this.value = String(value);
                    if (!state.connected) return;
                    try {
                        await writeRegisterAndWaitAck(register, value);
                        log(`[INFO] ${label}=${value} RPM（固件回读一致）`);
                    } catch (error) {
                        log('[ERROR] ' + label + '写入失败：' + error.message);
                    }
                });
            }

            // ===== 堵转阈值（0x174/0x175，存 Flash v6）=====
            // 钳位口径与固件路由一致：写超界值固件会钳位，这里先钳一遍再写，
            // 输入框留下的就是固件真实生效的值（写完不重读，回读一致靠钳位对齐）。
            for (const [inputId, register, label, lo, hi] of [
                ['stallQuietMsInput', CUSTOM_REGS.STALL_QUIET_MS, '堵转时长', 50, 3000],
                ['stallMoveLsbInput', CUSTOM_REGS.STALL_MOVE_LSB, '堵转位移', 40, 1500]]) {
                const el = document.getElementById(inputId);
                if (!el) continue;
                el.addEventListener('change', async function () {
                    const value = clamp(Math.round(Number(this.value) || 0), lo, hi);
                    this.value = String(value);
                    if (!state.connected) return;
                    try {
                        await writeRegisterAndWaitAck(register, value);
                        log(`[INFO] ${label}=${value}${register === CUSTOM_REGS.STALL_QUIET_MS ? 'ms' : 'LSB'}（固件回读一致）`
                            + '；「写入参数到 Flash」后掉电保持');
                    } catch (error) {
                        log('[ERROR] ' + label + '写入失败：' + error.message);
                    }
                });
            }

            // 解除堵转（右下角全局按钮，堵转锁存时由 ui.js updateFaultStatus 显示）
            const unlockStallBtn = document.getElementById('unlockStallBtn');
            if (unlockStallBtn) unlockStallBtn.addEventListener('click', async () => {
                if (!state.connected) return;
                try {
                    await writeRegisterAndWaitAck(CUSTOM_REGS.STALL_CLEAR, 1);
                    log('[INFO] 已清除堵转锁存（0x15F=1）。建议先确认卡滞原因再启动。');
                    await refreshStallStatus();
                } catch (error) {
                    log('[ERROR] 清除堵转失败：' + error.message);
                }
            });

            document.getElementById('extFollowMaxStepSInput').addEventListener('change', async function () {
                const value = clamp(Math.round(Number(this.value) || 0), 0, 2560000);
                this.value = String(value);
                if (!state.connected) return;
                await writeRegister(CUSTOM_REGS.EXT_FOLLOW_MAX_STEP_S, value);
                log(`[INFO] 跟随限速=${value === 0 ? '自动（按角度模式 RPM 上限换算）' : value + ' 步/s'}`);
            });

            document.getElementById('extFollowResetBtn').addEventListener('click', async () => {
                if (!state.connected) return;
                await writeRegister(CUSTOM_REGS.EXT_FOLLOW_RESET, 1);
                log('[INFO] 外部跟随计数已清零（命令/已转发/积压/丢步）');
            });

            // 【VLOOP 诊断】写 0x155=1 让固件打印一次快照，打完自动清零。
            // 为什么不是周期打印：三行浮点 ≈ 23ms 的 printf 会干扰被诊断的闭环本身；
            // 也不能“一直在后台存着不说”——之前几次误判都是因为没有现场数据。
            document.getElementById('vloopSnapshotBtn').addEventListener('click', async () => {
                if (!state.connected) return;
                await writeRegister(CUSTOM_REGS.VLOOP_DEBUG_PRINT, 1);
                log('[INFO] 已请求 [VLOOP] 快照：约 1 秒内出现在串口。'
                    + 'err=跟踪误差, P/I/D=速度环三分量, ff=轮廓前馈速度, kscale=live/base 增益比（应为 1.000）');
            });

            ['posKp', 'posKi', 'posKd', 'velKp', 'velKi', 'velKd', 'posLeadAlpha'].forEach((id) => {
                document.getElementById(id).addEventListener('input', (event) => {
                    void writePidParameter(event.currentTarget);
                });
            });
        }

        function handleCustomTelemetry(frame) {
            if (frame.reg === 0x11F) {
                const running = (frame.value | 0) !== 0;
                if (running !== state.motionRunning) {
                    state.motionRunning = running;
                    // 运行/停止切换时重新开窗：既不能把运动过程算进空闲抖动，
                    // 也不能让运动前的旧样本污染刚停下时的统计。
                    resetEncoderJitterWindow();
                }
            }
            if (frame.reg === 0x120) {
                state.motionSpeedRpm = frame.value | 0;
                // 【整定器】用转速判断“运动已开始”。0x11F 是双向寄存器（网页自己
                // 有 7 处 writeRegister(0x11F, 0)），靠它判断启动会超时报“运动失败”；
                // 0x120 是纯只读遥测，> 20RPM 就证明电机真在转。
                if (state.tuneTrialActive && Math.abs(state.motionSpeedRpm) > 20) {
                    state.tuneMotionObserved = true;
                }
            }
            if (frame.reg === 0x121) {
                const followErrorDeg = (frame.value | 0) / 1000.0;
                state.motionFollowErrorDeg = followErrorDeg;
            }
            if (frame.reg === CUSTOM_REGS.LAG_ANGLE_MDEG) {
                // 实时滞后角（mdeg 有符号）：按当前转速从 lag_angle_ 拟合模型算出。
                // 未标定滞后角表时固件恒发 0。
                state.motionLagAngleDeg = (frame.value | 0) / 1000.0;
            }
            if (frame.reg === 0x122) {
                const windowValue = clamp(Number(frame.value || 10000), 10000, 10000);
                document.getElementById('waveWindowMsInput').value = String(windowValue);
            }
            if (frame.reg === CUSTOM_REGS.ENCODER_FILTER_WINDOW) {
                // 【不再回写输入框】
                // 这里原来把固件回报的 0x148 写回 encoderFilterWindowInput，
                // 等于让“固件当前值”覆盖“用户设定值”。只要有一帧 value 不对
                // （dump 流量混进来、字节错位等），输入框就被永久污染，
                // 之后每次启动都发那个脏值——实测表现为“点开校准表后再跑就丢步”。
                // 输入框归用户，固件到底生效了多少只放在只读显示里。
                state.encoderFilterWindowActive = clamp(Number(frame.value) || 0, 0, 256);
                const activeText = document.getElementById('encoderFilterWindowActive');
                if (activeText) {
                    activeText.textContent = `实际生效：${state.encoderFilterWindowActive} 样本`;
                }
            }
            // 【波形采集不再依赖 0x11F 自动收尾】
            //
            // 原因：0x11F 是双向寄存器（写=启动/停止命令，读=运动状态），网页里有 7 处
            // writeRegister(0x11F, 0)，而这些写入路径不一定真的停电机（有些只写了
            // custom_parameters_ 而没调 stopMotion）。一旦某一处把 0x11F 刷成 0，
            // 这里的收尾逻辑就会把波形冻结，但电机仍在正常跑 —— 表现为“波形走一段就停住”。
            //
            // 现在完全由用户控制：
            //   点「暂停查看」-> 冻结在那一刻，再点恢复
            //   切 tab / 关弹窗 -> stopWaveCapture()
            //   运动结束后波形继续滚（能看到稳态误差），想看结果就手动暂停
            //
            // 下面这段只保留观测记录（给整定器 / 调试用），不再触发任何停止。
            if (frame.reg === 0x11F && (frame.cmd === 0x10 || frame.cmd === 0x81 || frame.cmd === 0x82) && state.waveCaptureActive) {
                if ((frame.value | 0) !== 0) {
                    state.waveMotionObserved = true;
                }
            }
            if (frame.reg === 0x11F && (frame.cmd === 0x10 || frame.cmd === 0x81) && state.tuneTrialActive) {
                const motionStatus = frame.value | 0;
                const previousStatus = state.tuneLastMotionStatus;
                state.tuneLastMotionTelemetryAt = performance.now();
                state.tuneLastMotionStatus = motionStatus;
                state.tuneLastMotionStatusCmd = frame.cmd;
                if (previousStatus !== motionStatus) {
                    const elapsedMs = Math.round(performance.now() - state.tuneTrialStartedAt);
                    appendTuneLog(`[状态] 0x011F=${motionStatus} cmd=0x${frame.cmd.toString(16)}；距试验启动 ${elapsedMs}ms；速度=${Number(state.motionSpeedRpm || 0).toFixed(2)}RPM，误差=${Number(state.motionFollowErrorDeg || 0).toFixed(3)}°`);
                }
                if (motionStatus !== 0) {
                    state.tuneMotionObserved = true;
                    state.tuneTrialStage = '已观察到固件运行状态，等待停止状态';
                } else if (state.tuneMotionObserved) {
                    state.tuneTrialStage = '已观察到固件停止状态';
                    if (state.tuneMotionTimeout !== null) {
                        window.clearTimeout(state.tuneMotionTimeout);
                        state.tuneMotionTimeout = null;
                    }
                    const resolveMotion = state.tuneMotionResolver;
                    state.tuneMotionResolver = null;
                    if (resolveMotion) resolveMotion();
                }
            }
        }

        bindDialDrag();
        bindWavePopupDrag();
        bindControls();
        // 【调试便利】把内部 state 挂到 window，卡住时可在控制台直接看：
        //   __waveState.waveCaptureActive / motionWaveTimer / waveHistory.length / waveMotionObserved
        // active=true 且 timer=null  -> 定时器丢了（看门狗会重启）
        // active=false              -> stopWaveCapture 被调了（看控制台的 [WAVE] stop 调用栈）
        // active=true 但 samples 不涨 -> pushWaveSample 内部抛异常
        window.__waveState = state;
        buildMotionControlOptions();
        // 波形模块里的 updateMotionDurationEstimate 不再在文件加载时自动调用
        // （它依赖的 getEffectivePulseCount / getStepsPerRevolution 在本脚本里定义），
        // 改在这里统一初始化，确保依赖已就绪。
        updateMotionDurationEstimate();
        renderTuneActionRows();
        renderRegisterMap();
        updateDialVisual();
        updateFaultStatus();
        updateTelemetryDisplay();

        setInterval(() => {
            updateTelemetryDisplay();
            ensureWaveTimerAlive();
        }, 200);

        // 外部 EN 电平随时可能被 PLC 改变：每秒回读一次使能状态（不参与控制，只显示）
        setInterval(() => {
            if (state.connected) void refreshEnableState();
        }, 1000);

        const originalHandleIncomingFrame = handleIncomingFrame;
        handleIncomingFrame = function (frame) {
            originalHandleIncomingFrame(frame);
            handleCustomTelemetry(frame);
        };

