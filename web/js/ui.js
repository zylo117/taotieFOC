// UI helpers: logging, badges, telemetry display, encoder jitter, register map (module 6).
// Depends on: state, CUSTOM_REGS, ENCODER_LSB_DEG (constants), REGISTER_DESCRIPTIONS (still inline).
// Load order: constants -> utils -> state -> protocol -> rx -> ui -> ...

        function log(message) {
            // 【默认屏蔽参数收发】[TX]/[RX] 是每帧都打的，每秒几百行，会把 [INFO]/[SETTLE]/[冲击]
            // 这些真正有用的日志冲掉。默认隐藏，需要看通信细节时在控制台执行：
            //   __hideTxRx = false   显示
            //   __hideTxRx = true    再屏蔽
            if (window.__hideTxRx !== false && (message.startsWith('[TX]') || message.startsWith('[RX]'))) {
                return;
            }
            const line = document.createElement('p');
            line.className = 'log-line';
            line.textContent = message;
            logArea.appendChild(line);
            while (logArea.children.length > 200) {
                logArea.removeChild(logArea.firstElementChild);
            }
            logArea.scrollTop = logArea.scrollHeight;
        }

        function setConnBadge(status, isOk = true) {
            connBadge.textContent = status;
            connBadge.className = 'badge ' + (isOk ? 'ok' : 'warn');
            connBadge.style.setProperty('border-color', isOk ? 'rgba(34,197,94,0.3)' : 'rgba(245,158,11,0.3)');
        }

        // 主动读一次堵转状态（0x15D/0x15E）并刷新显示。
        // 遥测里 0x108 的 bit3 也会更新 state.stallDetected，但那是 0.7s 才轮一次，
        // 所以"清除堵转锁存"按钮点完要立刻回读一次，让界面马上变成"未触发"。
        async function refreshStallStatus() {
            if (!state.connected) return;
            try {
                const detected = (await readRegisterValue(CUSTOM_REGS.STALL_DETECTED, 750, 0)) | 0;
                const events = (await readRegisterValue(CUSTOM_REGS.STALL_EVENTS, 750, 0)) >>> 0;
                state.stallDetected = detected !== 0;
                const element = document.getElementById('stallStatus');
                if (element) {
                    element.textContent = state.stallDetected
                        ? `已触发（累计 ${events} 次）：固件已停机并失能，需点“清除堵转锁存”`
                        : '未触发';
                }
                updateFaultStatus();
            } catch (error) {
                // 读不到就维持遥测显示，不要因为一次诊断读失败打断用户操作
            }
        }

        // 读一次使能状态（0x162）与外部 EN 电平（0x164）并刷新显示。
        // 外部模式下引脚随时可能被 PLC 拉动，所以主循环里每秒轮询一次。
        async function refreshEnableState() {
            if (!state.connected) return;
            try {
                const enabled = (await readRegisterValue(CUSTOM_REGS.ENABLE_STATE, 750, 0)) | 0;
                const level = (await readRegisterValue(CUSTOM_REGS.EXT_EN_LEVEL, 750, 0)) | 0;
                state.driverEnabled = enabled !== 0;
                state.externalEnLevel = level;
                const element = document.getElementById('enableStateReadout');
                if (element) {
                    if (level === 2) {
                        element.textContent = state.driverEnabled ? '已使能（上位机控制）' : '已失能（上位机控制）';
                    } else {
                        element.textContent = `${level === 0 ? '引脚低=使能' : '引脚高=失能'} / 驱动器${state.driverEnabled ? '已使能' : '未使能'}`;
                    }
                }
            } catch (error) {
                // 读不到就保持上次显示，不打断用户操作
            }
        }

        // ===================== 低频配置回读（UI ↔ MCU 一致性）=====================
        // 需求：界面显示的参数必须等于 MCU 里的真实值，但同步速率不需要高。
        // 这些都是"人工点击级"频率的参数框，3 秒一轮完全够；一轮最多 6 帧 READ
        //（每帧 10 字节 @115200 ≈ 0.87ms），占空比不到 1%，挤不掉遥测也不会让
        // 界面有"跟不上"的感觉。
        //
        // 为什么需要它：网页的输入框只在「连接时」回填一次（见 actions.js 的
        // pullPidFromDeviceOnce），之后固件值因为任何原因变了（另开一个上位机、
        // 重启、固件自己钳位如 0x11B 被钳到 0x182 量程），界面都继续显示旧数字。
        // "我以为参数是 A，MCU 实际是 B"正是这一整轮细分口径事故的形状
        //（网页输入框默认 64 / 固件 32µ），所以这里把"显示即真值"变成持续保证。
        //
        // 【两条必须守住的红线】
        //  ① 不覆盖用户正在编辑的控件：activeElement 就是它 → 本轮跳过这个控件。
        //     否则用户打字打到一半就被固件旧值顶回去。
        //  ② 只给 .value 赋值，绝不 dispatchEvent：程序赋值本身不会触发
        //     input/change（浏览器只在用户交互时触发），所以不会反过来触发
        //     writePidParameter / 各种 change 监听把值写回 MCU，形成读→写→读死循环。
        const CONFIG_SYNC_ITEMS = [
            {reg: CUSTOM_REGS.MICROSTEPS,        id: 'microsteps',            label: '微步'},
            {reg: 0x11A,                         id: 'startRpmSelect',        label: '起始转速'},
            {reg: 0x11B,                         id: 'maxRpmSelect',          label: '最高转速'},
            {reg: 0x11C,                         id: 'accelRpmSelect',        label: '加速度'},
            {reg: 0x123,                         id: 'stepPulseWidthNsInput', label: 'STEP 脉宽'},
            {reg: CUSTOM_REGS.LAG_DEADBAND_MDEG, id: 'lagDeadbandMdeg',       label: '滞后角死区'}
        ];

        // 串口是否"正在忙"：低频回读是礼貌访客，只要有任何别人在等回包 / 有写命令
        // 在等 ACK / 标定整定校准表这类连续流量在进行，本轮直接放弃，绝不抢帧。
        function serialBusBusy() {
            if (state.registerReadWaiters.size > 0) return true;        // 有人正在等 0x81
            if (state.registerWriteAckWaiters.size > 0) return true;    // 有人正在等 0x82 ACK
            if (state.targetWriteInFlight) return true;                 // 目标角写入进行中
            if (state.calibrationLookupSerialToken !== null) return true;
            if (state.calibrationDumpKind) return true;                 // 校准表 dump 在刷屏
            if (state.autoTuneActive || state.tuneTrialActive || state.tuneActionBusy) return true;
            if (state.resonanceScanActive || state.lagAngleCalibrationRunning) return true;
            return false;
        }

        async function syncConfigFromDevice() {
            if (!state.connected || !state.writer) return;
            if (state.configSyncInFlight) return;   // 上一轮还没收完（最坏 6×超时），不叠第二遍
            if (serialBusBusy()) return;
            state.configSyncInFlight = true;
            const changes = [];
            try {
                for (const item of CONFIG_SYNC_ITEMS) {
                    if (!state.connected) break;         // 中途拔线：立刻收手
                    if (serialBusBusy()) break;          // 中途插进来写/读命令：本轮到此为止
                    const element = document.getElementById(item.id);
                    if (!element) continue;
                    if (document.activeElement === element) continue;   // 红线① 正在编辑
                    let raw;
                    try {
                        raw = await readRegisterValue(item.reg, 800, 0);
                    } catch (error) {
                        continue;   // 读不到就维持现状：低频任务下一轮再来，不打断任何流程
                    }
                    if (!Number.isFinite(raw)) continue;
                    const text = String(raw >>> 0);
                    if (element.tagName === 'SELECT') {
                        // 离散下拉框只认自己那几个档位：固件值不在档位里就不动它。
                        // 赋一个不存在的 option 会把 selectedIndex 变成 -1，界面显示
                        // 空白，之后 Number('' || 默认) 又悄悄换成另一个数 —— 比继续
                        // 显示旧值更糟。这种不一致值得留给人看见，不值得自动编一个。
                        if (![...element.options].some((option) => option.value === text)) continue;
                    }
                    if (String(element.value) === text) continue;   // 没变就不碰 DOM
                    element.value = text;                            // 红线② 只赋值，不派发事件
                    if (item.reg === CUSTOM_REGS.MICROSTEPS) state.microsteps = raw >>> 0;
                    changes.push(`${item.label} ${text}`);
                }
            } finally {
                state.configSyncInFlight = false;
            }
            // 只在真的对不齐时才吭声：对齐是常态，常态不该刷屏。
            if (changes.length > 0) {
                log('[SYNC] 输入框已与 MCU 对齐：' + changes.join('，'));
            }
        }

        function updateFaultStatus() {
            const hasFault = state.encoderFault || state.magneticFault || state.outputStopped || state.stallDetected;
            const summary = state.stallDetected ? '堵转锁存' : state.encoderFault ? '编码器读取失败' : state.magneticFault ? '磁场报警' : state.outputStopped ? '已停止输出' : '正常';
            state.faultSummary = summary;
            const stallElement = document.getElementById('stallStatus');
            if (stallElement) {
                stallElement.textContent = state.stallDetected
                    ? '已触发：固件已停机并失能，需点“清除堵转锁存”'
                    : '未触发';
            }
            // 全局「解除堵转」按钮（右下角急停上方）：堵转锁存期间常驻显示。
            // 遥测 bit3 一清就自动消失，不需要用户去整定页找那个小按钮。
            const unlockBtn = document.getElementById('unlockStallBtn');
            if (unlockBtn) unlockBtn.hidden = !state.stallDetected;

            faultBadge.innerHTML = `<span class="led ${hasFault ? (state.encoderFault || state.magneticFault ? 'warn' : 'danger') : ''}"></span> ${summary}`;
            faultBadge.className = 'badge ' + (hasFault ? (state.encoderFault || state.magneticFault ? 'warn' : 'danger') : 'ok');
            document.getElementById('faultSummaryValue').textContent = summary;
            if (hasFault) {
                log('[WARN] 故障状态：' + summary);
            }
        }

        // ===== 磁编码器空闲抖动统计 =====
        // 只在电机不动时统计，用来区分“随机噪声”和“系统偏差”：
        //   抖动范围 ≈ 1 LSB 且很久不再变大 → 随机噪声已被压到量化极限，
        //       剩下的角度差就都是可重复的系统误差（磁铁偏心 / 传感器 INL / 校准残差 / 微步定位）。
        //   抖动范围有好几个 LSB → 随机噪声占主导，单次读数本身就不准，不能拿它当真值。
        const ENCODER_JITTER_WINDOW_MS = 4000;
        const ENCODER_JITTER_RESET_DEG = 0.2;   // 位移超过这个值就认为轴被挪动过，重新开窗

        function resetEncoderJitterWindow() {
            state.encoderJitterSamples = [];
            state.encoderJitterRangeDeg = 0;
            state.encoderJitterSampleCount = 0;
        }

        function pushEncoderJitterSample(angleDeg) {
            const now = performance.now();
            const samples = state.encoderJitterSamples;
            const previous = samples.length ? samples[samples.length - 1] : null;
            if (previous && Math.abs(normalizeAngle(angleDeg - previous.deg)) > ENCODER_JITTER_RESET_DEG) {
                resetEncoderJitterWindow();
            }
            state.encoderJitterSamples.push({t: now, deg: angleDeg});
            while (state.encoderJitterSamples.length &&
                   now - state.encoderJitterSamples[0].t > ENCODER_JITTER_WINDOW_MS) {
                state.encoderJitterSamples.shift();
            }
            // 以最新样本为基准做环绕归一化，轴停在 0°/360° 附近也不会把范围算成 359°
            const list = state.encoderJitterSamples;
            const reference = list[list.length - 1].deg;
            let low = 0;
            let high = 0;
            list.forEach((entry) => {
                const delta = normalizeAngle(entry.deg - reference);
                if (delta < low) low = delta;
                if (delta > high) high = delta;
            });
            state.encoderJitterRangeDeg = high - low;
            state.encoderJitterSampleCount = list.length;
        }

        function updateTelemetryDisplay() {
            document.getElementById('phaseACurrentValue').textContent = `${state.phaseACurrentA.toFixed(2)} A`;
            document.getElementById('phaseBCurrentValue').textContent = `${state.phaseBCurrentA.toFixed(2)} A`;
            document.getElementById('posLoopRateValue').textContent = `${state.posLoopHz} Hz`;
            document.getElementById('velLoopRateValue').textContent = `${state.velLoopHz} Hz`;
            document.getElementById('curLoopRateValue').textContent = `${state.curLoopHz} Hz`;
            document.getElementById('cpuUsageBadge').textContent = `CPU ${state.cpuUsagePercent.toFixed(1)}%`;
            document.getElementById('controlUsageBadge').textContent = `控制 ${state.controlTaskUsagePercent.toFixed(1)}%`;
            document.getElementById('faultSummaryValue').textContent = state.faultSummary;
            document.getElementById('encoderAngleValue').textContent = `${state.encoderAngleDeg.toFixed(2)}°`;
            const jitterElement = document.getElementById('encoderJitterValue');
            if (state.motionRunning) {
                jitterElement.textContent = '运行中';
            } else if (state.encoderJitterSampleCount < 5) {
                jitterElement.textContent = '采集中…';
            } else {
                const jitterLsb = state.encoderJitterRangeDeg / ENCODER_LSB_DEG;
                jitterElement.textContent = `${state.encoderJitterRangeDeg.toFixed(4)}° (${jitterLsb.toFixed(1)} LSB)`;
            }
            document.getElementById('magneticLevelValue').textContent = `${state.magneticHigh ? '报警' : '正常'} / ${state.magneticLow ? '报警' : '正常'}`;
            document.getElementById('speedValue').textContent = Number(state.motionSpeedRpm || 0).toFixed(2);
            document.getElementById('accelValue').textContent = (state.accel || 0).toFixed(2);
            document.getElementById('currentValue').textContent = (state.currentA || 0).toFixed(2);
        }

        function renderRegisterMap() {
            const rows = [...REGISTER_DESCRIPTIONS.values()].map(([address, name, description]) =>
                `<div>${address}　<strong>${name}</strong>　${description}</div>`).join('');
            document.getElementById('registerMap').innerHTML = rows;
        }

