// User actions: connect, tabs, dial, config apply, motor control (module 10b).
// Depends on: state, CUSTOM_REGS, protocol primitives, ui, waveform, pidtune.

        async function armPositionFollowMode() {
            if (!state.connected) return;
            const startRpm = clamp(Math.round(Number(document.getElementById('startRpmSelect').value || 120)), 1, 12000);
            const maxRpm = clamp(Math.round(Number(document.getElementById('maxRpmSelect').value || 1200)), 1, 12000);
            const accelRpm = clamp(Math.round(Number(document.getElementById('accelRpmSelect').value || 6000)), 1, 12000);
            const angleMaxRpm = clamp(Math.round(Number(document.getElementById('angleMaxRpmInput').value || 1200)), 1, 12000);
            const angleAccelRpm = clamp(Math.round(Number(document.getElementById('angleAccelRpmSInput').value || 6000)), 1, 12000);
            await writeRegister(0x11A, startRpm);
            await writeRegister(0x11B, maxRpm);
            await writeRegister(0x11C, accelRpm);
            await writeRegister(CUSTOM_REGS.ANGLE_MAX_RPM, angleMaxRpm);
            await writeRegister(CUSTOM_REGS.ANGLE_ACCEL_RPM_S, angleAccelRpm);
        }

        // ===== 共振带自动避让（滞后角校准 / 冲击标定等"速度归属型"扫速共用）=====
        // 固件 0x16F/0x170：巡航目标落进 [lo,hi] 会被钳到共振带下限 —— 对正常运动是对的
        // （不在共振点上驻留），但对标定是灾难：这个采样点的"速度"是假的，
        // 滞后角/冲击峰值会被归属到电机根本没跑过的速度上，拟合曲线直接失真。
        // 所以扫速采样开始前把落带内的点剔除（0 保留：它不参与运动）。
        // hi<=lo（含 hi=0）= 固件共振带关闭，不筛。旧固件读不到寄存器也不筛。
        async function excludeResonanceBandSpeeds(speedList, label) {
            let lo = NaN;
            let hi = NaN;
            try {
                lo = await readRegisterValue(CUSTOM_REGS.RESONANCE_LO_RPM);
                hi = await readRegisterValue(CUSTOM_REGS.RESONANCE_HI_RPM);
            } catch (e) {
                return speedList;
            }
            if (!Number.isFinite(lo) || !Number.isFinite(hi)) return speedList;
            lo = lo | 0;
            hi = hi | 0;
            if (hi <= lo) return speedList;
            const kept = speedList.filter((r) => !(r > 0 && r >= lo && r <= hi));
            const dropped = speedList.length - kept.length;
            if (dropped > 0) {
                log(`[${label}] 共振带 [${lo}, ${hi}] RPM：自动剔除 ${dropped} 个扫速点（剩 ${kept.length} 个，`
                    + '落带内的点会被固件钳速，测出来的滞后角/冲击数据会归属到错误的速度上）');
            }
            return kept;
        }

        // ===== 串口生命周期：断开 / 拔线检测 / 安全重连 =====
        // 没有这些，拔线后 state.connected 永远卡在 true、reader 泄漏、
        // 重连还会在旧 port 上叠第二个 reader；所有 await 中的读写请求也永远挂着。

        function rejectAllPendingWaiters(message) {
            for (const waiters of state.registerReadWaiters.values()) {
                for (const w of (Array.isArray(waiters) ? waiters : [waiters])) {
                    window.clearTimeout(w.timeout);
                    try { w.reject(new Error(message)); } catch (e) { /* 已结束，忽略 */ }
                }
            }
            state.registerReadWaiters.clear();
            for (const w of state.registerWriteAckWaiters.values()) {
                window.clearTimeout(w.timeout);
                try { w.reject(new Error(message)); } catch (e) { /* 已结束，忽略 */ }
            }
            state.registerWriteAckWaiters.clear();
        }

        async function disconnectSerial(reason) {
            const port = state.port;
            if (!port && !state.connected) return;
            // 先清状态：所有 await 中的调用再检查 connected 就会提前失败，
            // 而不是拿着已释放的 writer 继续写。
            state.connected = false;
            state.readGeneration += 1;   // 作废当代 readLoop 的收尾权
            stopWaveCapture();
            if (state.waveStartDelayTimer) {
                window.clearTimeout(state.waveStartDelayTimer);
                state.waveStartDelayTimer = null;
            }
            rejectAllPendingWaiters('串口已断开');
            const reader = state.reader;
            const writer = state.writer;
            state.reader = null;
            state.writer = null;
            // 顺序：cancel 让 read() 带 done 返回 → releaseLock → close。
            try { if (reader) await reader.cancel(); } catch (e) { /* 已断，正常 */ }
            try { if (reader) { await reader.closed.catch(() => {}); reader.releaseLock(); } } catch (e) {}
            try { if (writer) writer.releaseLock(); } catch (e) {}
            try { if (port) { port.ondisconnect = null; await port.close(); } } catch (e) {}
            if (state.port === port) state.port = null;
            setConnBadge('未连接', false);
            log('[INFO] 串口已断开' + (reason ? '（' + reason + '）' : '') + ' —— 电机状态不变（固件保持当前模式继续运行）');
        }

        async function connectSerial() {
            let portObj = null;
            try {
                if (!('serial' in navigator)) {
                    log('[WARN] 当前浏览器不支持 Web Serial API，请使用 Chrome / Edge');
                    return;
                }
                // 已经连着就先干净地断开：否则旧 reader 还在消费，新连接会叠两个读循环。
                if (state.connected || state.port) {
                    await disconnectSerial('重连前清理');
                }

                portObj = await navigator.serial.requestPort();
                const baud = Number(document.getElementById('baudRate').value || 115200);
                await portObj.open({baudRate: baud});
                // 物理拔出（没走我们按钮的那种）：浏览器在原 port 对象上发 disconnect 事件。
                // 挂上它，拔线 1 秒内界面就变"未连接"并释放资源，而不是等读超时。
                portObj.ondisconnect = () => {
                    if (state.port === portObj) void disconnectSerial('设备拔出');
                };

                state.port = portObj;
                state.connected = true;
                state.readGeneration += 1;
                // 新连接：允许 ensureMotionDefaultsForAngleControl 重新下发一次默认值
                state.angleDefaultsApplied = false;
                // 新连接：允许 PID 输入框从固件回填一次（之后输入框归用户，不再被固件值覆盖）
                state.pidSyncedFromDevice = false;
                state.reader = portObj.readable.getReader();
                state.writer = portObj.writable.getWriter();
                // 【连上就主动读一次 PID 回填】不能只靠遥测：
                //   - readRegisterValue 的 0x81 回包只 resolve 等待器，不会去写输入框
                //   - 遥测回填又有 1.5 秒窗口限制
                // 结果就是“保存成功、重启加载成功，但页面上一片默认值”。
                // 这里直接用 0x81 路径读一次，拿到值自己写输入框。
                void (async function pullPidFromDeviceOnce() {
                    // 等 readLoop 起来再发读请求（刚设好 reader 时还没开始收帧）
                    await new Promise((resolve) => window.setTimeout(resolve, 600));
                    const pidRegs = [
                        [CUSTOM_REGS.POSITION_KP, 'posKp'],
                        [CUSTOM_REGS.POSITION_KI, 'posKi'],
                        [CUSTOM_REGS.POSITION_KD, 'posKd'],
                        [CUSTOM_REGS.VELOCITY_KP, 'velKp'],
                        [CUSTOM_REGS.VELOCITY_KI, 'velKi'],
                        [CUSTOM_REGS.VELOCITY_KD, 'velKd']
                    ];
                    for (const [reg, id] of pidRegs) {
                        try {
                            const raw = await readRegisterValue(reg);
                            if (!Number.isFinite(raw)) continue;
                            const el = document.getElementById(id);
                            if (el) el.value = (Number(raw) / 1000).toFixed(3);
                        } catch (e) {
                            log(`[PID] 读取 0x${reg.toString(16)} 失败：${e.message}`);
                        }
                    }
                    // 读完就把回填窗口关掉，之后输入框归用户
                    state.pidSyncedFromDevice = true;
                    log('[PID] 已从固件读取当前 PID');
                    // 【使能来源也要同步】0x161 是掉电保持的：固件里可能是"外部 EN 引脚"，
                    // 网页下拉框如果还显示"上位机"，用户会以为使能按钮能直接控制芯片 EN。
                    try {
                        const mode = await readRegisterValue(CUSTOM_REGS.ENABLE_MODE);
                        const select = document.getElementById('enableModeSelect');
                        if (select && Number.isFinite(mode)) select.value = String((mode | 0) & 1);
                        log(`[INFO] 使能来源 = ${((mode | 0) & 1) === 1 ? '外部 EN 引脚 (PC13 低有效)' : '上位机'}`);
                    } catch (e) {
                        log(`[EN] 读取使能来源失败：${e.message}`);
                    }
                    await refreshEnableState();
                    // 滞后角模型参数（0x166/0x167）也要在连接时同步：它们是存 Flash 的
                    try {
                        const accelCoef = await readRegisterValue(CUSTOM_REGS.LAG_ACCEL_COEF, 750, 0);
                        const deadband = await readRegisterValue(CUSTOM_REGS.LAG_DEADBAND_MDEG, 750, 0);
                        const a = document.getElementById('lagAccelCoef');
                        const d = document.getElementById('lagDeadbandMdeg');
                        if (a && Number.isFinite(accelCoef)) a.value = String(accelCoef | 0);
                        if (d && Number.isFinite(deadband)) d.value = String(deadband | 0);
                        log(`[INFO] 滞后角模型：加减速系数 ${(accelCoef | 0)} mdeg/(RPM/s)，死区 ${(deadband | 0)} mdeg`);
                    } catch (e) {
                        log(`[LAG] 读取滞后角模型参数失败：${e.message}`);
                    }
                    // 【连接时同步运动源 + 共振带】两者都存 Flash，重启后和页面默认值
                    // 不一定一致；尤其固件上电默认外部跟随 —— 界面必须跟着切过去，
                    // 否则用户对着灰不拉几还能点的“内部”按钮以为能动车。
                    try {
                        const src = await readRegisterValue(CUSTOM_REGS.MOTION_SOURCE);
                        const lo = await readRegisterValue(CUSTOM_REGS.RESONANCE_LO_RPM);
                        const hi = await readRegisterValue(CUSTOM_REGS.RESONANCE_HI_RPM);
                        if (Number.isFinite(src)) {
                            const s = (src | 0) !== 0 ? 1 : 0;
                            if (s !== state.motionSource) switchWorkTab(s === 0 ? 'motion' : 'extfollow', true);
                        }
                        const loEl = document.getElementById('resonanceLoRpmInput');
                        const hiEl = document.getElementById('resonanceHiRpmInput');
                        if (loEl && Number.isFinite(lo)) loEl.value = String(lo | 0);
                        if (hiEl && Number.isFinite(hi)) hiEl.value = String(hi | 0);
                        if (Number.isFinite(src)) {
                            log('[INFO] 运动源 = ' + ((src | 0) !== 0 ? '外部脉冲跟随（内部运动已互斥禁用）' : '内部规划')
                                + `，共振带 [${(lo | 0)}, ${(hi | 0)}] RPM`);
                        }
                    } catch (e) {
                        log('[INFO] 运动源/共振带回读失败（旧固件没有 0x16D~0x170 就是这个）：' + e.message);
                    }
                })();

                setConnBadge('已连接', true);
                protoBadge.textContent = 'TMC 兼容';
                log('[INFO] 串口已连接，开始监听返回数据');
                await armPositionFollowMode();
                if (state.pendingTargetAngle !== null) void flushTargetAngleWrites();

                readLoop();
            } catch (err) {
                // 半途失败（用户取消选择/占用中打开失败）：把半途打开的 port 收拾干净，
                // 不留 connected=true 的假连接。
                if (portObj) {
                    if (state.port === portObj) {
                        await disconnectSerial('连接流程中断').catch(() => {});
                    } else {
                        try { portObj.ondisconnect = null; await portObj.close(); } catch (e) {}
                    }
                }
                state.connected = false;
                state.port = null;
                state.reader = null;
                state.writer = null;
                log('[ERROR] 串口连接失败：' + (err && err.message ? err.message : String(err)));
                setConnBadge('未连接', false);
            }
        }

        // ===================== 运动源门控（与固件 0x16D 的绝对互斥对齐）=====================
        // 固件在外部脉冲模式下会拒绝一切内部运动（startMotion / 目标角 / 校准会打
        // [SOURCE] rejected）。这里把对应控件置灰：不是安全边界（边界在固件），
        // 而是让用户不用先踩一次拒绝才知道不能用。
        const INTERNAL_MOTION_CONTROL_IDS = [
            'sendTargetBtn', 'zeroBtn',                       // 目标角/归零（角度模式）
            'startMotionBtn',                                 // 规划运动启动（tab 里，双保险）
            'tuneBtn', 'startTuneBtn',                        // PID 自动整定（会驱动电机）
            'lagAngleCalibrationBtn', 'startLagAngleCalibrationBtn',  // 滞后角校准
            'startShockCalibrationBtn'                        // 冲击标定
        ];

        function applyMotionSourceGate() {
            const external = state.motionSource !== 0;
            for (const id of INTERNAL_MOTION_CONTROL_IDS) {
                const el = document.getElementById(id);
                if (el) el.disabled = external;
            }
            let banner = document.getElementById('motionSourceBanner');
            if (!banner) {
                banner = document.createElement('div');
                banner.id = 'motionSourceBanner';
                banner.className = 'tiny';
                banner.style.cssText = 'margin:8px 0;padding:6px 10px;border-radius:8px;'
                    + 'background:rgba(245,158,11,0.14);color:#f59e0b;border:1px solid rgba(245,158,11,0.45);';
                const tabBtn = document.getElementById('tabMotionBtn');
                if (tabBtn && tabBtn.parentElement) tabBtn.parentElement.appendChild(banner);
            }
            banner.style.display = external ? '' : 'none';
            banner.textContent = '当前运动源：外部脉冲跟随（固件上电默认）。内部规划/角度/整定/标定已禁用 —— '
                + '上面切到「内部规划运动」tab 才会交还运动通道；两种模式的运动信号绝对互斥。';
        }

        // ===================== 工作模式 tab =====================
        // 测速控制和外部脉冲跟随共用同一个 STEP 输出通道（TMR2/DMA），固件里本来就是互斥的：
        // 外部跟随使能时 rampUpdate 直接走 updateExternalPulseChannel 并 return。
        // 所以这里不让它们同时出现：切 tab 就同时切固件的 0x149。
        function switchWorkTab(tab, skipWrite) {
            const isMotion = tab === 'motion';
            document.getElementById('cardMotion').style.display = isMotion ? '' : 'none';
            document.getElementById('cardExtFollow').style.display = isMotion ? 'none' : '';
            document.getElementById('tabMotionBtn').className = isMotion ? 'primary' : '';
            document.getElementById('tabExtFollowBtn').className = isMotion ? '' : 'primary';
            // 门控状态跟 tab 一致（外部 tab 禁用内部运动控件）
            state.motionSource = isMotion ? 0 : 1;
            applyMotionSourceGate();
            const extInput = document.getElementById('extFollowEnableInput');
            if (extInput && document.activeElement !== extInput) extInput.checked = !isMotion;

            // 【波形采集开关跟着 tab 走】
            // pushWaveSample 只看 state.waveCaptureActive，而这个标志原来只由「启动」按钮置位。
            // 外部跟随模式根本不会点「启动」，所以波形图一直是空的。
            //
            // 【先使能、延迟再采集】切到外部脉冲运动时，固件会 setEnable(true)，
            // 电机从松轴到锁轴，转子会对齐到最近的整步，编码器读数突变、
            // 跟随误差出现尖峰。如果同时开采集，这个抖动会被画进波形，
            // 看起来像“跟随误差很大”。所以先停采集，等使能完成 + 500ms 稳定后再开（见下面 then）。
            if (state.waveStartDelayTimer) {
                window.clearTimeout(state.waveStartDelayTimer);
                state.waveStartDelayTimer = null;
            }
            if (isMotion) {
                stopWaveCapture();
            } else {
                stopWaveCapture();  // 先清空历史，延迟启动在使能完成后的 then 里
                const closedLoopInput = document.getElementById('closedLoopEnableInput');
                if (closedLoopInput) closedLoopInput.checked = false;
                // skipWrite：显示同步（boot 回读），不回写补偿开关 —— 固件跟随通道
                // 本来就不走 PID，多写一枪只是徒增串口流量。
                if (state.connected && !skipWrite) {
                    writeRegister(CUSTOM_REGS.CLOSED_LOOP_ENABLE, 0)
                        .catch((error) => log('[ERROR] 关闭闭环补偿失败：' + error.message));
                }
            }

            if (!state.connected) {
                log('[INFO] 已切到' + (isMotion ? '内部规划运动' : '外部脉冲运动') + '（未连接串口，模式未下发）');
                return;
            }
            if (skipWrite) {
                // 状态本来就是从固件读回来的，不用再写回去（写了反而可能触发一次重放日志）。
                // 外部模式顺手恢复波形采集（延迟避开创动），让回读同步后面板有数据。
                if (!isMotion) {
                    state.waveStartDelayTimer = window.setTimeout(() => {
                        state.waveStartDelayTimer = null;
                        startWaveCapture();
                    }, 500);
                }
                log('[INFO] 工作界面已同步为' + (isMotion ? '内部规划运动' : '外部脉冲运动（固件上电默认）'));
                return;
            }
            writeRegister(CUSTOM_REGS.MOTION_SOURCE, isMotion ? 0 : 1)
                .then(() => {
                    const input = document.getElementById('extFollowEnableInput');
                    if (input) input.checked = !isMotion;
                    log('[INFO] 已切到' + (isMotion ? '内部规划运动（跟随已关）' : '外部脉冲运动（跟随已开）'));
                    // 使能指令已下达到固件（驱动已锁轴），再等 500ms 让转子对齐稳定，
                    // 才开始采集波形——否则使能瞬间的机械抖动会被当成跟随误差。
                    if (!isMotion) {
                        state.waveStartDelayTimer = window.setTimeout(() => {
                            state.waveStartDelayTimer = null;
                            startWaveCapture();
                            log('[INFO] 已开始采集波形（使能后延迟 500ms）');
                        }, 500);
                    }
                })
                .catch((error) => {
                    // 写失败：固件没切换，UI 也得跟着回滚，否则显示和现实相反
                    state.motionSource = isMotion ? 1 : 0;
                    applyMotionSourceGate();
                    log('[ERROR] 切换模式失败：' + error.message);
                });
        }

        async function readLoop() {
            if (!state.reader) return;
            const myReader = state.reader;
            const myGen = state.readGeneration;
            try {
                while (true) {
                    const {value, done} = await myReader.read();
                    if (done) break;
                    if (!value || value.length === 0) continue;
                    // 逐帧 try/catch：单个坏帧（解析器抛错）绝不能连 readLoop 一起带走，
                    // 否则 reader 不再被消费，之后所有回包堆积、界面彻底静止。
                    try {
                        appendRxBytes(value);
                    } catch (frameError) {
                        log('[WARN] 处理一帧返回数据出错（已跳过该帧）：' + frameError.message);
                    }
                }
            } catch (err) {
                // 断开/重连会让旧 reader 抛错，这属于正常收尾；只有当代连接才算真错误。
                if (state.readGeneration === myGen) {
                    log('[ERROR] 串口读取出现错误：' + err.message);
                }
            }
            // 只有仍是当代连接时才更新徽标/清状态（重连后的旧循环不能踩新连接的状态）。
            if (state.readGeneration === myGen && state.connected) {
                log('[WARN] 串口读取结束（设备可能已被拔出）');
                void disconnectSerial('设备拔出 / 读取结束');
            }
        }

        function updateDialVisual() {
            const targetPointerGroup = document.getElementById('dialTargetPointerGroup');
            const actualPointerGroup = document.getElementById('dialActualPointerGroup');
            const valueText = document.getElementById('posValue');
            const currentPosDisplay = document.getElementById('currentPosDisplay');
            const targetPosReadout = document.getElementById('targetPosReadout');
            const currentPosReadout = document.getElementById('currentPosReadout');

            const targetDisplayAngle = normalize360Angle(state.targetAngle);
            const currentDisplayAngle = normalize360Angle(state.currentAngle);

            targetPointerGroup.setAttribute('transform', `rotate(${targetDisplayAngle} 50 50)`);
            actualPointerGroup.setAttribute('transform', `rotate(${currentDisplayAngle} 50 50)`);
            valueText.textContent = `${Math.round(targetDisplayAngle)}°`;
            currentPosDisplay.value = `${Math.round(currentDisplayAngle)}°`;
            currentPosReadout.textContent = `${Math.round(currentDisplayAngle)}°`;
            targetPosReadout.textContent = `${Math.round(targetDisplayAngle)}°`;
        }

        function setCurrentAngle(angle) {
            state.currentAngle = normalize360Angle(angle);
            updateDialVisual();
        }

        function setTargetAngle(angle) {
            state.targetAngle = normalize360Angle(angle);
            document.getElementById('targetPosInput').value = Math.round(state.targetAngle);
            updateDialVisual();
        }

        async function ensureMotionDefaultsForAngleControl() {
            if (!state.connected) return;
            // 【只下发一次】这个名字叫 ensure，但原来没有任何缓存，
            // 而 flushTargetAngleWrites 每次拖拨盘都调它 —— 拖一次就多发 6 条重复写，
            // 60 次/秒的拖动等于每秒 420 帧（其中只有 60 帧是角度）→ USB 排队 → 角度传递慢。
            // 缓存标记在连上新串口时清掉（见 connectSerial）。
            if (state.angleDefaultsApplied) return;
            state.angleDefaultsApplied = true;
            const startRpm = clamp(Math.round(Number(document.getElementById('startRpmSelect').value || 120)), 1, 12000);
            const maxRpm = clamp(Math.round(Number(document.getElementById('maxRpmSelect').value || 1200)), 1, 12000);
            const accelRpm = clamp(Math.round(Number(document.getElementById('accelRpmSelect').value || 6000)), 1, 12000);
            const angleMaxRpm = clamp(Math.round(Number(document.getElementById('angleMaxRpmInput').value || 1200)), 1, 12000);
            const angleAccelRpm = clamp(Math.round(Number(document.getElementById('angleAccelRpmSInput').value || 6000)), 1, 12000);
            await writeRegister(CUSTOM_REGS.CLOSED_LOOP_ENABLE, 1);
            await writeRegister(0x11A, startRpm);
            await writeRegister(0x11B, maxRpm);
            await writeRegister(0x11C, accelRpm);
            await writeRegister(CUSTOM_REGS.ANGLE_MAX_RPM, angleMaxRpm);
            await writeRegister(CUSTOM_REGS.ANGLE_ACCEL_RPM_S, angleAccelRpm);
            document.getElementById('closedLoopEnableInput').checked = true;
        }

        // 当前是否停在“内部规划运动” tab（只有这里绕盘/归零/发目标角度才有意义）
        function isMotionTabActive() {
            const card = document.getElementById('cardMotion');
            return !!card && card.style.display !== 'none';
        }

        // 【位置角度控制 = 内部规划运动】
        // 外部脉冲模式下 rampUpdate 会先走 updateExternalPulseChannel 并 return，
        // 角度模式整段被跳过 —— 此时写 TARGET_ANGLE_DEG / 绕盘都不会有任何反应。
        // 所以只要用户碰了位置监控区（绕盘、归零、发目标角度），就强制切回内部规划 tab，
        // 同时把 0x149 写 0（switchWorkTab 里做），让 UI 和固件一起回到正确模式。
        function ensureMotionTabForPositionControl() {
            if (isMotionTabActive()) return;
            switchWorkTab('motion');
            log('[INFO] 位置控制需要内部规划运动模式，已自动切换');
        }

        function queueTargetAngleWrite() {
            if (state.autoTuneActive) return;
            ensureMotionTabForPositionControl();
            const target = normalize360Angle(Number(document.getElementById('targetPosInput').value || 0));
            setTargetAngle(target);
            state.targetPosition = target;
            state.pendingTargetAngle = target;
            if (state.connected) void flushTargetAngleWrites();
        }

        async function flushTargetAngleWrites() {
            if (state.targetWriteInFlight || !state.connected) return;
            state.targetWriteInFlight = true;
            try {
                await ensureMotionDefaultsForAngleControl();
                while (state.connected && !state.autoTuneActive && state.pendingTargetAngle !== null) {
                    const target = state.pendingTargetAngle;
                    state.pendingTargetAngle = null;
                    await writeRegister(CUSTOM_REGS.TARGET_ANGLE_DEG, Math.round(target * 1000));
                }
            } catch (error) {
                log('[ERROR] 目标角度写入失败：' + error.message);
            } finally {
                state.targetWriteInFlight = false;
            }
        }

        function bindDialDrag() {
            const dial = document.getElementById('dial');
            let dragging = false;
            let lastAngle = 0;

            function normalizeToAngle(clientX, clientY) {
                const rect = dial.getBoundingClientRect();
                const cx = rect.left + rect.width / 2;
                const cy = rect.top + rect.height / 2;
                const dx = clientX - cx;
                const dy = clientY - cy;
                let rad = Math.atan2(dy, dx);
                let deg = (rad * 180 / Math.PI) + 90;
                if (deg < -180) deg += 360;
                if (deg > 180) deg -= 360;
                return deg;
            }

            dial.addEventListener('pointerdown', (e) => {
                if (state.autoTuneActive) return;
                dragging = true;
                dial.setPointerCapture(e.pointerId);
                lastAngle = normalizeToAngle(e.clientX, e.clientY);
                state.dragging = true;
            });

            dial.addEventListener('pointermove', (e) => {
                if (!dragging) return;
                const angle = normalizeToAngle(e.clientX, e.clientY);
                let delta = angle - lastAngle;
                if (delta > 180) delta -= 360;
                if (delta < -180) delta += 360;
                const nextTarget = normalize360Angle(state.targetAngle + delta);
                state.targetAngle = nextTarget;
                document.getElementById('targetPosInput').value = Math.round(nextTarget);
                updateDialVisual();
                queueTargetAngleWrite();
                lastAngle = angle;
            });

            dial.addEventListener('pointerup', () => {
                if (dragging) {
                    dragging = false;
                    state.dragging = false;
                    queueTargetAngleWrite();
                }
            });

            dial.addEventListener('pointerleave', () => {
                if (dragging) {
                    dragging = false;
                    state.dragging = false;
                    queueTargetAngleWrite();
                }
            });
        }

        async function sendTargetPosition() {
            if (state.autoTuneActive) return;
            const target = normalize360Angle(Number(document.getElementById('targetPosInput').value || 0));
            setTargetAngle(target);
            state.targetPosition = target;
            log('[INFO] 发送目标位置：' + target + '°');
            queueTargetAngleWrite();
        }

        async function applyAllConfig() {
            const config = {
                microsteps: Number(document.getElementById('microsteps').value || 32),
                interpMicrosteps: Number(document.getElementById('interpMicrosteps').value || 256),
                runCurrent: Number(document.getElementById('runCurrent').value || 0.8),
                holdCurrent: Number(document.getElementById('holdCurrent').value || 0.2),
                silent: Number(document.getElementById('silentMode').value || 1)
            };

            if (state.connected) {
                await writeRegister(CUSTOM_REGS.MICROSTEPS, config.microsteps);
                await writeRegister(CUSTOM_REGS.INTERPOLATION, config.interpMicrosteps);
                await writeRegister(CUSTOM_REGS.RUN_CURRENT_A, Math.round(config.runCurrent * 1000));
                await writeRegister(CUSTOM_REGS.HOLD_CURRENT_A, Math.round(config.holdCurrent * 1000));
                await writeRegister(CUSTOM_REGS.SILENT_MODE, config.silent);
            }
            log('[INFO] 已应用电机基础参数：' + JSON.stringify(config));
        }

        function getPidParameters() {
            return {
                posKp: Number(document.getElementById('posKp').value || 0),
                posKi: Number(document.getElementById('posKi').value || 0),
                posKd: Number(document.getElementById('posKd').value || 0),
                velKp: Number(document.getElementById('velKp').value || 0),
                velKi: Number(document.getElementById('velKi').value || 0),
                velKd: Number(document.getElementById('velKd').value || 0)
            };
        }

        async function writePidParameters(params = getPidParameters()) {
            // 【只下发规划运动/外部跟随那一组】故意不把角度模式的六个寄存器加进来：
            // 自动整定/应用默认值的结果属于规划运动回路，写它会误改角度模式的参数。
            // 角度增益的持久化由 0x154「写入参数到 Flash」负责（固件会两组一起存）。
            // 【单位】输入框里填的是“实际增益”（默认 60 就是增益 60），发送时乘 1000
            // 变成固件口径。上限和固件 parameter_router.cpp 的 maxPidGain() 严格一致：
            //   位置 Kp/Ki <= 1000，位置 Kd <= 100；速度 Kp/Ki <= 10，速度 Kd <= 1。
            // 旧版这里写的是 30000/3000（把“实际增益”当成了“×1000 的口径”），
            // 于是用户填 60 会被固件钳成 30、填 20 的 Ki 被钳成 3 —— 网页值与生效值不符。
            const registers = [
                [CUSTOM_REGS.POSITION_KP, params.posKp, 1000],
                [CUSTOM_REGS.POSITION_KI, params.posKi, 1000],
                [CUSTOM_REGS.POSITION_KD, params.posKd, 100],
                [CUSTOM_REGS.VELOCITY_KP, params.velKp, 10],
                [CUSTOM_REGS.VELOCITY_KI, params.velKi, 10],
                [CUSTOM_REGS.VELOCITY_KD, params.velKd, 1]
            ];
            // 【必须等 ACK】原来用 writeRegister（不等确认），看校准表后 USB 缓冲
            // 被 dump 流量占着时会丢写，导致 PID 没更新而用户毫无感知。
            for (const [register, gain, maxGain] of registers) {
                await writeRegisterAndWaitAck(register, Math.round(clamp(gain, 0, maxGain) * 1000));
            }
        }

        async function writePidParameter(input) {
            if (!state.connected || state.autoTuneActive) return;
            const registerByInput = {
                posKp: [CUSTOM_REGS.POSITION_KP, 1000],
                posKi: [CUSTOM_REGS.POSITION_KI, 1000],
                posKd: [CUSTOM_REGS.POSITION_KD, 100],
                velKp: [CUSTOM_REGS.VELOCITY_KP, 10],
                velKi: [CUSTOM_REGS.VELOCITY_KI, 10],
                velKd: [CUSTOM_REGS.VELOCITY_KD, 1],
                // 位置环超前系数 α：输入框里填的是 α 本身（1.0~5.0），发送时同样 ×1000
                posLeadAlpha: [CUSTOM_REGS.POSITION_LEAD_ALPHA, 5]
            };
            const parameter = registerByInput[input.id];
            if (!parameter) return;
            const [register, maxGain] = parameter;
            const gain = clamp(Number(input.value) || 0, 0, maxGain);
            try {
                await writeRegister(register, Math.round(gain * 1000));
            } catch (error) {
                log('[ERROR] PID 参数写入失败：' + error.message);
            }
        }

        function readAllConfig() {
            log('[INFO] 发送读取全部参数请求');
            const regs = [
                OFFICIAL_REGS.IHOLD_IRUN,
                OFFICIAL_REGS.CHOPCONF,
                OFFICIAL_REGS.GCONF,
                OFFICIAL_REGS.DRV_CONF,
                OFFICIAL_REGS.TSTEP,
                OFFICIAL_REGS.DRV_STATUS,
                CUSTOM_REGS.CLOSED_LOOP_ENABLE,
                CUSTOM_REGS.RUN_CURRENT_A,
                CUSTOM_REGS.HOLD_CURRENT_A,
                CUSTOM_REGS.ANGLE_MAX_RPM,
                CUSTOM_REGS.ANGLE_ACCEL_RPM_S,
                CUSTOM_REGS.ENCODER_RAW,
                CUSTOM_REGS.ENCODER_ANGLE_MDEG,
                CUSTOM_REGS.MAGNETIC_HIGH,
                CUSTOM_REGS.MAGNETIC_LOW,
                CUSTOM_REGS.ENCODER_ZERO,
                CUSTOM_REGS.ENCODER_FILTER_WINDOW,
                CUSTOM_REGS.EXT_FOLLOW_ENABLE,
                CUSTOM_REGS.EXT_FOLLOW_MAX_STEP_S,
                CUSTOM_REGS.POSITION_KP,
                CUSTOM_REGS.POSITION_KI,
                CUSTOM_REGS.POSITION_KD,
                CUSTOM_REGS.VELOCITY_KP,
                CUSTOM_REGS.VELOCITY_KI,
                CUSTOM_REGS.VELOCITY_KD,
                CUSTOM_REGS.POSITION_LEAD_ALPHA,
                CUSTOM_REGS.MOTION_SOURCE,
                CUSTOM_REGS.RESONANCE_LO_RPM,
                CUSTOM_REGS.RESONANCE_HI_RPM
            ];
            // 【串行读，不要 26 个并发】forEach(async) 会把 26 个读请求一口气全压进
            // 写通道：固件单包响应，队列深了会触发丢帧（0x16B）和等待超时假象。
            // 逐条 await，每条等自己的 0x82 回包再发下一条。
            (async () => {
                for (const reg of regs) {
                    try {
                        await readRegister(reg);
                    } catch (error) {
                        log('[WARN] 读取 0x' + reg.toString(16).toUpperCase() + ' 失败：' + error.message);
                    }
                }
            })();
        }

        async function stopMotor() {
            stopWaveCapture();
            if (state.autoTuneActive) {
                state.autoTuneCancelled = true;
                appendTuneLog('收到停止命令，正在安全终止整定');
            }
            // 【缓停】写 0x163 而不是 0x11F=0：按 0x11C 的加速度减速到 0 就地收尾。
            // 高速运行中直接断脉冲（原行为），绕组反电动势会经 H 桥泵电流冲击驱动器；
            // 斜坡减速把这股能量摊到减速全程。立即断+失能是右下角红色「急停」(0x16E) 的事。
            log('[WARN] 缓停：按减速斜坡停住（比直接断脉冲反电动势温和）');
            if (state.connected) {
                await writeRegister(CUSTOM_REGS.QUICK_STOP, 1);
            }
        }

        // 【急停】右下角红色圆钮：立即断脉冲 + 失能驱动器（固件 [ESTOP]）。
        // 恢复要显式点「使能」（固件置了用户失能锁，自动测试循环唤不醒它）。
        async function emergencyStop() {
            stopWaveCapture();
            if (state.autoTuneActive) {
                state.autoTuneCancelled = true;
                appendTuneLog('急停：整定中止');
            }
            if (!state.connected) {
                log('[WARN] 未连接串口，急停按钮无法下发（必要时直接断电）');
                return;
            }
            try {
                await writeRegisterAndWaitAck(CUSTOM_REGS.EMERGENCY_STOP, 1);
                log('[WARN] ⚠ 急停已执行：脉冲立断 + 驱动器失能（垂直轴会掉！恢复点「使能」）');
            } catch (error) {
                log('[ERROR] 急停下发失败：' + error.message + ' —— 电机在动请直接断电！');
            }
        }

        async function zeroEncoder() {
            const tuning = state.autoTuneActive;
            if (tuning) state.autoTuneCancelled = true;
            state.targetAngle = 0;
            state.targetPosition = 0;
            document.getElementById('targetPosInput').value = '0';
            updateDialVisual();
            log('[INFO] 已发送电机归零目标：0°');
            if (tuning && state.connected) {
                await writeRegister(CUSTOM_REGS.TARGET_ANGLE_DEG, 0);
            } else {
                queueTargetAngleWrite();
            }
        }

