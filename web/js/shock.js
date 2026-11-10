// ===========================================================================
// 阶段 2：急刹冲击标定（module 12）
//
// 目的：在「速度 × 加速度倍数 M」的组合下，测出急刹车时跟随误差的
//   ① 峰值（超调量 → Kd 够不够）
//   ② 恢复时间（阻尼 → Kp/Kd 配合）
//   ③ 稳态 RMS（基础精度 → 开环/闭环对比基线）
//
// 参数完全独立：不与规划运动共用任何输入框（包括加速度），
// 因为两者的使用场景不同——规划运动是连续定位，这里是离散冲击组合。
//
// 依赖（都是全局）：state / CUSTOM_REGS / writeRegisterAndWaitAck / log
// ===========================================================================

(function () {
    'use strict';

    const SHOCK = {
        running: false,
        cancelled: false,
        rows: [],
        lastEmittedSteps: 0
    };

    function $(id) { return document.getElementById(id); }

    function setStatus(text) {
        const el = $('shockStatus');
        if (el) el.textContent = text;
    }

    function setProgress(done, total) {
        const el = $('shockProgress');
        if (el) el.textContent = `${done} / ${total}`;
    }

    function parseNumberList(text, fallback) {
        const list = String(text || '')
            .split(/[,，\s]+/)
            .map((t) => Number(t))
            .filter((v) => Number.isFinite(v) && v > 0);
        return list.length > 0 ? list : fallback;
    }

    function readConfig() {
        return {
            speeds: parseNumberList($('shockSpeedList') && $('shockSpeedList').value, [300, 600, 1200]),
            multipliers: parseNumberList($('shockAccelMultipliers') && $('shockAccelMultipliers').value, [1, 2, 4, 8]),
            baseAccelRpmS: Math.max(1, Number($('shockBaseAccelRpmS') && $('shockBaseAccelRpmS').value) || 6000),
            turnsPerSegment: Math.max(1, Number($('shockTurnsPerSegment') && $('shockTurnsPerSegment').value) || 2),
            tolerancePct: Math.max(1, Number($('shockTolerancePct') && $('shockTolerancePct').value) || 12),
            testOpenLoop: !!($('shockTestOpenLoop') && $('shockTestOpenLoop').checked),
            testClosedLoop: !!($('shockTestClosedLoop') && $('shockTestClosedLoop').checked)
        };
    }

    function renderRows() {
        const tbody = $('shockResultTable') && $('shockResultTable').querySelector('tbody');
        if (!tbody) return;
        tbody.innerHTML = '';
        for (const row of SHOCK.rows) {
            const tr = document.createElement('tr');
            const cells = [
                row.speedRpm,
                row.multiplier,
                row.mode,
                row.peakDeg === null ? '--' : row.peakDeg.toFixed(2),
                row.recoverMs === null ? '--' : row.recoverMs.toFixed(0),
                row.steadyRmsDeg === null ? '--' : row.steadyRmsDeg.toFixed(3)
            ];
            for (const c of cells) {
                const td = document.createElement('td');
                td.textContent = String(c);
                tr.appendChild(td);
            }
            tbody.appendChild(tr);
        }
    }

    function delay(ms) {
        return new Promise((resolve) => window.setTimeout(resolve, Math.max(0, ms)));
    }

    async function ampWaitWhileRunning(ms, stepMs = 50) {
        const deadline = performance.now() + ms;
        while (performance.now() < deadline) {
            if (SHOCK.cancelled || !state.connected) return false;
            await delay(Math.min(stepMs, deadline - performance.now()));
        }
        return true;
    }

    async function setSpeedAndWait(targetRpm, timeoutMs) {
        await writeRegisterAndWaitAck(0x11B, Math.max(10, Math.round(targetRpm)));
        const deadline = performance.now() + timeoutMs;
        const tolerance = Math.max(5, targetRpm * (readConfig().tolerancePct / 100));
        while (performance.now() < deadline) {
            if (SHOCK.cancelled) return false;
            const actual = Math.abs(Number(state.motionSpeedRpm || 0));
            if (actual > 0 && Math.abs(actual - targetRpm) <= tolerance) {
                return true;
            }
            await delay(80);
        }
        // 超时也继续（和滞后角校准一致的策略：按时间判到位，避免跳点）
        return true;
    }

    async function runOneCombination(speedRpm, multiplier, closedLoop, config) {
        const modeLabel = closedLoop ? '闭环' : '开环';
        setStatus(`${speedRpm} RPM × M${multiplier}（${modeLabel}）准备中...`);

        // 闭环补偿开关交给本次测试显式设置（不影响用户界面上的勾选）
        await writeRegisterAndWaitAck(CUSTOM_REGS.CLOSED_LOOP_ENABLE, closedLoop ? 1 : 0);

        // 加速度基准 × M：急刹用它，正常加速也用它（保证两段对称）
        const accel = Math.min(12000, Math.round(config.baseAccelRpmS * multiplier));
        await writeRegisterAndWaitAck(0x11C, accel);

        // 启动（无限行程，速度模式），先到目标速度
        await writeRegisterAndWaitAck(0x11D, 0);
        await writeRegisterAndWaitAck(0x11E, 2);
        await writeRegisterAndWaitAck(0x11F, 1);
        await setSpeedAndWait(speedRpm, Math.max(3000, speedRpm / Math.max(1, config.baseAccelRpmS) * 3000 + 1500));

        // 匀速段：按【已发步数】精确数圈，不再用时间估算
        // 0x134 返回 motion_steps_emitted_（本段已发步数，startMotion 时清零）
        // 原来用 steadyMs = 圈数×60000/RPM 并强制 min 1500ms，导致 2 圈 @300RPM
        // 实际转了 7.5 圈（0.4 秒被拉到 1.5 秒），而且转速波动时圈数根本不可能准。
        const stepsPerRound = Math.max(1, getStepsPerRevolution());
        const targetSteps = Math.round(config.turnsPerSegment * stepsPerRound);
        const steadySamples = [];
        // 匀速段超时上限：理论时间 × 2，避免数圈判断失效时一直跑到天花乱坠
        const theoreticalCruiseMs = (config.turnsPerSegment * 60000) / Math.max(60, speedRpm);
        const steadyDeadline = performance.now() + Math.max(2000, theoreticalCruiseMs * 2);
        let reachedTurns = false;
        let pollCounter = 0;
        while (performance.now() < steadyDeadline) {
            if (SHOCK.cancelled) return null;
            steadySamples.push(Number(state.motionFollowErrorDeg || 0));
            // 每 5 次采样（约 100ms）才读一次 0x134：readRegisterValue 是“发请求+等回包”，
            // 每次都读会把采样间隔拉到 50ms 以上，而且万一没有回包会阻塞 1.5 秒。
            if ((++pollCounter % 5) === 0) {
                const emitted = await readRegisterValue(0x134);
                if (Number.isFinite(emitted)) {
                    SHOCK.lastEmittedSteps = Number(emitted);
                }
                if (Number.isFinite(emitted) && Number(emitted) >= targetSteps) {
                    reachedTurns = true;
                    break;
                }
            }
            await delay(20);
        }
        if (!reachedTurns) {
            log(`[冲击] ${speedRpm}RPM 未在超时内走完 ${config.turnsPerSegment} 圈（目标 ${targetSteps} 步，最后读到 ${SHOCK.lastEmittedSteps}），按已采样本继续`);
        } else {
            log(`[冲击] ${speedRpm}RPM 匀速段完成：已发 ${SHOCK.lastEmittedSteps} 步（目标 ${targetSteps}）`);
        }
        const steadyRms = steadySamples.length > 0
            ? Math.sqrt(steadySamples.reduce((s, v) => s + v * v, 0) / steadySamples.length)
            : 0;

        // 冲击：以 accel × M 反向（固件会先按加速度刹到 0 再反向）
        setStatus(`${speedRpm} RPM × M${multiplier}（${modeLabel}）急刹反向...`);
        let peak = 0;
        let peakAt = 0;
        try {
            await writeRegisterAndWaitAck(0x11C, Math.min(12000, Math.round(config.baseAccelRpmS * multiplier)));
            await writeRegisterAndWaitAck(0x11E, 3);
        } catch (error) {
            // 写请求超时会直接抛到这里。原来没有日志，表现为“反转刚起步就结束了”。
            log(`[冲击] ${speedRpm}RPM × M${multiplier}（${modeLabel}）切换反向失败：${error.message}`);
            throw error;
        }
        const reverseStart = performance.now();

        // 冲击段采样：找峰值 + 恢复时间
        // 【窗口必须覆盖完整换向周期】从 +target 到 -target 要经过：
        //   ① 刹停（target/accel 秒）
        //   ② 反向加速到 target（再 target/accel 秒）
        //   ③ 反向匀速（turnsPerSegment 圈）
        // 原来只算了 ①+半段 ②（shockSeconds×1000+800），导致反转刚起速就被停机，
        // 实测反向只走了 0.3 圈就出结果。
        const shockSeconds = speedRpm / Math.max(1, accel);
        const reverseCruiseMs = (config.turnsPerSegment * 60000) / Math.max(60, speedRpm);
        const shockWindowMs = Math.max(1500, shockSeconds * 2000 + reverseCruiseMs + 800);
        const shockDeadline = reverseStart + shockWindowMs;
        while (performance.now() < shockDeadline) {
            if (SHOCK.cancelled) return null;
            const err = Math.abs(Number(state.motionFollowErrorDeg || 0));
            if (err > peak) {
                peak = err;
                peakAt = performance.now();
            }
            await delay(10);
        }
        // 【诊断】把实际时间线和转速打出来：如果“窗口”远小于预期，或者结束时转速还是 0，
        // 就能区分是窗口算错、还是固件没真的换向。
        log(`[冲击] ${speedRpm}RPM × M${multiplier}（${modeLabel}）冲击窗口 ${Math.round(performance.now() - reverseStart)}ms`
            + `（预期 ${Math.round(shockWindowMs)}ms），峰值 ${peak.toFixed(2)}°，结束时转速 ${Number(state.motionSpeedRpm || 0).toFixed(1)} RPM`);

        // 恢复时间：峰值之后首次回落到 steadyRms × 1.2 的时间差
        // 由于采样窗口已经过去，这里用“峰值出现在窗口的位置”做近似：
        // 真正精确的恢复时间需要连续记录，下一版再改成数组回放。
        const recoverMs = peak > 0 && steadyRms > 0 && peak <= steadyRms * 1.2
            ? 0
            : Math.max(0, performance.now() - peakAt);

        // 停止：0x11F=0 只清 motion_running_，如果当时正在换向（RAMP_STAGE_ACCEL）
        // 或速度模式的目标速度还没清，可能出现“停了又自己转”。
        // 所以：先停 → 复位方向 0x11E=2 → 等实际转速掉到 20RPM 以下（最多 3 秒）
        // → 再补一次停止，确保真的停下。
        await writeRegisterAndWaitAck(0x11F, 0);
        await writeRegisterAndWaitAck(0x11E, 2);
        const stopDeadline = performance.now() + 3000;
        while (performance.now() < stopDeadline) {
            if (Math.abs(Number(state.motionSpeedRpm || 0)) <= 20) break;
            await delay(80);
        }
        await writeRegisterAndWaitAck(0x11F, 0);
        await ampWaitWhileRunning(400);

        return {
            speedRpm,
            multiplier,
            mode: modeLabel,
            peakDeg: peak,
            recoverMs,
            steadyRmsDeg: steadyRms
        };
    }

    async function startShockCalibration() {
        if (SHOCK.running) return;
        if (!state.connected) {
            setStatus('尚未连接串口');
            return;
        }
        // 外部跟随模式下自动接管运动通道（同滞后角标定：否则冲击运动被固件互斥门拒）。
        if (!(await ensureInternalMotionSource('冲击标定'))) {
            setStatus('运动通道接管失败（见日志），未启动');
            return;
        }
        const config = readConfig();
        const modes = [];
        if (config.testOpenLoop) modes.push(false);
        if (config.testClosedLoop) modes.push(true);
        if (modes.length === 0) {
            setStatus('请至少勾选开环或闭环');
            return;
        }
        // 【共振带自动避让】默认速度列表 [300,600,1200] 正好压在默认共振带 300~600 上：
        // 落带内的组合会被固件钳到带下限巡航，峰值冲击数据归属到假速度上。剔除。
        config.speeds = await excludeResonanceBandSpeeds(config.speeds, '冲击标定');
        if (config.speeds.length === 0) {
            setStatus('速度全部落在共振带内，请调整速度列表或共振带范围');
            return;
        }

        SHOCK.running = true;
        SHOCK.cancelled = false;
        SHOCK.rows = [];
        renderRows();
        const startBtn = $('startShockCalibrationBtn');
        const cancelBtn = $('cancelShockCalibrationBtn');
        if (startBtn) startBtn.disabled = true;
        if (cancelBtn) cancelBtn.disabled = false;

        const total = config.speeds.length * config.multipliers.length * modes.length;
        let done = 0;
        try {
            // 【安全锁】固件在用户点过「失能」后会拒绝所有 0x11F=1（日志 [SAFETY]）。
            // 用户点「开始标定」本身就是明确要让电机转的意图，所以这里先显式使能一次。
            // 注意：只在这里（用户主动开始）做，测试循环内部不再重复使能，
            // 所以中途点「失能」依然会立即生效并中止后续全部组合。
            await writeRegisterAndWaitAck(0x124, 1);   // MOTOR_ENABLE
            // 统一准备：无限行程 + 速度模式 + 步宽
            await writeRegisterAndWaitAck(0x123, 200);
            for (const speedRpm of config.speeds) {
                for (const multiplier of config.multipliers) {
                    for (const closedLoop of modes) {
                        if (SHOCK.cancelled) throw new Error('已停止');
                        const row = await runOneCombination(speedRpm, multiplier, closedLoop, config);
                        if (row) {
                            SHOCK.rows.push(row);
                            renderRows();
                        }
                        done++;
                        setProgress(done, total);
                    }
                }
            }
            setStatus(`完成：${SHOCK.rows.length} 组`);
        } catch (error) {
            setStatus(`中止：${error.message}`);
        } finally {
            try {
                await writeRegisterAndWaitAck(0x11F, 0);
            } catch (e) { /* 忽略：可能已断开 */ }
            SHOCK.running = false;
            if (startBtn) startBtn.disabled = false;
            if (cancelBtn) cancelBtn.disabled = true;
        }
    }

    // PID 面板与主界面 <-> 双向同步，改完即时写固件。
    // 主界面 id：posKp/posKi/posKd/velKp/velKi/velKd
    // 弹窗 id：  shockPosKp/...
    const PID_FIELDS = [
        ['shockPosKp', 'posKp'],
        ['shockPosKi', 'posKi'],
        ['shockPosKd', 'posKd'],
        ['shockVelKp', 'velKp'],
        ['shockVelKi', 'velKi'],
        ['shockVelKd', 'velKd']
    ];

    // 主界面 -> 弹窗（打开弹窗时调用；也用于外部改动后回填）
    function syncPidFromMain() {
        for (const [shockId, mainId] of PID_FIELDS) {
            const s = $(shockId);
            const m = $(mainId);
            if (s && m) s.value = m.value;
        }
    }

    // 弹窗 -> 主界面 -> 固件（即时）
    async function syncPidToMain(changedId) {
        for (const [shockId, mainId] of PID_FIELDS) {
            if (shockId === changedId) {
                const s = $(shockId);
                const m = $(mainId);
                if (s && m) m.value = s.value;
            }
        }
        if (typeof writePidParameters === 'function') {
            try {
                await writePidParameters();
            } catch (error) {
                log(`[PID] 写入失败：${error.message}`);
            }
        }
    }

    function bindShockPidPanel() {
        for (const [shockId] of PID_FIELDS) {
            const s = $(shockId);
            if (!s) continue;
            s.addEventListener('change', function () { void syncPidToMain(shockId); });
        }
        // 主界面改动后回填（弹窗开着时保持两边一致）
        for (const [, mainId] of PID_FIELDS) {
            const m = $(mainId);
            if (!m) continue;
            m.addEventListener('change', function () {
                const dialog = $('shockDialog');
                if (dialog && dialog.open) syncPidFromMain();
            });
        }
    }

    // 写入参数到 Flash（0x154）。固件会把当前 PID + 角度限速存进 0x0801E000，
    // 下次上电时 init() 自动读回。运动中固件会拒绝（会 stall Flash 总线丢脉冲）。
    async function saveParamsToFlash() {
        if (!state.connected) {
            setStatus('尚未连接串口');
            return;
        }
        const btn = $('saveParamsBtn');
        if (btn) btn.disabled = true;
        try {
            // 先把面板上的 PID 写进固件 RAM，再让它保存，避免存的是旧值
            if (typeof writePidParameters === 'function') {
                await writePidParameters();
            }
            await writeRegisterAndWaitAck(0x154, 1);
            const result = await readRegisterValue(0x154);
            if (Number(result) === 1) {
                log('[PARAM] 参数已写入 Flash，下次上电自动恢复');
                setStatus('参数已保存到 Flash');
            } else {
                log(`[PARAM] 保存失败（0x154 读回 ${result}），运动中会被拒绝，静止后再试`);
                setStatus('保存失败：请确保电机静止');
            }
        } catch (error) {
            log(`[PARAM] 写入异常：${error.message}`);
            setStatus(`保存失败：${error.message}`);
        } finally {
            if (btn) btn.disabled = false;
        }
    }

    // 主界面也有一个「写入 PID 参数到 Flash」按钮（与弹窗内那个共用同一套逻辑）。
    // 弹窗未打开时也要能用，所以这里单独绑定，不依赖 bindShockUi 的弹窗存在性检查。
    async function saveParamsFromMainPanel() {
        if (!state.connected) {
            const s = $('savePidToFlashStatus');
            if (s) s.textContent = '尚未连接串口';
            return;
        }
        const btn = $('savePidToFlashBtn');
        const status = $('savePidToFlashStatus');
        if (btn) btn.disabled = true;
        if (status) status.textContent = '写入中...';
        try {
            if (typeof writePidParameters === 'function') {
                await writePidParameters();
            }
            await writeRegisterAndWaitAck(0x154, 1);
            const result = await readRegisterValue(0x154);
            if (Number(result) === 1) {
                log('[PARAM] PID 已写入 Flash，下次上电自动恢复');
                if (status) status.textContent = '已保存，上电自动恢复';
            } else {
                log(`[PARAM] 保存失败（0x154 读回 ${result}），运动中会被拒绝，静止后再试`);
                if (status) status.textContent = '保存失败：请确保电机静止';
            }
        } catch (error) {
            log(`[PARAM] 写入异常：${error.message}`);
            if (status) status.textContent = `保存失败：${error.message}`;
        } finally {
            if (btn) btn.disabled = false;
        }
    }

    function bindShockUi() {
        const openBtn = $('shockCalibrationBtn');
        const dialog = $('shockDialog');
        if (!openBtn || !dialog) return;
        openBtn.addEventListener('click', function () {
            syncPidFromMain();
            if (!dialog.open) dialog.showModal();
        });
        const closeBtn = $('closeShockDialogBtn');
        if (closeBtn) {
            closeBtn.addEventListener('click', function () {
                if (dialog.open) dialog.close();
            });
        }
        const startBtn = $('startShockCalibrationBtn');
        if (startBtn) startBtn.addEventListener('click', startShockCalibration);
        bindShockPidPanel();
        const cancelBtn = $('cancelShockCalibrationBtn');
        const saveBtn = $('saveParamsBtn');
        if (saveBtn) saveBtn.addEventListener('click', function () { void saveParamsToFlash(); });
        const saveMainBtn = $('savePidToFlashBtn');
        if (saveMainBtn) saveMainBtn.addEventListener('click', function () { void saveParamsFromMainPanel(); });
        if (cancelBtn) {
            cancelBtn.addEventListener('click', function () {
                SHOCK.cancelled = true;
                setStatus('正在停止...');
            });
        }
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', bindShockUi);
    } else {
        bindShockUi();
    }

    // 暴露给控制台调试
    window.__shock = SHOCK;
})();
