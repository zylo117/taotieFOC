// ===========================================================================
// 共振区间自动检测（网页版 v2，2026-11-11 指标重做）
//
// 【v1 为什么错（用户实机反馈 + 数据复核）】
//  v1 用「编码器角度的去趋势残差 σ」当纹波指标，两个系统性错误：
//   A. 假阳性：角度的时间戳来自 JS 轮询（~100Hz 遥测 × 轮询相位抖动 ±1.3ms）。
//      匀速段角度速率 2160°/s(60RPM) ~ 23400°/s(650RPM)，纯抖动就能造出
//      σ=2.8°/11.5° —— 与首轮实测（基线 2.76°、250RPM 11.48°）吻合：
//      量到的主要是采样抖动，σ 随速度线性膨胀，250RPM 平滑也被判 4.16x。
//   B. 假阴性：转子打滑/转不动时角度近似静止，去趋势把斜坡当速度减掉，
//      残差反而极小 → 被判「平静」。**斜率才是失步证据，v1 把它扔了。**
//
// 【v2 指标：跟随误差 FE(0x121) = 编码器行程 - 命令行程（±180° 回绕，已解卷绕）】
//   * 匀速跟随：FE ≈ 常数滞后。值对时间的一阶导≈0 ⇒ 同样的时间戳抖动
//     **不再产生残差**（误差 = 斜率×δt，斜率≈0 则残差≈0）。
//   * 共振：转子绕命令位置大幅摆动，FE 如实放大 —— σ/pp 就是用户肉眼看到的抖幅。
//   * 失步/打滑：编码器行程跟不上命令行程，FE 以 (实际-命令)速率持续爬升，
//     |斜率| > 25% 命令速率 = 失步证据（即使没触发堵转锁存也逃不掉）；
//     失步点在分带里按「热」证据处理（同堵转），绝不再算平静。
//   * 双保险：角度斜率估计转速 <70% 命令、或 0x120 实测中位 <60% 命令，
//     同样判失步（三个独立观测量，任何一个露馅都算）。
//
// 【分带常数沿用脚本标定值】ratio=FEσ/基线FEσ：连续 2 点 ≥3× = 带起（回看一点），
//  之后 ≤1.6× 且离起点>1 步 = 带止（该点前一步）；堵转/失步点直接算 2 个热计数。
// 【安全】开扫显式确认、自动接管内部模式、0x171 旁路 finally 必恢复、堵转清锁存
//  跳邻域连炸 3 次中止、非模态 dialog 保急停可点、急停/缓停联动中止、
//  结果两段式（点「应用」才写 0x16F/0x170）。
// ===========================================================================
(function () {
    const START_RATIO = 3.0;   // 连续 2 点 FEσ ≥ 3× 基线 => 共振开始
    const END_RATIO = 1.6;     // 之后连续回落到 ≤ 1.6× => 共振结束
    const MAX_STALLS = 3;      // 堵转次数上限（失步不算堵转，只有真堵转锁存算）
    const BASE_RPM = 60;       // 基线转速
    const SLIP_SLOPE_FRAC = 0.25;   // |FE斜率| > 25% 命令速率 = 失步
    const SLIP_EST_FRAC = 0.70;     // 角度估速 < 70% 命令 = 失步
    const SLIP_MED_FRAC = 0.60;     // 0x120 中位 < 60% 命令 = 失步

    const REG = {
        START_RPM: 0x11A, MAX_RPM: 0x11B, ACCEL: 0x11C, PULSE_COUNT: 0x11D,
        MODE: 0x11E, MOTION: 0x11F, ENABLE: 0x124, DISABLE: 0x125,
        QUICK_STOP: 0x163, STALL_DETECTED: 0x15D, STALL_CLEAR: 0x15F,
        BYPASS: 0x171, BAND_LO: 0x16F, BAND_HI: 0x170
    };
    const MODE_VELOCITY_FORWARD = 2;

    const RS = { active: false, cancel: false, lastBand: null };

    function $(id) { return document.getElementById(id); }
    function delay(ms) { return new Promise((r) => setTimeout(r, Math.max(0, ms))); }

    function setStatus(text) { const el = $('resonanceScanStatus'); if (el) el.textContent = text; }
    function setProgress(text) { const el = $('resonanceScanProgress'); if (el) el.textContent = text; }
    function rsLog(msg) {
        const el = $('resonanceScanLog');
        if (el) {
            el.textContent += `[${(performance.now() / 1000).toFixed(1)}s] ${msg}\n`;
            el.parentElement.scrollTop = el.parentElement.scrollHeight;
        }
        try { log('[共振扫描] ' + msg); } catch (e) { /* 主日志不可用时忽略 */ }
    }

    // ---- 采样：角度(算估计转速) 与 FE(算纹波) 同步采，±180° 解卷绕 ----
    // 时间戳仍来自轮询 —— 但 FE 指标对时间抖动免疫（匀速段 FE 斜率≈0），
    // 这正是 v2 换指标的全部意义，不要再改回角度残差。
    async function captureWindow(seconds) {
        const ang = [];
        const fe = [];
        const speeds = [];
        const t0 = performance.now();
        let prevA = null;
        let unwA = null;
        let prevF = null;
        let unwF = null;
        while (performance.now() - t0 < seconds * 1000) {
            if (RS.cancel) break;
            const t = (performance.now() - t0) / 1000;
            const a = state.encoderAngleDeg;
            if (typeof a === 'number' && Number.isFinite(a)) {
                if (prevA === null) { unwA = a; }
                else {
                    let d = a - prevA;
                    while (d > 180) d -= 360;
                    while (d < -180) d += 360;
                    unwA += d;
                }
                prevA = a;
                ang.push([t, unwA]);
            }
            const f = state.motionFollowErrorDeg;
            if (typeof f === 'number' && Number.isFinite(f)) {
                if (prevF === null) { unwF = f; }
                else {
                    let d = f - prevF;
                    while (d > 180) d -= 360;
                    while (d < -180) d += 360;
                    unwF += d;
                }
                prevF = f;
                fe.push([t, unwF]);
            }
            speeds.push(Math.abs(Number(state.motionSpeedRpm || 0)));
            await delay(10);
        }
        return { ang, fe, speeds };
    }

    // 去线性趋势：返回 {σ, pp, slope}。slope 是关键信息，绝不许再扔。
    function detrend(rows) {
        if (rows.length < 10) return { std: NaN, pp: NaN, slope: NaN };
        const trim = Math.max(1, Math.floor(rows.length / 10));
        const core = rows.slice(trim, rows.length - trim);
        const pts = core.length >= 8 ? core : rows;
        const n = pts.length;
        const mt = pts.reduce((s, r) => s + r[0], 0) / n;
        const ma = pts.reduce((s, r) => s + r[1], 0) / n;
        let cov = 0;
        let varT = 0;
        for (const r of pts) { cov += (r[0] - mt) * (r[1] - ma); varT += (r[0] - mt) * (r[0] - mt); }
        const slope = varT ? cov / varT : 0;
        let sum2 = 0;
        let min = Infinity;
        let max = -Infinity;
        for (const r of pts) {
            const e = r[1] - (ma + slope * (r[0] - mt));
            sum2 += e * e;
            if (e < min) min = e;
            if (e > max) max = e;
        }
        return { std: Math.sqrt(sum2 / n), pp: max - min, slope };
    }

    async function stallActive() {
        if (state.stallDetected) return true;
        try {
            const v = await readRegisterValue(REG.STALL_DETECTED, 750, 0);
            return ((v | 0) & 1) !== 0;
        } catch (e) { return false; }
    }

    // 单点：速度模式巡航 → 等稳态 → 采样 → 缓停/堵转处理。
    async function runPoint(rpm, accel, windowS) {
        await writeRegisterAndWaitAck(REG.START_RPM, Math.max(10, Math.min(rpm, 120)));
        await writeRegisterAndWaitAck(REG.MAX_RPM, rpm);
        await writeRegisterAndWaitAck(REG.ACCEL, accel);
        await writeRegisterAndWaitAck(REG.PULSE_COUNT, 0);          // 0 = 无限距离巡航
        await writeRegisterAndWaitAck(REG.MODE, MODE_VELOCITY_FORWARD);
        await writeRegisterAndWaitAck(REG.MOTION, 1);

        const tolerance = Math.max(15, rpm * 0.15);
        const settleDeadline = performance.now() + Math.min(10000, 2000 + (rpm / accel) * 2000);
        while (performance.now() < settleDeadline) {
            if (RS.cancel) break;
            if (await stallActive()) break;
            const actual = Math.abs(Number(state.motionSpeedRpm || 0));
            if (actual > 0 && Math.abs(actual - rpm) <= tolerance) break;
            await delay(100);
        }

        const win = await captureWindow(windowS);
        const stalled = await stallActive();
        if (stalled) {
            await writeRegister(REG.MOTION, 0);
            await writeRegister(REG.STALL_CLEAR, 1);
            await delay(200);
        } else {
            await writeRegister(REG.QUICK_STOP, 1);
            await delay(300 + Math.round((rpm / accel) * 1000));
        }
        const fe = detrend(win.fe);
        const ang = detrend(win.ang);
        const estRpm = ang.slope / 6.0;                 // °/s → RPM
        const sorted = win.speeds.slice().sort((x, y) => x - y);
        const medSpeed = sorted.length ? sorted[Math.floor(sorted.length / 2)] : NaN;
        // 失步判定（三个独立观测量，任一露馅即失步）：
        //   FE 斜率：编码器行程没跟上命令行程（转不动/打滑的最直接信号）
        //   角度估速：直接从解卷绕角度算实测转速
        //   0x120 中位：固件 20ms 平滑的编码器实测转速
        const cmdDegPerS = rpm * 6.0;
        const slip = !stalled && (
            (Number.isFinite(fe.slope) && Math.abs(fe.slope) > cmdDegPerS * SLIP_SLOPE_FRAC) ||
            (Number.isFinite(estRpm) && estRpm < rpm * SLIP_EST_FRAC) ||
            (Number.isFinite(medSpeed) && medSpeed < rpm * SLIP_MED_FRAC));
        return { fe, estRpm, medSpeed, stalled, slip, samples: win.fe.length };
    }

    // ---- 分带（v2）：堵转/失步点直接算 2 个热计数且不被 ratio NaN 清零；
    //      平稳热态：连续 2 点 ratio ≥3×，起点回看一点；终点：ratio ≤1.6× 且距起点>1步。
    function extractBand(points, step) {
        let bandLo = null;
        let bandHi = null;
        let hot = 0;
        for (let i = 0; i < points.length; i++) {
            const p = points[i];
            if (p.hotEvidence) {
                hot = Math.max(hot, 2);
            } else if (Number.isFinite(p.ratio) && p.ratio >= START_RATIO) {
                hot += 1;
            } else {
                hot = 0;
            }
            if (hot >= 2 && bandLo === null) bandLo = i > 0 ? points[i - 1].rpm : p.rpm;
        }
        if (bandLo !== null) {
            for (const p of points) {
                if (p.rpm < bandLo) continue;
                if (p.hotEvidence) { bandHi = p.rpm; continue; }
                if (Number.isFinite(p.ratio) && p.ratio <= END_RATIO && p.rpm > bandLo + step) {
                    bandHi = p.rpm - step;
                    break;
                }
            }
            if (bandHi === null && points.length) {
                const last = points[points.length - 1];
                if (last.hotEvidence || (Number.isFinite(last.ratio) && last.ratio > END_RATIO)) {
                    bandHi = last.rpm;
                }
            }
        }
        if (bandLo !== null && (bandHi === null || bandHi <= bandLo)) bandHi = bandLo + step;
        return { lo: bandLo, hi: bandHi };
    }

    // ---- 主流程 ----
    async function runScan(cfg) {
        RS.active = true;
        RS.cancel = false;
        RS.lastBand = null;
        state.resonanceScanActive = true;
        let stalls = 0;
        const results = [];
        // 共振带自 2026-11-12 起为纯信息标记，固件不再钳位任何巡航，
        // 扫描无需 0x171 旁路。
        $('resonanceScanAbortBtn').disabled = false;
        $('resonanceScanApplyBtn').disabled = true;
        $('resonanceScanStartBtn').disabled = true;
        try {
            await writeRegisterAndWaitAck(CUSTOM_REGS.CLOSED_LOOP_ENABLE, 0);
            const cb = $('closedLoopEnableInput');
            if (cb) cb.checked = false;
            await writeRegister(REG.STALL_CLEAR, 1);
            await writeRegister(REG.ENABLE, 1);
            rsLog(`扫描 ${cfg.lo}→${cfg.hi} RPM，步长 ${cfg.step}，每点 ${cfg.windowS}s，`
                + `基线 ${BASE_RPM} RPM，加速度 ${cfg.accel} RPM/s（指标=跟随误差残差，v2）`);

            setStatus('采基线…');
            const base = await runPoint(BASE_RPM, cfg.accel, cfg.windowS);
            if (base.stalled || base.slip || !Number.isFinite(base.fe.std) || base.fe.std <= 0) {
                rsLog(`!! 基线无效（堵转=${base.stalled} 失步=${base.slip} σ=${base.fe.std}）`
                    + ' —— 低速都跑不稳/跟不上，先解决基础问题再测共振');
                setStatus('基线无效，已中止');
                return null;
            }
            rsLog(`基线 ${BASE_RPM}RPM：FEσ=${base.fe.std.toFixed(4)}° pp=${base.fe.pp.toFixed(3)}° `
                + `估速=${base.estRpm.toFixed(0)} 实测中位=${base.medSpeed.toFixed(0)}（${base.samples} 样本）`);

            const sweep = [];
            for (let rpm = cfg.lo; rpm <= cfg.hi; rpm += cfg.step) sweep.push(rpm);
            let skipUntil = -1;
            let done = 0;
            for (const rpm of sweep) {
                if (RS.cancel) { rsLog('用户中止'); break; }
                if (rpm <= skipUntil) {
                    rsLog(`${rpm} RPM 跳过（堵转禁区邻近）`);
                    done += 1;
                    setProgress(`${done} / ${sweep.length}`);
                    continue;
                }
                setStatus(`测 ${rpm} RPM…`);
                setProgress(`${done} / ${sweep.length}`);
                const p = await runPoint(rpm, cfg.accel, cfg.windowS);
                const ratio = (Number.isFinite(p.fe.std) && p.fe.std >= 0) ? p.fe.std / base.fe.std : NaN;
                results.push({ rpm, ratio, fe: p.fe, estRpm: p.estRpm, hotEvidence: p.stalled || p.slip });
                if (p.stalled) {
                    stalls += 1;
                    skipUntil = rpm + Math.max(2 * cfg.step, 50);
                    rsLog(`${rpm} RPM 堵转 #${stalls} —— 已清锁存，跳过 ≤${skipUntil} RPM 邻近点`);
                    if (stalls >= MAX_STALLS) {
                        rsLog(`堵转次数达到 ${MAX_STALLS}，中止扫描，用已有数据出结论`);
                        done += 1;
                        break;
                    }
                } else {
                    const verdict = p.slip ? '失步/打滑(热证据)'
                        : (ratio >= START_RATIO ? '剧烈' : (ratio > END_RATIO ? '偏高' : '平静'));
                    rsLog(`${rpm} RPM：FEσ=${Number.isFinite(p.fe.std) ? p.fe.std.toFixed(4) : 'nan'}° `
                        + `pp=${Number.isFinite(p.fe.pp) ? p.fe.pp.toFixed(3) : 'nan'}° `
                        + `(${Number.isFinite(ratio) ? ratio.toFixed(2) : 'nan'}x) `
                        + `估速=${Number.isFinite(p.estRpm) ? p.estRpm.toFixed(0) : 'nan'} `
                        + `FE斜率=${Number.isFinite(p.fe.slope) ? p.fe.slope.toFixed(0) : 'nan'}°/s ${verdict}`);
                }
                done += 1;
                setProgress(`${done} / ${sweep.length}`);
                await delay(150);
            }

            const band = extractBand(results, cfg.step);
            if (band.lo !== null) {
                rsLog(`== 检测到共振区间约 [${band.lo}, ${band.hi}] RPM（固件现值 [${cfg.bandLo}, ${cfg.bandHi}]）==`);
                rsLog('点「应用为共振带」写入 0x16F/0x170；「写入参数到 Flash」才会掉电保持');
                setStatus('检测完成');
                RS.lastBand = { lo: band.lo, hi: band.hi };
                $('resonanceScanApplyBtn').disabled = false;
                return band;
            }
            const ratios = results.filter((r) => Number.isFinite(r.ratio)).map((r) => r.ratio);
            rsLog(`本次未发现 ≥${START_RATIO}x 热点或失步段（最大 ${ratios.length ? Math.max(...ratios).toFixed(2) : 'nan'}x）；`
                + '保持现设置，或加严阈值/加密步长再扫');
            setStatus('未检出共振');
            return null;
        } catch (e) {
            rsLog(`异常中止：${e && e.message ? e.message : e}`);
            setStatus('异常中止');
            return null;
        } finally {
            try { await writeRegister(REG.MOTION, 0); } catch (e) { /* 尽力 */ }
            try { await writeRegister(REG.DISABLE, 1); } catch (e) { /* 尽力 */ }
            RS.active = false;
            state.resonanceScanActive = false;
            $('resonanceScanAbortBtn').disabled = true;
            $('resonanceScanStartBtn').disabled = false;
            setProgress(`${results.length} 点，堵转 ${stalls} 次`);
        }
    }

    // ---- UI ----
    function openDialog() {
        if (!state.connected) { log('[WARN] 未连接串口，无法共振自动检测'); return; }
        if (RS.active) { $('resonanceScanDialog').show(); return; }
        const lo = Math.round(Number($('resonanceLoRpmInput').value) || 300);
        const hi = Math.round(Number($('resonanceHiRpmInput').value) || 600);
        $('resonanceScanLoRpm').value = String(Math.max(20, lo - 50));
        $('resonanceScanHiRpm').value = String(Math.min(6000, Math.max(hi, lo + 100) + 50));
        $('resonanceScanLog').textContent = '';
        setStatus('等待开始');
        setProgress('0 / 0');
        $('resonanceScanDialog').show();   // 非模态：急停钮必须始终可点
    }

    async function onStart() {
        if (RS.active) return;
        if (!state.connected) { rsLog('未连接串口'); return; }
        if (!(await ensureInternalMotionSource('共振自动检测'))) {
            rsLog('运动通道接管失败（见日志），未启动扫描');
            return;
        }
        const cfg = {
            lo: Math.round(Number($('resonanceScanLoRpm').value) || 250),
            hi: Math.round(Number($('resonanceScanHiRpm').value) || 650),
            step: Math.max(5, Math.round(Number($('resonanceScanStepRpm').value) || 25)),
            windowS: Math.min(5, Math.max(0.5, Number($('resonanceScanWindowS').value) || 1.2)),
            accel: Math.max(100, Math.round(Number($('resonanceScanAccelRpmS').value) || 1500)),
            bandLo: Number($('resonanceLoRpmInput').value) | 0,
            bandHi: Number($('resonanceHiRpmInput').value) | 0
        };
        cfg.lo = Math.max(20, cfg.lo);
        cfg.hi = Math.min(6000, Math.max(cfg.lo + cfg.step, cfg.hi));
        if (!window.confirm('共振检测会让电机在全速度域自行运转，带内会真实抖动，'
            + '并可能触发堵转/失步（失步段会按热点处理）。\n\n请确认：电机空载、固定在位、周围无人无物。继续？')) {
            rsLog('用户取消，未启动');
            return;
        }
        await runScan(cfg);
    }

    function onApply() {
        const band = RS.lastBand;
        if (!band || !state.connected) return;
        (async () => {
            try {
                await writeRegisterAndWaitAck(REG.BAND_LO, band.lo);
                await writeRegisterAndWaitAck(REG.BAND_HI, band.hi);
                $('resonanceLoRpmInput').value = String(band.lo);
                $('resonanceHiRpmInput').value = String(band.hi);
                rsLog(`已应用共振带 [${band.lo}, ${band.hi}] RPM（0x16F/0x170 回读一致）`
                    + ' —— 记得「写入参数到 Flash」才掉电保持');
                $('resonanceScanApplyBtn').disabled = true;
            } catch (e) {
                rsLog(`应用失败：${e.message}`);
            }
        })();
    }

    // 急停/缓停联动（actions.js 调）：置 cancel，扫描循环在测点边界收尾，清理走 finally
    window.rsAbortScan = function () {
        if (RS.active) {
            RS.cancel = true;
            rsLog('急停/停止触发，扫描中止');
        }
    };

    function initBindings() {
        if (window.__rsBound) return;
        window.__rsBound = true;
        const bind = (id, ev, fn) => { const el = $(id); if (el) el.addEventListener(ev, fn); };
        bind('resonanceScanBtn', 'click', openDialog);
        bind('resonanceScanStartBtn', 'click', () => { onStart(); });
        bind('resonanceScanAbortBtn', 'click', () => { RS.cancel = true; });
        bind('resonanceScanApplyBtn', 'click', onApply);
        bind('resonanceScanCloseBtn', 'click', () => {
            if (RS.active) { RS.cancel = true; rsLog('已请求中止，等当前测点收尾…'); }
            $('resonanceScanDialog').close();
        });
    }
    document.addEventListener('DOMContentLoaded', initBindings);
    initBindings();
})();
