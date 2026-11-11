// ===========================================================================
// 共振区间自动检测（网页版，移植自 scripts/tt_resonance.py，2026-11-11）
//
// 判据与脚本完全一致 —— 两处实测标定的经验常数不要随手改：
//   1) 60 RPM 超低速采一段编码器角度，去线性趋势后算"纹波"（σ 与峰峰值）。
//      匀速段角度≈直线，共振让直线变成毛刷，去趋势就是把这些毛毛量出来。
//   2) 从 [带下沿-边距] 到 [带上沿+边距] 逐点升速，每点等稳态采同样窗口。
//   3) 共振的特征是**突然开始剧烈抖动、突然结束**：比例阈值找跳变 ——
//      连续 2 点 ratio ≥ 3×基线 = 带起点；之后回落 ≤ 1.6× 连续 2 点 = 带终点。
//   4) 剧烈共振大概率触发堵转：每点盯 0x15D/0x108(bit3)，堵转即停车、清锁存、
//      把该速 ±步长 邻域标记为禁区跳过；整轮连炸 3 次带结论中止。
// 安全措施（同脚本，一条不能少）：
//   * 开扫前显式确认（现场必须空载）；自动接管运动通道（ensureInternalMotionSource）。
//   * 扫描期间 0x171=1 旁路固件共振钳位（否则带内目标被钳到下限，测不到共振本身），
//     **无论正常结束/中止/异常都恢复 0x171=0**，失败要红字提醒手动恢复。
//   * 每点测完先缓停（0x163）再升速，不来回猛冲。
//   * 检测出的区间不自动生效：用户点「应用为共振带」才写 0x16F/0x170。
//   * 结束留在内部规划模式并失能驱动器（同 tt_autotune 约定）。
// 与脚本的差异只在数据通路：脚本经 tty 读 0x116 的最新遥测值；网页直接读
// state.encoderAngleDeg —— 它由 rx.js 从 ~100Hz 遥测帧持续刷新（见 rx.js 的
// ENCODER_ANGLE_MDEG 分支），10ms 轮询能拿到和脚本同量级的样本密度。
// ===========================================================================
(function () {
    const START_RATIO = 3.0;   // 连续 2 点纹波 ≥ 3× 基线 => 共振开始
    const END_RATIO = 1.6;     // 之后连续 2 点回落到 ≤ 1.6× => 共振结束
    const MAX_STALLS = 3;      // 整轮堵转超过 3 次：带结论中止
    const BASE_RPM = 60;       // 基线转速（超低速，稳、离共振远）

    const REG = {
        START_RPM: 0x11A, MAX_RPM: 0x11B, ACCEL: 0x11C, PULSE_COUNT: 0x11D,
        MODE: 0x11E, MOTION: 0x11F, ENABLE: 0x124, DISABLE: 0x125,
        QUICK_STOP: 0x163, STALL_DETECTED: 0x15D, STALL_CLEAR: 0x15F,
        BYPASS: 0x171, BAND_LO: 0x16F, BAND_HI: 0x170
    };
    const MODE_VELOCITY_FORWARD = 2;

    const RS = { active: false, cancel: false };

    function $(id) { return document.getElementById(id); }
    function delay(ms) { return new Promise((r) => setTimeout(r, Math.max(0, ms))); }

    function setStatus(text) {
        const el = $('resonanceScanStatus');
        if (el) el.textContent = text;
    }
    function setProgress(text) {
        const el = $('resonanceScanProgress');
        if (el) el.textContent = text;
    }
    // 扫描日志：对话框内一份（带时间戳），主日志一份（进用户存档）
    function rsLog(msg) {
        const el = $('resonanceScanLog');
        if (el) {
            const t = (performance.now() / 1000).toFixed(1);
            el.textContent += `[${t}s] ${msg}\n`;
            el.parentElement.scrollTop = el.parentElement.scrollHeight;
        }
        try { log('[共振扫描] ' + msg); } catch (e) { /* 主日志不可用时忽略 */ }
    }

    // ---- 采样：10ms 轮询 state.encoderAngleDeg，环绕解卷绕（±180° 回折） ----
    async function captureUnwrapped(seconds) {
        const rows = [];
        const t0 = performance.now();
        let unwrapped = null;
        let prev = null;
        while (performance.now() - t0 < seconds * 1000) {
            if (RS.cancel) break;
            const deg = state.encoderAngleDeg;
            if (typeof deg === 'number' && Number.isFinite(deg)) {
                if (prev === null) {
                    unwrapped = deg;
                } else {
                    let d = deg - prev;
                    while (d > 180) d -= 360;
                    while (d < -180) d += 360;
                    unwrapped += d;
                }
                prev = deg;
                rows.push([(performance.now() - t0) / 1000, unwrapped]);
            }
            await delay(10);
        }
        return rows;
    }

    // 去线性趋势后的 (σ, 峰峰值)：匀速段角度≈直线，残差就是抖动
    function detrendRipple(rows) {
        if (rows.length < 10) return { std: NaN, pp: NaN };
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
        return { std: Math.sqrt(sum2 / n), pp: max - min };
    }

    async function stallActive() {
        if (state.stallDetected) return true;
        try {
            const v = await readRegisterValue(REG.STALL_DETECTED, 750, 0);
            return ((v | 0) & 1) !== 0;
        } catch (e) { return false; }
    }

    // 单点：速度模式无限跑 rpm → 等稳态 → 采窗口 → 缓停。返回 {std, pp, stalled}
    async function runPoint(rpm, accel, windowS) {
        await writeRegisterAndWaitAck(REG.START_RPM, Math.max(10, Math.min(rpm, 120)));
        await writeRegisterAndWaitAck(REG.MAX_RPM, rpm);
        await writeRegisterAndWaitAck(REG.ACCEL, accel);
        await writeRegisterAndWaitAck(REG.PULSE_COUNT, 0);          // 0 = 无限距离巡航
        await writeRegisterAndWaitAck(REG.MODE, MODE_VELOCITY_FORWARD);
        await writeRegisterAndWaitAck(REG.MOTION, 1);

        // 等稳态：|实测-目标| ≤ max(15, 15%)；超时不判失败（同脚本：按时间放行，
        // 真要没到速，纹波比会异常，日志里看得见）。堵转提前终止等待。
        const tolerance = Math.max(15, rpm * 0.15);
        const settleDeadline = performance.now() + Math.min(10000, 2000 + (rpm / accel) * 2000);
        while (performance.now() < settleDeadline) {
            if (RS.cancel) break;
            if (await stallActive()) break;
            const actual = Math.abs(Number(state.motionSpeedRpm || 0));
            if (actual > 0 && Math.abs(actual - rpm) <= tolerance) break;
            await delay(100);
        }

        const rows = await captureUnwrapped(windowS);
        const stalled = await stallActive();
        if (stalled) {
            await writeRegister(REG.MOTION, 0);          // 硬停：堵转态别再做斜坡
            await writeRegister(REG.STALL_CLEAR, 1);
            await delay(200);
        } else {
            await writeRegister(REG.QUICK_STOP, 1);      // 缓停：斜坡收尾
            await delay(300 + Math.round((rpm / accel) * 1000));
        }
        const r = detrendRipple(rows);
        return { std: r.std, pp: r.pp, stalled, samples: rows.length };
    }

    // ---- 分带：逐行移植脚本 239~268 行，语义不许改 ----
    // 骤起：连续 2 点 ratio ≥3x（堵转点直接算满 2 个热计数，比测出来的数更有说服力）；
    //       起点 = 连续两点中的第一个（回看上一条）。
    // 骤落：起点之后，ratio ≤1.6x 且 rpm > lo+step 的第一个点，终点 = 该点前一步；
    //       若扫到尽头仍热/堵，终点 = 最后一点。兜底 hi = lo+step。
    function extractBand(points, step) {
        let bandLo = null;
        let bandHi = null;
        let hot = 0;
        for (let i = 0; i < points.length; i++) {
            const p = points[i];
            if (p.stalled) hot = Math.max(hot, 2);
            if (Number.isFinite(p.ratio) && p.ratio >= START_RATIO) hot += 1;
            else hot = 0;
            if (hot >= 2 && bandLo === null) bandLo = i > 0 ? points[i - 1].rpm : p.rpm;
        }
        if (bandLo !== null) {
            for (const p of points) {
                if (p.rpm < bandLo) continue;
                if (p.stalled) { bandHi = p.rpm; continue; }
                if (Number.isFinite(p.ratio) && p.ratio <= END_RATIO && p.rpm > bandLo + step) {
                    bandHi = p.rpm - step;
                    break;
                }
            }
            if (bandHi === null && points.length) {
                const last = points[points.length - 1];
                if (last.stalled || (Number.isFinite(last.ratio) && last.ratio > END_RATIO)) {
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
        state.resonanceScanActive = true;
        let stalls = 0;
        const results = [];
        let bypassOn = false;
        $('resonanceScanAbortBtn').disabled = false;
        $('resonanceScanApplyBtn').disabled = true;
        $('resonanceScanStartBtn').disabled = true;
        try {
            // 基线口径：开环 + 清堵转 + 使能（外部 EN 模式下 0x124 只是撤销失能锁，
            // 真使能还得引脚低 —— 不使能电机根本不动，扫描会在第一点就报无法稳速）
            await writeRegisterAndWaitAck(CUSTOM_REGS.CLOSED_LOOP_ENABLE, 0);
            const cb = $('closedLoopEnableInput');
            if (cb) cb.checked = false;
            await writeRegister(REG.STALL_CLEAR, 1);
            await writeRegister(REG.ENABLE, 1);
            await writeRegisterAndWaitAck(REG.BYPASS, 1);
            bypassOn = true;
            rsLog(`扫描 ${cfg.lo}→${cfg.hi} RPM，步长 ${cfg.step}，每点 ${cfg.windowS}s，基线 ${BASE_RPM} RPM，加速度 ${cfg.accel} RPM/s`);

            setStatus('采基线…');
            const base = await runPoint(BASE_RPM, cfg.accel, cfg.windowS);
            if (base.stalled || !Number.isFinite(base.std) || base.std <= 0) {
                rsLog('!! 超低速基准都堵转/采不到有效纹波（σ=' + base.std + '）——先解决基础问题（卡滞？电流？）再测共振');
                setStatus('基线无效，已中止');
                return null;
            }
            rsLog(`基线 ${BASE_RPM} RPM：σ=${base.std.toFixed(4)}° 峰峰=${base.pp.toFixed(4)}°（${base.samples} 样本）`);

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
                const ratio = (Number.isFinite(p.std) && p.std >= 0) ? p.std / base.std : NaN;
                results.push({ rpm, ratio, std: p.std, pp: p.pp, stalled: p.stalled });
                if (p.stalled) {
                    stalls += 1;
                    rsLog(`${rpm} RPM 堵转 #${stalls} —— 已清锁存，跳过 ≤${rpm + Math.max(2 * cfg.step, 50)} RPM 的邻近点继续`);
                    skipUntil = rpm + Math.max(2 * cfg.step, 50);
                    if (stalls >= MAX_STALLS) {
                        rsLog(`堵转次数达到 ${MAX_STALLS}，中止扫描，用已有数据出结论`);
                        done += 1;
                        break;
                    }
                } else {
                    const verdict = ratio >= START_RATIO ? '剧烈' : (ratio > END_RATIO ? '偏高' : '平静');
                    rsLog(`${rpm} RPM：σ=${Number.isFinite(p.std) ? p.std.toFixed(4) : 'nan'}° `
                        + `峰峰=${Number.isFinite(p.pp) ? p.pp.toFixed(4) : 'nan'}° `
                        + `(${Number.isFinite(ratio) ? ratio.toFixed(2) : 'nan'}x) ${verdict}`);
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
            rsLog(`本次未发现 ≥${START_RATIO}x 基准的骤发剧烈区间（最大 ${ratios.length ? Math.max(...ratios).toFixed(2) : 'nan'}x）；`
                + `保持现设置，或加严阈值/加密步长再扫`);
            setStatus('未检出共振');
            return null;
        } catch (e) {
            rsLog(`异常中止：${e && e.message ? e.message : e}`);
            setStatus('异常中止');
            return null;
        } finally {
            try { await writeRegister(REG.MOTION, 0); } catch (e) { /* 尽力 */ }
            if (bypassOn) {
                try {
                    await writeRegisterAndWaitAck(REG.BYPASS, 0);
                    rsLog('0x171 钳位旁路已恢复');
                } catch (e) {
                    rsLog('!! 0x171 恢复失败 —— 请手动写 0x171=0，否则固件共振保护不生效！');
                }
            }
            try { await writeRegister(REG.DISABLE, 1); } catch (e) { /* 尽力 */ }
            RS.active = false;
            state.resonanceScanActive = false;
            $('resonanceScanAbortBtn').disabled = true;
            $('resonanceScanStartBtn').disabled = false;
            setProgress(`${results.length} 点，堵转 ${stalls} 次`);
        }
    }

    // ---- UI 绑定 ----
    function openDialog() {
        if (!state.connected) { log('[WARN] 未连接串口，无法共振自动检测'); return; }
        if (RS.active) { $('resonanceScanDialog').show(); return; }
        // 默认扫描窗 = 当前共振带 ±50（同脚本 edge 默认）
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
        if (!window.confirm('共振检测会让电机在全速度域自行运转，带内会剧烈抖动（钳位临时旁路），并可能触发堵转。\n\n'
            + '请确认：电机空载、固定在位、周围无人无物。继续？')) {
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
                rsLog(`已应用共振带 [${band.lo}, ${band.hi}] RPM（0x16F/0x170 回读一致）—— 记得「写入参数到 Flash」才掉电保持`);
                $('resonanceScanApplyBtn').disabled = true;
            } catch (e) {
                rsLog(`应用失败：${e.message}`);
            }
        })();
    }

    // 急停联动：emergencyStop() 会调这个把扫描叫停（清理走 finally）
    window.rsAbortScan = function () {
        if (RS.active) {
            RS.cancel = true;
            rsLog('急停触发，扫描中止');
        }
    };

    // ---- 绑定 ----
    // 本脚本在 </body> 前执行（此时所有控件已在 DOM，readyState 为 interactive），
    // 立即绑定即可；DOMContentLoaded 只是保险（若将来被挪进 <head>）。
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
