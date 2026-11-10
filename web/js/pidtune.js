// PID auto-tune: trial profiles, scoring, geometric search (module 9).
// Depends on: state, CUSTOM_REGS, writeRegister/readRegisterValue, log, writePidParameters.
// Load order: ... -> lagangle -> pidtune -> ...
//
// 【2026-11 重写】目标：和 scripts/tt_autotune.py（串口版）用同一套逻辑，效果对齐。
//   1) 六个工况：慢速/中速/高速/反向/急加速/急转弯往返，全部 <=250RPM（本机 300~800 共振）
//   2) 指标 = 固件评分(0x132) + 0.5*|停稳后残余(0x121)|，既罚抖动也罚"最后没停准"
//   3) 每个工况重复 N 次取平均（单次批间差有 0.1~0.2°）
//   4) 搜索：vel_kp -> pos_kp -> pos_kd -> vel_kd -> pos_ki -> vel_ki 的几何步进坐标下降
//   5) 安全门：最差工况 > 5° 说明是功能问题（方向/丢步/堵转），拒绝写 Flash
//   6) 每个试验：清堵转 -> 使能 -> 开闭环补偿 -> 跑完等停 -> 失能

        const TUNE_MAX_RPM_HARD = 250;      // 本机 300~800RPM 剧烈共振，先低速整定
        const TUNE_SAFETY_LIMIT_DEG = 5.0;  // 任一工况超过它就不是调参能解决的
        const TUNE_GAIN_KEYS = ['posKp', 'posKi', 'posKd', 'velKp', 'velKi', 'velKd'];
        // 搜索顺序：先速度环（抖动/超调主因），再位置环，最后积分项
        const TUNE_SEARCH_ORDER = ['velKp', 'posKp', 'posKd', 'velKd', 'posKi', 'velKi'];
        const TUNE_GAIN_MAX = {posKp: 1000, posKi: 1000, posKd: 100, velKp: 10, velKi: 10, velKd: 1};

        function appendTuneLog(message) {
            const logElement = document.getElementById('tuneLog');
            const line = document.createElement('p');
            line.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
            logElement.appendChild(line);
            while (logElement.children.length > 800) {
                logElement.removeChild(logElement.firstElementChild);
            }
            logElement.scrollTop = logElement.scrollHeight;
        }

        // 六个整定工况 + 一个单独的急刹冲击测试。
        // 工况比例与串口版一致：慢 15%→25%、中 35%→65%、高 50%→100%、急加速/急转弯 14 倍加速度。
        function createPulseTuneProfiles(maxSpeedRpm, accelRpmS, turnsPerDirection, fullRun) {
            const m = Math.max(10, Math.min(TUNE_MAX_RPM_HARD, Number(maxSpeedRpm) || 200));
            const accel = Math.max(60, Number(accelRpmS) || 400);
            const fastAccel = Math.min(4000, Math.max(accel, m * 14));
            const turns = Math.max(1, Math.min(5, Number(turnsPerDirection) || 3));
            const steadyTurns = fullRun ? turns : Math.max(1, turns / 2);
            const profiles = [
                {label: '慢速正向', mode: 0, direction: 0, startRpm: Math.max(10, Math.round(m * 0.15)),
                 maxRpm: Math.max(20, Math.round(m * 0.25)), accelRpmS: Math.max(60, Math.round(m * 1.2)),
                 turns: 1.0, weight: 1.0},
                {label: '中速正向', mode: 0, direction: 0, startRpm: Math.max(20, Math.round(m * 0.35)),
                 maxRpm: Math.max(40, Math.round(m * 0.65)), accelRpmS: Math.max(120, Math.round(m * 2.4)),
                 turns: steadyTurns, weight: 1.0},
                {label: '高速正向', mode: 0, direction: 0, startRpm: Math.max(40, Math.round(m * 0.5)),
                 maxRpm: m, accelRpmS: Math.max(200, Math.round(m * 5)),
                 turns: steadyTurns + 0.5, weight: 1.0},
                {label: '中速反向', mode: 1, direction: 1, startRpm: Math.max(20, Math.round(m * 0.35)),
                 maxRpm: Math.max(40, Math.round(m * 0.65)), accelRpmS: Math.max(120, Math.round(m * 2.4)),
                 turns: steadyTurns, weight: 1.0},
                {label: '急加速', mode: 0, direction: 0, startRpm: Math.max(20, Math.round(m * 0.30)),
                 maxRpm: Math.max(40, Math.round(m * 0.85)), accelRpmS: fastAccel,
                 turns: steadyTurns, weight: 1.0},
                {label: '急转弯往返', mode: 6, direction: 0, startRpm: Math.max(20, Math.round(m * 0.35)),
                 maxRpm: Math.max(40, Math.round(m * 0.75)), accelRpmS: fastAccel,
                 turns: Math.max(2, steadyTurns), weight: 1.0},
                {label: '急刹冲击', mode: 2, direction: 0, startRpm: Math.max(20, Math.round(m * 0.6)),
                 maxRpm: m, accelRpmS: Math.max(200, Math.round(m * 5)), turns: 0.6,
                 weight: 1.0, quickStop: true},
            ];
            if (!fullRun) {
                // 搜索阶段只用有代表性的四个（含反向！反向补偿是独立的一条符号路径）
                const keep = ['中速正向', '高速正向', '中速反向', '急加速'];
                return profiles.filter((p) => keep.includes(p.label));
            }
            return profiles;
        }

        function getTuneActionProfiles() {
            const maxSpeedRpm = clamp(Number(document.getElementById('tuneMaxSpeed').value || 200), 5, TUNE_MAX_RPM_HARD);
            const accelRpmS = clamp(Number(document.getElementById('tuneAccel').value || 400), 10, 3000);
            const turns = clamp(Number(document.getElementById('tuneTestTurns').value || 3), 1, 5);
            return createPulseTuneProfiles(maxSpeedRpm, accelRpmS, turns, true);
        }

        function renderTuneActionRows(reset = true) {
            const container = document.getElementById('tuneActionList');
            if (!container) return;
            state.tuneActionProfiles = getTuneActionProfiles();
            container.replaceChildren();
            state.tuneActionProfiles.forEach((profile, index) => {
                const row = document.createElement('div');
                row.className = 'tune-action-row';
                const desc = profile.quickStop
                    ? `${profile.maxRpm} RPM 恒速 -> 快速停车`
                    : `${profile.direction === 0 ? '正向' : '反向'} · ${profile.startRpm}→${profile.maxRpm} RPM · ${profile.accelRpmS} RPM/s · ${profile.turns} 圈${profile.mode === 6 ? '/程' : ''}`;
                row.innerHTML = `<div class="tune-action-name">${String(index + 1).padStart(2, '0')} · ${profile.label}</div><div class="tune-action-result" id="tuneActionResult${index}">${desc} · 等待执行</div><button class="tune-action-button" id="tuneActionButton${index}" type="button">执行</button>`;
                container.appendChild(row);
                row.querySelector('button').addEventListener('click', () => {
                    void executeTuneAction(index);
                });
            });
            if (!reset) return;
            document.getElementById('tuneResults').textContent = '尚未完成动作评分';
        }

        function setTuneActionState(index, status, result, completed = false) {
            const resultElement = document.getElementById(`tuneActionResult${index}`);
            const button = document.getElementById(`tuneActionButton${index}`);
            if (!resultElement || !button) return;
            resultElement.textContent = `${status} · ${result}`;
            button.disabled = completed || state.autoTuneActive || state.tuneActionBusy;
            button.textContent = completed ? '已完成' : '执行';
        }

        // ---------------- 固件侧的读数 ----------------
        async function readTuneGainsFromHardware() {
            const registers = [
                [CUSTOM_REGS.POSITION_KP, 'posKp'], [CUSTOM_REGS.POSITION_KI, 'posKi'],
                [CUSTOM_REGS.POSITION_KD, 'posKd'], [CUSTOM_REGS.VELOCITY_KP, 'velKp'],
                [CUSTOM_REGS.VELOCITY_KI, 'velKi'], [CUSTOM_REGS.VELOCITY_KD, 'velKd']
            ];
            const gains = {};
            for (const [register, key] of registers) {
                gains[key] = (await readRegisterValue(register)) / 1000;
            }
            writeTuneGainsToInputs(gains);
            return gains;
        }

        function formatTuneGains(gains) {
            return `P:${gains.posKp.toFixed(2)}/${gains.posKi.toFixed(2)}/${gains.posKd.toFixed(2)} V:${gains.velKp.toFixed(3)}/${gains.velKi.toFixed(3)}/${gains.velKd.toFixed(4)}`;
        }

        function writeTuneGainsToInputs(gains) {
            for (const key of TUNE_GAIN_KEYS) {
                const input = document.getElementById(key);
                if (input) input.value = Number(gains[key]).toFixed(key.startsWith('vel') ? 4 : 3);
            }
        }

        async function pushGainsToDevice(gains) {
            for (const key of TUNE_GAIN_KEYS) {
                const regName = {posKp: 'POSITION_KP', posKi: 'POSITION_KI', posKd: 'POSITION_KD',
                                 velKp: 'VELOCITY_KP', velKi: 'VELOCITY_KI', velKd: 'VELOCITY_KD'}[key];
                await writeRegisterAndWaitAck(CUSTOM_REGS[regName], Math.round(gains[key] * 1000));
            }
        }

        function clampGain(key, value) {
            const max = TUNE_GAIN_MAX[key] || 1;
            const v = Math.max(0, Math.min(max, Number(value) || 0));
            return Number(v.toFixed(key === 'velKd' ? 4 : 3));
        }

        // ---------------- 一次试验 ----------------
        async function runPulseTuneTrial(profile, stepsPerRevolution, gains) {
            if (gains) await pushGainsToDevice(gains);
            const isAlternating = profile.mode === 6 || profile.mode === 7;
            const pulseCount = profile.quickStop
                ? Math.max(2, Math.round(profile.turns * stepsPerRevolution))
                : Math.max(2, Math.round(profile.turns * stepsPerRevolution * (isAlternating ? 2 : 1)));
            const accelTimeMs = Math.max(0, (profile.maxRpm - profile.startRpm) / profile.accelRpmS * 1000);
            const timeoutMs = Math.max(6000, pulseCount / stepsPerRevolution * 60 / Math.max(20, profile.maxRpm) * 3000
                                              + accelTimeMs * 4 + 4000);
            state.tuneTrialProfile = {...profile, pulseCount};
            state.tuneTrialStage = '配置运动参数';

            // 每个试验都从干净状态开始：清堵转锁存 -> 使能 -> 开闭环补偿（掉电默认是关的）
            await writeRegister(CUSTOM_REGS.STALL_CLEAR, 1).catch(() => {});
            const faultFlags = await readRegisterValue(CUSTOM_REGS.FAULT_STATUS, 1000, 1);
            if ((faultFlags & 0x03) !== 0) {
                throw new Error(`固件报故障 0x${faultFlags.toString(16)}（编码器/磁场），拒绝试验`);
            }
            await writeRegister(0x124, 1);                 // MOTOR_ENABLE
            await writeRegister(CUSTOM_REGS.CLOSED_LOOP_ENABLE, 1);

            await writeRegisterAndWaitAck(0x11A, Math.round(profile.startRpm));
            await writeRegisterAndWaitAck(0x11B, Math.round(profile.maxRpm));
            await writeRegisterAndWaitAck(0x11C, Math.round(profile.accelRpmS));
            await writeRegisterAndWaitAck(0x11D, pulseCount);
            await writeRegisterAndWaitAck(0x11E, profile.mode);
            await writeRegister(CUSTOM_REGS.PID_TUNE_CONTROL, 1);

            let scoreMilli = null;
            let samples = 0;
            try {
                state.tuneTrialStage = '启动运动';
                await writeRegister(0x11F, 1);
                if (profile.quickStop) {
                    // 先恒速跑一小段，再下"快速停车"，让减速段也能被评分采样
                    await new Promise((resolve) => window.setTimeout(resolve,
                        Math.round(pulseCount / stepsPerRevolution * 60 / Math.max(20, profile.maxRpm) * 380)));
                    state.tuneTrialStage = '快速停车';
                    await writeRegister(CUSTOM_REGS.QUICK_STOP, 1);
                }
                const completed = await waitMotionStopped(timeoutMs);
                if (!completed) throw new Error('等待固件运动结束超时');
                const stall = (await readRegisterValue(CUSTOM_REGS.STALL_DETECTED, 750, 0)) | 0;
                if (stall !== 0) {
                    throw new Error('固件判定堵转（0x015D=1），已停机失能；检查机械/接线后点“清除堵转锁存”');
                }
                samples = (await readRegisterValue(CUSTOM_REGS.PID_TUNE_SAMPLE_COUNT)) | 0;
                scoreMilli = (await readRegisterValue(CUSTOM_REGS.PID_TUNE_SCORE_MDEG)) >>> 0;
                const peakMilli = (await readRegisterValue(CUSTOM_REGS.PID_TUNE_PEAK_MDEG, 750, 0)) | 0;
                const residual = (await readRegisterValue(0x121, 750, 0)) | 0;
                if (samples < 10 || scoreMilli === 0xFFFFFFFF) {
                    throw new Error(`固件评分样本不足（${samples}）`);
                }
                return {
                    fwScore: scoreMilli / 1000.0,
                    peak: (peakMilli | 0) / 1000.0,
                    residual: residual / 1000.0,
                    // 综合分：固件评分（运动中）+ 0.5×停稳后残余（"最后停准了没有"）
                    score: scoreMilli / 1000.0 + 0.5 * Math.abs(residual / 1000.0),
                    samples,
                };
            } finally {
                await writeRegister(CUSTOM_REGS.PID_TUNE_CONTROL, 0).catch(() => {});
                // 【阶段收尾】等运动真的结束之后再失能；提前失能会丢最后几十个脉冲
                try {
                    await writeRegister(0x11F, 0);
                    await new Promise((resolve) => window.setTimeout(resolve, 120));
                    await writeRegister(0x125, 1);
                } catch (error) {
                    appendTuneLog(`阶段收尾失败：${error.message}`);
                }
                state.tuneTrialProfile = null;
                state.tuneTrialStage = '';
            }
        }

        // 轮询 0x11F，直到 running=0（带超时）。比等遥测更直接，也不依赖 rx 的状态机。
        async function waitMotionStopped(timeoutMs) {
            const deadline = performance.now() + timeoutMs;
            let observedRunning = false;
            const start = performance.now();
            while (performance.now() < deadline) {
                let running;
                try {
                    running = (await readRegisterValue(0x11F, 750, 0)) | 0;
                } catch (error) {
                    running = 0;
                }
                if (running !== 0) {
                    observedRunning = true;
                } else if (observedRunning || performance.now() - start > 400) {
                    return true;
                }
                if (state.autoTuneCancelled) return true;
                await new Promise((resolve) => window.setTimeout(resolve, 120));
            }
            return false;
        }

        // ---------------- 一组参数的综合评分 ----------------
        async function scoreGains(gains, profiles, repeats) {
            const details = [];
            let weighted = 0;
            let weightSum = 0;
            let worst = 0;
            for (const profile of profiles) {
                if (state.autoTuneCancelled) return null;
                const runs = [];
                let peak = 0;
                let residual = 0;
                for (let i = 0; i < repeats; i++) {
                    const trial = await runPulseTuneTrial(profile, 200 * state.microsteps, gains);
                    runs.push(trial.score);
                    peak = Math.max(peak, trial.peak);
                    residual += trial.residual;
                }
                const score = runs.reduce((a, b) => a + b, 0) / runs.length;
                residual /= repeats;
                details.push({profile: profile.label, score, peak, residual, runs});
                weighted += score * (profile.weight || 1);
                weightSum += (profile.weight || 1);
                worst = Math.max(worst, score);
                const index = state.tuneActionProfiles.findIndex((p) => p.label === profile.label);
                if (index >= 0) {
                    setTuneActionState(index, '候选评分',
                        `${score.toFixed(3)}°（峰值 ${peak.toFixed(3)}° / 残余 ${residual.toFixed(3)}°）`);
                }
                appendTuneLog(`      ${profile.label}：${score.toFixed(3)}°（峰值 ${peak.toFixed(3)}°，残余 ${residual.toFixed(3)}°）`);
            }
            const mean = weightSum > 0 ? weighted / weightSum : Number.POSITIVE_INFINITY;
            return {mean, worst, combined: mean + 0.25 * worst, details};
        }

        // ---------------- 自动整定主流程 ----------------
        async function startAutoTune() {
            if (state.autoTuneActive) {
                state.autoTuneCancelled = true;
                appendTuneLog('请求取消整定（当前试验结束后停止）');
                document.getElementById('startTuneBtn').textContent = '正在停止';
                return;
            }
            if (state.tuneActionBusy) {
                appendTuneLog('单项动作正在运行，请稍候');
                return;
            }
            renderTuneActionRows(true);
            if (!state.connected) {
                appendTuneLog('未连接串口，无法启动整定');
                return;
            }
            // 整定会驱动电机：外部跟随模式下先自动接管运动通道（固件互斥门）。
            if (!(await ensureInternalMotionSource('PID 自动整定'))) {
                appendTuneLog('运动通道接管失败，未启动整定');
                renderTuneActionRows(false);
                return;
            }

            state.autoTuneActive = true;
            state.autoTuneCancelled = false;
            state.pendingTargetAngle = null;
            state.tuneActionProfiles.forEach((_, index) => {
                const button = document.getElementById(`tuneActionButton${index}`);
                if (button) button.disabled = true;
            });
            document.getElementById('startTuneBtn').textContent = '停止整定';
            document.getElementById('tuneState').textContent = '运行中';
            document.getElementById('tuneLogBadge').textContent = '整定中';
            document.getElementById('tuneResults').textContent = '正在读取硬件参数';
            appendTuneLog('开始自动整定：六工况（含反向/急加速/急转弯），≤250RPM，每工况重复 2 次');

            const repeats = 2;
            const maxSpeedRpm = clamp(Number(document.getElementById('tuneMaxSpeed').value || 200), 5, TUNE_MAX_RPM_HARD);
            const accelRpmS = clamp(Number(document.getElementById('tuneAccel').value || 400), 10, 3000);
            const turns = clamp(Number(document.getElementById('tuneTestTurns').value || 3), 1, 5);
            const stepRatio = clamp(Number(document.getElementById('tuneGainStep').value || 0.25), 0.05, 0.5);
            const factor = Math.max(1.3, 1.0 + stepRatio * 4.0);   // 0.25 -> ×2.0
            let savedConfig = null;
            let bestGains = null;
            let bestScore = Number.POSITIVE_INFINITY;
            let tuneOutcome = '已完成';

            try {
                state.microsteps = (await readRegisterValue(CUSTOM_REGS.MICROSTEPS)) | 0;
                if (!state.microsteps) state.microsteps = 64;
                document.getElementById('microsteps').value = state.microsteps;
                savedConfig = {
                    closedLoopEnabled: await readRegisterValue(CUSTOM_REGS.CLOSED_LOOP_ENABLE),
                    startRpm: await readRegisterValue(0x11A),
                    maxRpm: await readRegisterValue(0x11B),
                    accelRpmS: await readRegisterValue(0x11C),
                };
                bestGains = await readTuneGainsFromHardware();
                appendTuneLog(`起始参数：${formatTuneGains(bestGains)}；微步 ${state.microsteps}（${200 * state.microsteps}/圈）`);

                const searchProfiles = createPulseTuneProfileSearch(maxSpeedRpm, accelRpmS, turns);
                appendTuneLog(`搜索工况：${searchProfiles.map((p) => p.label).join('、')}`);

                let baseline = await scoreGains(bestGains, searchProfiles, repeats);
                if (!baseline) { tuneOutcome = '已停止'; return; }
                bestScore = baseline.combined;
                appendTuneLog(`基准综合分 ${bestScore.toFixed(3)}°（平均 ${baseline.mean.toFixed(3)}° / 最差 ${baseline.worst.toFixed(3)}°）`);

                const maxRounds = 3;
                for (let round = 0; round < maxRounds && !state.autoTuneCancelled; round++) {
                    let improved = false;
                    appendTuneLog(`── 第 ${round + 1} 轮（步进 ×${factor.toFixed(2)}）──`);
                    for (const key of TUNE_SEARCH_ORDER) {
                        if (state.autoTuneCancelled) break;
                        const center = bestGains[key];
                        const candidates = [];
                        if (center > 1e-9) candidates.push(clampGain(key, center / factor));
                        candidates.push(clampGain(key, center > 1e-9 ? center * factor : 0.05));
                        for (const candidate of candidates) {
                            if (state.autoTuneCancelled) break;
                            if (Math.abs(candidate - bestGains[key]) < 1e-6) continue;
                            const trialGains = {...bestGains, [key]: candidate};
                            appendTuneLog(`  试 ${key}: ${center.toFixed(4)} -> ${candidate.toFixed(4)}`);
                            let result;
                            try {
                                result = await scoreGains(trialGains, searchProfiles, repeats);
                            } catch (error) {
                                appendTuneLog(`    跳过：${error.message}`);
                                continue;
                            }
                            if (!result) { tuneOutcome = '已停止'; break; }
                            if (result.combined < bestScore - 1e-4) {
                                bestScore = result.combined;
                                bestGains = trialGains;
                                improved = true;
                                appendTuneLog(`    ✔ 新最佳 ${bestScore.toFixed(3)}°  ${formatTuneGains(bestGains)}`);
                            } else {
                                appendTuneLog(`    ✘ ${result.combined.toFixed(3)}°`);
                            }
                        }
                        await pushGainsToDevice(bestGains).catch(() => {});
                    }
                    if (!improved) {
                        appendTuneLog('本轮无改善，搜索结束');
                        break;
                    }
                }

                if (state.autoTuneCancelled) {
                    tuneOutcome = '已停止';
                } else {
                    // 全工况验证（含急转弯与急刹冲击）
                    appendTuneLog(`搜索完成：${formatTuneGains(bestGains)}；开始全工况验证`);
                    const allProfiles = createPulseTuneProfiles(maxSpeedRpm, accelRpmS, turns, true);
                    let finalResult = null;
                    try {
                        finalResult = await scoreGains(bestGains, allProfiles, repeats);
                    } catch (error) {
                        tuneOutcome = '失败，可重试失败项';
                        document.getElementById('tuneResults').textContent = `全工况验证失败：${error.message}`;
                    }
                    if (finalResult) {
                        const worstProfile = finalResult.details.reduce((a, b) => (a.score > b.score ? a : b));
                        appendTuneLog(`全工况：平均 ${finalResult.mean.toFixed(3)}° / 最差 ${finalResult.worst.toFixed(3)}°（${worstProfile.profile}）`);
                        if (finalResult.worst > TUNE_SAFETY_LIMIT_DEG) {
                            tuneOutcome = '拒绝写入';
                            document.getElementById('tuneResults').textContent =
                                `拒绝写入 Flash：最差工况 ${worstProfile.profile} 评分 ${finalResult.worst.toFixed(3)}° > ${TUNE_SAFETY_LIMIT_DEG}°，先修功能问题（方向/丢步/堵转）`;
                            appendTuneLog(`!! 最差工况超过 ${TUNE_SAFETY_LIMIT_DEG}°，不写 Flash`);
                        } else {
                            writeTuneGainsToInputs(bestGains);
                            await pushGainsToDevice(bestGains);
                            // 0x154 = 写入参数到 Flash（固件要求静止：整定刚结束是静止的）
                            await writeRegisterAndWaitAck(0x154, 1);
                            const saveResult = (await readRegisterValue(0x154)) | 0;
                            const saved = saveResult === 1;
                            document.getElementById('tuneResults').textContent =
                                `位置 PID Kp=${bestGains.posKp.toFixed(3)} Ki=${bestGains.posKi.toFixed(3)} Kd=${bestGains.posKd.toFixed(3)}\n` +
                                `速度 PID Kp=${bestGains.velKp.toFixed(3)} Ki=${bestGains.velKi.toFixed(3)} Kd=${bestGains.velKd.toFixed(4)}\n` +
                                `全工况平均=${finalResult.mean.toFixed(3)}°，最差=${finalResult.worst.toFixed(3)}°，` +
                                (saved ? '已写入 Flash' : '写入 Flash 失败（0x154 读回 0）');
                            appendTuneLog(saved ? '全部工况通过；最佳 PID 已写入并提交 Flash' : '全部工况通过，但 Flash 写入失败');
                        }
                    }
                }
            } catch (error) {
                tuneOutcome = '已中止';
                appendTuneLog(`整定中止：${error.message}`);
                document.getElementById('tuneResults').textContent = `整定未完成：${error.message}`;
            } finally {
                state.autoTuneActive = false;
                document.getElementById('startTuneBtn').textContent = '开始整定';
                document.getElementById('tuneState').textContent = state.autoTuneCancelled ? '已停止' : tuneOutcome;
                document.getElementById('tuneLogBadge').textContent = state.autoTuneCancelled ? '已停止' : tuneOutcome;
                if (state.connected && savedConfig) {
                    try {
                        await writeRegister(0x11F, 0);
                        await new Promise((resolve) => window.setTimeout(resolve, 120));
                        await writeRegister(0x125, 1);      // 整定结束：失能，避免持续灌电流发热
                        await writeRegister(0x11A, savedConfig.startRpm);
                        await writeRegister(0x11B, savedConfig.maxRpm);
                        await writeRegister(0x11C, savedConfig.accelRpmS);
                        await writeRegister(CUSTOM_REGS.CLOSED_LOOP_ENABLE, savedConfig.closedLoopEnabled);
                        document.getElementById('closedLoopEnableInput').checked = savedConfig.closedLoopEnabled !== 0;
                    } catch (error) {
                        appendTuneLog(`恢复运动配置失败：${error.message}`);
                    }
                }
                state.tuneActionProfiles.forEach((_, index) => {
                    const button = document.getElementById(`tuneActionButton${index}`);
                    if (button) button.disabled = state.autoTuneCancelled;
                });
            }
        }

        // 搜索阶段用的四工况（与串口版一致）
        function createPulseTuneProfileSearch(maxSpeedRpm, accelRpmS, turns) {
            return createPulseTuneProfiles(maxSpeedRpm, accelRpmS, turns, false);
        }

        // ---------------- 单项执行（手动点“执行”） ----------------
        async function runTuneAction(index, options = {}) {
            if (state.tuneActionBusy || (state.autoTuneActive && !options.fromAuto)) return null;
            if (!state.connected) {
                setTuneActionState(index, '失败，可重试', '串口未连接');
                appendTuneLog(`${state.tuneActionProfiles[index].label}：未连接串口`);
                return null;
            }
            const profile = state.tuneActionProfiles[index];
            const button = document.getElementById(`tuneActionButton${index}`);
            if (!profile || !button) return null;

            state.tuneActionBusy = true;
            state.tuneActionProfiles.forEach((_, actionIndex) => {
                const actionButton = document.getElementById(`tuneActionButton${actionIndex}`);
                if (actionButton) actionButton.disabled = true;
            });
            button.textContent = '运行中';
            setTuneActionState(index, '运行中', '读取当前 PID 并采样');
            let savedConfig = null;
            try {
                state.microsteps = (await readRegisterValue(CUSTOM_REGS.MICROSTEPS)) | 0;
                if (!state.microsteps) state.microsteps = 64;
                savedConfig = {
                    closedLoopEnabled: await readRegisterValue(CUSTOM_REGS.CLOSED_LOOP_ENABLE),
                    startRpm: await readRegisterValue(0x11A),
                    maxRpm: await readRegisterValue(0x11B),
                    accelRpmS: await readRegisterValue(0x11C),
                };
                const gains = await readTuneGainsFromHardware();
                const trial = await runPulseTuneTrial(profile, 200 * state.microsteps, gains);
                const details = `评分=${trial.score.toFixed(3)}°（固件 ${trial.fwScore.toFixed(3)} + 残余 ${trial.residual.toFixed(3)}），峰值 ${trial.peak.toFixed(3)}°，样本 ${trial.samples}`;
                setTuneActionState(index, '已完成', details, true);
                appendTuneLog(`[动作完成] ${profile.label}：${details}`);
                return trial.score;
            } catch (error) {
                setTuneActionState(index, '失败，可重试', error.message);
                appendTuneLog(`[动作失败] ${profile.label}：${error.message}`);
                throw error;
            } finally {
                if (savedConfig && state.connected && !state.autoTuneActive) {
                    try {
                        await writeRegister(0x11F, 0);
                        await writeRegister(0x11A, savedConfig.startRpm);
                        await writeRegister(0x11B, savedConfig.maxRpm);
                        await writeRegister(0x11C, savedConfig.accelRpmS);
                        await writeRegister(CUSTOM_REGS.CLOSED_LOOP_ENABLE, savedConfig.closedLoopEnabled);
                        document.getElementById('closedLoopEnableInput').checked = savedConfig.closedLoopEnabled !== 0;
                    } catch (error) {
                        appendTuneLog(`恢复动作前配置失败：${error.message}`);
                    }
                }
                state.tuneActionBusy = false;
                if (state.autoTuneActive) {
                    state.tuneActionProfiles.forEach((_, actionIndex) => {
                        const actionButton = document.getElementById(`tuneActionButton${actionIndex}`);
                        if (actionButton && !actionButton.textContent.includes('已完成')) actionButton.disabled = true;
                    });
                } else {
                    state.tuneActionProfiles.forEach((_, actionIndex) => {
                        const actionButton = document.getElementById(`tuneActionButton${actionIndex}`);
                        if (actionButton && !actionButton.textContent.includes('已完成')) actionButton.disabled = false;
                    });
                }
            }
        }

        async function executeTuneAction(index) {
            try {
                await runTuneAction(index, {manual: true});
            } catch (error) {
                document.getElementById('tuneResults').textContent = `${state.tuneActionProfiles[index].label} 失败：${error.message}`;
            }
        }
