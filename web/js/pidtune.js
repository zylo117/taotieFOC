// PID auto-tune: trial profiles, scoring, geometric search (module 9).
// Depends on: state, CUSTOM_REGS, writeRegister/readRegisterValue, log, writePidParameters.
// Load order: ... -> lagangle -> pidtune -> ...

        function appendTuneLog(message) {
            const logElement = document.getElementById('tuneLog');
            const line = document.createElement('p');
            line.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
            logElement.appendChild(line);
            while (logElement.children.length > 500) {
                logElement.removeChild(logElement.firstElementChild);
            }
            logElement.scrollTop = logElement.scrollHeight;
        }

        function createPulseTuneProfiles(maxSpeedRpm, accelRpmS, turnsPerDirection, fullRun) {
            const slowSpeedRpm = Math.max(5, Math.round(maxSpeedRpm * 0.25));
            const slowAccelRpmS = Math.max(10, Math.round(accelRpmS * 0.35));
            const fastAccelRpmS = Math.min(12000, Math.max(slowAccelRpmS, accelRpmS));
            const steadyTurns = fullRun ? turnsPerDirection / 2 : 0.25;
            const reversalTurns = fullRun ? 0.25 : 0.125;
            const profiles = [
                {direction: 0, startRpm: Math.max(2, slowSpeedRpm / 2), maxRpm: slowSpeedRpm, accelRpmS: slowAccelRpmS, turns: steadyTurns, label: '正向慢速'},
                {direction: 0, startRpm: slowSpeedRpm, maxRpm: maxSpeedRpm, accelRpmS: fastAccelRpmS, turns: steadyTurns, label: '正向快速'},
                {direction: 1, startRpm: Math.max(2, slowSpeedRpm / 2), maxRpm: slowSpeedRpm, accelRpmS: slowAccelRpmS, turns: steadyTurns, label: '反向慢速'},
                {direction: 1, startRpm: slowSpeedRpm, maxRpm: maxSpeedRpm, accelRpmS: fastAccelRpmS, turns: steadyTurns, label: '反向快速'}
            ];
            for (let index = 0; index < 4; index++) {
                profiles.push({
                    direction: index % 2 === 0 ? 0 : 1,
                    mode: index % 2 === 0 ? 6 : 7,
                    startRpm: Math.max(5, Math.round(maxSpeedRpm * 0.5)),
                    maxRpm: maxSpeedRpm,
                    accelRpmS: Math.min(12000, Math.max(fastAccelRpmS, accelRpmS * 2)),
                    turns: reversalTurns,
                    label: `快速往返 ${index + 1}`
                });
            }
            return profiles;
        }

        function waitForTuneMotionCompletion(timeoutMs) {
            return new Promise((resolve, reject) => {
                state.tuneMotionObserved = false;
                state.tuneTrialStartedAt = performance.now();
                state.tuneLastMotionTelemetryAt = 0;
                state.tuneLastMotionStatus = null;
                state.tuneLastMotionStatusCmd = null;
                state.tuneTrialStage = '等待固件运动状态变为运行';
                const profile = state.tuneTrialProfile;
                const profileDescription = profile
                    ? `${profile.label}, mode=${profile.mode ?? profile.direction}, dir=${profile.direction}, rpm=${profile.startRpm}->${profile.maxRpm}, accel=${profile.accelRpmS}, turnsPerDirection=${profile.turns}, pulses=${profile.pulseCount}`
                    : 'profile=unknown';
                appendTuneLog(`[诊断] ${profileDescription}；等待 0x011F: 1->0，超时=${timeoutMs}ms`);
                state.tuneMotionResolver = resolve;
                state.tuneMotionTimeout = window.setTimeout(async () => {
                    state.tuneMotionTimeout = null;
                    const stageAtTimeout = state.tuneTrialStage;
                    const commandAtTimeout = state.tuneLastCommand;
                    let liveStatus = '读取失败';
                    let scheduledSteps = '读取失败';
                    let commandedRpm = '读取失败';
                    try {
                        state.tuneTrialStage = '超时诊断：主动读取 0x011F';
                        const value = await readRegisterValue(0x11F, 750, 0);
                        liveStatus = `${value | 0} (cmd=0x81)`;
                    } catch (error) {
                        liveStatus = error.message;
                    }
                    try {
                        state.tuneTrialStage = '超时诊断：读取已提交 STEP 数 0x0134';
                        scheduledSteps = String(await readRegisterValue(CUSTOM_REGS.MOTION_SCHEDULED_STEPS, 750, 0));
                    } catch (error) {
                        scheduledSteps = error.message;
                    }
                    try {
                        state.tuneTrialStage = '超时诊断：读取命令速度 0x0135';
                        const rawCommandedSpeed = await readRegisterValue(CUSTOM_REGS.MOTION_COMMAND_SPEED_MRPM, 750, 0);
                        commandedRpm = `${(rawCommandedSpeed | 0) / 1000.0}RPM`;
                    } catch (error) {
                        commandedRpm = error.message;
                    }
                    if (state.tuneMotionResolver === null) return;

                    const now = performance.now();
                    const elapsedMs = Math.round(now - state.tuneTrialStartedAt);
                    const telemetryAge = state.tuneLastMotionTelemetryAt > 0
                        ? `${Math.round(now - state.tuneLastMotionTelemetryAt)}ms`
                        : '本试验未收到';
                    const lastStatus = state.tuneLastMotionStatus === null
                        ? '未收到'
                        : `${state.tuneLastMotionStatus} (cmd=0x${state.tuneLastMotionStatusCmd.toString(16).padStart(2, '0')})`;
                    const reason = state.tuneMotionObserved
                        ? '已观察到运行状态，但在期限内没有观察到停止状态'
                        : '未观察到运行状态；需检查启动命令应答、固件运动启动及 0x011F 状态遥测';
                    const message = `等待固件运动完成状态超时：阶段=${stageAtTimeout}；${reason}；试验=${profileDescription}；已等待=${elapsedMs}ms/${timeoutMs}ms；最后 0x011F 状态=${lastStatus}，状态遥测距今=${telemetryAge}，超时后主动读取=${liveStatus}；STEP 已提交=${scheduledSteps}/${profile.pulseCount}；命令速度=${commandedRpm}，编码器实测速度=${Number(state.motionSpeedRpm || 0).toFixed(2)}RPM，跟随误差=${Number(state.motionFollowErrorDeg || 0).toFixed(3)}°；超时前最近命令=${commandAtTimeout}`;
                    state.tuneMotionResolver = null;
                    appendTuneLog(`[超时诊断] ${message}`);
                    reject(new Error(message));
                }, timeoutMs);
            });
        }

        async function runPulseTuneTrial(profile, stepsPerRevolution) {
            const isAlternating = profile.mode === 6 || profile.mode === 7;
            const totalTurns = profile.turns * (isAlternating ? 2 : 1);
            const pulseCount = Math.max(2, Math.round(totalTurns * stepsPerRevolution));
            const accelTimeMs = Math.max(0, (profile.maxRpm - profile.startRpm) / profile.accelRpmS * 1000);
            const timeoutMs = Math.max(5000, totalTurns * 60 / profile.maxRpm * 3000 + accelTimeMs * (isAlternating ? 6 : 3) + 3000);
            state.tuneTrialProfile = {...profile, pulseCount};
            state.tuneTrialStage = '配置运动参数';
            appendTuneLog(`[试验] ${profile.label} 参数：方向=${profile.direction === 0 ? '正向' : '反向'}，起始=${profile.startRpm}RPM，最高=${profile.maxRpm}RPM，加速度=${profile.accelRpmS}RPM/s，脉冲数=${pulseCount}，完成超时=${Math.round(timeoutMs)}ms`);
            await writeRegister(0x11A, Math.round(profile.startRpm));
            await writeRegister(0x11B, Math.round(profile.maxRpm));
            await writeRegister(0x11C, Math.round(profile.accelRpmS));
            await writeRegister(0x11D, pulseCount);
            await writeRegister(0x11E, profile.mode ?? profile.direction);
            state.tuneTrialStage = '启动前读取故障位图 0x0108';
            const faultFlags = await readRegisterValue(CUSTOM_REGS.FAULT_STATUS, 1000, 1);
            const encoderFault = (faultFlags & 0x01) !== 0;
            const magneticFault = (faultFlags & 0x02) !== 0;
            const outputStopped = (faultFlags & 0x04) !== 0;
            appendTuneLog(`[启动门控] 0x0108=0x${faultFlags.toString(16).padStart(8, '0')}；编码器故障=${encoderFault ? 1 : 0}，磁场报警=${magneticFault ? 1 : 0}，输出停止=${outputStopped ? 1 : 0}`);
            if (encoderFault || magneticFault) {
                throw new Error(`固件拒绝启动：${encoderFault ? '编码器故障 ' : ''}${magneticFault ? '磁场报警 ' : ''}(0x0108=0x${faultFlags.toString(16)})`);
            }
            state.tuneTrialActive = true;
            state.tuneTrialStage = '启动固件评分采集';
            // 【每个试验都从干净状态开始】
            //   0x15F=1 清堵转锁存（上一个试验如果卡住，锁存会让 startMotion 直接拒绝）
            //   0x124=1 显式使能（EN 低有效，固件里 setEnable(true) 会拉低）
            // 试验结束（无论成功/超时）在 finally 里 0x125=1 失能：驱动不能一直灌电流发热。
            await writeRegister(CUSTOM_REGS.STALL_CLEAR, 1).catch(() => {});
            await writeRegister(0x124, 1);
            const motionCompleted = waitForTuneMotionCompletion(timeoutMs);
            let score = Number.POSITIVE_INFINITY;
            try {
                state.tuneTrialStage = '发送评分采集命令 0x0131=1';
                await writeRegister(CUSTOM_REGS.PID_TUNE_CONTROL, 1);
                state.tuneTrialStage = '发送运动启动命令 0x011F=1';
                await writeRegister(0x11F, 1);
                state.tuneTrialStage = '等待固件运行状态与完成状态';
                appendTuneLog(`${profile.label}：${isAlternating ? `${profile.turns.toFixed(2)} 圈/方向，往返` : `${profile.turns.toFixed(2)} 圈`}，${profile.maxRpm.toFixed(0)} RPM，${profile.accelRpmS.toFixed(0)} RPM/s`);
                await motionCompleted;
                // 【堵转判据】试验“跑完了”但编码器全程没动 = 堵转/线松/电流太小。
                // 这种数据不能拿去评分，否则整定器会把 PID 越调越凶。
                const stall = await readRegisterValue(CUSTOM_REGS.STALL_DETECTED, 750, 0).catch(() => 0);
                if ((stall | 0) !== 0) {
                    throw new Error('固件判定堵转（0x015D=1），已停机失能；请检查机械/接线后点“清除堵转锁存”');
                }
                state.tuneTrialStage = '读取固件评分样本数 0x0133';
                const sampleCount = await readRegisterValue(CUSTOM_REGS.PID_TUNE_SAMPLE_COUNT);
                state.tuneTrialStage = '读取固件评分 0x0132';
                const scoreMilliDegrees = await readRegisterValue(CUSTOM_REGS.PID_TUNE_SCORE_MDEG);
                if (sampleCount < 4 || scoreMilliDegrees === 0xFFFFFFFF) {
                    throw new Error(`固件评分样本不足（${sampleCount}）`);
                }
                score = scoreMilliDegrees / 1000.0;
                return {score, sampleCount};
            } finally {
                state.tuneTrialStage = '关闭固件评分采集';
                state.tuneTrialActive = false;
                await writeRegister(CUSTOM_REGS.PID_TUNE_CONTROL, 0).catch((error) => {
                    appendTuneLog(`关闭固件评分采集失败：${error.message}`);
                });
                if (state.tuneMotionTimeout !== null) {
                    window.clearTimeout(state.tuneMotionTimeout);
                    state.tuneMotionTimeout = null;
                }
                state.tuneMotionResolver = null;
                state.tuneTrialProfile = null;
                state.tuneTrialStage = '';
                // 【阶段收尾：停脉冲 + 失能】必须等到运动真的结束（上面的 await）之后再做，
                // 提前失能会把最后几十个脉冲连同对应角度一起丢掉（"最后一点没走完"）。
                try {
                    await writeRegister(0x11F, 0);
                    await new Promise((resolve) => window.setTimeout(resolve, 120));
                    await writeRegister(0x125, 1);   // MOTOR_DISABLE：本阶段结束，断电流
                } catch (error) {
                    appendTuneLog(`阶段收尾（停止/失能）失败：${error.message}`);
                }
            }
            if (state.autoTuneCancelled) return null;
            return {score, sampleCount: 0};
        }

        function getTuneActionProfiles() {
            // 本机 300~800RPM 剧烈共振：低速整定阶段先把上限钉在 250RPM。
            const maxSpeedRpm = clamp(Number(document.getElementById('tuneMaxSpeed').value || 200), 5, 250);
            const accelRpmS = clamp(Number(document.getElementById('tuneAccel').value || 400), 10, 3000);
            const turnsPerDirection = clamp(Number(document.getElementById('tuneTestTurns').value || 3), 1, 5);
            return createPulseTuneProfiles(maxSpeedRpm, accelRpmS, turnsPerDirection, true);
        }

        function renderTuneActionRows(reset = true) {
            const container = document.getElementById('tuneActionList');
            if (!container) return;
            state.tuneActionProfiles = getTuneActionProfiles();
            container.replaceChildren();
            state.tuneActionProfiles.forEach((profile, index) => {
                const row = document.createElement('div');
                row.className = 'tune-action-row';
                const turnsDescription = profile.mode === 6 || profile.mode === 7
                    ? `${profile.turns} 圈/方向，往返`
                    : `${profile.turns} 圈`;
                row.innerHTML = `<div class="tune-action-name">${String(index + 1).padStart(2, '0')} · ${profile.label}</div><div class="tune-action-result" id="tuneActionResult${index}">${profile.direction === 0 ? '正向' : '反向'} · ${profile.startRpm}→${profile.maxRpm} RPM · ${profile.accelRpmS} RPM/s · ${turnsDescription} · 等待执行</div><button class="tune-action-button" id="tuneActionButton${index}" type="button">执行</button>`;
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
            return `P:${gains.posKp.toFixed(2)}/${gains.posKi.toFixed(2)}/${gains.posKd.toFixed(2)} V:${gains.velKp.toFixed(2)}/${gains.velKi.toFixed(2)}/${gains.velKd.toFixed(2)}`;
        }

        async function runPulseTuneTrialWithRetries(profile, stepsPerRevolution, actionIndex = -1) {
            for (let attempt = 0; ; attempt++) {
                try {
                    return await runPulseTuneTrial(profile, stepsPerRevolution);
                } catch (error) {
                    if (state.autoTuneCancelled) return null;
                    const isMotionTimeout = error.message.includes('等待固件运动完成状态超时');
                    if (!state.autoTuneActive || !isMotionTimeout || attempt >= 2) throw error;

                    if (actionIndex >= 0) {
                        setTuneActionState(actionIndex, '超时，正在恢复', `自动重试 ${attempt + 1}/2`);
                    }
                    appendTuneLog(`[自动重试] ${profile.label} 第 ${attempt + 1}/2 次：停止当前动作后重跑`);
                    await writeRegister(0x11F, 0);
                    let stopped = false;
                    const stopDeadline = performance.now() + 2000;
                    while (!stopped && performance.now() < stopDeadline) {
                        try {
                            stopped = (await readRegisterValue(0x11F, 500, 0)) === 0;
                        } catch {
                            stopped = false;
                        }
                        if (!stopped) await new Promise((resolve) => window.setTimeout(resolve, 100));
                    }
                    if (!stopped) throw new Error(`自动重试前无法确认运动已停止：${error.message}`);
                    await new Promise((resolve) => window.setTimeout(resolve, 300));
                }
            }
        }

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
                const microsteps = await readRegisterValue(CUSTOM_REGS.MICROSTEPS);
                savedConfig = {
                    closedLoopEnabled: await readRegisterValue(CUSTOM_REGS.CLOSED_LOOP_ENABLE),
                    startRpm: await readRegisterValue(0x11A),
                    maxRpm: await readRegisterValue(0x11B),
                    accelRpmS: await readRegisterValue(0x11C)
                };
                const gains = await readTuneGainsFromHardware();
                await writeRegister(CUSTOM_REGS.CLOSED_LOOP_ENABLE, 1);
                document.getElementById('closedLoopEnableInput').checked = true;
                const trialResult = await runPulseTuneTrialWithRetries(profile, 200 * microsteps, index);
                if (!trialResult) return null;
                const details = `评分=${trialResult.score.toFixed(3)}°，样本=${trialResult.sampleCount}，${formatTuneGains(gains)}`;
                setTuneActionState(index, '已完成', details, true);
                appendTuneLog(`[动作完成] ${profile.label}：${details}`);
                return trialResult.score;
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

        function writeTuneGainsToInputs(gains) {
            for (const [key, inputId] of Object.entries({
                posKp: 'posKp', posKi: 'posKi', posKd: 'posKd',
                velKp: 'velKp', velKi: 'velKi', velKd: 'velKd'
            })) {
                document.getElementById(inputId).value = Number(gains[key]).toFixed(3);
            }
        }

        async function startAutoTune() {
            if (state.autoTuneActive) {
                state.autoTuneCancelled = true;
                if (state.tuneMotionResolver) state.tuneMotionResolver();
                document.getElementById('startTuneBtn').textContent = '正在停止';
                renderTuneActionRows(false);
                state.tuneActionProfiles.forEach((_, index) => {
                    const button = document.getElementById(`tuneActionButton${index}`);
                    if (button) button.disabled = true;
                });
                appendTuneLog('请求取消整定');
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
            appendTuneLog('开始多速度、多加速度的正反向脉冲闭环整定');

            const maxSpeedRpm = clamp(Number(document.getElementById('tuneMaxSpeed').value || 60), 5, 6000);
            const accelRpmS = clamp(Number(document.getElementById('tuneAccel').value || 180), 10, 3000);
            const stepRatio = clamp(Number(document.getElementById('tuneGainStep').value || 0.25), 0.05, 0.5);
            let bestGains = null;
            let bestScore = Number.POSITIVE_INFINITY;
            let finalScore = Number.POSITIVE_INFINITY;
            let savedConfig = null;
            let tuneOutcome = '已完成';
            try {
                const microsteps = await readRegisterValue(CUSTOM_REGS.MICROSTEPS);
                state.microsteps = microsteps;
                document.getElementById('microsteps').value = microsteps;
                savedConfig = {
                    closedLoopEnabled: await readRegisterValue(CUSTOM_REGS.CLOSED_LOOP_ENABLE),
                    startRpm: await readRegisterValue(0x11A),
                    maxRpm: await readRegisterValue(0x11B),
                    accelRpmS: await readRegisterValue(0x11C)
                };
                bestGains = await readTuneGainsFromHardware();

                if (performance.now() - state.lastEncoderTelemetryAt > 1000) {
                    await readRegisterValue(CUSTOM_REGS.ENCODER_ANGLE_MDEG);
                    if (performance.now() - state.lastEncoderTelemetryAt > 1000) {
                        throw new Error('没有收到实时编码器角度遥测');
                    }
                }

                await writeRegister(CUSTOM_REGS.CLOSED_LOOP_ENABLE, 1);
                document.getElementById('closedLoopEnableInput').checked = true;
                appendTuneLog(`开始候选 PID 搜索：${formatTuneGains(bestGains)}；编码器细分=${microsteps}`);
                const candidateProfiles = createPulseTuneProfiles(maxSpeedRpm, accelRpmS, 5, false).slice(0, 4);
                const actionIndexForProfile = (profile) => state.tuneActionProfiles.findIndex((item) => item.label === profile.label);
                const scoreProfiles = async (gains, profiles) => {
                    await writePidParameters(gains);
                    let totalScore = 0;
                    for (const profile of profiles) {
                        if (state.autoTuneCancelled) return Number.POSITIVE_INFINITY;
                        const actionIndex = actionIndexForProfile(profile);
                        try {
                            const trial = await runPulseTuneTrialWithRetries(profile, 200 * microsteps, actionIndex);
                            if (!trial || state.autoTuneCancelled) return Number.POSITIVE_INFINITY;
                            totalScore += trial.score;
                            if (actionIndex >= 0) {
                                setTuneActionState(actionIndex, '候选 PID 评分', `评分=${trial.score.toFixed(3)}°，样本=${trial.sampleCount}，${formatTuneGains(gains)}`);
                            }
                        } catch (error) {
                            if (actionIndex >= 0) setTuneActionState(actionIndex, '失败，可重试', error.message);
                            throw error;
                        }
                    }
                    return totalScore / profiles.length;
                };

                bestScore = await scoreProfiles(bestGains, candidateProfiles);
                const gainKeys = ['posKp', 'posKi', 'posKd', 'velKp', 'velKi', 'velKd'];
                // 【几何搜索 + 多轮】原来单轮 + 每候选只试 ±stepRatio（默认 ±25%），
                // 从 Kp=1 一轮最多到 1.25，而最优值可能在 50~200（差 100 倍）—— 永远到不了。
                // 改成：候选 ×2 / ÷2，每轮 6 个参数全试，有改善就继续，无改善就停。
                // 8 轮就能覆盖 256 倍。几何因子仍受 tuneGainStep 影响。
                const geometricFactor = Math.max(1.2, 1.0 + stepRatio * 4.0);
                const kMaxSearchRounds = 10;
                for (let round = 0; round < kMaxSearchRounds && !state.autoTuneCancelled; round++) {
                    let improvedThisRound = false;
                    for (const key of gainKeys) {
                        if (state.autoTuneCancelled) break;
                        const center = bestGains[key];
                        // 上限与固件 maxPidGain() 一致（这里是“实际增益”口径）
                        const maxGain = key === 'posKp' || key === 'posKi' ? 1000
                            : key === 'posKd' ? 100
                            : key === 'velKp' || key === 'velKi' ? 10 : 1;
                        const lower = center / geometricFactor;
                        const upper = Math.min(maxGain, center * geometricFactor);
                        const candidates = [
                            lower > 0.001 ? lower : 0,
                            center < 0.001 ? Math.min(maxGain, stepRatio) : upper
                        ];
                        for (const candidateValue of candidates) {
                            if (state.autoTuneCancelled) break;
                            const candidate = {...bestGains, [key]: candidateValue};
                            const score = await scoreProfiles(candidate, candidateProfiles);
                            if (score < bestScore) {
                                bestScore = score;
                                bestGains = candidate;
                                improvedThisRound = true;
                                appendTuneLog(`第 ${round + 1} 轮：${key}=${candidateValue.toFixed(3)} 成为当前最佳，评分 ${score.toFixed(3)}`);
                            }
                        }
                        await writePidParameters(bestGains);
                    }
                    if (!improvedThisRound) {
                        appendTuneLog(`第 ${round + 1} 轮无改善，搜索结束（几何因子 ${geometricFactor.toFixed(2)}）`);
                        break;
                    }
                }

                if (state.autoTuneCancelled) {
                    tuneOutcome = '已停止';
                    appendTuneLog('整定已取消；动作按钮状态已重置');
                } else {
                    appendTuneLog(`候选 PID 搜索完成：${formatTuneGains(bestGains)}；开始逐项全场景验证`);
                    let totalFinalScore = 0;
                    let completedProfiles = 0;
                    for (let index = 0; index < state.tuneActionProfiles.length; index++) {
                        if (state.autoTuneCancelled) break;
                        try {
                            const score = await runTuneAction(index, {fromAuto: true});
                            if (score === null) break;
                            totalFinalScore += score;
                            completedProfiles++;
                        } catch (error) {
                            tuneOutcome = '失败，可重试失败项';
                            document.getElementById('tuneResults').textContent = `${state.tuneActionProfiles[index].label} 失败：${error.message}`;
                            break;
                        }
                    }
                    if (!state.autoTuneCancelled && completedProfiles === state.tuneActionProfiles.length) {
                        finalScore = totalFinalScore / completedProfiles;
                        state.bestTuneGains = {...bestGains};
                        writeTuneGainsToInputs(bestGains);
                        document.getElementById('tuneResults').textContent = `位置 PID Kp=${bestGains.posKp.toFixed(3)} Ki=${bestGains.posKi.toFixed(3)} Kd=${bestGains.posKd.toFixed(3)}\n速度 PID Kp=${bestGains.velKp.toFixed(3)} Ki=${bestGains.velKi.toFixed(3)} Kd=${bestGains.velKd.toFixed(3)}\n短行程最佳评分=${bestScore.toFixed(3)}°；全场景验证评分=${finalScore.toFixed(3)}°`;
                        appendTuneLog('全部动作通过；最佳 PID 已即时写入，每组评分和 PID 已记录');
                    } else if (state.autoTuneCancelled) {
                        tuneOutcome = '已停止';
                    }
                }
            } catch (error) {
                tuneOutcome = '已中止';
                state.tuneTrialActive = false;
                appendTuneLog(`整定中止：${error.message}`);
                document.getElementById('tuneResults').textContent = `整定未完成：${error.message}`;
            } finally {
                state.tuneTrialActive = false;
                state.autoTuneActive = false;
                document.getElementById('startTuneBtn').textContent = '开始整定';
                document.getElementById('tuneState').textContent = state.autoTuneCancelled ? '已停止' : tuneOutcome;
                document.getElementById('tuneLogBadge').textContent = state.autoTuneCancelled ? '已停止' : tuneOutcome;
                if (state.connected) {
                    if (bestGains && Number.isFinite(bestScore)) {
                        try {
                            await writePidParameters(bestGains);
                        } catch (error) {
                            appendTuneLog(`恢复最佳参数失败：${error.message}`);
                        }
                    }
                    try {
                        await writeRegister(0x11F, 0);
                    } catch (error) {
                        appendTuneLog(`发送停止命令失败：${error.message}`);
                    }
                    try {
                        if (savedConfig) {
                            await writeRegister(0x11A, savedConfig.startRpm);
                            await writeRegister(0x11B, savedConfig.maxRpm);
                            await writeRegister(0x11C, savedConfig.accelRpmS);
                            await writeRegister(CUSTOM_REGS.CLOSED_LOOP_ENABLE, savedConfig.closedLoopEnabled);
                            document.getElementById('closedLoopEnableInput').checked = savedConfig.closedLoopEnabled !== 0;
                        }
                    } catch (error) {
                        appendTuneLog(`恢复运动配置失败：${error.message}`);
                    }
                }
                if (state.autoTuneCancelled) renderTuneActionRows(true);
                else state.tuneActionProfiles.forEach((_, index) => {
                    const button = document.getElementById(`tuneActionButton${index}`);
                    if (button && button.textContent !== '已完成') button.disabled = false;
                });
            }
        }

        async function executeTuneAction(index) {
            try {
                await runTuneAction(index, {manual: true});
            } catch (error) {
                document.getElementById('tuneResults').textContent = `${state.tuneActionProfiles[index].label} 失败：${error.message}`;
            }
        }

