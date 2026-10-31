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

        async function connectSerial() {
            try {
                if (!('serial' in navigator)) {
                    log('[WARN] 当前浏览器不支持 Web Serial API，请使用 Chrome / Edge');
                    return;
                }

                const portObj = await navigator.serial.requestPort();
                const baud = Number(document.getElementById('baudRate').value || 115200);
                await portObj.open({baudRate: baud});

                state.port = portObj;
                state.connected = true;
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
                })();

                setConnBadge('已连接', true);
                protoBadge.textContent = 'TMC 兼容';
                log('[INFO] 串口已连接，开始监听返回数据');
                await armPositionFollowMode();
                if (state.pendingTargetAngle !== null) void flushTargetAngleWrites();

                readLoop();
            } catch (err) {
                log('[ERROR] 串口连接失败：' + (err && err.message ? err.message : String(err)));
                setConnBadge('未连接', false);
            }
        }

        // ===================== 工作模式 tab =====================
        // 测速控制和外部脉冲跟随共用同一个 STEP 输出通道（TMR2/DMA），固件里本来就是互斥的：
        // 外部跟随使能时 rampUpdate 直接走 updateExternalPulseChannel 并 return。
        // 所以这里不让它们同时出现：切 tab 就同时切固件的 0x149。
        function switchWorkTab(tab) {
            const isMotion = tab === 'motion';
            document.getElementById('cardMotion').style.display = isMotion ? '' : 'none';
            document.getElementById('cardExtFollow').style.display = isMotion ? 'none' : '';
            document.getElementById('tabMotionBtn').className = isMotion ? 'primary' : '';
            document.getElementById('tabExtFollowBtn').className = isMotion ? '' : 'primary';

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
                if (state.connected) {
                    writeRegister(CUSTOM_REGS.CLOSED_LOOP_ENABLE, 0)
                        .catch((error) => log('[ERROR] 关闭闭环补偿失败：' + error.message));
                }
            }

            if (!state.connected) {
                log('[INFO] 已切到' + (isMotion ? '内部规划运动' : '外部脉冲运动') + '（未连接串口，模式未下发）');
                return;
            }
            writeRegister(CUSTOM_REGS.EXT_FOLLOW_ENABLE, isMotion ? 0 : 1)
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
                .catch((error) => log('[ERROR] 切换模式失败：' + error.message));
        }

        async function readLoop() {
            if (!state.reader) return;
            try {
                while (true) {
                    const {value, done} = await state.reader.read();
                    if (done) break;
                    if (!value || value.length === 0) continue;
                    appendRxBytes(value);
                }
            } catch (err) {
                log('[ERROR] 串口读取出现错误：' + err.message);
                setConnBadge('未连接', false);
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
            // 【单位】输入框里填的是“实际增益”（默认 60.0 就是增益 60），
            // 发送时乘 1000 变成固件口径。上限和固件 clampPidGain 保持一致：
            //   比例/微分 实际 <= 30，积分 实际 <= 3
            // 以前这里写的是 30000/3000（当成“已 ×1000 的口径”），
            // 导致用户填 9000 也直接发 9,000,000，固件再钳到 30，看起来像“填 9000 变 30000”。
            const registers = [
                [CUSTOM_REGS.POSITION_KP, params.posKp, 30000],
                [CUSTOM_REGS.POSITION_KI, params.posKi, 3000],
                [CUSTOM_REGS.POSITION_KD, params.posKd, 30000],
                [CUSTOM_REGS.VELOCITY_KP, params.velKp, 30000],
                [CUSTOM_REGS.VELOCITY_KI, params.velKi, 3000],
                [CUSTOM_REGS.VELOCITY_KD, params.velKd, 30000]
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
                posKp: [CUSTOM_REGS.POSITION_KP, 30000],
                posKi: [CUSTOM_REGS.POSITION_KI, 3000],
                posKd: [CUSTOM_REGS.POSITION_KD, 30000],
                velKp: [CUSTOM_REGS.VELOCITY_KP, 30000],
                velKi: [CUSTOM_REGS.VELOCITY_KI, 3000],
                velKd: [CUSTOM_REGS.VELOCITY_KD, 30000]
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
                CUSTOM_REGS.VELOCITY_KD
            ];
            regs.forEach(async (reg) => {
                await readRegister(reg);
            });
        }

        async function stopMotor() {
            stopWaveCapture();
            if (state.autoTuneActive) {
                state.autoTuneCancelled = true;
                appendTuneLog('收到停止命令，正在安全终止整定');
            }
            log('[WARN] 发送停止命令，释放闭环保持');
            if (state.connected) {
                await writeRegister(0x11F, 0);
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

