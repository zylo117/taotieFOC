// Motion waveform: sampling, drawing, popup window (module 7).
// Depends on: state, ENCODER_LSB_DEG, log (ui), CUSTOM_REGS.
// Load order: ... -> ui -> waveform -> ...

        function updateMotionDurationEstimate() {
            const pulseCount = Math.max(2, getEffectivePulseCount());
            const startRpm = Number(document.getElementById('startRpmSelect').value || 0);
            const maxRpm = Number(document.getElementById('maxRpmSelect').value || 0);
            const accelRpm = Number(document.getElementById('accelRpmSelect').value || 0);
            const stepsPerRevolution = getStepsPerRevolution();
            const accelSeconds = accelRpm > 0 && maxRpm > startRpm ? (maxRpm - startRpm) / accelRpm : 0;
            const accelPulses = ((startRpm + maxRpm) * 0.5) * accelSeconds / 60 * stepsPerRevolution;
            const physicalSeconds = accelPulses < pulseCount && maxRpm > 0
                ? accelSeconds + (pulseCount - accelPulses) / (maxRpm / 60 * stepsPerRevolution)
                : pulseCount / Math.max(1, startRpm / 60 * stepsPerRevolution);
            document.getElementById('motionDurationEstimate').textContent =
                `脉冲数：${pulseCount}（${(pulseCount / stepsPerRevolution).toFixed(2)} 圈），理论时间（${maxRpm} RPM）：约 ${physicalSeconds.toFixed(1)} s；STEP 基准：${stepsPerRevolution} 个/圈`;
        }

        // 【不要在这里立即调用】
        // 原来这里有一句 updateMotionDurationEstimate()，它依赖的 getEffectivePulseCount /
        // getStepsPerRevolution 还在内联脚本里（本文件先加载），会抛 ReferenceError，
        // 导致本文件后续的 const WAVE_POST_ROLL_MS 永远得不到初始化（暂时性死区），
        // 进而让 pushWaveSample 报 “Cannot access WAVE_POST_ROLL_MS before initialization”。
        // 初始化调用已移到内联脚本的 buildMotionControlOptions() 之后。

        // 固件报“运动结束”后再留一段采样尾巴，等机械轴把最后的负载角走完，
        // 波形图才能把“误差收敛到稳态”这一段画出来（而不是停在还没停转的那一帧）。
        const WAVE_POST_ROLL_MS = 600;

        function pushWaveSample() {
            if (!state.waveCaptureActive || state.wavePaused) return;
            const now = performance.now();
            const sample = {
                t: now,
                rpm: Number(state.motionSpeedRpm || 0),
                followError: Number(state.motionFollowErrorDeg || 0),
                // 历史样本里也存下当时的负载角，勾选“误差不计负载角”时用它做扣除，
                // 而不是用当前值近似（否则波形在速度变化段会失真）。
                loadAngle: Number(state.motionLoadAngleDeg || 0)
            };
            state.waveHistory.push(sample);
            const windowMs = clamp(Number(document.getElementById('waveWindowMsInput').value || 10000), 10000, 10000);
            const cutoff = now - windowMs;
            state.waveHistory = state.waveHistory.filter((entry) => entry.t >= cutoff);
            if (state.waveHistory.length > 2000) {
                state.waveHistory = state.waveHistory.slice(-2000);
            }

            // 【运动结束检测】用「即时转速」判断，不用 0x11F。
            // 不用 0x11F 的原因：它是双向寄存器（写=启动/停止命令，读=运动状态），
            // 网页里有 7 处 writeRegister(0x11F, 0)，一旦被刷成 0 就会误判“运动结束”，
            // 表现为“波形走一段就停住但电机还在转”。
            // 而 0x120（即时转速）是纯只读遥测，不参与任何命令，可靠得多。
            //
            // 判据：
            //   转速 > 20 RPM         -> 确认运动开始（编码器静止噪声约 ±2RPM，20 足够区分）
            //   连续 25 帧（约 1 秒）都 <= 20 RPM -> 确认停稳，再留 WAVE_POST_ROLL_MS 尾迹后冻结
            const speedAbs = Math.abs(sample.rpm);
            if (speedAbs > 20) {
                state.waveMotionObserved = true;
                state.waveZeroStreak = 0;
            } else if (state.waveMotionObserved && state.waveStopFreezeTimer === null) {
                state.waveZeroStreak = (state.waveZeroStreak || 0) + 1;
                if (state.waveZeroStreak >= 25) {
                    // 停稳了：补一个末点、记下停止时刻（画一条虚线），再留一段尾迹后冻结
                    state.waveStoppedAt = now;
                    state.waveStopFreezeTimer = window.setTimeout(() => {
                        state.waveStopFreezeTimer = null;
                        stopWaveCapture();
                    }, WAVE_POST_ROLL_MS);
                }
            }
        }

        function startWaveCapture() {
            if (state.motionWaveTimer !== null) {
                window.clearInterval(state.motionWaveTimer);
                state.motionWaveTimer = null;   // 先置 null：万一下面抛异常，看门狗能感知到“无定时器”并重启
            }
            // 防抖计数器：连续 3 帧 0x11F==0 才认定运动结束
            state.waveZeroStreak = 0;
            state.waveHistory = [];
            state.motionSpeedRpm = 0;
            state.motionFollowErrorDeg = 0;
            state.wavePaused = false;
            state.wavePausedAt = 0;
            state.waveCaptureStartedAt = performance.now();
            state.waveMotionObserved = false;
            state.waveFrozenAt = 0;
            state.waveStoppedAt = 0;
            if (state.waveStopFreezeTimer !== null) {
                window.clearTimeout(state.waveStopFreezeTimer);
                state.waveStopFreezeTimer = null;
            }
            state.waveCaptureActive = true;
            document.getElementById('pauseWaveBtn').textContent = '暂停查看';
            state.motionWaveTimer = window.setInterval(() => {
                if (!state.waveCaptureActive) return;
                pushWaveSample();
                drawWaveform();
                if (document.getElementById('wavePopup').classList.contains('visible')) {
                    drawWaveform('motionWavePopupCanvas');
                }
            }, 40);
        }

        // 波形定时器自愈（看门狗）。
        // 由独立的 200ms 心跳调用，不依赖波形自己的定时器（否则死循环）。
        // 僵尸状态：waveCaptureActive 为 true 但 motionWaveTimer 为 null
        // —— 表现为“波形永久不再推进，但电机还在转”。
        // 可能来源：startWaveCapture 里 clear 后抛异常、或切 tab/stop 的异步竞争。
        let waveWatchdogLogCount = 0;

        function drawWaveform(canvasId = 'motionWaveCanvas') {
            const canvas = document.getElementById(canvasId);
            if (!canvas) return;
            const ctx = canvas.getContext('2d');
            const width = canvas.width;
            const height = canvas.height;
            const leftPad = 48;
            const rightPad = 52;
            const plotLeft = leftPad;
            const plotRight = width - rightPad;
            const plotTop = 18;
            const plotBottom = height - 24;
            const windowMs = clamp(Number(document.getElementById('waveWindowMsInput').value || 10000), 10000, 10000);
            if (!state.waveCaptureActive && state.waveHistory.length === 0) return;
            const now = state.wavePaused ? state.wavePausedAt :
                state.waveCaptureActive ? performance.now() : state.waveFrozenAt;

            ctx.clearRect(0, 0, width, height);
            ctx.fillStyle = 'rgba(2,6,23,0.82)';
            ctx.fillRect(0, 0, width, height);

            const points = state.waveHistory.length ? state.waveHistory : [{t: now - windowMs, rpm: 0, followError: 0}];
            const filtered = points.filter((entry) => now - entry.t <= windowMs);

            // 跟随误差的量级只有零点几度（1200 RPM 稳态负载角约 0.3°），
            // 固定 ±30° 的轴会把曲线压成一条直线，看不出“结束运动到底少走了多少”。
            // 这里按窗口内实际幅值自动挑一档对称刻度，最小 ±0.2°，最大 ±30°。
            const errorAxisSteps = [0.2, 0.5, 1, 2, 3, 5, 10, 15, 20, 30];
            let errorAbsPeak = 0;
            filtered.forEach((entry) => {
                const magnitude = Math.abs(Number(entry.followError) || 0);
                if (magnitude > errorAbsPeak) errorAbsPeak = magnitude;
            });
            let followErrorRange = errorAxisSteps[errorAxisSteps.length - 1];
            for (const candidate of errorAxisSteps) {
                if (errorAbsPeak <= candidate * 0.85) {
                    followErrorRange = candidate;
                    break;
                }
            }
            const errorAxisDecimals = followErrorRange <= 0.5 ? 2 : followErrorRange <= 3 ? 1 : 0;
            const formatErrorTick = (value) => {
                if (Math.abs(value) < 1e-9) return `0.${'0'.repeat(errorAxisDecimals)}°`;
                return `${value > 0 ? '+' : '-'}${Math.abs(value).toFixed(errorAxisDecimals)}°`;
            };

            ctx.strokeStyle = 'rgba(148,163,184,0.24)';
            ctx.lineWidth = 1;
            const gridCount = Math.floor(windowMs / 100);
            for (let i = 0; i <= gridCount; i++) {
                const x = plotLeft + ((plotRight - plotLeft) * i) / gridCount;
                const isMajorGrid = i % 10 === 0;
                ctx.strokeStyle = isMajorGrid ? 'rgba(148,163,184,0.32)' : 'rgba(148,163,184,0.14)';
                ctx.lineWidth = isMajorGrid ? 1 : 0.7;
                ctx.beginPath();
                ctx.moveTo(x, plotTop);
                ctx.lineTo(x, plotBottom);
                ctx.stroke();
            }
            ctx.textBaseline = 'middle';
            ctx.font = '11px sans-serif';
            for (let i = 0; i <= 12; i++) {
                const y = plotTop + ((plotBottom - plotTop) / 12) * i;
                const isMajorGrid = i % 3 === 0;
                const rpmTick = 3000 - (i * 500);
                const errorTick = followErrorRange * (1 - (2 * i) / 12);
                ctx.strokeStyle = isMajorGrid ? 'rgba(148,163,184,0.36)' : 'rgba(148,163,184,0.2)';
                ctx.lineWidth = isMajorGrid ? 1 : 0.7;
                ctx.beginPath();
                ctx.moveTo(plotLeft, y);
                ctx.lineTo(plotRight, y);
                ctx.stroke();
                ctx.fillStyle = 'rgba(226,232,240,0.78)';
                ctx.textAlign = 'right';
                ctx.fillText(String(rpmTick), plotLeft - 6, y);
                // 误差轴只标主刻度（±满量程 / ±半量程 / 0），自动换挡后右侧也不会糊成一片
                if (isMajorGrid) {
                    ctx.fillStyle = 'rgba(134,239,172,0.9)';
                    ctx.textAlign = 'left';
                    ctx.fillText(formatErrorTick(errorTick), plotRight + 6, y);
                }
            }

            const speedLine = (rpm) => {
                return plotBottom - ((rpm + 3000) / 6000) * (plotBottom - plotTop);
            };
            const followErrorLine = (error) => {
                return plotBottom - ((error + followErrorRange) / (2 * followErrorRange)) * (plotBottom - plotTop);
            };
            const speedReadoutId = canvasId === 'motionWavePopupCanvas' ? 'wavePopupSpeedReadout' : 'waveSpeedReadout';
            const errorReadoutId = canvasId === 'motionWavePopupCanvas' ? 'wavePopupErrorReadout' : 'waveErrorReadout';
            const peakReadoutId = canvasId === 'motionWavePopupCanvas' ? 'wavePopupErrorPeakReadout' : 'waveErrorPeakReadout';
            const lastReadoutId = canvasId === 'motionWavePopupCanvas' ? 'wavePopupErrorLastReadout' : 'waveErrorLastReadout';
            const lastSample = filtered.length ? filtered[filtered.length - 1] : {followError: 0};
            document.getElementById(speedReadoutId).textContent = `${Number(state.motionSpeedRpm || 0).toFixed(2)} RPM`;
            const loadAngleReadout = document.getElementById('waveLoadAngleReadout');
            if (loadAngleReadout) {
                loadAngleReadout.textContent = `${Number(state.motionLoadAngleDeg || 0).toFixed(3)}°`;
            }
            document.getElementById(errorReadoutId).textContent = `${Number(state.motionFollowErrorDeg || 0).toFixed(3)}°`;
            document.getElementById(peakReadoutId).textContent = `${errorAbsPeak.toFixed(3)}°`;
            document.getElementById(lastReadoutId).textContent = `${Number(lastSample.followError || 0).toFixed(3)}°`;

            ctx.strokeStyle = 'rgba(56,189,248,0.7)';
            ctx.fillStyle = '#e2e8f0';
            ctx.font = '12px sans-serif';
            ctx.textBaseline = 'alphabetic';
            ctx.textAlign = 'left';
            ctx.fillText('RPM', 10, 14);
            ctx.textAlign = 'right';
            ctx.fillText(`跟随误差 °（满量程 ±${followErrorRange}°）`, width - 4, 14);
            ctx.textAlign = 'left';

            ctx.strokeStyle = '#38bdf8';
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.moveTo(plotLeft + 8, plotTop + 12);
            ctx.lineTo(plotLeft + 28, plotTop + 12);
            ctx.stroke();
            ctx.fillStyle = '#38bdf8';
            ctx.fillText('编码器速度 RPM', plotLeft + 34, plotTop + 16);
            ctx.strokeStyle = '#22c55e';
            ctx.beginPath();
            ctx.moveTo(plotLeft + 170, plotTop + 12);
            ctx.lineTo(plotLeft + 190, plotTop + 12);
            ctx.stroke();
            ctx.fillStyle = '#22c55e';
            ctx.fillText('瞬时跟随误差 °', plotLeft + 196, plotTop + 16);

            ctx.beginPath();
            ctx.moveTo(plotLeft, plotTop);
            ctx.lineTo(plotLeft, plotBottom);
            ctx.lineTo(plotRight, plotBottom);
            ctx.stroke();

            if (filtered.length > 1) {
                ctx.beginPath();
                filtered.forEach((entry, i) => {
                    const x = plotLeft + ((now - entry.t) / windowMs) * (plotRight - plotLeft);
                    const y = speedLine(entry.rpm);
                    if (i === 0) ctx.moveTo(x, y);
                    else ctx.lineTo(x, y);
                });
                ctx.strokeStyle = '#38bdf8';
                ctx.lineWidth = 2;
                ctx.stroke();

                ctx.save();
                ctx.beginPath();
                ctx.rect(plotLeft, plotTop, plotRight - plotLeft, plotBottom - plotTop);
                ctx.clip();
                // clip() 不会清空当前路径，必须重新 beginPath，
                // 否则最后那次 stroke() 会把上面这个裁剪矩形也描出来（多一圈绿框）。
                ctx.beginPath();
                // 勾选“误差不计负载角”时，把当时的负载角加回去：
                // follow_error 稳定在 -L，加上 L 后归零，图上就能看出“真实偏差”。
                // 用 entry.loadAngle（采样时刻的值）而不是当前值，速度变化段才不失真。
                // 【性能】勾选状态提到循环外：原来每个点都查一次 DOM，2000 个点就是 2000 次查询/帧。
                const excludeLoadAngle = document.getElementById('waveExcludeLoadAngleInput');
                const excludeLoadAngleChecked = !!(excludeLoadAngle && excludeLoadAngle.checked);
                filtered.forEach((entry, i) => {
                    const x = plotLeft + ((now - entry.t) / windowMs) * (plotRight - plotLeft);
                    const loadAngleComp = excludeLoadAngleChecked
                        ? Number(entry.loadAngle || 0)
                        : 0;
                    // 勾选「误差不计负载角」时扣除负载角：
                    // 注意固件发来的负载角是**带符号**的，且和 follow_error 同号：
                    //   正向运动：follow_error = -1.5°，loadAngle = -1.5°
                    //   反向运动：follow_error = +1.5°，loadAngle = +1.5°
                    // 所以是「减去」而不是「加上」（旧版本固件发正值时才用 +）。
                    const y = followErrorLine(entry.followError - loadAngleComp);
                    if (i === 0) ctx.moveTo(x, y);
                    else ctx.lineTo(x, y);
                });
                ctx.strokeStyle = '#22c55e';
                ctx.lineWidth = 2;
                ctx.stroke();
                ctx.restore();
            }

            // 标出固件报告“运动结束”的时刻：这条线左侧是运动段，右侧是沉降/静止段。
            // 有了它就能一眼看出末值是收敛到 0 附近，还是停在了某个负载角上。
            if (state.waveStoppedAt && now >= state.waveStoppedAt && now - state.waveStoppedAt <= windowMs) {
                const stopX = plotLeft + ((now - state.waveStoppedAt) / windowMs) * (plotRight - plotLeft);
                ctx.save();
                ctx.setLineDash([4, 4]);
                ctx.strokeStyle = 'rgba(251,191,36,0.85)';
                ctx.lineWidth = 1.5;
                ctx.beginPath();
                ctx.moveTo(stopX, plotTop);
                ctx.lineTo(stopX, plotBottom);
                ctx.stroke();
                ctx.restore();
            }

            ctx.fillStyle = 'rgba(148,163,184,0.75)';
            ctx.textAlign = 'right';
            ctx.fillText('10 s / 100 ms grid', plotRight, height - 6);
            ctx.textAlign = 'left';
        }

        function ensureWaveTimerAlive() {
            if (!state.waveCaptureActive || state.motionWaveTimer !== null) return;
            // 重建定时器，内容和 startWaveCapture 里完全一致
            state.motionWaveTimer = window.setInterval(() => {
                if (!state.waveCaptureActive) return;
                pushWaveSample();
                drawWaveform();
                if (document.getElementById('wavePopup').classList.contains('visible')) {
                    drawWaveform('motionWavePopupCanvas');
                }
            }, 40);
            // 只在头几次打日志，避免控制台被刷屏
            if (waveWatchdogLogCount < 5) {
                waveWatchdogLogCount++;
                console.warn('[WAVE] 检测到采集标志为真但定时器丢失，已自动重启（第 ' + waveWatchdogLogCount + ' 次）');
            }
        }

        function stopWaveCapture() {
            // 【临时诊断】把调用栈打出来：波形“时不时卡住”时，先确认是不是这里被误调用。
            // 控制台会看到 [WAVE] stop 加一串调用行，第一个业务函数就是元凶。
            // 定位完可以删掉这一行。
            console.trace('[WAVE] stopWaveCapture');
            if (state.waveStopFreezeTimer !== null) {
                window.clearTimeout(state.waveStopFreezeTimer);
                state.waveStopFreezeTimer = null;
            }
            if (!state.waveCaptureActive) return;
            state.waveFrozenAt = state.wavePaused ? state.wavePausedAt : performance.now();
            state.waveCaptureActive = false;
            state.wavePaused = false;
            state.wavePausedAt = 0;
            if (state.motionWaveTimer !== null) {
                window.clearInterval(state.motionWaveTimer);
                state.motionWaveTimer = null;
            }
            document.getElementById('pauseWaveBtn').textContent = '暂停查看';
        }

        function toggleWavePopup(forceOpen) {
            const popup = document.getElementById('wavePopup');
            if (!popup) return;
            const nextVisible = typeof forceOpen === 'boolean' ? forceOpen : !popup.classList.contains('visible');
            popup.classList.toggle('visible', nextVisible);
            if (nextVisible && (state.waveCaptureActive || state.waveHistory.length > 0)) {
                drawWaveform('motionWavePopupCanvas');
            }
        }

        function toggleWavePopupMaximize() {
            const popup = document.getElementById('wavePopup');
            const button = document.getElementById('maximizeWavePopupBtn');
            if (!popup || !button) return;
            const isMaximized = popup.classList.toggle('maximized');
            button.textContent = isMaximized ? '还原' : '最大化';
            if (state.waveCaptureActive || state.waveHistory.length > 0) {
                drawWaveform('motionWavePopupCanvas');
            }
        }

        function bindWavePopupDrag() {
            const popup = document.getElementById('wavePopup');
            const header = document.getElementById('wavePopupHeader');
            if (!popup || !header) return;

            let dragState = null;

            header.addEventListener('mousedown', (event) => {
                if (popup.classList.contains('maximized')) return;
                dragState = {
                    startX: event.clientX,
                    startY: event.clientY,
                    left: popup.offsetLeft,
                    top: popup.offsetTop
                };
            });

            window.addEventListener('mousemove', (event) => {
                if (!dragState) return;
                const dx = event.clientX - dragState.startX;
                const dy = event.clientY - dragState.startY;
                popup.style.left = `${dragState.left + dx}px`;
                popup.style.top = `${dragState.top + dy}px`;
                popup.style.right = 'auto';
                popup.style.bottom = 'auto';
            });

            window.addEventListener('mouseup', () => {
                dragState = null;
            });
        }

