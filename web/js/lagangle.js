// Lag-angle (lag angle) calibration: sweep, sampling, fit upload (module 8).
// Depends on: state, CUSTOM_REGS, writeRegister/writeRegisterAndWaitAck/readRegisterValue, log, drawLagAngleChart.
// Load order: ... -> ui -> waveform -> lagangle -> ...

        function drawLagAngleChart(points, maxSpeedRpm = 3000) {
            const canvas = document.getElementById('lagAngleChart');
            const rect = canvas.getBoundingClientRect();
            const pixelRatio = window.devicePixelRatio || 1;
            canvas.width = Math.max(1, Math.round(rect.width * pixelRatio));
            canvas.height = Math.max(1, Math.round(rect.height * pixelRatio));
            const context = canvas.getContext('2d');
            context.scale(pixelRatio, pixelRatio);
            context.clearRect(0, 0, rect.width, rect.height);

            const left = 58;
            const right = rect.width - 18;
            const top = 18;
            const bottom = rect.height - 42;
            const values = points.flatMap((point) => Number.isFinite(point.fitAngleDeg)
                ? [point.angleDeg, point.fitAngleDeg]
                : [point.angleDeg]);
            const minValue = values.length ? Math.min(...values) : -1;
            const maxValue = values.length ? Math.max(...values) : 1;
            const padding = Math.max(0.5, (maxValue - minValue) * 0.12);
            const minAngle = minValue - padding;
            const maxAngle = maxValue + padding;
            const x = (rpm) => left + (rpm / Math.max(1, maxSpeedRpm)) * (right - left);
            const y = (angle) => bottom - ((angle - minAngle) / (maxAngle - minAngle)) * (bottom - top);

            context.font = '12px Segoe UI, sans-serif';
            context.lineWidth = 1;
            context.strokeStyle = 'rgba(148, 163, 184, 0.2)';
            context.fillStyle = '#94a3b8';
            for (let tick = 0; tick <= 4; tick++) {
                const tickY = top + (tick / 4) * (bottom - top);
                const angle = maxAngle - (tick / 4) * (maxAngle - minAngle);
                context.beginPath();
                context.moveTo(left, tickY);
                context.lineTo(right, tickY);
                context.stroke();
                context.fillText(`${angle.toFixed(2)}°`, 6, tickY + 4);
            }
            const speedTicks = 6;
            for (let tick = 0; tick <= speedTicks; tick++) {
                const rpm = maxSpeedRpm * tick / speedTicks;
                const tickX = x(rpm);
                context.beginPath();
                context.moveTo(tickX, top);
                context.lineTo(tickX, bottom);
                context.stroke();
                context.fillText(String(Math.round(rpm)), tickX - 12, bottom + 20);
            }

            if (points.length > 0) {
                context.strokeStyle = '#f59e0b';
                context.lineWidth = 2;
                context.beginPath();
                const fittedPoints = points.filter((point) => Number.isFinite(point.fitAngleDeg))
                    .sort((a, b) => a.speedRpm - b.speedRpm);
                fittedPoints.forEach((point, index) => {
                    if (index === 0) context.moveTo(x(point.speedRpm), y(point.fitAngleDeg));
                    else context.lineTo(x(point.speedRpm), y(point.fitAngleDeg));
                });
                if (fittedPoints.length > 1) context.stroke();

                context.fillStyle = '#38bdf8';
                for (const point of points) {
                    context.beginPath();
                    context.arc(x(point.speedRpm), y(point.angleDeg), 2.5, 0, Math.PI * 2);
                    context.fill();
                }
            } else {
                context.fillStyle = '#94a3b8';
                context.fillText('尚无固件校准点', left + 12, top + 24);
            }
            context.fillStyle = '#cbd5e1';
            context.fillText('速度 (RPM)', (left + right) / 2 - 30, rect.height - 8);
            context.save();
            context.translate(15, (top + bottom) / 2);
            context.rotate(-Math.PI / 2);
            context.fillText('滞后角 (°)', -34, 0);
            context.restore();
        }

        async function refreshLagAngleCalibration() {
            if (!state.connected) throw new Error('串口未连接');
            const fitted = await readRegisterValue(CUSTOM_REGS.LAG_ANGLE_FIT);
            if ((fitted >>> 0) !== 1) throw new Error('固件中没有已保存的有效拟合');
            const count = await readRegisterValue(CUSTOM_REGS.LAG_ANGLE_POINT_COUNT);
            const safeCount = Math.min(1024, count >>> 0);
            const points = [];
            for (let index = 0; index < safeCount; index++) {
                await writeRegisterAndWaitAck(CUSTOM_REGS.LAG_ANGLE_POINT_INDEX, index);
                const speedRpm = await readRegisterValue(CUSTOM_REGS.LAG_ANGLE_POINT_SPEED_RPM);
                const angleMdeg = await readRegisterValue(CUSTOM_REGS.LAG_ANGLE_POINT_MDEG);
                await writeRegisterAndWaitAck(CUSTOM_REGS.LAG_ANGLE_QUERY_SPEED_RPM, speedRpm >>> 0);
                const fitMdeg = await readRegisterValue(CUSTOM_REGS.LAG_ANGLE_QUERY_MDEG);
                points.push({
                    speedRpm: speedRpm >>> 0,
                    angleDeg: (angleMdeg | 0) / 1000,
                    fitAngleDeg: (fitMdeg | 0) / 1000
                });
            }
            const coefficient = (await readRegisterValue(CUSTOM_REGS.LAG_ANGLE_FIT_SLOPE) | 0) / 1000000;
            const intercept = (await readRegisterValue(CUSTOM_REGS.LAG_ANGLE_FIT_INTERCEPT) | 0) / 1000;
            const exponent = (await readRegisterValue(CUSTOM_REGS.LAG_ANGLE_FIT_EXPONENT) | 0) / 1000000;
            document.getElementById('lagAngleProgress').textContent = `${safeCount} / ${safeCount}`;
            document.getElementById('lagAngleEquation').textContent =
                `固件幂律拟合：滞后角 = ${intercept.toFixed(3)}° + ${coefficient.toFixed(8)} × RPM^${exponent.toFixed(4)}`;
            state.lagAngleMaxSpeedRpm = Math.max(1, ...points.map((point) => point.speedRpm));
            drawLagAngleChart(points, state.lagAngleMaxSpeedRpm);
            document.getElementById('lagAngleStatus').textContent = `已读取 ${safeCount} 个固件保存点及拟合查询值`;
        }

        function lagAngleSweepConfig() {
            const startRpm = Math.round(Number(document.getElementById('lagAngleStartRpm').value));
            const stopRpm = Math.round(Number(document.getElementById('lagAngleStopRpm').value));
            const stepRpm = Math.round(Number(document.getElementById('lagAngleStepRpm').value));
            const intervalSec = Number(document.getElementById('lagAngleIntervalSec').value);
            const sampleWindowSec = Number(document.getElementById('lagAngleSampleWindowSec').value);
            const samplePeriodMs = Number(document.getElementById('lagAngleSamplePeriodMs').value);
            const accelRpmS = Math.round(Number(document.getElementById('lagAngleAccelRpmS').value));
            // 速度容差（%）：实际转速与目标偏差超过这个比例就判“未达标”，触发重试。
            // 默认 12%：既不会因为编码器噪声误判，又能及时发现电机真的跑不到。
            const toleranceEl = document.getElementById('lagAngleTolerancePct');
            const tolerancePct = toleranceEl ? Number(toleranceEl.value) || 12 : 12;
            if (![startRpm, stopRpm, stepRpm, intervalSec, sampleWindowSec, samplePeriodMs, accelRpmS].every(Number.isFinite) ||
                startRpm < 0 || stopRpm < 0 || startRpm > 12000 || stopRpm > 12000 || startRpm === stopRpm ||
                stepRpm < 1 || intervalSec < 2 || sampleWindowSec <= 0 || sampleWindowSec > intervalSec ||
                samplePeriodMs < 20 || accelRpmS < 1) {
                throw new Error('请检查速度范围、步进量、采样窗口和间隔设置');
            }
            const direction = stopRpm > startRpm ? 1 : -1;
            // 固件 side 的 LagAngle::MAX_POINTS 已经从 1024 降到 32（省 14.9KB RAM、
            // 启动拟合从 1.6s 降到 50ms）。点数超限时 beginUpload 会直接返回 false，
            // 后面每个 setPoint 都静默失败，最后只报一句“拟合失败”，很难查。
            // 所以在前端就卡住，并直接告诉用户步进要调到多少。
            const firmwareMaxPoints = 128;
            const maxSingleLegPoints = Math.ceil(firmwareMaxPoints / 2);   // 单程 64，往返 2*64-1 = 127
            const speeds = [startRpm];
            while (speeds[speeds.length - 1] !== stopRpm) {
                const next = speeds[speeds.length - 1] + direction * stepRpm;
                speeds.push(direction > 0 ? Math.min(next, stopRpm) : Math.max(next, stopRpm));
                if (speeds.length > maxSingleLegPoints) {
                    const suggested = Math.max(stepRpm, Math.ceil(Math.abs(stopRpm - startRpm) / (maxSingleLegPoints - 1)));
                    throw new Error(`单程速度点 ${speeds.length} 个，往返后超过固件上限 ${firmwareMaxPoints} 点；请把速度步进改到 ${suggested} RPM 或更大`);
                }
            }
            for (let index = speeds.length - 2; index >= 0; index--) {
                speeds.push(speeds[index]);
            }
            if (speeds.length < 2) throw new Error('拟合至少需要两个速度点');
            return {startRpm, stopRpm, stepRpm, intervalSec, sampleWindowSec, samplePeriodMs, accelRpmS, tolerancePct, speeds};
        }

        function lagAngleDelay(ms) {
            return new Promise((resolve) => window.setTimeout(resolve, Math.max(0, ms)));
        }

        async function stopLagAngleMotion() {
            if (state.connected) await writeRegister(0x11F, 0);
        }

        async function waitForLagAngleSpeed(targetRpm, accelRpmS, intervalMs) {
            if (targetRpm === 0) {
                await stopLagAngleMotion();
                await lagAngleDelay(300);
                return;
            }
            await writeRegisterAndWaitAck(CUSTOM_REGS.CLOSED_LOOP_ENABLE, 0);  // 关闭闭环
            // 关键修正：测速控制/滞后角校准的起步动作，必须先给 startMotion() 一个有限且合理的 pulseCount。
            // 如果不先写 0x11D，就会让 firmware 在 startMotion() 里拿到旧值/脏值，出现超大脉冲数和起步卡死。
            const pulseCount = targetRpm / 60 * 360 / Number(document.getElementById('stepAngle').value || 1.8) * Number(document.getElementById('microsteps').value || 32) * Number(document.getElementById('lagAngleIntervalSec').value);;
            await writeRegisterAndWaitAck(0x11A, Math.min(10, targetRpm));
            await writeRegisterAndWaitAck(0x11B, targetRpm);
            await writeRegisterAndWaitAck(0x11C, accelRpmS);
            await writeRegisterAndWaitAck(0x11D, pulseCount);
            await writeRegisterAndWaitAck(0x11E, 2);
            await writeRegisterAndWaitAck(0x123, 200);
            await writePidParameters();
            await writeRegisterAndWaitAck(0x11F, 1);

            const timeoutMs = Math.max(15000, Math.min(120000, (targetRpm / accelRpmS * 2 + 5) * 1000));
            const deadline = performance.now() + timeoutMs;
            const tolerance = Math.max(6, Math.min(24, targetRpm * 0.12));
            const avgWindowMs = Math.max(1500, Math.min(5000, targetRpm * 30));
            const avgPeriodMs = 80;
            const speedHistory = [];
            let stableReads = 0;
            let lastMeasuredRpm = 0;
            const startWindowAt = performance.now();
            while (performance.now() < deadline && !state.lagAngleCalibrationCancelled) {
                const measuredRpm = Math.abs(readSigned32(await readRegisterValue(CUSTOM_REGS.SPEED_RPM)));
                const now = performance.now();
                speedHistory.push({time: now, rpm: measuredRpm});
                while (speedHistory.length > 0 && now - speedHistory[0].time > avgWindowMs) {
                    speedHistory.shift();
                }
                const averagedMeasuredRpm = speedHistory.length === 0
                    ? measuredRpm
                    : speedHistory.reduce((sum, sample) => sum + sample.rpm, 0) / speedHistory.length;
                lastMeasuredRpm = averagedMeasuredRpm;
                const withinTolerance = Math.abs(averagedMeasuredRpm - targetRpm) <= tolerance;
                stableReads = withinTolerance ? stableReads + 1 : 0;
                const statusText = `目标 ${targetRpm} RPM / 平均 ${averagedMeasuredRpm.toFixed(1)} RPM / 容差 ±${tolerance.toFixed(1)} RPM / 稳定 ${stableReads}/3`;
                document.getElementById('lagAngleStatus').textContent = `等待稳定：${statusText}`;
                if (stableReads >= 3) return;
                await lagAngleDelay(Math.max(avgPeriodMs, 200));
            }
            if (state.lagAngleCalibrationCancelled) throw new Error('已停止校准');
            throw new Error(`转速 ${targetRpm} RPM 未能在限定时间内稳定（平均 ${lastMeasuredRpm.toFixed(1)} RPM，目标 ${targetRpm} RPM，容差 ±${tolerance.toFixed(1)} RPM）`);
        }

        function readSigned32(value) {
            return value | 0;
        }

        async function collectLagAngleMean(config) {
            const windowMs = config.sampleWindowSec * 1000;
            const startAt = performance.now();
            const endAt = startAt + windowMs;
            const samples = [];
            let nextAt = startAt;
            while (performance.now() < endAt && !state.lagAngleCalibrationCancelled) {
                const errorMdeg = readSigned32(await readRegisterValue(CUSTOM_REGS.FOLLOW_ERROR_DEG));
                samples.push(-errorMdeg);
                nextAt += config.samplePeriodMs;
                await lagAngleDelay(Math.max(0, nextAt - performance.now()));
            }
            if (state.lagAngleCalibrationCancelled) throw new Error('已停止校准');
            if (samples.length < 3) throw new Error('采样窗口内有效样本不足，至少需要 3 个');
            let sum = 0;
            let min = Infinity;
            let max = -Infinity;
            for (const sample of samples) {
                sum += sample;
                min = Math.min(min, sample);
                max = Math.max(max, sample);
            }
            return Math.round((sum - min - max) / (samples.length - 2));
        }

        // ===== 连续扫速：启动 / 变速 / 等达标 =====
        // 旧流程每点都要 stopLagAngleMotion() + 重新启动，每次从 0 加速，
        // 一段一段测极慢。现在固件支持速度模式连续运行（0x11D = 0 → 无限跑）
        // 和运行时变速（改 0x11B 即生效），所以这里只启动一次。
        async function startLagAngleSweep(targetRpm, accelRpmS) {
            await writeRegisterAndWaitAck(CUSTOM_REGS.CLOSED_LOOP_ENABLE, 0);
            // 【必须非零】固件 startMotion() 里有兜底：
            //   if (motion_max_rpm_ <= 0) motion_max_rpm_ = angle_max_rpm_ ?: 1200;
            // 所以校准从 0 RPM 开始时，如果直接把 0x11B 写成 0，
            // 固件会拿“网页闭环最大速度”（比如 250）当目标 -> 电机直接冲到 250，
            // 而网页还以为在测 50 RPM 的点。
            // 这里强制最小 10 RPM 起步；0 RPM 那个点本来就不采样（滞后角=0）。
            const safeRpm = Math.max(10, Math.round(targetRpm));
            await writeRegisterAndWaitAck(0x11A, Math.min(10, safeRpm));
            await writeRegisterAndWaitAck(0x11B, safeRpm);
            await writeRegisterAndWaitAck(0x11C, accelRpmS);
            await writeRegisterAndWaitAck(0x11D, 0);   // ★ 0 = 无限跑，不停机
            await writeRegisterAndWaitAck(0x11E, 2);   // 速度正转
            await writeRegisterAndWaitAck(0x123, 200);
            await writePidParameters();
            await writeRegisterAndWaitAck(0x11F, 1);
        }

        // 只改目标速度，不碰其它寄存器
        // 同样要防 0：写 0 会让固件 startMotion 的兜底逻辑拿“闭环最大速度”当目标。
        async function changeLagAngleSweepSpeed(targetRpm) {
            await writeRegisterAndWaitAck(0x11B, Math.max(10, Math.round(targetRpm)));
        }

        // 等转速**稳定**在容差范围内，返回 true/false（不抛异常）
        //
        // 【关键】不能只判一次“进入容差”就返回：
        //   目标 600RPM、容差 12%（±72）时，速度刚到 528 就满足条件，
        //   但那时还在加速上升段，接下来 1 秒的采样窗口会覆盖 528→600 整个过程，
        //   测出来的滞后角对应的其实是“比标签偏低的一截转速”，时机也对不上。
        //   必须“连续 kStableSamples 次（80ms x 6 = 480ms）都在容差内”才算真稳。
        // 等转速到位，返回 true/false。
        //
        // 【重要修正】早期版本完全依赖 0x120（即时转速）判断，一旦遥测没到或延迟大，
        // 就会永远判“未达标” -> 重试 3 次 -> 跳过这个点 -> 下一个点更高 ->
        // 表现为“电机一直在加速、速度远超目标”（实测目标 250 却跑到 500+）。
        //
        // 现在的策略：
        //   主判据 = 理论加速时间（accel 段 + 稳定段），到点就认为到位；
        //   辅助   = 期间如果收到 0x120 且连续 6 次在容差内，提前返回（省时间）。
        // 这样即使遥测完全没到，扫速也能按预期跑完，不会自己抬高目标。
        async function waitForSpeedReached(targetRpm, tolerancePct, timeoutMs) {
            const start = performance.now();
            const deadline = start + timeoutMs;
            const tolerance = Math.max(5, targetRpm * tolerancePct / 100);
            // 【稳定判定放宽】原来是“连续 6 次都必须精确落在容差内” ——
            // 编码器测速本身有 ±2~5 RPM 噪声，低速时（50RPM、容差 6）很容易被噪声
            // 打断一次，stableCount 归零重来，表现为“明明到了又说没到”。
            // 改成“8 次里至少 6 次在容差内”（允许 2 次抖动），鲁棒得多。
            const kWindowSamples = 8;
            const kRequiredInBand = 6;
            let stableCount = 0;
            let windowCount = 0;
            let maxActual = 0;
            while (performance.now() < deadline) {
                if (state.lagAngleCalibrationCancelled) return false;
                const actual = Math.abs(Number(state.motionSpeedRpm || 0));
                if (actual > maxActual) maxActual = actual;
                if (actual > 0 && Math.abs(actual - targetRpm) <= tolerance) {
                    stableCount++;
                }
                // 滑动窗口：8 个采样里至少 6 个在带内就判稳定（允许 2 次噪声抖动）
                windowCount++;
                if (windowCount >= kWindowSamples) {
                    if (stableCount >= kRequiredInBand) {
                        log(`[滞后角] ${targetRpm}RPM 遥测确认到位（实测 ${actual.toFixed(1)}，${kWindowSamples} 次中 ${stableCount} 次在带内）`);
                        return true;
                    }
                    // 窗口清空重新统计
                    windowCount = 0;
                    stableCount = 0;
                }
                await lagAngleDelay(80);
            }
            // 【兜底与告警】
            // 遥测完全没到（maxActual == 0）时没有依据判断速度，只能按时间推进，
            // 但必须明确告警——否则“目标 50 却跑到 250”这种情况会被静默放过。
            // 遥测正常但速度不在容差内 → 返回 false，让调用方重试，
            // 重试 3 次都不行就中止整个扫速（而不是自欺欺人地继续）。
            if (maxActual <= 0.5) {
                log(`[滞后角] ${targetRpm}RPM 遥测完全缺失（maxActual=0），按时间推进；请检查 0x120 遥测链路`);
                return true;
            }
            log(`[滞后角] ${targetRpm}RPM 未达标：遥测最大 ${maxActual.toFixed(1)}，容差 ±${tolerance.toFixed(1)}`);
            return false;
        }

        async function captureLagAnglePoints(config) {
            const points = [];
            const windowMs = config.sampleWindowSec * 1000;
            const intervalMs = config.intervalSec * 1000;
            const centeredDelayMs = (intervalMs - windowMs) / 2;
            const tolerancePct = Number(config.tolerancePct) > 0 ? Number(config.tolerancePct) : 12;
            const maxRetries = 3;
            let consecutiveFailures = 0;
            let directionReversed = false;   // 往返：只切一次方向

            // 【只启动一次】用第一个**非零**速度启动：
            // 0 RPM 那个点不参与运动（没有转速就没有滞后角），下面循环里直接记 0 并跳过。
            // 如果拿 0 去启动，0x11B 会被钳到 10，而前端以为在测 0 -> 速度对不上就报错。
            const startupRpm = config.speeds.find((s) => s > 0) ?? 10;
            await startLagAngleSweep(startupRpm, config.accelRpmS);
            await lagAngleDelay(500);

            for (let index = 0; index < config.speeds.length; index++) {
                if (state.lagAngleCalibrationCancelled) throw new Error('已停止校准');
                const speedRpm = config.speeds[index];

                // 【往返换向】第一次出现“速度比上一个点小”说明进入下降段。
                // 固件速度模式只认 0x11E（2=正转 3=反转），改它会先刹到 0 再反向加速，
                // 所以这里切完要留一段换向时间，再去等目标转速。
                if (!directionReversed && index > 0 && speedRpm < config.speeds[index - 1]) {
                    document.getElementById('lagAngleStatus').textContent =
                        `到达最高点，切换反向（先刹到 0）...`;
                    await writeRegisterAndWaitAck(0x11E, 3);
                    directionReversed = true;
                    await lagAngleDelay(800);
                }

                // 0 RPM：不启动、不等待、不采样（滞后角定义为 0）
                if (speedRpm === 0) {
                    points.push({speedRpm: 0, angleMdeg: 0, angleDeg: 0, fitAngleDeg: null});
                    document.getElementById('lagAngleProgress').textContent = `${index + 1} / ${config.speeds.length}`;
                    document.getElementById('lagAngleStatus').textContent = `0 RPM 直接记 0（不采样）`;
                    continue;
                }

                await changeLagAngleSweepSpeed(speedRpm);
                // 【状态栏提前到这个点】原来是在采样完成后才写，导致“采样 200 却显示 150”
                // 的错位——用户看到的状态总是落后一拍。
                document.getElementById('lagAngleStatus').textContent =
                    `等待稳定：${speedRpm} RPM，${index + 1}/${config.speeds.length}`;

                // 【同点重试】速度不达标最多重试 maxRetries 次（重新下发目标）
                let reached = false;
                const prevRpm = index > 0 ? config.speeds[index - 1] : 0;
                const settleMs = Math.max(2500,
                    Math.abs(speedRpm - prevRpm) / Math.max(1, config.accelRpmS) * 3000 + 1500);
                for (let attempt = 0; attempt < maxRetries && !reached; attempt++) {
                    if (state.lagAngleCalibrationCancelled) throw new Error('已停止校准');
                    reached = await waitForSpeedReached(speedRpm, tolerancePct, settleMs);
                    if (!reached) {
                        document.getElementById('lagAngleStatus').textContent =
                            `${speedRpm} RPM 第 ${attempt + 1} 次未达标，重试中...`;
                        await changeLagAngleSweepSpeed(speedRpm);
                    }
                }

                if (!reached) {
                    // 【连续 3 次不达标】判定已达电机极限，停止扫速
                    consecutiveFailures++;
                    document.getElementById('lagAngleStatus').textContent =
                        `${speedRpm} RPM 无法达标（连续 ${consecutiveFailures} 次）`;
                    if (consecutiveFailures >= 3) {
                        await stopLagAngleMotion();
                        throw new Error(
                            `连续 ${consecutiveFailures} 个速度点无法达标（最后：${speedRpm} RPM，容差 ${tolerancePct}%），已停止扫速`);
                    }
                    continue;   // 跳过这个点，继续下一个
                }
                consecutiveFailures = 0;

                const holdStart = performance.now();
                await lagAngleDelay(centeredDelayMs);
                // 【拟合用幅值，不能带符号】
                // collectLagAngleMean 读的是 0x121（follow_error = 实际 - 命令）：
                //   正转时滞后角是负的（转子落后）
                //   反转时是正的
                // 两者幅值相同、符号相反。如果直接上传，往返序列会在同一点上出现
                // 一正一负两个值，幂律拟合会被拉成一条接近零的斜线，完全失真。
                // 滞后角的物理量是「幅值」，方向由转速符号决定（固件里已处理）。
                const rawAngleMdeg = speedRpm === 0 ? 0 : await collectLagAngleMean(config);
                const angleMdeg = Math.abs(rawAngleMdeg);
                points.push({speedRpm, angleMdeg, angleDeg: angleMdeg / 1000, fitAngleDeg: null});
                document.getElementById('lagAngleProgress').textContent = `${index + 1} / ${config.speeds.length}`;
                document.getElementById('lagAngleStatus').textContent = `采样中：${speedRpm} RPM，${index + 1}/${config.speeds.length}`;
                drawLagAngleChart(points, Math.max(config.startRpm, config.stopRpm, 1));
                await lagAngleDelay(Math.max(0, holdStart + intervalMs - performance.now()));
            }
            await stopLagAngleMotion();
            return points;
        }

        async function uploadLagAnglePoints(points) {
            if (points.length > 32) {
                throw new Error(`拟合点 ${points.length} 个，超过固件上限 32`);
            }
            await writeRegisterAndWaitAck(CUSTOM_REGS.LAG_ANGLE_UPLOAD_COUNT, points.length);
            for (let index = 0; index < points.length; index++) {
                const point = points[index];
                await writeRegisterAndWaitAck(CUSTOM_REGS.LAG_ANGLE_POINT_INDEX, index);
                await writeRegisterAndWaitAck(CUSTOM_REGS.LAG_ANGLE_POINT_SPEED_RPM, point.speedRpm);
                await writeRegisterAndWaitAck(CUSTOM_REGS.LAG_ANGLE_POINT_MDEG, point.angleMdeg >>> 0);
                await writeRegisterAndWaitAck(CUSTOM_REGS.LAG_ANGLE_POINT_COMMIT, 1);
                document.getElementById('lagAngleStatus').textContent = `上传均值点：${index + 1}/${points.length}`;
            }
            await writeRegisterAndWaitAck(CUSTOM_REGS.LAG_ANGLE_FIT, 1);
            const fitSaved = await readRegisterValue(CUSTOM_REGS.LAG_ANGLE_FIT);
            if ((fitSaved >>> 0) !== 1) throw new Error('固件拟合或 Flash 保存失败');
        }

        async function startLagAngleCalibration() {
            if (!state.connected) {
                document.getElementById('lagAngleStatus').textContent = '请先连接串口';
                return;
            }
            let config;
            try {
                config = lagAngleSweepConfig();
                // 【共振带自动避让】见 excludeResonanceBandSpeeds 注释。
                config.speeds = await excludeResonanceBandSpeeds(config.speeds, '滞后角校准');
                if (config.speeds.length < 2) {
                    throw new Error('避开共振带后不足 2 个速度点，无法拟合；请调整速度范围或共振带设置');
                }
            } catch (error) {
                document.getElementById('lagAngleStatus').textContent = error.message;
                return;
            }
            state.lagAngleCalibrationRunning = true;
            state.lagAngleCalibrationCancelled = false;
            const startButton = document.getElementById('startLagAngleCalibrationBtn');
            const cancelButton = document.getElementById('cancelLagAngleCalibrationBtn');
            startButton.disabled = true;
            cancelButton.disabled = false;
            document.getElementById('closeLagAngleDialogBtn').disabled = true;
            document.getElementById('lagAngleProgress').textContent = `0 / ${config.speeds.length}`;
            document.getElementById('lagAngleEquation').textContent = '固件拟合函数：等待本轮采样上传与拟合';
            try {
                const points = await captureLagAnglePoints(config);
                document.getElementById('lagAngleStatus').textContent = '上传采样均值并由固件拟合...';
                await uploadLagAnglePoints(points);
                document.getElementById('lagAngleStatus').textContent = '拟合完成，读取固件查询值...';
                await refreshLagAngleCalibration();
                document.getElementById('lagAngleStatus').textContent = '采样完成；曲线查询值及拟合系数均来自固件';
            } catch (error) {
                await stopLagAngleMotion().catch(() => {});
                document.getElementById('lagAngleStatus').textContent = `校准失败：${error.message}`;
                log(`[滞后角校准] ${error.message}`);
            } finally {
                state.lagAngleCalibrationRunning = false;
                startButton.disabled = false;
                cancelButton.disabled = true;
                document.getElementById('closeLagAngleDialogBtn').disabled = false;
            }
        }

        async function openLagAngleCalibration() {
            const dialog = document.getElementById('lagAngleDialog');
            if (!dialog.open) dialog.showModal();
            if (!state.connected) {
                document.getElementById('lagAngleStatus').textContent = '请先连接串口以读取固件数据';
                drawLagAngleChart([], 3000);
                return;
            }
            try {
                await refreshLagAngleCalibration();
            } catch (error) {
                document.getElementById('lagAngleStatus').textContent = `读取失败：${error.message}`;
                drawLagAngleChart([], 3000);
            }
        }

