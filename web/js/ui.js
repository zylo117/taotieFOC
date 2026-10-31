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

        function updateFaultStatus() {
            const hasFault = state.encoderFault || state.magneticFault || state.outputStopped;
            const summary = state.encoderFault ? '编码器读取失败' : state.magneticFault ? '磁场报警' : state.outputStopped ? '已停止输出' : '正常';
            state.faultSummary = summary;

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

