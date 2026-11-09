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
                context.fillText(`${angle.toFixed(2)}掳`, 6, tickY + 4);
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
                context.fillText('灏氭棤鍥轰欢鏍″噯鐐?, left + 12, top + 24);
            }
            context.fillStyle = '#cbd5e1';
            context.fillText('閫熷害 (RPM)', (left + right) / 2 - 30, rect.height - 8);
            context.save();
            context.translate(15, (top + bottom) / 2);
            context.rotate(-Math.PI / 2);
            context.fillText('婊炲悗瑙?(掳)', -34, 0);
            context.restore();
        }

        async function refreshLagAngleCalibration() {
            if (!state.connected) throw new Error('涓插彛鏈繛鎺?);
            const fitted = await readRegisterValue(CUSTOM_REGS.LAG_ANGLE_FIT);
            if ((fitted >>> 0) !== 1) throw new Error('鍥轰欢涓病鏈夊凡淇濆瓨鐨勬湁鏁堟嫙鍚?);
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
                `鍥轰欢骞傚緥鎷熷悎锛氭粸鍚庤 = ${intercept.toFixed(3)}掳 + ${coefficient.toFixed(8)} 脳 RPM^${exponent.toFixed(4)}`;
            state.lagAngleMaxSpeedRpm = Math.max(1, ...points.map((point) => point.speedRpm));
            drawLagAngleChart(points, state.lagAngleMaxSpeedRpm);
            document.getElementById('lagAngleStatus').textContent = `宸茶鍙?${safeCount} 涓浐浠朵繚瀛樼偣鍙婃嫙鍚堟煡璇㈠€糮;
        }

        function lagAngleSweepConfig() {
            const startRpm = Math.round(Number(document.getElementById('lagAngleStartRpm').value));
            const stopRpm = Math.round(Number(document.getElementById('lagAngleStopRpm').value));
            const stepRpm = Math.round(Number(document.getElementById('lagAngleStepRpm').value));
            const intervalSec = Number(document.getElementById('lagAngleIntervalSec').value);
            const sampleWindowSec = Number(document.getElementById('lagAngleSampleWindowSec').value);
            const samplePeriodMs = Number(document.getElementById('lagAngleSamplePeriodMs').value);
            const accelRpmS = Math.round(Number(document.getElementById('lagAngleAccelRpmS').value));
            // 閫熷害瀹瑰樊锛?锛夛細瀹為檯杞€熶笌鐩爣鍋忓樊瓒呰繃杩欎釜姣斾緥灏卞垽鈥滄湭杈炬爣鈥濓紝瑙﹀彂閲嶈瘯銆?            // 榛樿 12%锛氭棦涓嶄細鍥犱负缂栫爜鍣ㄥ櫔澹拌鍒わ紝鍙堣兘鍙婃椂鍙戠幇鐢垫満鐪熺殑璺戜笉鍒般€?            const toleranceEl = document.getElementById('lagAngleTolerancePct');
            const tolerancePct = toleranceEl ? Number(toleranceEl.value) || 12 : 12;
            if (![startRpm, stopRpm, stepRpm, intervalSec, sampleWindowSec, samplePeriodMs, accelRpmS].every(Number.isFinite) ||
                startRpm < 0 || stopRpm < 0 || startRpm > 12000 || stopRpm > 12000 || startRpm === stopRpm ||
                stepRpm < 1 || intervalSec < 2 || sampleWindowSec <= 0 || sampleWindowSec > intervalSec ||
                samplePeriodMs < 20 || accelRpmS < 1) {
                throw new Error('璇锋鏌ラ€熷害鑼冨洿銆佹杩涢噺銆侀噰鏍风獥鍙ｅ拰闂撮殧璁剧疆');
            }
            const direction = stopRpm > startRpm ? 1 : -1;
            // 鍥轰欢 side 鐨?LagAngle::MAX_POINTS 宸茬粡浠?1024 闄嶅埌 32锛堢渷 14.9KB RAM銆?            // 鍚姩鎷熷悎浠?1.6s 闄嶅埌 50ms锛夈€傜偣鏁拌秴闄愭椂 beginUpload 浼氱洿鎺ヨ繑鍥?false锛?            // 鍚庨潰姣忎釜 setPoint 閮介潤榛樺け璐ワ紝鏈€鍚庡彧鎶ヤ竴鍙モ€滄嫙鍚堝け璐モ€濓紝寰堥毦鏌ャ€?            // 鎵€浠ュ湪鍓嶇灏卞崱浣忥紝骞剁洿鎺ュ憡璇夌敤鎴锋杩涜璋冨埌澶氬皯銆?            const firmwareMaxPoints = 128;
            const maxSingleLegPoints = Math.ceil(firmwareMaxPoints / 2);   // 鍗曠▼ 64锛屽線杩?2*64-1 = 127
            const speeds = [startRpm];
            while (speeds[speeds.length - 1] !== stopRpm) {
                const next = speeds[speeds.length - 1] + direction * stepRpm;
                speeds.push(direction > 0 ? Math.min(next, stopRpm) : Math.max(next, stopRpm));
                if (speeds.length > maxSingleLegPoints) {
                    const suggested = Math.max(stepRpm, Math.ceil(Math.abs(stopRpm - startRpm) / (maxSingleLegPoints - 1)));
                    throw new Error(`鍗曠▼閫熷害鐐?${speeds.length} 涓紝寰€杩斿悗瓒呰繃鍥轰欢涓婇檺 ${firmwareMaxPoints} 鐐癸紱璇锋妸閫熷害姝ヨ繘鏀瑰埌 ${suggested} RPM 鎴栨洿澶);
                }
            }
            for (let index = speeds.length - 2; index >= 0; index--) {
                speeds.push(speeds[index]);
            }
            if (speeds.length < 2) throw new Error('鎷熷悎鑷冲皯闇€瑕佷袱涓€熷害鐐?);
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
            await writeRegisterAndWaitAck(CUSTOM_REGS.CLOSED_LOOP_ENABLE, 0);  // 鍏抽棴闂幆
            // 鍏抽敭淇锛氭祴閫熸帶鍒?婊炲悗瑙掓牎鍑嗙殑璧锋鍔ㄤ綔锛屽繀椤诲厛缁?startMotion() 涓€涓湁闄愪笖鍚堢悊鐨?pulseCount銆?            // 濡傛灉涓嶅厛鍐?0x11D锛屽氨浼氳 firmware 鍦?startMotion() 閲屾嬁鍒版棫鍊?鑴忓€硷紝鍑虹幇瓒呭ぇ鑴夊啿鏁板拰璧锋鍗℃銆?            const pulseCount = targetRpm / 60 * 360 / Number(document.getElementById('stepAngle').value || 1.8) * Number(document.getElementById('microsteps').value || 32) * Number(document.getElementById('lagAngleIntervalSec').value);;
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
                const statusText = `鐩爣 ${targetRpm} RPM / 骞冲潎 ${averagedMeasuredRpm.toFixed(1)} RPM / 瀹瑰樊 卤${tolerance.toFixed(1)} RPM / 绋冲畾 ${stableReads}/3`;
                document.getElementById('lagAngleStatus').textContent = `绛夊緟绋冲畾锛?{statusText}`;
                if (stableReads >= 3) return;
                await lagAngleDelay(Math.max(avgPeriodMs, 200));
            }
            if (state.lagAngleCalibrationCancelled) throw new Error('宸插仠姝㈡牎鍑?);
            throw new Error(`杞€?${targetRpm} RPM 鏈兘鍦ㄩ檺瀹氭椂闂村唴绋冲畾锛堝钩鍧?${lastMeasuredRpm.toFixed(1)} RPM锛岀洰鏍?${targetRpm} RPM锛屽宸?卤${tolerance.toFixed(1)} RPM锛塦);
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
            if (state.lagAngleCalibrationCancelled) throw new Error('宸插仠姝㈡牎鍑?);
            if (samples.length < 3) throw new Error('閲囨牱绐楀彛鍐呮湁鏁堟牱鏈笉瓒筹紝鑷冲皯闇€瑕?3 涓?);
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

        // ===== 杩炵画鎵€燂細鍚姩 / 鍙橀€?/ 绛夎揪鏍?=====
        // 鏃ф祦绋嬫瘡鐐归兘瑕?stopLagAngleMotion() + 閲嶆柊鍚姩锛屾瘡娆′粠 0 鍔犻€燂紝
        // 涓€娈典竴娈垫祴鏋佹參銆傜幇鍦ㄥ浐浠舵敮鎸侀€熷害妯″紡杩炵画杩愯锛?x11D = 0 鈫?鏃犻檺璺戯級
        // 鍜岃繍琛屾椂鍙橀€燂紙鏀?0x11B 鍗崇敓鏁堬級锛屾墍浠ヨ繖閲屽彧鍚姩涓€娆°€?        async function startLagAngleSweep(targetRpm, accelRpmS) {
            await writeRegisterAndWaitAck(CUSTOM_REGS.CLOSED_LOOP_ENABLE, 0);
            // 銆愬繀椤婚潪闆躲€戝浐浠?startMotion() 閲屾湁鍏滃簳锛?            //   if (motion_max_rpm_ <= 0) motion_max_rpm_ = angle_max_rpm_ ?: 1200;
            // 鎵€浠ユ牎鍑嗕粠 0 RPM 寮€濮嬫椂锛屽鏋滅洿鎺ユ妸 0x11B 鍐欐垚 0锛?            // 鍥轰欢浼氭嬁鈥滅綉椤甸棴鐜渶澶ч€熷害鈥濓紙姣斿 250锛夊綋鐩爣 -> 鐢垫満鐩存帴鍐插埌 250锛?            // 鑰岀綉椤佃繕浠ヤ负鍦ㄦ祴 50 RPM 鐨勭偣銆?            // 杩欓噷寮哄埗鏈€灏?10 RPM 璧锋锛? RPM 閭ｄ釜鐐规湰鏉ュ氨涓嶉噰鏍凤紙婊炲悗瑙?0锛夈€?            const safeRpm = Math.max(10, Math.round(targetRpm));
            await writeRegisterAndWaitAck(0x11A, Math.min(10, safeRpm));
            await writeRegisterAndWaitAck(0x11B, safeRpm);
            await writeRegisterAndWaitAck(0x11C, accelRpmS);
            await writeRegisterAndWaitAck(0x11D, 0);   // 鈽?0 = 鏃犻檺璺戯紝涓嶅仠鏈?            await writeRegisterAndWaitAck(0x11E, 2);   // 閫熷害姝ｈ浆
            await writeRegisterAndWaitAck(0x123, 200);
            await writePidParameters();
            await writeRegisterAndWaitAck(0x11F, 1);
        }

        // 鍙敼鐩爣閫熷害锛屼笉纰板叾瀹冨瘎瀛樺櫒
        // 鍚屾牱瑕侀槻 0锛氬啓 0 浼氳鍥轰欢 startMotion 鐨勫厹搴曢€昏緫鎷库€滈棴鐜渶澶ч€熷害鈥濆綋鐩爣銆?        async function changeLagAngleSweepSpeed(targetRpm) {
            await writeRegisterAndWaitAck(0x11B, Math.max(10, Math.round(targetRpm)));
        }

        // 绛夎浆閫?*绋冲畾**鍦ㄥ宸寖鍥村唴锛岃繑鍥?true/false锛堜笉鎶涘紓甯革級
        //
        // 銆愬叧閿€戜笉鑳藉彧鍒や竴娆♀€滆繘鍏ュ宸€濆氨杩斿洖锛?        //   鐩爣 600RPM銆佸宸?12%锛埪?2锛夋椂锛岄€熷害鍒氬埌 528 灏辨弧瓒虫潯浠讹紝
        //   浣嗛偅鏃惰繕鍦ㄥ姞閫熶笂鍗囨锛屾帴涓嬫潵 1 绉掔殑閲囨牱绐楀彛浼氳鐩?528鈫?00 鏁翠釜杩囩▼锛?        //   娴嬪嚭鏉ョ殑婊炲悗瑙掑搴旂殑鍏跺疄鏄€滄瘮鏍囩鍋忎綆鐨勪竴鎴浆閫熲€濓紝鏃舵満涔熷涓嶄笂銆?        //   蹇呴』鈥滆繛缁?kStableSamples 娆★紙80ms x 6 = 480ms锛夐兘鍦ㄥ宸唴鈥濇墠绠楃湡绋炽€?        // 绛夎浆閫熷埌浣嶏紝杩斿洖 true/false銆?        //
        // 銆愰噸瑕佷慨姝ｃ€戞棭鏈熺増鏈畬鍏ㄤ緷璧?0x120锛堝嵆鏃惰浆閫燂級鍒ゆ柇锛屼竴鏃﹂仴娴嬫病鍒版垨寤惰繜澶э紝
        // 灏变細姘歌繙鍒も€滄湭杈炬爣鈥?-> 閲嶈瘯 3 娆?-> 璺宠繃杩欎釜鐐?-> 涓嬩竴涓偣鏇撮珮 ->
        // 琛ㄧ幇涓衡€滅數鏈轰竴鐩村湪鍔犻€熴€侀€熷害杩滆秴鐩爣鈥濓紙瀹炴祴鐩爣 250 鍗磋窇鍒?500+锛夈€?        //
        // 鐜板湪鐨勭瓥鐣ワ細
        //   涓诲垽鎹?= 鐞嗚鍔犻€熸椂闂达紙accel 娈?+ 绋冲畾娈碉級锛屽埌鐐瑰氨璁や负鍒颁綅锛?        //   杈呭姪   = 鏈熼棿濡傛灉鏀跺埌 0x120 涓旇繛缁?6 娆″湪瀹瑰樊鍐咃紝鎻愬墠杩斿洖锛堢渷鏃堕棿锛夈€?        // 杩欐牱鍗充娇閬ユ祴瀹屽叏娌″埌锛屾壂閫熶篃鑳芥寜棰勬湡璺戝畬锛屼笉浼氳嚜宸辨姮楂樼洰鏍囥€?        async function waitForSpeedReached(targetRpm, tolerancePct, timeoutMs) {
            const start = performance.now();
            const deadline = start + timeoutMs;
            const tolerance = Math.max(5, targetRpm * tolerancePct / 100);
            // 銆愮ǔ瀹氬垽瀹氭斁瀹姐€戝師鏉ユ槸鈥滆繛缁?6 娆￠兘蹇呴』绮剧‘钀藉湪瀹瑰樊鍐呪€?鈥斺€?            // 缂栫爜鍣ㄦ祴閫熸湰韬湁 卤2~5 RPM 鍣０锛屼綆閫熸椂锛?0RPM銆佸宸?6锛夊緢瀹规槗琚櫔澹?            // 鎵撴柇涓€娆★紝stableCount 褰掗浂閲嶆潵锛岃〃鐜颁负鈥滄槑鏄庡埌浜嗗張璇存病鍒扳€濄€?            // 鏀规垚鈥? 娆￠噷鑷冲皯 6 娆″湪瀹瑰樊鍐呪€濓紙鍏佽 2 娆℃姈鍔級锛岄瞾妫掑緱澶氥€?            const kWindowSamples = 8;
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
                // 婊戝姩绐楀彛锛? 涓噰鏍烽噷鑷冲皯 6 涓湪甯﹀唴灏卞垽绋冲畾锛堝厑璁?2 娆″櫔澹版姈鍔級
                windowCount++;
                if (windowCount >= kWindowSamples) {
                    if (stableCount >= kRequiredInBand) {
                        log(`[婊炲悗瑙抅 ${targetRpm}RPM 閬ユ祴纭鍒颁綅锛堝疄娴?${actual.toFixed(1)}锛?{kWindowSamples} 娆′腑 ${stableCount} 娆″湪甯﹀唴锛塦);
                        return true;
                    }
                    // 绐楀彛娓呯┖閲嶆柊缁熻
                    windowCount = 0;
                    stableCount = 0;
                }
                await lagAngleDelay(80);
            }
            // 銆愬厹搴曚笌鍛婅銆?            // 閬ユ祴瀹屽叏娌″埌锛坢axActual == 0锛夋椂娌℃湁渚濇嵁鍒ゆ柇閫熷害锛屽彧鑳芥寜鏃堕棿鎺ㄨ繘锛?            // 浣嗗繀椤绘槑纭憡璀︹€斺€斿惁鍒欌€滅洰鏍?50 鍗磋窇鍒?250鈥濊繖绉嶆儏鍐典細琚潤榛樻斁杩囥€?            // 閬ユ祴姝ｅ父浣嗛€熷害涓嶅湪瀹瑰樊鍐?鈫?杩斿洖 false锛岃璋冪敤鏂归噸璇曪紝
            // 閲嶈瘯 3 娆￠兘涓嶈灏变腑姝㈡暣涓壂閫燂紙鑰屼笉鏄嚜娆烘浜哄湴缁х画锛夈€?            if (maxActual <= 0.5) {
                log(`[婊炲悗瑙抅 ${targetRpm}RPM 閬ユ祴瀹屽叏缂哄け锛坢axActual=0锛夛紝鎸夋椂闂存帹杩涳紱璇锋鏌?0x120 閬ユ祴閾捐矾`);
                return true;
            }
            log(`[婊炲悗瑙抅 ${targetRpm}RPM 鏈揪鏍囷細閬ユ祴鏈€澶?${maxActual.toFixed(1)}锛屽宸?卤${tolerance.toFixed(1)}`);
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
            let directionReversed = false;   // 寰€杩旓細鍙垏涓€娆℃柟鍚?
            // 銆愬彧鍚姩涓€娆°€戠敤绗竴涓?*闈為浂**閫熷害鍚姩锛?            // 0 RPM 閭ｄ釜鐐逛笉鍙備笌杩愬姩锛堟病鏈夎浆閫熷氨娌℃湁婊炲悗瑙掞級锛屼笅闈㈠惊鐜噷鐩存帴璁?0 骞惰烦杩囥€?            // 濡傛灉鎷?0 鍘诲惎鍔紝0x11B 浼氳閽冲埌 10锛岃€屽墠绔互涓哄湪娴?0 -> 閫熷害瀵逛笉涓婂氨鎶ラ敊銆?            const startupRpm = config.speeds.find((s) => s > 0) ?? 10;
            await startLagAngleSweep(startupRpm, config.accelRpmS);
            await lagAngleDelay(500);

            for (let index = 0; index < config.speeds.length; index++) {
                if (state.lagAngleCalibrationCancelled) throw new Error('宸插仠姝㈡牎鍑?);
                const speedRpm = config.speeds[index];

                // 銆愬線杩旀崲鍚戙€戠涓€娆″嚭鐜扳€滈€熷害姣斾笂涓€涓偣灏忊€濊鏄庤繘鍏ヤ笅闄嶆銆?                // 鍥轰欢閫熷害妯″紡鍙 0x11E锛?=姝ｈ浆 3=鍙嶈浆锛夛紝鏀瑰畠浼氬厛鍒瑰埌 0 鍐嶅弽鍚戝姞閫燂紝
                // 鎵€浠ヨ繖閲屽垏瀹岃鐣欎竴娈垫崲鍚戞椂闂达紝鍐嶅幓绛夌洰鏍囪浆閫熴€?                if (!directionReversed && index > 0 && speedRpm < config.speeds[index - 1]) {
                    document.getElementById('lagAngleStatus').textContent =
                        `鍒拌揪鏈€楂樼偣锛屽垏鎹㈠弽鍚戯紙鍏堝埞鍒?0锛?..`;
                    await writeRegisterAndWaitAck(0x11E, 3);
                    directionReversed = true;
                    await lagAngleDelay(800);
                }

                // 0 RPM锛氫笉鍚姩銆佷笉绛夊緟銆佷笉閲囨牱锛堟粸鍚庤瀹氫箟涓?0锛?                if (speedRpm === 0) {
                    points.push({speedRpm: 0, angleMdeg: 0, angleDeg: 0, fitAngleDeg: null});
                    document.getElementById('lagAngleProgress').textContent = `${index + 1} / ${config.speeds.length}`;
                    document.getElementById('lagAngleStatus').textContent = `0 RPM 鐩存帴璁?0锛堜笉閲囨牱锛塦;
                    continue;
                }

                await changeLagAngleSweepSpeed(speedRpm);
                // 銆愮姸鎬佹爮鎻愬墠鍒拌繖涓偣銆戝師鏉ユ槸鍦ㄩ噰鏍峰畬鎴愬悗鎵嶅啓锛屽鑷粹€滈噰鏍?200 鍗存樉绀?150鈥?                // 鐨勯敊浣嶁€斺€旂敤鎴风湅鍒扮殑鐘舵€佹€绘槸钀藉悗涓€鎷嶃€?                document.getElementById('lagAngleStatus').textContent =
                    `绛夊緟绋冲畾锛?{speedRpm} RPM锛?{index + 1}/${config.speeds.length}`;

                // 銆愬悓鐐归噸璇曘€戦€熷害涓嶈揪鏍囨渶澶氶噸璇?maxRetries 娆★紙閲嶆柊涓嬪彂鐩爣锛?                let reached = false;
                const prevRpm = index > 0 ? config.speeds[index - 1] : 0;
                const settleMs = Math.max(2500,
                    Math.abs(speedRpm - prevRpm) / Math.max(1, config.accelRpmS) * 3000 + 1500);
                for (let attempt = 0; attempt < maxRetries && !reached; attempt++) {
                    if (state.lagAngleCalibrationCancelled) throw new Error('宸插仠姝㈡牎鍑?);
                    reached = await waitForSpeedReached(speedRpm, tolerancePct, settleMs);
                    if (!reached) {
                        document.getElementById('lagAngleStatus').textContent =
                            `${speedRpm} RPM 绗?${attempt + 1} 娆℃湭杈炬爣锛岄噸璇曚腑...`;
                        await changeLagAngleSweepSpeed(speedRpm);
                    }
                }

                if (!reached) {
                    // 銆愯繛缁?3 娆′笉杈炬爣銆戝垽瀹氬凡杈剧數鏈烘瀬闄愶紝鍋滄鎵€?                    consecutiveFailures++;
                    document.getElementById('lagAngleStatus').textContent =
                        `${speedRpm} RPM 鏃犳硶杈炬爣锛堣繛缁?${consecutiveFailures} 娆★級`;
                    if (consecutiveFailures >= 3) {
                        await stopLagAngleMotion();
                        throw new Error(
                            `杩炵画 ${consecutiveFailures} 涓€熷害鐐规棤娉曡揪鏍囷紙鏈€鍚庯細${speedRpm} RPM锛屽宸?${tolerancePct}%锛夛紝宸插仠姝㈡壂閫焋);
                    }
                    continue;   // 璺宠繃杩欎釜鐐癸紝缁х画涓嬩竴涓?                }
                consecutiveFailures = 0;

                const holdStart = performance.now();
                await lagAngleDelay(centeredDelayMs);
                // 銆愭嫙鍚堢敤骞呭€硷紝涓嶈兘甯︾鍙枫€?                // collectLagAngleMean 璇荤殑鏄?0x121锛坒ollow_error = 瀹為檯 - 鍛戒护锛夛細
                //   姝ｈ浆鏃舵粸鍚庤鏄礋鐨勶紙杞瓙钀藉悗锛?                //   鍙嶈浆鏃舵槸姝ｇ殑
                // 涓よ€呭箙鍊肩浉鍚屻€佺鍙风浉鍙嶃€傚鏋滅洿鎺ヤ笂浼狅紝寰€杩斿簭鍒椾細鍦ㄥ悓涓€鐐逛笂鍑虹幇
                // 涓€姝ｄ竴璐熶袱涓€硷紝骞傚緥鎷熷悎浼氳鎷夋垚涓€鏉℃帴杩戦浂鐨勬枩绾匡紝瀹屽叏澶辩湡銆?                // 婊炲悗瑙掔殑鐗╃悊閲忔槸銆屽箙鍊笺€嶏紝鏂瑰悜鐢辫浆閫熺鍙峰喅瀹氾紙鍥轰欢閲屽凡澶勭悊锛夈€?                const rawAngleMdeg = speedRpm === 0 ? 0 : await collectLagAngleMean(config);
                const angleMdeg = Math.abs(rawAngleMdeg);
                points.push({speedRpm, angleMdeg, angleDeg: angleMdeg / 1000, fitAngleDeg: null});
                document.getElementById('lagAngleProgress').textContent = `${index + 1} / ${config.speeds.length}`;
                document.getElementById('lagAngleStatus').textContent = `閲囨牱涓細${speedRpm} RPM锛?{index + 1}/${config.speeds.length}`;
                drawLagAngleChart(points, Math.max(config.startRpm, config.stopRpm, 1));
                await lagAngleDelay(Math.max(0, holdStart + intervalMs - performance.now()));
            }
            await stopLagAngleMotion();
            return points;
        }

        async function uploadLagAnglePoints(points) {
            if (points.length > 32) {
                throw new Error(`鎷熷悎鐐?${points.length} 涓紝瓒呰繃鍥轰欢涓婇檺 32`);
            }
            await writeRegisterAndWaitAck(CUSTOM_REGS.LAG_ANGLE_UPLOAD_COUNT, points.length);
            for (let index = 0; index < points.length; index++) {
                const point = points[index];
                await writeRegisterAndWaitAck(CUSTOM_REGS.LAG_ANGLE_POINT_INDEX, index);
                await writeRegisterAndWaitAck(CUSTOM_REGS.LAG_ANGLE_POINT_SPEED_RPM, point.speedRpm);
                await writeRegisterAndWaitAck(CUSTOM_REGS.LAG_ANGLE_POINT_MDEG, point.angleMdeg >>> 0);
                await writeRegisterAndWaitAck(CUSTOM_REGS.LAG_ANGLE_POINT_COMMIT, 1);
                document.getElementById('lagAngleStatus').textContent = `涓婁紶鍧囧€肩偣锛?{index + 1}/${points.length}`;
            }
            await writeRegisterAndWaitAck(CUSTOM_REGS.LAG_ANGLE_FIT, 1);
            const fitSaved = await readRegisterValue(CUSTOM_REGS.LAG_ANGLE_FIT);
            if ((fitSaved >>> 0) !== 1) throw new Error('鍥轰欢鎷熷悎鎴?Flash 淇濆瓨澶辫触');
        }

        async function startLagAngleCalibration() {
            if (!state.connected) {
                document.getElementById('lagAngleStatus').textContent = '璇峰厛杩炴帴涓插彛';
                return;
            }
            let config;
            try {
                config = lagAngleSweepConfig();
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
            document.getElementById('lagAngleEquation').textContent = '鍥轰欢鎷熷悎鍑芥暟锛氱瓑寰呮湰杞噰鏍蜂笂浼犱笌鎷熷悎';
            try {
                const points = await captureLagAnglePoints(config);
                document.getElementById('lagAngleStatus').textContent = '涓婁紶閲囨牱鍧囧€煎苟鐢卞浐浠舵嫙鍚?..';
                await uploadLagAnglePoints(points);
                document.getElementById('lagAngleStatus').textContent = '鎷熷悎瀹屾垚锛岃鍙栧浐浠舵煡璇㈠€?..';
                await refreshLagAngleCalibration();
                document.getElementById('lagAngleStatus').textContent = '閲囨牱瀹屾垚锛涙洸绾挎煡璇㈠€煎強鎷熷悎绯绘暟鍧囨潵鑷浐浠?;
            } catch (error) {
                await stopLagAngleMotion().catch(() => {});
                document.getElementById('lagAngleStatus').textContent = `鏍″噯澶辫触锛?{error.message}`;
                log(`[婊炲悗瑙掓牎鍑哴 ${error.message}`);
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
                document.getElementById('lagAngleStatus').textContent = '璇峰厛杩炴帴涓插彛浠ヨ鍙栧浐浠舵暟鎹?;
                drawLagAngleChart([], 3000);
                return;
            }
            try {
                await refreshLagAngleCalibration();
            } catch (error) {
                document.getElementById('lagAngleStatus').textContent = `璇诲彇澶辫触锛?{error.message}`;
                drawLagAngleChart([], 3000);
            }
        }

