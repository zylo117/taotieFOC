// RX byte-stream assembly + frame dispatch (module 5).
// Depends on: state, parseFrame, readRegisterValue, and various UI/calibration handlers (all global).
// Load order: constants -> utils -> state -> protocol -> rx -> ...

        function appendRxBytes(value) {
            const incoming = new Uint8Array(value);
            const merged = new Uint8Array(state.rxBuffer.length + incoming.length);
            merged.set(state.rxBuffer);
            merged.set(incoming, state.rxBuffer.length);
            state.rxBuffer = merged;

            while (state.rxBuffer.length >= 10) {
                let start = 0;
                while (start + 1 < state.rxBuffer.length &&
                (state.rxBuffer[start] !== 0x55 || state.rxBuffer[start + 1] !== 0xAA)) start++;
                if (start > 0) state.rxBuffer = state.rxBuffer.slice(start);
                if (state.rxBuffer.length < 10) return;
                const frame = parseFrame(state.rxBuffer.slice(0, 10));
                if (!frame) {
                    state.rxBuffer = state.rxBuffer.slice(1);
                    continue;
                }
                state.rxBuffer = state.rxBuffer.slice(10);
                handleIncomingFrame(frame);
            }
        }

        function handleIncomingFrame(frame) {
            if (frame.cmd === 0x83 || frame.cmd === 0x84) {
                handleCalibrationTablePair(frame);
                return;
            }
            if (frame.cmd === 0x85) {
                state.calibrationTableChecksum = frame.value & 0xFFFF;
                document.getElementById('calibrationTableProgress').value = 1;
                document.getElementById('calibrationTableChecksum').textContent =
                    `0x${state.calibrationTableChecksum.toString(16).padStart(4, '0').toUpperCase()} (${state.calibrationTableChecksum})`;
                const allFastPagesLoaded = state.calibrationFastPagesLoaded.every((loaded) => loaded !== 0);
                const canCheckFullFastTable = allFastPagesLoaded && state.calibrationFastTable && state.calibrationFastTable.length === 65536;
                const computed = canCheckFullFastTable
                    ? state.calibrationFastTable.reduce((sum, value) => (sum + value) & 0xFFFF, 0)
                    : null;
                const checksumStatus = !canCheckFullFastTable
                    ? '，当前快速表未加载完整，不能用部分页结果做全表校验'
                    : (computed === state.calibrationTableChecksum ? '，校验通过' : '，校验不匹配');
                document.getElementById('calibrationTableStatus').textContent = `读取完成${checksumStatus}`;
                state.calibrationDumpKind = null;
                return;
            }
            if (frame.cmd === 0x86) {
                if (frame.value === 0xFFFFFFFF) {
                    document.getElementById('calibrationTableStatus').textContent =
                        '读取失败：校准表不可用，或校准正在写入';
                    state.calibrationDumpKind = null;
                    return;
                }
                if (state.calibrationDumpKind === 'main') {
                    if (state.calibrationDumpReceivedPairs !== state.calibrationDumpExpectedPairs) {
                        document.getElementById('calibrationTableStatus').textContent = '主表数据不完整';
                        state.calibrationDumpKind = null;
                        return;
                    }
                    document.getElementById('calibrationTableStatus').textContent = '主表读取完成';
                    state.calibrationDumpKind = null;
                    renderCalibrationTablePage();
                    return;
                }
                if (state.calibrationDumpKind === 'fast') {
                    if (state.calibrationDumpReceivedPairs !== state.calibrationDumpExpectedPairs) {
                        document.getElementById('calibrationTableStatus').textContent = '快速表数据不完整';
                        state.calibrationDumpKind = null;
                        return;
                    }
                    const pageIndex = Math.floor(state.calibrationDumpPageOffset / 128);
                    state.calibrationFastPagesLoaded[pageIndex] = 1;
                    const allFastPagesLoaded = state.calibrationFastPagesLoaded.every((loaded) => loaded !== 0);
                    document.getElementById('calibrationTableStatus').textContent = `快速表第 ${pageIndex + 1} 页读取完成`;
                    state.calibrationDumpKind = null;
                    renderCalibrationTablePage();
                    if (allFastPagesLoaded && state.calibrationFastTable && state.calibrationFastTable.length === 65536) {
                        void requestCalibrationTableDump('checksum');
                    }
                    return;
                }
                return;
            }

            const registerWaiters = state.registerReadWaiters.get(frame.reg);
            const writeAckWaiter = state.registerWriteAckWaiters.get(frame.reg);
            if (frame.cmd === 0x82 && writeAckWaiter) {
                window.clearTimeout(writeAckWaiter.timeout);
                state.registerWriteAckWaiters.delete(frame.reg);
                if (frame.value !== writeAckWaiter.expectedValue) {
                    writeAckWaiter.reject(new Error(
                        `WRITE ACK 值不匹配：期望 0x${writeAckWaiter.expectedValue.toString(16).padStart(8, '0').toUpperCase()}，收到 0x${frame.value.toString(16).padStart(8, '0').toUpperCase()}`
                    ));
                } else {
                    writeAckWaiter.resolve(frame.value);
                }
            }
            // 只接受真正的“读寄存器回包”（0x81）。周期遥测（0x10）是广播，不应被当成
            // 读取命令的答复，否则同一寄存器的等待器会被提前消费，出现超时和样本丢失。
            if (registerWaiters && registerWaiters.length > 0 && frame.cmd === 0x81) {
                const waiter = registerWaiters.shift();
                window.clearTimeout(waiter.timeout);
                waiter.resolve(frame.value);
                if (registerWaiters.length === 0) state.registerReadWaiters.delete(frame.reg);
            }
            const description = REGISTER_DESCRIPTIONS.get(frame.reg);
            const name = description ? description[1] : 'UNKNOWN';
            const isTelemetry = frame.cmd === 0x10;
            const diagnosticRegister = frame.reg === CUSTOM_REGS.ENCODER_RAW ||
                frame.reg === CUSTOM_REGS.ENCODER_ANGLE_MDEG ||
                frame.reg === CUSTOM_REGS.MAGNETIC_HIGH ||
                frame.reg === CUSTOM_REGS.MAGNETIC_LOW ||
                frame.reg === CUSTOM_REGS.CPU_USAGE_CENTIPERCENT ||
                frame.reg === CUSTOM_REGS.CONTROL_TASK_USAGE_CENTIPERCENT;
            const routineMotionStatus = isTelemetry && frame.reg === 0x11F;
            const isSignedRegister = frame.reg === 0x0113 || frame.reg === 0x0120 ||
                frame.reg === 0x0121 || frame.reg === 0x0135 ||
                frame.reg === 0x014B || frame.reg === 0x014C;
            const displayValue = isSignedRegister ? (frame.value | 0) : Number(frame.value >>> 0);
            if (!routineMotionStatus && (!isTelemetry || !diagnosticRegister)) {
                log(`[RX] cmd=0x${frame.cmd.toString(16).padStart(2, '0')} ${description ? description[0] : '0x' + frame.reg.toString(16).padStart(4, '0')} ${name} = ${displayValue}`);
            }

            if (frame.reg === CUSTOM_REGS.PHASE_A_CURRENT_MA) state.phaseACurrentA = (frame.value | 0) / 1000;
            if (frame.reg === CUSTOM_REGS.PHASE_B_CURRENT_MA) state.phaseBCurrentA = (frame.value | 0) / 1000;
            if (frame.reg === CUSTOM_REGS.POSITION_LOOP_HZ) state.posLoopHz = Number(frame.value >>> 0);
            if (frame.reg === CUSTOM_REGS.VELOCITY_LOOP_HZ) state.velLoopHz = Number(frame.value >>> 0);
            if (frame.reg === CUSTOM_REGS.CURRENT_LOOP_HZ) state.curLoopHz = Number(frame.value >>> 0);
            if (frame.reg === CUSTOM_REGS.CPU_USAGE_CENTIPERCENT) state.cpuUsagePercent = frame.value / 100;
            if (frame.reg === CUSTOM_REGS.CONTROL_TASK_USAGE_CENTIPERCENT) {
                state.controlTaskUsagePercent = frame.value / 100;
            }
            if (frame.reg === CUSTOM_REGS.MICROSTEPS) {
                state.microsteps = frame.value;
                document.getElementById('microsteps').value = frame.value;
            }
            if (frame.reg === CUSTOM_REGS.CLOSED_LOOP_ENABLE) {
                document.getElementById('closedLoopEnableInput').checked = frame.value !== 0;
            }
            if (frame.reg === CUSTOM_REGS.EXT_FOLLOW_ENABLE) {
                const input = document.getElementById('extFollowEnableInput');
                if (document.activeElement !== input) input.checked = frame.value !== 0;
            }
            if (frame.reg === CUSTOM_REGS.EXT_FOLLOW_MAX_STEP_S) {
                const input = document.getElementById('extFollowMaxStepSInput');
                if (document.activeElement !== input) input.value = String(frame.value >>> 0);
            }
            if (frame.reg === CUSTOM_REGS.EXT_FOLLOW_CMD_STEPS) {
                document.getElementById('extFollowCmdValue').textContent = String(frame.value | 0);
            }
            if (frame.reg === CUSTOM_REGS.EXT_FOLLOW_EMITTED_STEPS) {
                document.getElementById('extFollowEmittedValue').textContent = String(frame.value | 0);
            }
            if (frame.reg === CUSTOM_REGS.EXT_FOLLOW_BACKLOG) {
                document.getElementById('extFollowBacklogValue').textContent = String(frame.value >>> 0);
            }
            if (frame.reg === CUSTOM_REGS.EXT_FOLLOW_DROPPED) {
                document.getElementById('extFollowDroppedValue').textContent = String(frame.value >>> 0);
            }
            if (frame.reg === CUSTOM_REGS.EXT_FOLLOW_WINDOW_REVERSALS) {
                document.getElementById('extFollowReversalValue').textContent = String(frame.value >>> 0);
            }
            if (frame.reg === CUSTOM_REGS.ENCODER_FAULT) state.encoderFault = frame.value !== 0;
            if (frame.reg === CUSTOM_REGS.MAGNETIC_FAULT) state.magneticFault = frame.value !== 0;
            if (frame.reg === CUSTOM_REGS.OUTPUT_STOP) state.outputStopped = frame.value !== 0;
            if (frame.reg === CUSTOM_REGS.ENCODER_RAW) state.encoderRaw = frame.value;
            if (frame.reg === CUSTOM_REGS.ENCODER_ANGLE_MDEG) {
                state.encoderAngleDeg = (frame.value | 0) / 1000;
                state.lastEncoderTelemetryAt = performance.now();
                setCurrentAngle(state.encoderAngleDeg);
                // 只有静止时才统计抖动；运行中的角度变化是真实位移，不是误差
                if (!state.motionRunning) pushEncoderJitterSample(state.encoderAngleDeg);
            }
            const pidInputByRegister = new Map([
                [CUSTOM_REGS.POSITION_KP, 'posKp'],
                [CUSTOM_REGS.POSITION_KI, 'posKi'],
                [CUSTOM_REGS.POSITION_KD, 'posKd'],
                [CUSTOM_REGS.VELOCITY_KP, 'velKp'],
                [CUSTOM_REGS.VELOCITY_KI, 'velKi'],
                [CUSTOM_REGS.VELOCITY_KD, 'velKd']
            ]);
            if (pidInputByRegister.has(frame.reg)) {
                // 【只在首次连接时从固件回填】原来每收到一条读回包就往输入框写，
                // 用户填了 9000 之后，固件回报的旧值（比如 21568 -> 21.57）会把输入框覆盖掉，
                // 于是“写入 Flash”实际存的是旧值（实测 save ok 打出的 pid 21568/20000/60000 就是这个原因）。
                // 现在只在刚连上、还没同步过时回填一次；之后输入框归用户，不再被固件值修改。
                if (!state.pidSyncedFromDevice) {
                    document.getElementById(pidInputByRegister.get(frame.reg)).value = (frame.value / 1000).toFixed(3);
                    // 6 个 PID 依次回报，给 1.5 秒窗口全部回填完，然后关闭回填（输入框归用户）
                    if (!state.pidSyncTimer) {
                        state.pidSyncTimer = window.setTimeout(function () {
                            state.pidSyncedFromDevice = true;
                            state.pidSyncTimer = null;
                        }, 1500);
                    }
                }
            }
            if (frame.reg === CUSTOM_REGS.MAGNETIC_HIGH) state.magneticHigh = frame.value !== 0;
            if (frame.reg === CUSTOM_REGS.MAGNETIC_LOW) state.magneticLow = frame.value !== 0;
            if (frame.reg === CUSTOM_REGS.ACTUAL_POSITION) {
                state.actualPosition = frame.value | 0;
            }
            if (frame.reg === CUSTOM_REGS.ANGLE_MAX_RPM) state.angleMaxRpm = frame.value;
            if (frame.reg === CUSTOM_REGS.ANGLE_ACCEL_RPM_S) state.angleAccelRpmS = frame.value;
            if (frame.reg === CUSTOM_REGS.ANGLE_MAX_RPM) {
                document.getElementById('angleMaxRpmInput').value = frame.value;
            }
            if (frame.reg === CUSTOM_REGS.ANGLE_ACCEL_RPM_S) {
                document.getElementById('angleAccelRpmSInput').value = frame.value;
            }
            updateFaultStatus();
            updateTelemetryDisplay();

            if (isTelemetry && frame.reg === CUSTOM_REGS.MAGNETIC_LOW) {
                const now = performance.now();
                if (!state.lastEncoderLogAt || now - state.lastEncoderLogAt >= 500) {
                    // log(`[KTH7823] raw=0x${state.encoderRaw.toString(16).padStart(4, '0').toUpperCase()} angle=${state.encoderAngleDeg.toFixed(3)}° MGH=${state.magneticHigh ? 1 : 0} MGL=${state.magneticLow ? 1 : 0}`);
                    state.lastEncoderLogAt = now;
                }
            }
        }

        // 【必须与固件一致】angle_encoder.h 的 CALIBRATION_TABLE_SIZE。
        // 项数改了不改这里，主表导出会错位或读到空页：页数、期望对数、数组长度全部由这两个常量派生。
        const CALIBRATION_MAIN_TABLE_ITEMS = 256;
        const CALIBRATION_MAIN_PAGE_PAIRS = 128;   // 一页 = 128 对 = 256 项
        const CALIBRATION_MAIN_PAGE_ITEMS = CALIBRATION_MAIN_PAGE_PAIRS * 2;

        async function requestCalibrationTableDump(kind, pageIndex = null) {
            const type = kind === 'main' ? 0 : (kind === 'fast' ? 1 : (kind === 'checksum' ? 2 : 3));
            const mainLastPage = Math.max(0, Math.ceil(CALIBRATION_MAIN_TABLE_ITEMS / CALIBRATION_MAIN_PAGE_ITEMS) - 1);
            const pageNumber = pageIndex === null ? 0 : clamp(Math.max(0, Math.round(Number(pageIndex))), 0, kind === 'main' ? mainLastPage : 255);
            const mainPageItems = Math.min(CALIBRATION_MAIN_PAGE_ITEMS,
                                           CALIBRATION_MAIN_TABLE_ITEMS - pageNumber * CALIBRATION_MAIN_PAGE_ITEMS);
            state.calibrationDumpKind = kind;
            state.calibrationDumpPageOffset = pageNumber * 128;
            state.calibrationDumpExpectedPairs = kind === 'main'
                ? Math.max(0, Math.ceil(mainPageItems / 2)) : 128;
            state.calibrationDumpReceivedPairs = 0;
            if (kind === 'main') {
                state.calibrationMainTable = new Uint16Array(CALIBRATION_MAIN_TABLE_ITEMS);
            }
            if (kind === 'fast') {
                if (!state.calibrationFastTable) state.calibrationFastTable = new Uint16Array(65536);
            }
            if (kind === 'checksum') {
                document.getElementById('calibrationTableStatus').textContent = '正在读取校验码';
                document.getElementById('calibrationTableProgress').max = 1;
            } else if (kind === 'main') {
                document.getElementById('calibrationTableStatus').textContent =
                    mainLastPage === 0
                        ? `正在读取主表：0/${CALIBRATION_MAIN_TABLE_ITEMS}`
                        : `正在读取主表第 ${pageNumber + 1} 页：0/${mainPageItems}`;
            } else {
                document.getElementById('calibrationTableStatus').textContent = `正在读取快速表第 ${pageNumber + 1} 页：0/256`;
            }
            document.getElementById('calibrationTableProgress').value = 0;
            document.getElementById('calibrationTableProgress').max = Math.max(1, state.calibrationDumpExpectedPairs);
            try {
                await writeRegister(CUSTOM_REGS.CALIBRATION_TABLE_DUMP, ((pageNumber & 0xFFFF) << 16) | (type & 0xFFFF));
            } catch (error) {
                state.calibrationDumpKind = null;
                document.getElementById('calibrationTableStatus').textContent = `读取失败：${error.message}`;
            }
        }

        function handleCalibrationTablePair(frame) {
            const isMain = frame.cmd === 0x83;
            const expectedKind = isMain ? 'main' : 'fast';
            const table = isMain ? state.calibrationMainTable : state.calibrationFastTable;
            if (state.calibrationDumpKind !== expectedKind || !table) return;
            const expectedPairIndex = state.calibrationDumpPageOffset + state.calibrationDumpReceivedPairs;
            if (frame.reg !== expectedPairIndex) {
                document.getElementById('calibrationTableStatus').textContent =
                    `数据帧索引错位：期望 ${expectedPairIndex}，收到 ${frame.reg}`;
                state.calibrationDumpKind = null;
                return;
            }
            const firstIndex = frame.reg * 2;
            if (firstIndex >= table.length) return;
            table[firstIndex] = (frame.value >>> 16) & 0xFFFF;
            if (firstIndex + 1 < table.length) table[firstIndex + 1] = frame.value & 0xFFFF;
            state.calibrationDumpReceivedPairs++;
            document.getElementById('calibrationTableProgress').value = state.calibrationDumpReceivedPairs;
            if ((state.calibrationDumpReceivedPairs & 0x7F) === 0 || state.calibrationDumpReceivedPairs === state.calibrationDumpExpectedPairs) {
                const received = state.calibrationDumpReceivedPairs * 2;
                const total = state.calibrationDumpExpectedPairs * 2;
                const pageText = state.calibrationDumpPageOffset === 0 ? '' : `第 ${state.calibrationDumpPageOffset / 128 + 1} 页 · `;
                document.getElementById('calibrationTableStatus').textContent =
                    `正在读取${isMain ? '主表' : '快速表'}${pageText}${received}/${total}`;
                if (document.getElementById('calibrationTableSelect').value === expectedKind) {
                    renderCalibrationTablePage();
                }
            }
        }

        function toCalibrationAngle(value) {
            return ((Number(value) % 65536) + 65536) % 65536 * 360.0 / 65536.0;
        }

        function updateLookupInputAvailability() {
            const rawInput = document.getElementById('calibrationRawInput');
            const rawBtn = document.getElementById('calibrationRawLookupBtn');
            const correctedInput = document.getElementById('calibrationCorrectedInput');
            const correctedBtn = document.getElementById('calibrationCorrectedLookupBtn');
            const busy = Boolean(state.calibrationLookupInFlight);
            rawInput.disabled = busy;
            rawInput.readOnly = busy;
            rawBtn.disabled = busy;
            correctedInput.disabled = busy;
            correctedInput.readOnly = busy;
            correctedBtn.disabled = busy;
        }

        function updateCalibrationLookupDisplays() {
            const rawInput = document.getElementById('calibrationRawInput');
            const correctedInput = document.getElementById('calibrationCorrectedInput');
            const rawOutput = document.getElementById('calibrationRawOutputValue');
            const rawAngle = document.getElementById('calibrationRawOutputAngle');
            const correctedOutput = document.getElementById('calibrationCorrectedOutputValue');
            const correctedAngle = document.getElementById('calibrationCorrectedOutputAngle');

            if (state.calibrationLookupInFlight) {
                updateLookupInputAvailability();
                return;
            }

            const resetForInvalid = (input, valueOutput, angleOutput) => {
                if (input.value.trim() === '' || !Number.isFinite(Number(input.value))) {
                    valueOutput.textContent = '--';
                    angleOutput.textContent = '--';
                }
            };

            resetForInvalid(rawInput, rawOutput, rawAngle);
            resetForInvalid(correctedInput, correctedOutput, correctedAngle);
        }

        async function requestFirmwareCalibrationLookup(direction, value) {
            if (!state.connected || !state.writer) {
                throw new Error('串口未连接');
            }
            if (state.calibrationLookupDesynchronized) {
                throw new Error('协议状态已因超时失去同步；为避免旧回包被误认，断开并重新连接后再查询');
            }
            if (state.calibrationLookupSerialToken !== null) {
                throw new Error('上一个校准查询仍在进行中，必须等待回包收回后再发下一条');
            }
            const isForward = direction === 'forward';
            const reg = isForward
                ? CUSTOM_REGS.CALIBRATION_LOOKUP_RAW_TO_CORRECTED
                : CUSTOM_REGS.CALIBRATION_LOOKUP_CORRECTED_TO_RAW;
            const token = Symbol('calibrationLookup');
            state.calibrationLookupSerialToken = token;
            const inputValue = clamp(Math.round(Number(value)), 0, 65535);
            const startedAt = performance.now();
            try {
                log(`[校准互查] 开始 direction=${direction} reg=0x${reg.toString(16).padStart(4, '0')} input=${inputValue}`);
                log(`[校准互查] 阶段 1/3：发送 WRITE，等待固件 0x82 确认`);
                await writeRegisterAndWaitAck(reg, inputValue, 1200);
                log(`[校准互查] 阶段 2/3：WRITE ACK 已收到，耗时 ${Math.round(performance.now() - startedAt)}ms；发送 READ`);
                const result = await readRegisterValue(reg, 1200, 0);
                log(`[校准互查] 阶段 3/3：READ 0x81 已收到，result=${result & 0xFFFF}，总耗时 ${Math.round(performance.now() - startedAt)}ms`);
                return (result >>> 0) & 0xFFFF;
            } catch (error) {
                if (error.lookupTransactionSent) {
                    state.calibrationLookupDesynchronized = true;
                    log('[校准互查] 协议状态标记为不同步；本次不自动重试，也不再发新查询，需重新连接以排除迟到回包');
                }
                throw error;
            } finally {
                if (state.calibrationLookupSerialToken === token) {
                    state.calibrationLookupSerialToken = null;
                }
            }
        }

