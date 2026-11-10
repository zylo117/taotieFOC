// Register read/write primitives (module 4).
// Depends on: CUSTOM_REGS (constants.js), state (state.js), buildFrame (utils.js), log (ui).
// Load order: constants -> utils -> state -> protocol -> ...

        // 【写超时】设备被拔掉时 writer.write() 的 Promise 可能永远不 settle
        // （既不 resolve 也不 reject），没有超时就会把所有调用链（含自动整定循环）
        // 永久卡住，界面看上去“死了”。2.5s 足够覆盖正常 USB 枚举抖动。
        async function writeFrameTimeout(frame, timeoutMs = 2500) {
            let timer = null;
            try {
                await Promise.race([
                    state.writer.write(frame),
                    new Promise((_, reject) => {
                        timer = window.setTimeout(() =>
                            reject(new Error('串口写入超时 ' + timeoutMs + 'ms（设备可能已被拔掉）')), timeoutMs);
                    })
                ]);
            } finally {
                if (timer !== null) window.clearTimeout(timer);
            }
        }

        async function readRegister(reg) {
            if (!state.connected || !state.writer) {
                log('[WARN] 尚未连接串口，无法读取寄存器');
                return null;
            }
            const frame = buildFrame(0x01, reg, 0);
            if (state.autoTuneActive) state.tuneLastCommand = `READ 0x${reg.toString(16).padStart(4, '0')} cmd=0x01`;
            await writeFrameTimeout(frame);
            log(`[TX] READ reg=0x${reg.toString(16).padStart(4, '0')} cmd=0x01`);
            return frame;
        }

        function clearPendingReadWaiters(reg) {
            const waiters = state.registerReadWaiters.get(reg);
            if (!waiters || waiters.length === 0) return;
            for (const waiter of waiters) {
                window.clearTimeout(waiter.timeout);
                try {
                    waiter.reject(new Error(`READ 0x${reg.toString(16).padStart(4, '0')} 等待 cmd=0x81 回包被新请求覆盖`));
                } catch (error) {
                    // 忽略重复结束的等待器
                }
            }
            state.registerReadWaiters.delete(reg);
        }

        async function readRegisterValue(reg, timeoutMs = 1500, retries = 2) {
            if (!state.connected || !state.writer) {
                throw new Error('串口未连接，无法读取寄存器');
            }
            // 这个函数是严格串行的（同一寄存器不允许重叠读），retries 参数实际上不起作用。
            // 默认值 retries=2 会让每个调用都打一条 WARN，高频读（如 0x134 数圈）时刷屏
            // 把真正的 [冲击] 日志冲掉。改成只在 retries 显式传非零值时提示一次。
            if (retries !== 0 && !readRegisterValue._warned) {
                readRegisterValue._warned = true;
                log('[INFO] READ 为严格串行模式，retries 参数已忽略（本条只提示一次）');
            }
            return await new Promise((resolve, reject) => {
                const waiters = state.registerReadWaiters.get(reg) || [];
                if (waiters.length > 0) {
                    reject(new Error(`READ 0x${reg.toString(16).padStart(4, '0')} 仍有未收回的 cmd=0x81 等待，拒绝重叠请求`));
                    return;
                }
                const waiter = {
                    resolve,
                    reject,
                    timeout: window.setTimeout(() => {
                        state.registerReadWaiters.delete(reg);
                        reject(new Error(`READ 0x${reg.toString(16).padStart(4, '0')} 等待 cmd=0x81 回包超时 ${timeoutMs}ms`));
                    }, timeoutMs)
                };
                state.registerReadWaiters.set(reg, [waiter]);
                readRegister(reg).catch((error) => {
                    window.clearTimeout(waiter.timeout);
                    state.registerReadWaiters.delete(reg);
                    reject(error);
                });
            });
        }

        async function writeRegister(reg, value) {
            if (!state.connected || !state.writer) {
                log('[WARN] 尚未连接串口，无法写入寄存器');
                return;
            }
            const frame = buildFrame(0x02, reg, value >>> 0);
            if (state.autoTuneActive) state.tuneLastCommand = `WRITE 0x${reg.toString(16).padStart(4, '0')} value=${value} cmd=0x02`;
            await writeFrameTimeout(frame);
            log(`[TX] WRITE reg=0x${reg.toString(16).padStart(4, '0')} cmd=0x02 value=${value}`);
        }

