// Global UI/motion state container.
// Split out of taotie_webui.html (module 3). Pure literal, no dependencies.
// Load order: constants.js -> utils.js -> state.js -> ...

        const state = {
            connected: false,
            port: null,
            reader: null,
            writer: null,
            // 每次连接代次 +1：readLoop 收尾时发现自己已经不是当代（被断开/重连顶替）
            // 就静默退出，不去清新的连接状态。
            readGeneration: 0,
            registerReadWaiters: new Map(),
            registerWriteAckWaiters: new Map(),
            lastEncoderTelemetryAt: 0,
            autoTuneActive: false,
            autoTuneCancelled: false,
            tuneTrialActive: false,
            tuneActionBusy: false,
            tuneActionProfiles: [],
            tuneMotionObserved: false,
            tuneMotionResolver: null,
            tuneMotionTimeout: null,
            tuneTrialStage: '',
            tuneTrialProfile: null,
            tuneTrialStartedAt: 0,
            tuneLastMotionTelemetryAt: 0,
            tuneLastMotionStatus: null,
            tuneLastMotionStatusCmd: null,
            tuneLastCommand: '无',
            bestTuneGains: null,
            currentAngle: 0,
            targetAngle: 0,
            targetPosition: 0,
            microsteps: 64,
            pendingTargetAngle: null,
            targetWriteInFlight: false,
            speed: 0,
            accel: 0,
            currentA: 0,
            dragging: false,
            encoderFault: false,
            magneticFault: false,
            outputStopped: false,
            // 堵转锁存（0x108 bit3 / 0x15D）：固件会停机并失能，必须写 0x15F=1 清除
            stallDetected: false,
            faultFlags: 0,
            driverEnabled: false,
            externalEnLevel: 2,
            faultSummary: '正常',
            phaseACurrentA: 0,
            phaseBCurrentA: 0,
            posLoopHz: 0,
            velLoopHz: 0,
            curLoopHz: 0
            , encoderRaw: 0
            , encoderAngleDeg: 0
            , magneticHigh: false
            , magneticLow: false
            , actualPosition: 0
            , lastEncoderLogAt: 0
            , rxBuffer: new Uint8Array(0)
            , motionSpeedRpm: 0
            , motionFollowErrorDeg: 0
            , angleMaxRpm: 1200
            , angleAccelRpmS: 6000
            , waveHistory: []
            , wavePaused: false
            , wavePausedAt: 0
            , motionWaveTimer: null
            // 0x11F 连续为 0 的帧数，用于防抖（见 handleCustomTelemetry 里的收尾逻辑）
            , waveZeroStreak: 0
            , waveCaptureActive: false
            , waveCaptureStartedAt: 0
            , waveMotionObserved: false
            , waveFrozenAt: 0
            , waveStoppedAt: 0
            , waveStopFreezeTimer: null
            , lastWaveStamp: 0
            , cpuUsagePercent: 0
            , controlTaskUsagePercent: 0
            , calibrationMainTable: null
            , calibrationFastTable: null
            , calibrationFastPagesLoaded: new Uint8Array(256)
            , calibrationTableChecksum: null
            , calibrationDumpKind: null
            , calibrationDumpExpectedPairs: 0
            , calibrationDumpReceivedPairs: 0
            , calibrationDumpPageOffset: 0
            , calibrationDumpNextKind: null
            , calibrationLookupInFlight: false
            , calibrationLookupSerialToken: null
            , calibrationLookupDesynchronized: false
            , calibrationLookupActiveDirection: null
            , calibrationLookupLastRaw: null
            , calibrationLookupLastCorrected: null
            , calibrationLookupRequestId: 0
            , lagAngleCalibrationRunning: false
            , lagAngleCalibrationCancelled: false
            , lagAngleMaxSpeedRpm: 3000
            , motionRunning: false
            // 运动源：0=内部规划 1=外部脉冲跟随（固件 0x16D）。默认按“内部”渲染，
            // 连上新固件后由 0x16D 回读纠正为外部；老固件没有 0x16D 时不会被误锁。
            // 真正的互斥权威在固件（它会拒绝并打印 [SOURCE]），UI 禁用只是防呆。
            , motionSource: 0
            , encoderJitterSamples: []
            , encoderJitterRangeDeg: 0
            , encoderJitterSampleCount: 0
        };

