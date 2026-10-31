// ===========================================================================
// 纯工具函数：数值归一 + 协议帧编解码
//
// 从 taotie_webui.html 拆出来的第 2 个模块。这里全是无副作用的纯函数，
// 不依赖 DOM、不依赖 state，所以拆出去风险为零。
//
// 加载顺序：constants.js 之后、其它业务模块之前。
// （crc8 被 buildFrame / parseFrame 使用；三者都被 protocol.js 使用。）
// ===========================================================================

function clamp(value, min, max) {
    return Math.min(max, Math.max(min, value));
}

// 归一到 (-180, 180]
function normalizeAngle(value) {
    let v = value;
    while (v > 180) v -= 360;
    while (v < -180) v += 360;
    return v;
}

// 归一到 [0, 360)
function normalize360Angle(value) {
    let v = value % 360;
    if (v < 0) v += 360;
    return v;
}

// CRC-8/MAXIM（多项式 0x07，MSB-first），与固件 crc8 一致
function crc8(data) {
    let crc = 0x00;
    for (let i = 0; i < data.length; i++) {
        crc ^= data[i];
        for (let bit = 0; bit < 8; bit++) {
            if ((crc & 0x80) !== 0) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc & 0xFF;
}

// 组帧：55 AA | cmd | regHi regLo | value(4B, 大端) | crc8
function buildFrame(cmd, reg, value) {
    const regHi = (reg >> 8) & 0xFF;
    const regLo = reg & 0xFF;
    const data = new Uint8Array([
        0x55, 0xAA, cmd,
        regHi, regLo,
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF,
        0x00
    ]);
    data[data.length - 1] = crc8(data.slice(0, -1));
    return data;
}

// 解帧：校验帧头 + CRC，失败返回 null
function parseFrame(raw) {
    if (!raw || raw.length < 10) return null;
    const arr = new Uint8Array(raw);
    if (arr[0] !== 0x55 || arr[1] !== 0xAA || crc8(arr.slice(0, 9)) !== arr[9]) return null;
    const cmd = arr[2];
    const reg = (arr[3] << 8) | arr[4];
    const value = (
        (arr[5] << 24) |
        (arr[6] << 16) |
        (arr[7] << 8) |
        arr[8]
    ) >>> 0;
    return {cmd, reg, value};
}
