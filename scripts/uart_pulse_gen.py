#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CH340G 硬件 UART 脉冲发生器：用 UART 比特流模拟 PUL/STEP 脉冲。

原理（不需要任何软件翻转电平、不需要定时器）：
  UART 8N1 的一帧 = START(0) + D0..D7 + STOP(1)，共 10 个位时间，位宽由波特率
  发生器硬件决定。把数据字节选成某个固定值，就能让线上出现严格周期性的方波：

    字节 0x55 = 0101 0101（LSB 先发） -> 线上波形 0,1,0,1,0,1,0,1,0,1
              -> 每 2 个位时间一个上升沿，占空比 50%
    字节 0xEF -> 每 5 个位时间一个上升沿，高电平 4 位（20% 低）
    字节 0xFF -> 每 10 个位时间一个上升沿，高电平 9 位

  所以脉冲频率 = 波特率 / 周期位数，精度完全由 CH340G 的波特率发生器保证，
  PC 只负责“把 TX FIFO 喂满”，喂慢了只会出现一次长空闲（TX 空闲为高电平，
  相当于把高电平拉长），不会丢脉冲、也不会产生多余脉冲。

接线（只要三根线）：
  CH340G GND  -> 控制器 GND        （必须共地，否则电平判不出来）
  CH340G TXD  -> PUL 输入 PB5      （TMR3_CH2，上升沿捕获）
  DIR 输入 PB4 -> GND 或 3V3       （别悬空！悬空时 DIR 快照随机，累计会正负抵消）
  CH340G 电平选择跳线打到 3.3V（AT32 的 IO 是 3.3V 逻辑，5V TTL 虽然有 FT 脚但没必要冒险）

用法示例：
  # 100kHz 连续发 2 秒（纯 0x55 波形流）
  python scripts/uart_pulse_gen.py --port COM5 --freq 100000 --duration 2

  # 精确发 20000 个脉冲（= 20000/12800 圈 ≈ 1.56 圈，便于对拍计数）
  python scripts/uart_pulse_gen.py --port COM5 --freq 50000 --count 20000

  # 只看计划不开串口
  python scripts/uart_pulse_gen.py --freq 100000 --count 100000 --dry-run

  # 接线/电平排查：先用 1kHz（脉宽 500us，万用表都能看出跳变），确认能计数再提速
  python scripts/uart_pulse_gen.py --port COM5 --freq 1000 --count 5000

  # 回环自检：把 CH340 的 TXD 与 RXD 短接，验证“脚本到底有没有在发数据”
  python scripts/uart_pulse_gen.py --port COM5 --freq 50000 --count 5000 --loopback

  # 一根 CH340 同时干两件事：TXD->PB5 发脉冲，RXD->PA9 收固件 [PUL] 日志。
  # （一个 UART 收发共用一个波特率，固件 printf 是 115200，所以 freq 只能取 57600）
  python scripts/uart_pulse_gen.py --port COM14 --freq 57600 --count 5760 --monitor

  注意：同一个 COM 口同一时刻只能被一个程序打开。要在串口里看 [PUL]，就不
  要再同时开 monitor_com14.py，用 --monitor 让本脚本把日志一并打出来。

对拍方法（默认就开着，不用改固件）：
  固件通过 printf（USART1 / PA9 / 115200）每秒打一行自检：

      [PUL] cap=100000 deg=2812.500 spr=12800 PUL=1 hi=50% DIR=0 pd=0 ovf=0

    cap   已捕获的上升沿总数，应该和本脚本打印的“脉冲数”完全一致（一个不差）
    deg   按 DIR 快照累加的有符号角度，DIR 接 GND 时应为 cap*360/每圈步数
          （每圈步数 = 200 x 驱动器实际微步，默认 12800，与 --steps-per-rev 保持一致）
    PUL   PUL 引脚此刻的电平（只采一点，仅供参考）
    hi    最近 1 秒内 PUL 被判为高电平的采样占比（每秒约 200 点，只读引脚、不经过捕获）
    DIR   DIR 引脚此刻的电平（接 GND 应为 0；悬空会乱跳，导致 deg 在 0 附近正负抵消）
    pd    DMA 积压深度，持续非 0 说明 rampUpdate 消费不过来
    ovf   环形缓冲溢出（丢脉冲）

  判读表：
    cap 在涨                  -> 全链路 OK，可以和脚本的脉冲数逐个对拍
    cap 不涨、hi≈100%         -> 引脚一直是高：要么根本没在发数据，要么高电平降不下来
                                （源极跟随没有下拉 / 负载电容太大 / 低电平时间太短）
    cap 不涨、hi≈0%          -> 引脚一直是低：高电平没被 MCU 认成高
                                （VIH 裕量不足，2.5V 很危险）、没共地，或源极根本没导通
    cap 不涨、hi 在 0~100 之间  -> 引脚确实在跳变，但捕获没记到；这是定时器/DMA 侧的问题，
                                把 include/pulse_capture.h 的 PULSE_CAPTURE_TICK_DMA 置 0 再试
    cap 只涨 1 就不动         -> 典型“只有第一个上升沿”：之后电平没降到 VIL 以下，再也没新上升沿
    cap 在涨、deg≈0           -> DIR 悬空

  注意：--period-bits 2（默认，0x55）是 50% 占空比，高/低电平都是 1 个位时间，对模拟通路最友好；
  --period-bits 10（0xFF）的低电平只有 1 个位时间，在源极跟随 + 弱下拉的电路上几乎必丢，
  除非已经换成推挽/共源极输出。

  想关掉这行打印：把 include/pulse_capture.h 里 PULSE_CAPTURE_SELFCHECK_LOG 置 0。

！！CH340G 实测速率误差（本机 COM14 实测，200000 字节取平均）：
    标称波特率   实测波特率    偏差    对应脉冲频率（period=2）
      115200       115739      +0.5%        57.9 kHz
      200000       200509      +0.3%       100.2 kHz
      500000       498464      -0.3%       249.2 kHz
      921600       836599     -10.2%       418.3 kHz
     1000000       902653     -10.7%       451.3 kHz
     1500000      1241394     -20.8%       620.7 kHz
     2000000      1524372     -31.2%       762.2 kHz
  也就是说 ≤500k 波特率基本准，1M 以上会明显偏慢（驱动分频精度不够）。
  要精确的高频，用 --trim 反比例补偿，例如想得到真 1 Mbaud 就写：
      --baud 1107000        # 实测得到 999567 baud
  或者直接 --trim 1.107。
"""

import argparse
import sys
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("缺少 pyserial，请先安装：pip install pyserial")

# Windows 控制台不是 UTF-8 时，遇到印不出来的字符直接降级，别让整个脚本崩掉。
try:
    sys.stdout.reconfigure(errors="replace")
except Exception:
    pass

# UART 8N1：1 开始位 + 8 数据位 + 1 停止位
BITS_PER_FRAME = 10

# 周期位数 -> (要发送的字节, 每个字节包含的完整脉冲数)
# 0x55: 线上 0101...，周期 2 位，每帧 5 个上升沿
# 0xEF: 线上 01111...，周期 5 位，每帧 2 个上升沿
# 0xFF: 线上 011111111...，周期 10 位，每帧 1 个上升沿
PERIOD_TABLE = {
    2: (0x55, 5),
    5: (0xEF, 2),
    10: (0xFF, 1),
}

CHUNK_BYTES = 1024          # 每次往驱动里塞的块大小，要比驱动缓冲小，才能把 TX 队列持续顶满
# 每圈 STEP 数 = 200 步/圈 x 驱动器实际微步数。这里只用于把频率换算成 RPM 显示，
# 必须和固件一致：1 个脉冲 = 1 个微步，不要再按 256 细分的 51200 算。
DEFAULT_STEPS_PER_REV = 12800  # 200 步/圈 x 64 微步


class Writer(threading.Thread):
    """后台把固定的波形字节流不断写进串口，写入速度由驱动/波特率自动限流。"""

    def __init__(self, ser, byte_value, budget_bytes, chunk_bytes=CHUNK_BYTES):
        super().__init__(daemon=True)
        self._ser = ser
        self._chunk = bytes([byte_value]) * chunk_bytes
        self._chunk_len = chunk_bytes
        self._budget = budget_bytes  # 0 表示不限量，靠外部 stop 事件停
        # 注意：不能叫 _stop，threading.Thread 内部有个同名方法，覆盖它会在线程收尾时
        # 抛 TypeError: 'Event' object is not callable
        self._stop_event = threading.Event()
        self.written = 0
        self.error = None

    def stop(self):
        self._stop_event.set()

    def run(self):
        try:
            while not self._stop_event.is_set():
                if self._budget:
                    remain = self._budget - self.written
                    if remain <= 0:
                        break
                    if remain < self._chunk_len:
                        self._ser.write(self._chunk[:remain])
                        self.written += remain
                        break
                self._ser.write(self._chunk)
                self.written += self._chunk_len
        except Exception as exc:  # SerialException / SerialTimeoutException 等
            self.error = exc
            self._stop_event.set()


class LinePrinter:
    """把串口收到的字节按行解出来打印（MCU 的 printf 用 \\r\\n 结尾）。"""

    def __init__(self, prefix="  [MCU] "):
        self.prefix = prefix
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)
        while True:
            idx = self.buf.find(b"\n")
            if idx < 0:
                break
            line = bytes(self.buf[:idx])
            del self.buf[:idx + 1]
            text = line.decode("utf-8", "replace").replace("\r", "").strip()
            if text:
                print(self.prefix + text, flush=True)
        # 一直没有换行（比如把 TXD 短到 RXD 做回环，收到的是 0x55 流）：
        # 攒到 64 字节就按十六进制吐一行，免得“明明收到了却一个字都不打印”。
        if len(self.buf) >= 64:
            chunk = bytes(self.buf[:64])
            del self.buf[:64]
            print(self.prefix + "HEX " + chunk.hex(" "), flush=True)


def drain_output(ser, timeout_s=10.0):
    """等驱动里的数据真正发完（Windows 上 pyserial 通过 ClearCommError 拿到待发字节数）。"""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            pending = ser.out_waiting
        except Exception:
            return
        if pending == 0:
            return
        time.sleep(0.005)


def pick_default_port():
    """自动找 CH340（USB VID:PID=1A86:7523），找不到就列出所有串口并报错。"""
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        hwid = (port.hwid or "").upper()
        desc = (port.description or "").upper()
        if "1A86:7523" in hwid or "CH340" in desc:
            return port.device
    print("没找到 CH340 串口，当前系统上的串口如下：")
    for port in ports:
        print("  %-6s %s" % (port.device, port.description))
    sys.exit("请用 --port 指定，例如 --port COM14")


def build_plan(args):
    if args.period_bits not in PERIOD_TABLE:
        sys.exit("--period-bits 只支持 2 / 5 / 10（原因见文件头说明）")
    byte_value, pulses_per_byte = PERIOD_TABLE[args.period_bits]

    baud = args.baud if args.baud else int(round(args.freq * args.period_bits * args.trim))
    if baud <= 0:
        sys.exit("频率/波特率必须为正数")
    if baud > args.max_baud:
        limit = args.max_baud / args.period_bits
        sys.exit(
            "算出来的波特率 %d 超过上限 %d（CH340G 实际最高 2M）. "
            "把 --freq 降到 %.0f Hz 以下再试。" % (baud, args.max_baud, limit)
        )

    real_freq = baud / args.period_bits
    bit_time = 1.0 / baud
    pulse_width_s = bit_time * (args.period_bits - 1)  # 高电平位数 = 周期 - 1

    if args.count is not None:
        if args.count <= 0:
            sys.exit("--count 必须大于 0")
        pulses = args.count
        if pulses % pulses_per_byte:
            pulses = (pulses // pulses_per_byte) * pulses_per_byte
            print("提示：脉冲数按每字节 %d 个对齐，实际发送 %d 个（丢掉 %d）"
                  % (pulses_per_byte, pulses, args.count - pulses))
        total_bytes = pulses // pulses_per_byte
        estimate_s = total_bytes * BITS_PER_FRAME * bit_time
    else:
        pulses = None
        total_bytes = 0  # 不限量，跑满 duration
        estimate_s = args.duration

    return {
        "byte_value": byte_value,
        "pulses_per_byte": pulses_per_byte,
        "baud": baud,
        "real_freq": real_freq,
        "pulse_width_s": pulse_width_s,
        "total_bytes": total_bytes,
        "pulses": pulses,
        "estimate_s": estimate_s,
    }


def print_plan(plan, args):
    print("=== 发送计划 ===")
    print("  端口        : %s%s" % (args.port, "（dry-run，不打开）" if args.dry_run else ""))
    print("  波形字节    : 0x%02X（线上周期 %d 位）" % (plan["byte_value"], args.period_bits))
    baud_note = ""
    if plan["baud"] > 500000:
        baud_note = "  ← 500k 以上 CH340G 会偏慢，必要时用 --trim 补偿（见文件头实测表）"
    print("  波特率      : %d bps%s" % (plan["baud"], baud_note))
    print("  脉冲频率    : %.3f Hz（每位 %.3f us，高电平 %.3f us）"
          % (plan["real_freq"], (1.0 / plan["baud"]) * 1e6, plan["pulse_width_s"] * 1e6))
    print("  低电平时间  : %.3f us（这一段要靠下拉才拉得下来，太短就会“只有第一个上升沿”）"
          % ((1.0 / plan["baud"]) * 1e6))
    print("  等效转速    : %.3f RPM（按 %d 脉冲/圈）"
          % (plan["real_freq"] / args.steps_per_rev * 60.0, args.steps_per_rev))
    if plan["pulses"] is not None:
        print("  脉冲数      : %d 个（%d 字节，理论耗时 %.3f s）"
              % (plan["pulses"], plan["total_bytes"], plan["estimate_s"]))
    else:
        print("  持续时间    : %.3f s（不限脉冲数，理论最多 %d 个）"
              % (args.duration, int(args.duration * plan["real_freq"])))
    print("  接线确认    : GND 共地 / TXD -> PB5(PUL) / PB4(DIR) 接 GND 或 3V3 不许悬空")
    if args.loopback:
        print("  回环自检    : 已开启（需将 TXD 与 RXD 短接；收不到就是脚本/驱动/端口侧的问题）")
    if args.monitor:
        note = "" if plan["baud"] == 115200 else (
            "  ← 当前 baud=%d，固件日志读出来会是乱码；改用 --freq 57600 让 baud=115200" % plan["baud"])
        print("  同时监控    : 已开启（TXD->PB5 / RXD->PA9，MCU 日志 115200 8N1）%s" % note)
    print("  电平判读    : 固件 [PUL] 行的 hi= 占比：发 0x55 时应≈50%；恒 100% = 降不下去；恒 0% = 高电平没被认成高")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="用 CH340G 的 UART 硬件比特流模拟 PUL/STEP 脉冲（无软件翻转）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port", default=None, help="CH340G 的串口号（不填则自动搜 CH340）")
    parser.add_argument("--freq", type=float, default=100000.0, help="目标脉冲频率（Hz）")
    parser.add_argument("--period-bits", type=int, default=2, choices=sorted(PERIOD_TABLE),
                        help="一个脉冲占几个 UART 位时间（2=50%% 占空比，10=10%% 占空比）")
    parser.add_argument("--baud", type=int, default=0,
                        help="直接指定波特率（0 = 自动取 freq*period_bits）")
    parser.add_argument("--count", type=int, default=None, help="精确发送这么多个脉冲")
    parser.add_argument("--duration", type=float, default=2.0,
                        help="没给 --count 时，持续发送多少秒")
    parser.add_argument("--max-baud", type=int, default=2000000, help="波特率上限保护")
    parser.add_argument("--trim", type=float, default=1.0,
                        help="波特率补偿系数（CH340G 高波特率偏慢：~1M 用 1.107，~1.5M 用 1.208）")
    parser.add_argument("--keep-dtr-rts", action="store_true",
                        help="不主动拉低 DTR/RTS（默认会拉低，避免误触发目标板复位）")
    parser.add_argument("--dry-run", action="store_true", help="只打印计划，不打开串口")
    parser.add_argument("--loopback", action="store_true",
                        help="回环自检：先把 CH340 的 TXD 与 RXD 短接，脚本边发边收，最后报收发字节数"
                             "（用来证明 python/驱动/端口到底有没有在发数据，建议 --freq 50000 以下）")
    parser.add_argument("--monitor", action="store_true",
                        help="边发脉冲边把 MCU 从 RXD 回来的日志按行打印（TXD->PB5 / RXD->PA9）。"
                             "注意波特率共用：固件 printf 是 115200，所以用 --freq 57600 才能同时读到日志。"
                             "如果只是把 TXD-RXD 短接做回环，请改用 --loopback（0x55 里没有换行，按行打不出来）")
    parser.add_argument("--monitor-after", type=float, default=2.0,
                        help="--monitor 时，脉冲发完后继续收多少秒日志")
    parser.add_argument("--steps-per-rev", type=int, default=DEFAULT_STEPS_PER_REV,
                        help="每圈 STEP 数（= 200 x 驱动器微步），仅用于把频率换算成 RPM 显示")
    args = parser.parse_args()

    if args.port is None:
        args.port = pick_default_port()

    plan = build_plan(args)
    print_plan(plan, args)

    if plan["pulse_width_s"] < 100e-9:
        sys.exit("高电平只剩下 %.1f ns，小于固件要求的 100ns 最窄脉宽" % (plan["pulse_width_s"] * 1e9))

    if args.dry_run:
        return 0

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=plan["baud"],
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,
            write_timeout=5,
        )
    except serial.SerialException as exc:
        print("当前串口：")
        for port in serial.tools.list_ports.comports():
            print("  %-6s %s" % (port.device, port.description))
        sys.exit("打开 %s 失败：%s（串口是否被别的程序占用？）" % (args.port, exc))

    try:
        if not args.keep_dtr_rts:
            try:
                ser.dtr = False
                ser.rts = False
            except Exception:
                pass
        ser.reset_output_buffer()

        writer = Writer(ser, plan["byte_value"], plan["total_bytes"])
        start = time.time()
        writer.start()

        # 回环模式下必须边发边收：Windows 的 RX 缓冲只有几 KB，等发完再读就等于丢数据
        received = 0
        printer = LinePrinter() if args.monitor else None

        def pump(seconds):
            """读串口：seconds=0 只读一次；>0 则读到“连续这么多秒没有新数据”为止。"""
            nonlocal received
            idle_deadline = time.time() + seconds
            while True:
                chunk = ser.read(ser.in_waiting or 1)
                if chunk:
                    received += len(chunk)
                    if printer is not None:
                        printer.feed(chunk)
                    idle_deadline = time.time() + seconds
                elif seconds <= 0:
                    time.sleep(0.0002)
                    return
                elif time.time() >= idle_deadline:
                    return
                else:
                    time.sleep(0.0005)

        try:
            while writer.is_alive():
                if plan["pulses"] is None and time.time() - start >= args.duration:
                    writer.stop()
                    break
                if args.loopback or args.monitor:
                    pump(0)
                else:
                    time.sleep(0.002)
        except KeyboardInterrupt:
            print("\n收到 Ctrl+C，停止发送")
            writer.stop()
        writer.join()

        if args.monitor:
            print("=== 脉冲已发完，继续接 MCU 日志 %.1f s ===" % args.monitor_after, flush=True)
            pump(args.monitor_after)
            print("  收到 MCU 字节: %d%s" % (received, "" if received else
                  "  ← 0 字节：RXD 没接到 PA9，或波特率不是固件 printf 的 115200"), flush=True)
        elif args.loopback:
            # 等驱动把最后一点回环数据交上来
            pump(0.2)

        drain_output(ser)
        elapsed = time.time() - start

        if writer.error is not None:
            print("发送过程出错：%s" % writer.error)

        sent_bytes = writer.written
        sent_pulses = sent_bytes * plan["pulses_per_byte"]
        ideal_s = sent_bytes * BITS_PER_FRAME / plan["baud"]
        print("=== 发送结束 ===")
        print("  字节数      : %d" % sent_bytes)
        print("  脉冲数      : %d" % sent_pulses)
        print("  理论耗时    : %.3f s（连续无空档时）" % ideal_s)
        print("  墙钟耗时    : %.3f s（多出来的部分是 USB/驱动缓冲与调度空档，不影响脉冲总数）"
              % elapsed)
        print("  对照固件    : pulse_capture_get_captured_count() 应等于 %d" % sent_pulses)
        if plan["pulses"] is not None and sent_pulses != plan["pulses"]:
            print("  警告        : 实际发送数与计划不一致（驱动提前返回/超时）")

        if args.loopback:
            print("=== 回环校验（TXD 与 RXD 必须短接）===")
            print("  应收到字节  : %d" % sent_bytes)
            print("  实收到字节  : %d" % received)
            if received == 0:
                print("  结论        : 一个字节都没回来 -> 端口选错 / TXD-RXD 没短接 / 驱动没在发，先解决这一层")
            elif received < sent_bytes:
                print("  结论        : 收回 %.1f%%，能收到就说明 TX 确实在发数据，差额是 Windows RX 缓冲溢出"
                      % (100.0 * received / max(1, sent_bytes)))
            else:
                print("  结论        : 回环完整，python + CH340 + 驱动都在正常发数据，问题在线路/电平侧")
        return 0 if writer.error is None else 1
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
