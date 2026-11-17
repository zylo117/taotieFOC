# 自定义 Bootloader（USB DFU，免按键刷机）

目标：**改代码 → 刷机 → 自动测试** 全程不碰板子上的任何按键，并且**永远刷不死**。

---

## 1. Flash 分区（单一真相源：`include/bl_layout.h`）

| 地址 | 大小 | 内容 | 谁能写 |
|---|---|---|---|
| `0x08000000`~`0x08001FFF` | 8 KB | **Bootloader**（`src/bootloader/`，`[env:bootloader]`） | 只有 ST-Link / 芯片内置 DFU；DFU 通路被守卫一票否决 |
| `0x08002000`~`0x0801BFFF` | 104 KB | **应用**（现有固件，`[env:genericAT32F403ACCT7]`） | DFU |
| `0x0801C000` | | 滞后角拟合元数据 | 只有应用（永不擦） |
| `0x0801D000`~`0x0801E7FF` | 6 KB | 滞后角拟合点 | 只有应用 |
| `0x0801F000` / `0x0801F800` | | 编码器校准元数据 / 主表 | 只有应用 |
| `0x08020000`~`0x0803FFFF` | 128 KB | FastCal 稠密表 | 只有应用 |

- 应用镜像最后 16 字节（`0x0801BFF0`）是**镜像尾签**：`magic('ZFIL') | payload_len | crc32 | flags`。
- RAM `0x20017C00` 的 1KB 是 **BL↔应用交接页**：两份链接脚本都把 RAM 从 96K 砍到 95K、`_estack` 从 `0x20018000` 降到 `0x20017C00`，所以它不被 `.bss` 清零、不被栈踩到，又能活过 `NVIC_SystemReset`。
- 应用当前 100,156 字节，**余 6,340 字节**。`board_upload.maximum_size = 106496` 是保险丝：超了编译期就报错，不会静默压到尾签或校准区上。

## 2. 首次烧 BL（唯一还需要手的一次）

BL 自己不能被自己刷（这正是"刷不死"的前提），所以第一次必须借芯片内置 DFU 或 ST-Link：

```
1) 断电 → 按住 BOOT0 → 上电 → 松手          # 进芯片内置 DFU
2) pio run -e bootloader                    # 编出 8KB BL
3) pio run -e bootloader -t upload          # = py -3 tools/dfu_flash.py --bootloader
4) 断电 → 正常上电（BOOT0 悬空/拉低）        # 从此以后不再需要按键
```
也可以 `pio run -e bootloader -t upload -t ... ` 之外用 ST-Link：
`openocd -f interface/stlink.cfg -f target/at32f403axx.cfg -c "program .pio/build/bootloader/firmware.bin 0x08000000 verify reset exit"`。

> **首次烧 BL 之前请先备份校准区**（滞后角 + 编码器校准 + FastCal 共 136KB）。
> 设计上不会碰它，但"先备份再动 flash"是纪律，不是多余。
> `dfu-util -u 0 -s 0x0801C000:0x20000 -U calib_backup.bin`（内置 DFU 下可读全片）。

## 3. 日常：改代码 → 刷机 → 测试

```bash
pio run                                          # 编应用（104KB 上限，超了直接报错）
pio run -t upload                                # 免按键：0x19A → DFU → 刷 → 回读校验 → 回应用
py -3 scripts/tt_autotest_angle.py               # 自动测试 1：0°→233°→90°→117°→0°
py -3 scripts/tt_autotest_planner.py             # 自动测试 2：开环/闭环 3 圈正 2 圈反
```

`pio run -t upload` 展开就是 `tools/dfu_flash.py`，它做的事：

1. 打开应用的 CDC 口，写 **`0x19A = 1`**；固件先 `stopMotion()` + 失能驱动，等 0x82 ACK 发完，
   在交接页写下 `request = DFU` 然后 `NVIC_SystemReset()`。
2. BL 复位起来看到 request → **枚举成 `VID_2E3C PID_DF11`**（和芯片内置 DFU 同一对 ID，
   Windows 早已绑好驱动，所以不用再装一次 Zadig）。
3. 生成定长镜像：`固件 + 0xFF 补齐到 0x1AFF0 + 16 字节尾签` = 正好 104KB，
   `dfu-util -a 0 -s 0x08002000 -D`。
4. **回读校验**：用 DFU_UPLOAD 把 104KB 整片读回来逐字节比对（不靠 dfu-util 的退出码说话）。
5. BL 在 manifest 校验通过后进入 `dfuMANIFEST-WAIT-RESET`，**等主机静默 2.5 秒**才自己复位
   —— 这个窗口就是留给第 4 步回读的；定长 400ms 复位会把回读抢掉，看起来像刷机失败。
   脚本再等 CDC 口回来 = 新固件确实跑起来了。

其它常用参数：`--no-enter`（已在 DFU 里）、`--enter-only`（只进不刷）、`--list`、
`--protect`（顺带给 BL 区上 EPP 硬件写保护）、`--port COM9`。

## 4. 为什么刷不死

| 风险 | 挡住它的机制 |
|---|---|
| DFU 把 BL 自己擦掉 | `bl_addr_in_app()`：任何擦/写地址必须整段落在 `[0x08002000, 0x0801C000)`，越界直接 `ERR_ADDR`，**一条 flash 命令都不发**。擦完还比对 BL 区首尾哨兵字。 |
| 校准数据被抹 | 同上，`0x0801C000` 以上全部在允许区间之外。 |
| 刷到一半拔线/断电 | 有效镜像的判据是"尾签 + 整镜像 CRC32"。尾签只是镜像的一部分，**没传完 = CRC 不符 = 停在 DFU**。下载前整片擦也是安全的：擦完断电同样停在 DFU。重跑 `pio run -t upload` 即可，不需要"续传"。 |
| 镜像完整但是个坏固件 | 应用启动后第一件事清 `boot_attempt`；BL 每次跳转前 +1，**连续 3 次启不来就常驻 DFU**，不会无限重启。 |
| 应用启不来又没有 BL 入口 | 兜底：按 BOOT0 上电进芯片内置 DFU（它永远在），或 ST-Link。 |
| 晶振/PLL 配错 | BL **跑在复位默认的 HICK 8MHz，不用 HEXT、不开 PLL**（USB 的 48MHz 来自校准 HICK，和 sysclk 无关）。外部晶振坏了 BL 照样能枚举刷机。 |

**BL 区硬件写保护（可选加固）**：`tools/dfu_flash.py --protect` 在尾签 flags 里置位，
BL 在 manifest 时调 `flash_epp_set()` 保护 EPP bit0/bit1（每 bit 管 4KB，正好 BL 的 8KB）。
之后连 BL 自己都擦不掉 BL 区；解除要 `flash_user_system_data_erase()` 之后用 ST-Link 重刷 BL。
应用可以用 **`0x19B`** 读回 BL 诊断（高 16 位状态码 `DF`/`JM`/`JF` + 原因，低 16 位启动尝试次数）。

> EPP 的 bit↔地址映射（"每 bit 4KB"）来自 flash 库注释，**尚未在本机芯片上实测确认**。
> 因此它被做成显式 opt-in，而不是默认行为；软件守卫才是默认且始终生效的那道闸。

## 5. 已知代价 / 未验证项

- **应用只剩 6,340 字节生长空间**（104KB 区 − 100,156）。8KB BL 是极限：BL 用 12KB 就只剩 2.5KB，
  用 16KB 直接装不下（应用 99,720 > 96KB）。真要更多空间，代价是把 FastCal 稠密表挪走 → 需要重新标定，
  那是另一个决定。
- BL 实测 **8,120 / 8,192 字节**，余 72 字节。加功能前先 `-t size` 看余量。
- **不要给 `[env:bootloader]` 开 `-flto`**：实测 LTO 会把 AT32 startup 的 `.weak → Default_Handler`
  弱别名解析赢掉，`USBFS_L_CAN1_RX0_IRQHandler` 被丢弃、向量表 word59 指向死循环，
  尺寸从 8,216 掉到 4,180 很好看，实际是 USB 一进中断就死、永远枚举不出 DFU，且**一个警告都没有**。
  平台无关的教训：省体积省到中断向量表上，必须逐条核对向量表。
- **在机验证进度**（2026-11-17 深夜，第一次上机）：BL 已通过内置 DFU 写入并回读逐字节一致、
  校准区 128KB 与刷前逐字节一致 —— 这两条是实测。**仍然未验证**：我的 BL 能否枚举
  （卡在需要一次真复位，见第 7 节）、64 字节分块刷 104KB 的实际耗时、EPP 生效、
  0x19A 进 DFU、两个自动测试脚本的阈值。
- 自动测试 2 里 `0x120` 速度寄存器的标度（RPM 还是 milli-RPM）需要在真机上看一眼，脚本里对两种
  标度都做了兼容打印，确认后可以写死。
- 顺手发现一个**不是我改出来的**问题：`src/parameter_router.cpp` 写分支里 `0x191`
  （`ANGLE_SERVO_HOLD_MS`）的处理块被嵌进了 `if (reg == ENGINE_TUNING)` 的花括号内部
  （约 549~570 行），条件永远不可能同时成立 → **写 0x191 现在是空操作**。这是 2026-11-17-B
  交棒功能合并时的缩进/花括号事故，我没动它，等你确认后再修。

## 6. 新增文件

| 文件 | 作用 |
|---|---|
| `include/bl_layout.h` | 分区/尾签/交接页的单一真相源（BL 与应用共用） |
| `bootloader/linker/bl_AT32F403AxC_FLASH.ld` | BL 8KB / RAM 95K |
| `bootloader/linker/app_AT32F403AxC_FLASH.ld` | 应用 ORIGIN=`0x08002000` 104K / RAM 95K |
| `src/bootloader/main.c` | 启动决策、USB 初始化、DWT 延时、manifest 后自动复位 |
| `src/bootloader/bl_flash.c/h` | 地址守卫、扇区表擦除、CRC32 镜像校验、跳应用、EPP |
| `src/bootloader/bl_dfu.c/h` | 手写 DFU 1.1 类（EP0-only），擦写全在主循环，中断只回 `dfuDNBUSY` |
| `src/bootloader/bl_desc.c/h` | DFU 设备/配置/功能/字符串描述符（`VID_2E3C PID_DF11`） |
| `src/bl_handshake.c/h` | 应用侧交接页：启动确认、武装进 DFU、诊断读取 |
| `tools/dfu_flash.py` | 进 DFU → 生成镜像 → 刷 → 回读校验 → 等应用回来；`--protect`/`--bootloader` |
| `scripts/tt_autotest_angle.py` | 自动测试 1：角度模式 0/233/90/117/0，到位时间+停稳抖动+定位误差 |
| `scripts/tt_autotest_planner.py` | 自动测试 2：开环/闭环 3 圈正 2 圈反，单段误差 + 累计误差 + 峰值速度 |

## 7. 上机实测记录

**2026-11-17 23:40，板子按 BOOT0 上电进芯片内置 DFU：**

| 动作 | 结果 |
|---|---|
| `dfu-util --list` | `[2e3c:df11] alt=0 "@Internal Flash /0x08000000/128*2Kg"`、`alt=1 "@Option Byte"`、serial `"AT32"` → 确认是内置 DFU |
| 全片 256KB 读出 | 成功（**没开读保护**），备份 `backup/flash_full_*.bin`，SHA1 `C51895584C84B2CF48C2BC0ADC74B74CA3183F5E` |
| 刷 BL 8,104 字节 @`0x08000000` | `Download done.` + 回读 8,192 字节**逐字节一致**；BL 向量 SP=`0x20017C00`、Reset=`0x08000A7D` |
| 校准区是否被波及 | 再读 `0x0801C000`+128KB，与刷前备份**完全一致** ✓ |
| `dfu-util -R` 想退出 DFU | **做不到**：`-R` 只是 USB 总线复位，不是芯片复位，内置 DFU 原样回来 |

三条实测得到的教训：

1. **`dfu-util -R` 逃不出内置 DFU。** 让新 BL 第一次跑起来只能靠断电重上电 / 按 RESET，
   且 BOOT0 必须为低。我没有去写 option byte 触发复位 —— 写错 BOOT 映射或误开 RDP
   会把芯片锁死，那是拿一块板子换一个省手的动作，不划算。
2. **内置 DFU 报的扇区布局是均匀 `128 × 2KB`**，不是我原先按 STM32 习惯推的"低 16 扇区 1KB"。
   对 BL 无害：`bl_app_erase_all()` 以 1KB 步进走扇区表，真实 2KB 扇区会被重复擦两次
   （幂等，只是慢一点），而 `0x08002000` 无论按 1K 还是 2K 都是扇区边界，**永远不会向下越进 BL 区**；
   擦完还有哨兵字比对 + 整区 0xFF 校验兜底。
3. **共用 VID/PID 的代价**：`2e3c:df11` 既是内置 DFU 也是我的 BL，只按 VID:PID 判断会张冠李戴。
   `dfu_flash.py` 现在按描述符区分（我的是 serial `TAOTIE` / 单个 alt / iProduct `taotie DFU`），
   `--list` 会直接打印"识别为: mine / builtin"。重刷 BL 时如果识别到 `mine` 会直接拒绝并提示按 BOOT0。

**2026-11-18 00:30，换上修好的描述符之后，三条真机结论（每条都值一次上机）：**

4. **AT32F403A 是均匀 2KB 扇区**，不是我按 STM32 习惯推的"1K/2K/4K 分段"。
   证据就在内置 DFU 自报的 `128*2Kg` 里，只是我第一版没读这句话。
   `flash_sector_erase(addr)` 只擦**地址所在那一个** 2KB 扇区，所以我按 4KB 跳步时
   `≥0x0800C000` 每个窗口的上半 2KB **从没被擦**，整区 0xFF 自检必然失败。
   现在擦除固定 **1KB 步进**，与真实扇区几何无关：最坏是重复擦同一扇区（幂等、多几毫秒），
   绝不会漏擦。`bl_sector_size()` 那张表已删除，避免下一个人再去"优化"步长。
5. **`bcdDFU` 必须报 `0x0110`，不能报 `0x011A`。** dfu-util 看到 `>= 0x011A` 就认定这是
   ST 的 **DfuSe** 变体，转而强制要求接口字符串是 `@Flash /0x08002000/52*2Kg` 这种内存布局，
   解析不到就 `Failed to parse memory layout for alternate interface 0` 直接退出（实测 exit 74，
   一个字节都不发）。报 `0x0110` 才走标准 DFU 1.1：主机只发**块号**，地址由设备算
   （这也正是我工具里不能带 `-s` 的原因）。改对之后 dfu-util 立刻输出
   `Device DFU version 0110` / `Device returned transfer size 64` 并开始拷贝。
6. **bStatus 必须严格按 DFU 1.1 Table 13 编号，设备侧自创编号会把诊断彻底带偏。**
   我原来按自己顺序排了 `02/04/06/07/0B`，于是"擦除失败"发出去的是 `0x07`，
   dfu-util 拿正确表翻译成 *"Programmed memory failed verification"* ——
   我顺着这句话查了半天 manifest 校验，而真凶是擦除（第 4 条）。
   现在照规范表：`03=WRITE 04=ERASE 05=CHECK_ERASED 06=PROG 07=VERIFY 08=ADDRESS`，
   并且把"擦除失败 / 擦不干净 / 编程失败 / 守卫拒绝 / CRC 不符"分别映射到
   `04 / 05 / 06 / 08 / 07` —— 上位机的报错文本第一次能直接对应到我的代码分支。
   （状态机 Table 14 那套 `dfuIDLE=2 … dfuERROR=10` 一开始就是对的，错的只有状态码。）
