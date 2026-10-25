#include "tmc2209_driver.h"
#include "at32f403a_407_board.h"
#include "core_cm4.h"

#include <stdio.h>

/*
 * TMC2209 单线 UART（PDN_UART）实现——时序与协议照 test/main_test_tmcuart.cpp 已跑通的单元测试移植
 *
 * 数据报格式（同步字节主机与驱动器都是 0x05，早前写成 0xF5 是错的，实测驱动器不收）：
 *   写：05 00 (REG|0x80) D3 D2 D1 D0 CRC    共 8 字节
 *   读：05 00 REG CRC                       共 4 字节
 *   答：05 FF REG D3 D2 D1 D0 CRC           共 8 字节
 *
 * 时序：DWT->CYCCNT 做基准，1bit = system_core_clock / baud
 *   TX：推挽输出，按绝对时刻翻转电平，多位发送误差不累积
 *   RX：输入上拉，检出起始位下降沿后 1.5bit 采第一位，之后每 1bit 采一位，并校验停止位
 *
 * 两条实测结论（都是从单元测试里踩出来的，别改回去）：
 *   1. 数据报之间必须等总线「连续静默 64bit」，只留 8bit 时驱动器还停在发送态，新请求整帧被丢；
 *   2. 这块模块回读 IHOLD_IRUN(0x10) 恒为 0，写是否生效只能看 IFCNT 增量 + DRV_STATUS.CS_ACTUAL；
 *   3. 进了 FreeRTOS 之后一次读写事务必须整段挡住 TMR3/TMR4，否则起始位会被 ISR 整段吃掉——
 *      表现就是「上电自检能读、连上位机全部读不到」（自检时两个定时器还没开）。
 */
namespace
{
  /* ---------------- 协议常量 ---------------- */
  constexpr uint8_t kTmcSync = 0x05U;
  constexpr uint8_t kTmcSlaveAddress = 0x00U;    /* MS1 = MS2 = GND */
  constexpr uint8_t kTmcMasterAddress = 0xFFU;
  constexpr uint8_t kTmcWriteFlag = 0x80U;

  constexpr uint8_t kRegGconf = 0x00U;
  constexpr uint8_t kRegIfcnt = 0x02U;
  constexpr uint8_t kRegIholdIrun = 0x10U;
  constexpr uint8_t kRegTpowerdown = 0x11U;
  constexpr uint8_t kRegChopconf = 0x6CU;
  constexpr uint8_t kRegDrvStatus = 0x6FU;
  constexpr uint8_t kRegPwmconf = 0x70U;

  /* GCONF 位定义 */
  constexpr uint32_t kGconfSpreadCycle = 1UL << 2U;    /* 1 = spreadCycle，0 = stealthChop */
  constexpr uint32_t kGconfPdnDisable = 1UL << 6U;     /* PDN 脚当 UART 用，必须置 1 */
  constexpr uint32_t kGconfMstepRegSelect = 1UL << 7U; /* 细分由 CHOPCONF.MRES 决定而不是 MS1/MS2 */

  /* CHOPCONF 位定义 */
  constexpr uint32_t kChopconfToffMask = 0x0FUL;
  constexpr uint32_t kChopconfVsense = 1UL << 17U;     /* 1 = Vfs 0.180V，0 = Vfs 0.325V */
  constexpr uint32_t kChopconfMresMask = 0x0FUL << 24U;
  constexpr uint32_t kChopconfIntpol = 1UL << 28U;     /* 插值到 256 微步 */
  constexpr uint32_t kChopconfDefaultToff = 0x03UL;
  constexpr uint32_t kChopconfResetValue = 0x10000053UL;

  /* PWM_CONF：stealthChop 自动电流标定 */
  constexpr uint32_t kPwmconfAutoscale = 1UL << 18U;
  constexpr uint32_t kPwmconfAutograd = 1UL << 19U;
  constexpr uint32_t kPwmconfDefault = 0xC10D0024UL;

  /* 电流换算：除了采样电阻，芯片内部上下桥导通电阻按 20mΩ 计入 */
  constexpr float kSenseInternalOhm = 0.020f;
  constexpr float kVfsLow = 0.180f;
  constexpr float kVfsHigh = 0.325f;
  constexpr float kSqrt2 = 1.41421356f;

  /* 总线时序 */
  constexpr uint32_t kInterPacketGapBits = 256U;  /* 数据报之间强制静默位数（实测靠电平判不可行，改固定等） */
  /* 等应答起始位的上限。这个等待现在整段处在临界区里，不能像以前那样给 200bit（5ms）。
   * TMC2209 收完读请求后几个 bit 内就开始回包，64bit 已经是 8 倍余量 */
  constexpr uint32_t kStartBitTimeoutBits = 64U;
  constexpr uint8_t kReadRetryCount = 3U;
  constexpr uint8_t kWriteRetryCount = 3U;
  constexpr uint32_t kIholdDelay = 8UL;            /* 停转后约 0.5s 降到保持电流 */

  /* UART 离线短路。
   * 模块烧毁 / PA4 断线 / 没供电时，每一次 readRegister 都要白跑 3 次重试（约 30ms），
   * writeRegister 要 300ms 上下；上位机一次“应用电机参数”会被拖到几秒，看起来就是卡死。
   * 连续 kUartOfflineThreshold 次完整读失败就判定离线，之后读写立即返回失败，
   * 由 init() / 重新绑定串口引脚时再试探。
   * 注意：这是单例驱动的文件级状态（全局只有一个 g_driver），不放在头文件里。 */
  bool g_uart_offline = false;
  uint8_t g_uart_fail_streak = 0U;
  uint8_t g_offline_skip = 0U;
  constexpr uint8_t kUartOfflineThreshold = 3U;
  constexpr uint8_t kUartOfflineProbeEvery = 32U;

  void uartNoteSuccess()
  {
    g_uart_fail_streak = 0U;
    g_offline_skip = 0U;
    g_uart_offline = false;
  }

  void uartNoteFailure(uint8_t reg)
  {
    if (!g_uart_offline && ++g_uart_fail_streak >= kUartOfflineThreshold)
    {
      g_uart_offline = true;
      printf("[TMC_UART] PA4 unreachable after %u consecutive failures (last reg=0x%02X); UART marked offline, reads/writes fail fast until re-init\r\n",
             static_cast<unsigned>(g_uart_fail_streak), reg);
    }
  }

  /* 离线状态下绝大多数请求立即失败，但每 kUartOfflineProbeEvery 次放一边真实探测。
   * 为什么不能一锁到底：上电瞬间 VM 未建立 / 总线被短暂钳住都会让自检连续失败，
   * 若永久锁死，就会出现“只是上电晚了一会儿，却再也读不到驱动器”的假故障。 */
  bool uartOfflineFastFail()
  {
    if (!g_uart_offline) return false;
    if (++g_offline_skip < kUartOfflineProbeEvery) return true;
    g_offline_skip = 0U;
    return false;
  }

  /* 电流安全上限（A, rms）。这不是拍脑袋的数：CS=31 + VSENSE=0 时芯片真的能输出 1.768A，
   * 板上 Rsense 110mΩ + 内阻 20mΩ，"软件设 2A" 不会被限幅，而是真的给 1.77A。
   * 超过电机额定电流会同时烧绕组和模块，要放开必须先确认电机额定值与散热。 */
  constexpr float kMaxCurrentA = 1.2f;

  /* 一次字节收发内关中断，保证位时序不被 ISR 打断：
   * 115200 下一字节约 87us，40000 下约 250us，不要在硬实时路径里高频调用 */
  struct CriticalSection
  {
    CriticalSection() : primask(__get_PRIMASK()) { __disable_irq(); }
    ~CriticalSection() { if (primask == 0U) { __enable_irq(); } }
    uint32_t primask;
  };

  /* 一次完整数据报事务（发请求 + 收应答）的原子保护。
   *
   * 为什么必须整段保护：读路径是纯软件位时序，靠「轮询抓起始位下降沿，再 1.5bit 后采样」。
   * 这块板上比 USB 任务优先级高的东西有两层：
   *   - TMR4 控制环 50us 一次（NVIC 1），ISR 里 vTaskNotifyGiveFromISR 唤醒优先级 3 的 control_task；
   *   - TMR3 编码器采样 25us 一次（NVIC 2），ISR 里直接跑阻塞 SPI2 读 + 三角滤波。
   * 起始位只有 25us 宽，被这两层一切，边缘必然抓歪，采样点整体偏移，整帧作废。
   * 实测表现：main() 里自检（两个定时器都没开）读得好好的，一进 FreeRTOS 连上位机就全读不到。
   *
   * 用 PRIMASK 关总中断而不是 FreeRTOS 临界区：这里只是想挡住 TMR3/TMR4 和 PendSV，
   * PRIMASK 一行搞定，而且不依赖 FreeRTOSConfig 里的可选 API（xTaskGetSchedulerState 默认为 0），
   * 单元测试那种不起调度器的环境也能直接跑。一次事务约 3ms，只在上位机读写参数时发生。 */
  struct UartBusGuard
  {
    UartBusGuard() : primask(__get_PRIMASK()) { __disable_irq(); }
    ~UartBusGuard() { if (primask == 0U) { __enable_irq(); } }

    uint32_t primask;
  };

  uint32_t bitCyclesFor(uint32_t baudrate)
  {
    const uint32_t baud = baudrate == 0U ? TMC2209_UART_BAUDRATE : baudrate;
    const uint32_t cycles = system_core_clock / baud;
    return cycles == 0U ? 1U : cycles;
  }

  inline bool lineIsHigh(gpio_type *port, uint16_t pin)
  {
    return gpio_input_data_bit_read(port, pin) != RESET;
  }

  inline void waitUntilCycle(uint32_t deadline)
  {
    /* 32 位回绕比较，等待时间远小于 2^31 个周期 */
    while (static_cast<int32_t>(DWT->CYCCNT - deadline) < 0)
    {
    }
  }

  /* 数据报之间必须留足静默。两条实测教训：
   *   1. 只留 8bit 时驱动器还在发送/处理上一帧，新报文整帧被丢；
   *   2. 这块板 PDN_UART 空闲电平被外部拉低（idle high 只有 29/4096），
   *      「线为高」的判据永远不成立，所以不能用电平判空闲，只能固定等。
   * 另外 readByte() 在停止位中点就返回了，剩下半个停止位驱动器还在发，
   * 这里不补够间隔就会跟驱动器抢线，写帧直接被丢（IFCNT 不涨就是这么来的）。 */
  bool busWaitIdle(gpio_type *port, uint16_t pin, uint32_t bit_cycles)
  {
    (void)port;
    (void)pin;
    waitUntilCycle(DWT->CYCCNT + kInterPacketGapBits * bit_cycles);
    return true;
  }

  /* TMC 数据报 CRC：多项式 x^8 + x^2 + x + 1，字节按 LSB 先处理 */
  uint8_t tmcCrc(const uint8_t *data, uint8_t length)
  {
    uint8_t crc = 0U;
    for (uint8_t index = 0U; index < length; ++index)
    {
      uint8_t current = data[index];
      for (uint8_t bit = 0U; bit < 8U; ++bit)
      {
        const bool mix = ((crc >> 7U) ^ (current & 0x01U)) != 0U;
        crc = static_cast<uint8_t>(crc << 1U);
        if (mix) crc ^= 0x07U;
        current >>= 1U;
      }
    }
    return crc;
  }

  /* 微步数转 CHOPCONF.MRES：256 -> 0，128 -> 1 ... 2 -> 7 */
  uint8_t microstepsToMres(uint16_t microsteps)
  {
    switch (microsteps)
    {
    case 256U: return 0U;
    case 128U: return 1U;
    case 64U: return 2U;
    case 32U: return 3U;
    case 16U: return 4U;
    case 8U: return 5U;
    case 4U: return 6U;
    case 2U: return 7U;
    default: return 3U;
    }
  }

  uint16_t normalizeMicrosteps(uint16_t value)
  {
    static const uint16_t supported[] = {2U, 4U, 8U, 16U, 32U, 64U, 128U, 256U};
    if (value < 2U) return 32U;
    uint16_t closest = supported[0];
    uint16_t closest_delta = value > closest ? value - closest : closest - value;
    for (uint8_t i = 1U; i < sizeof(supported) / sizeof(supported[0]); ++i)
    {
      const uint16_t delta = value > supported[i] ? value - supported[i] : supported[i] - value;
      if (delta < closest_delta)
      {
        closest = supported[i];
        closest_delta = delta;
      }
    }
    return closest;
  }

  /* 电流(A, RMS) 转 CS 档位：Irms = (CS+1)/32 * Vfs/(Rsense+20mΩ) / sqrt(2) */
  uint32_t currentToCs(float amps, float rsense_total, float vfs)
  {
    if (amps <= 0.0f || rsense_total <= 0.0f || vfs <= 0.0f) return 0U;
    const float scale = amps * kSqrt2 * rsense_total * 32.0f / vfs;
    int32_t cs = static_cast<int32_t>(scale + 0.5f) - 1;
    if (cs < 0) cs = 0;
    if (cs > 31) cs = 31;
    return static_cast<uint32_t>(cs);
  }
}

Tmc2209Driver::Tmc2209Driver()
  : enabled_(false), direction_(false), single_wire_uart_(false),
    config_{32U, 256U, 1.8f, 0.8f, 0.2f, true, false, 0.110f, 0.0f, TMC2209_UART_BAUDRATE, true, false, TMC2209_UART_GPIO, TMC2209_UART_PIN},
    uart_port_(TMC2209_UART_GPIO), uart_pin_(TMC2209_UART_PIN), chopconf_shadow_(kChopconfResetValue)
{
}

bool Tmc2209Driver::init()
{
  /* 重新初始化就清掉离线标记：换模块 / 重新接线后要能自动恢复 */
  uartNoteSuccess();

  stepper_common::stepper_init_step_gpio();
  stepper_common::stepper_init_uart_gpio(uart_port_, uart_pin_);

  /* DWT 周期计数器作为位时序基准 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* 上电先把总线交给驱动器：引脚作输入 + 上拉，空闲高电平 */
  releaseUartLine();

  /* 上电自检：空闲必须是高电平。这里只看电平不占时间，
   * 一直低说明模块没上电 / 线被钳住，后面的收发一定失败，先把现场打出来 */
  uint32_t high_samples = 0U;
  for (uint32_t index = 0U; index < 4096U; ++index)
  {
    if (lineIsHigh(uart_port_, uart_pin_)) ++high_samples;
  }
  printf("[TMC_UART] PA4 idle high %lu/4096, cfglr=0x%08lX odt=0x%04lX idt=0x%04lX baud=%lu bit=%lu cycles\r\n",
         static_cast<unsigned long>(high_samples),
         static_cast<unsigned long>(uart_port_->cfglr),
         static_cast<unsigned long>(uart_port_->odt),
         static_cast<unsigned long>(uart_port_->idt),
         static_cast<unsigned long>(config_.uart_baudrate),
         static_cast<unsigned long>(bitCyclesFor(config_.uart_baudrate)));

  /* 功能自检：把细分 / 插值 / 斩波模式各切一次再读回，确认这几条真的落进芯片。
   * 电流走 IHOLD_IRUN(0x10)，回读恒 0，只能看 DRV_STATUS.CS_ACTUAL，这里不测。 */
  {
    uint32_t chop = 0U;
    uint32_t gconf = 0U;

    (void)setMicrosteps(32U);
    if (readRegister(kRegChopconf, &chop))
    {
      const uint32_t mres = (chop >> 24U) & 0x0FU;
      printf("[TMC_CHK] microsteps=32 -> CHOPCONF=0x%08lX MRES=%lu (%lu microsteps) %s\r\n",
             static_cast<unsigned long>(chop), static_cast<unsigned long>(mres),
             static_cast<unsigned long>(256U >> mres),
             mres == 3U ? "OK" : "FAIL");
    }

    (void)setInterpolation(32U, 256U);
    if (readRegister(kRegChopconf, &chop))
    {
      printf("[TMC_CHK] interpolation=256 -> CHOPCONF=0x%08lX intpol=%u %s\r\n",
             static_cast<unsigned long>(chop),
             (chop & kChopconfIntpol) != 0U ? 1U : 0U,
             (chop & kChopconfIntpol) != 0U ? "OK" : "FAIL");
    }

    (void)setSilentMode(false);
    if (readRegister(kRegGconf, &gconf))
    {
      printf("[TMC_CHK] spreadCycle -> GCONF=0x%08lX en_spreadCycle=%u %s\r\n",
             static_cast<unsigned long>(gconf),
             (gconf & kGconfSpreadCycle) != 0U ? 1U : 0U,
             (gconf & kGconfSpreadCycle) != 0U ? "OK" : "FAIL");
    }

    (void)setSilentMode(true);
    if (readRegister(kRegGconf, &gconf))
    {
      printf("[TMC_CHK] stealthChop -> GCONF=0x%08lX en_spreadCycle=%u %s\r\n",
             static_cast<unsigned long>(gconf),
             (gconf & kGconfSpreadCycle) != 0U ? 1U : 0U,
             (gconf & kGconfSpreadCycle) == 0U ? "OK" : "FAIL");
    }
  }

  single_wire_uart_ = config_.use_single_wire_uart;
  return true;
}

bool Tmc2209Driver::configure(const StepperDriverConfig &config)
{
  config_ = config;
  uart_port_ = config_.uart_port != nullptr ? config_.uart_port : TMC2209_UART_GPIO;
  uart_pin_ = config_.uart_pin != 0U ? config_.uart_pin : TMC2209_UART_PIN;
  single_wire_uart_ = config_.use_single_wire_uart;
  if (config_.uart_baudrate == 0U)
  {
    config_.uart_baudrate = TMC2209_UART_BAUDRATE;
  }
  if (config_.microsteps == 0U)
  {
    config_.microsteps = 32U;
  }
  if (config_.sense_resistor_ohm <= 0.0f)
  {
    config_.sense_resistor_ohm = 0.110f;
  }
  config_.microsteps = normalizeMicrosteps(config_.microsteps);
  if (config_.step_angle_deg < 0.5f || config_.step_angle_deg > 3.0f)
  {
    config_.step_angle_deg = 1.8f;
  }
  if (config_.interpolation_microsteps != 256U)
  {
    config_.interpolation_microsteps = 0U;
  }
  return applyConfiguration();
}

const StepperDriverConfig &Tmc2209Driver::config() const
{
  return config_;
}

bool Tmc2209Driver::writeRegister(uint8_t reg, uint32_t value)
{
  /* 判定写是否被芯片接收：
   *   1. IFCNT 增量 —— 理论上最权威，但这块模块 IFCNT 实测恒为 64 不递增，不能单靠它；
   *   2. 回读寄存器 —— 可回读的寄存器（GCONF / CHOPCONF / PWM_CONF）用值比对确认；
   *   3. 0x10 / 0x11 —— 回读恒为 0（芯片特性），无法回读验证，写帧发出去且总线正常即认为成功。
   * 注意：不要把 0x10 / 0x11 当探针用，它们的“读回 0”不代表写失败。 */
  const bool write_only_reg = (reg == kRegIholdIrun || reg == kRegTpowerdown);
  /* UART 已知离线：不要再每写一个寄存器白等 300ms */
  if (uartOfflineFastFail()) return false;
  for (uint8_t retry = 0U; retry < kWriteRetryCount; ++retry)
  {
    uint32_t ifcnt_before = 0U;
    const bool ifcnt_before_ok = readRegister(kRegIfcnt, &ifcnt_before);

    sendWriteFrame(reg, value);

    uint32_t ifcnt_after = 0U;
    const bool ifcnt_after_ok = readRegister(kRegIfcnt, &ifcnt_after);
    const uint32_t delta = (ifcnt_after - ifcnt_before) & 0xFFU;

    if (ifcnt_before_ok && ifcnt_after_ok && delta != 0U)
    {
      return true;    /* 至少被接收一次；重试时可能累计到 +2 */
    }

    /* IFCNT 没动：不可回读的寄存器发送成功即算成功，其它靠回读比对 */
    if (write_only_reg)
    {
      printf("[TMC_UART] write 0x%02X=0x%08lX sent (not readable, IFCNT %lu->%lu %s)\r\n",
             reg, static_cast<unsigned long>(value),
             static_cast<unsigned long>(ifcnt_before), static_cast<unsigned long>(ifcnt_after),
             ifcnt_after_ok ? "ok" : "fail");
      return ifcnt_after_ok;
    }

    uint32_t readback = 0U;
    const bool readback_ok = readRegister(reg, &readback);
    if (readback_ok && readback == value)
    {
      printf("[TMC_UART] write 0x%02X=0x%08lX confirmed by readback (IFCNT %lu->%lu)\r\n",
             reg, static_cast<unsigned long>(value),
             static_cast<unsigned long>(ifcnt_before), static_cast<unsigned long>(ifcnt_after));
      return true;
    }

    printf("[TMC_UART] write 0x%02X attempt %u: IFCNT %lu->%lu, readback=0x%08lX expect=0x%08lX\r\n",
           reg, static_cast<unsigned>(retry + 1U),
           static_cast<unsigned long>(ifcnt_before), static_cast<unsigned long>(ifcnt_after),
           static_cast<unsigned long>(readback), static_cast<unsigned long>(value));
  }

  printf("[TMC_UART] write reg=0x%02X value=0x%08lX failed\r\n",
         reg, static_cast<unsigned long>(value));
  return false;
}

void Tmc2209Driver::sendWriteFrame(uint8_t reg, uint32_t value)
{
  uint8_t frame[8];
  frame[0] = kTmcSync;
  frame[1] = kTmcSlaveAddress;
  frame[2] = static_cast<uint8_t>(reg | kTmcWriteFlag);
  frame[3] = static_cast<uint8_t>(value >> 24U);
  frame[4] = static_cast<uint8_t>(value >> 16U);
  frame[5] = static_cast<uint8_t>(value >> 8U);
  frame[6] = static_cast<uint8_t>(value);
  frame[7] = tmcCrc(frame, 7U);

  (void)busWaitIdle(uart_port_, uart_pin_, bitCyclesFor(config_.uart_baudrate));

  /* 整帧 8 字节放一个临界区里：一帧 2ms，只在改配置 / 参数下发时发生，不进出硬实时路径 */
  UartBusGuard guard;
  setUartTransmitMode();
  for (uint8_t index = 0U; index < sizeof(frame); ++index)
  {
    if (!writeByte(frame[index]))
    {
      releaseUartLine();
      return;
    }
  }
  releaseUartLine();    /* 写报文没有应答，发完立刻释放总线 */
}

bool Tmc2209Driver::readRegister(uint8_t reg, uint32_t *value)
{
  if (value == nullptr) return false;
  /* UART 已知离线：直接失败，不要每读一个寄存器白等 30ms */
  if (uartOfflineFastFail()) return false;

  const uint32_t bit_cycles = bitCyclesFor(config_.uart_baudrate);

  uint8_t request[4];
  request[0] = kTmcSync;
  request[1] = kTmcSlaveAddress;
  request[2] = static_cast<uint8_t>(reg & 0x7FU);
  request[3] = tmcCrc(request, 3U);

  for (uint8_t attempt = 0U; attempt < kReadRetryCount; ++attempt)
  {
    (void)busWaitIdle(uart_port_, uart_pin_, bit_cycles);

    uint8_t response[8] = {};
    bool framing_ok = false;
    {
      /* 发请求 + 收 8 字节应答在同一个临界区里，中间不能被打断（见 UartBusGuard 注释）。
       * printf 必须放在临界区外面：115200 下一行日志要几 ms，带中断跑不起。 */
      UartBusGuard guard;

      setUartTransmitMode();
      bool tx_ok = true;
      for (uint8_t index = 0U; index < sizeof(request); ++index)
      {
        if (!writeByte(request[index]))
        {
          tx_ok = false;
          break;
        }
      }
      releaseUartLine();    /* 同一根线，请求发完立刻切接收等应答 */

      if (tx_ok)
      {
        framing_ok = true;
        for (uint8_t index = 0U; index < sizeof(response); ++index)
        {
          if (!readByte(&response[index]))
          {
            framing_ok = false;
            break;
          }
        }
      }
    }

    if (!framing_ok)
    {
      printf("[TMC_UART] PA4 no reply for reg=0x%02X (attempt %u)\r\n",
             reg, static_cast<unsigned>(attempt + 1U));
      continue;
    }

    if (response[0] != kTmcSync || response[1] != kTmcMasterAddress ||
        response[2] != static_cast<uint8_t>(reg & 0x7FU) ||
        response[7] != tmcCrc(response, 7U))
    {
      printf("[TMC_UART] PA4 invalid reply reg=0x%02X frame=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
             reg, response[0], response[1], response[2], response[3],
             response[4], response[5], response[6], response[7]);
      continue;
    }

    *value = (static_cast<uint32_t>(response[3]) << 24U) |
             (static_cast<uint32_t>(response[4]) << 16U) |
             (static_cast<uint32_t>(response[5]) << 8U) |
             static_cast<uint32_t>(response[6]);
    uartNoteSuccess();
    return true;
  }

  uartNoteFailure(reg);
  return false;
}

bool Tmc2209Driver::setMicrosteps(uint16_t microsteps)
{
  config_.microsteps = normalizeMicrosteps(microsteps);
  return applyChopperConfig();
}

bool Tmc2209Driver::setRunCurrent(float amps)
{
  config_.run_current_a = amps;
  return applyCurrentConfig();
}

bool Tmc2209Driver::setHoldCurrent(float amps, bool enabled)
{
  config_.hold_current_a = amps;
  config_.enable_hold_current = enabled;
  return applyCurrentConfig();
}

bool Tmc2209Driver::setSilentMode(bool enable)
{
  config_.silent_mode = enable;
  return applyModeConfig();
}

bool Tmc2209Driver::setInterpolation(uint16_t input_microsteps, uint16_t target_microsteps)
{
  config_.microsteps = normalizeMicrosteps(input_microsteps);
  config_.interpolation_microsteps = target_microsteps >= 256U ? 256U : 0U;
  return applyChopperConfig();
}

bool Tmc2209Driver::setSenseResistor(float ohm)
{
  if (ohm <= 0.0f)
  {
    return false;
  }
  config_.sense_resistor_ohm = ohm;
  return applyCurrentConfig();
}

bool Tmc2209Driver::setSingleWireUart(gpio_type *port, uint16_t pin, bool enable)
{
  if (port == nullptr)
  {
    return false;
  }

  uart_port_ = port;
  uart_pin_ = pin;
  single_wire_uart_ = enable;
  config_.uart_port = port;
  config_.uart_pin = pin;
  config_.use_single_wire_uart = enable;
  releaseUartLine();
  /* 重新绑定串口引脚也算是“换了个现场”，清掉离线标记重新探测 */
  uartNoteSuccess();
  return true;
}

bool Tmc2209Driver::applyConfiguration()
{
  /* 顺序有讲究：GCONF（pdn_disable / mstep_reg_select / 斩波模式）→ CHOPCONF（细分、插值）
   * → VSENSE + 电流档位 → TPOWERDOWN。VSENSE 在 CHOPCONF 里，所以电流必须在 CHOPCONF 之后写。 */
  const bool ok = applyModeConfig() && applyChopperConfig() && applyCurrentConfig() &&
                  writeRegister(kRegTpowerdown, 20U);

  /* 配置落地自检：GCONF / CHOPCONF 能直接回读；IHOLD_IRUN 这块模块回读恒为 0，
   * 电流是否生效要看 DRV_STATUS.CS_ACTUAL，不看 0x10 的回读值。 */
  uint32_t gconf = 0U;
  uint32_t chopconf = 0U;
  if (readRegister(kRegGconf, &gconf) && readRegister(kRegChopconf, &chopconf))
  {
    printf("[TMC_UART] config %s: GCONF=0x%08lX%s CHOPCONF=0x%08lX microsteps=%lu intpol=%u vsense=%u\r\n",
           ok ? "OK" : "FAIL",
           static_cast<unsigned long>(gconf),
           (gconf & kGconfSpreadCycle) != 0U ? "(spreadCycle)" : "(stealthChop)",
           static_cast<unsigned long>(chopconf),
           static_cast<unsigned long>(256U >> ((chopconf >> 24U) & 0x0FU)),
           (chopconf & kChopconfIntpol) != 0U ? 256U : 0U,
           (chopconf & kChopconfVsense) != 0U ? 1U : 0U);
  }
  return ok;
}

bool Tmc2209Driver::applyModeConfig()
{
  /* pdn_disable = 1：PDN 脚当 UART 用，否则驱动器不收串口；
   * mstep_reg_select = 1：细分由 CHOPCONF.MRES 决定，不看 MS1/MS2；
   * en_spreadCycle = 1：spreadCycle，0：stealthChop 静音模式 */
  uint32_t gconf = kGconfPdnDisable | kGconfMstepRegSelect;
  if (!config_.silent_mode) gconf |= kGconfSpreadCycle;
  if (!writeRegister(kRegGconf, gconf)) return false;

  if (config_.silent_mode)
  {
    /* stealthChop：打开 pwm_autoscale / pwm_autograd，让芯片自己标定电流 */
    uint32_t pwmconf = kPwmconfDefault;
    if (!readRegister(kRegPwmconf, &pwmconf))
    {
      pwmconf = kPwmconfDefault;
    }
    pwmconf |= kPwmconfAutoscale | kPwmconfAutograd;
    return writeRegister(kRegPwmconf, pwmconf);
  }
  return true;    /* spreadCycle 不依赖 stealthChop 的 PWM 自动标定 */
}

bool Tmc2209Driver::applyChopperConfig()
{
  /* 先读回芯片真实的 CHOPCONF 再改 MRES / intpol，保留 TOFF、HSTRT、HEND 等斩波参数 */
  uint32_t actual_chopconf = 0U;
  if (readRegister(kRegChopconf, &actual_chopconf))
  {
    chopconf_shadow_ = actual_chopconf;
  }

  chopconf_shadow_ &= ~(kChopconfMresMask | kChopconfIntpol);
  chopconf_shadow_ |= static_cast<uint32_t>(microstepsToMres(config_.microsteps)) << 24U;
  if (config_.interpolation_microsteps >= 256U)
  {
    chopconf_shadow_ |= kChopconfIntpol;
  }
  if ((chopconf_shadow_ & kChopconfToffMask) == 0U)
  {
    chopconf_shadow_ |= kChopconfDefaultToff;    /* TOFF = 0 会让斩波器停摆 */
  }

  return writeRegister(kRegChopconf, chopconf_shadow_);
}

bool Tmc2209Driver::applyCurrentConfig()
{
  if (config_.sense_resistor_ohm <= 0.0f) return false;

  /* VSENSE 判断必须基于芯片“现在真实”的 CHOPCONF，不能用可能过期的影子值：
   * 影子值一旦和芯片不一致，下面的“先清零电流再切 VSENSE”保护就整体失效。
   * 这里多花一次回读（约 10ms），只在改电流时发生。 */
  uint32_t actual_chopconf = 0U;
  if (readRegister(kRegChopconf, &actual_chopconf)) chopconf_shadow_ = actual_chopconf;

  /* 电流安全上限：超过就夹住并报错，绝不把超额定电流的档位写进芯片。
   * 夹完同时回写到 config_，避免下一次 apply 又重新算一遍超限值。 */
  float run_current = config_.run_current_a < 0.0f ? 0.0f : config_.run_current_a;
  float hold_current = config_.hold_current_a < 0.0f ? 0.0f : config_.hold_current_a;
  if (run_current > kMaxCurrentA)
  {
    printf("[TMC_SAFE] run current %.3fA over ceiling %.3fA, clamped\r\n", run_current, kMaxCurrentA);
    run_current = kMaxCurrentA;
    config_.run_current_a = kMaxCurrentA;
  }
  if (hold_current > kMaxCurrentA)
  {
    hold_current = kMaxCurrentA;
    config_.hold_current_a = kMaxCurrentA;
  }
  const float rsense_total = config_.sense_resistor_ohm + kSenseInternalOhm;

  /* 小电流优先用 VSENSE = 1（Vfs 0.180V），分辨率更高；超过它的能力才切 0.325V */
  const float max_current_low_vfs = kVfsLow / (rsense_total * kSqrt2);
  const bool use_low_vfs = run_current <= max_current_low_vfs * 0.98f;
  const float vfs = use_low_vfs ? kVfsLow : kVfsHigh;

  /* VSENSE 改变会直接改掉“同一个 CS 对应的实际电流”（0.325V / 0.180V = 1.8 倍）。
   * 若先切 VSENSE 再改 CS，中间那几十毫秒（每次寄存器写前后各一次 IFCNT 读，共 20ms+）
   * 芯片跑的是“新 Vfs + 旧 CS”，从低档跳到高档就是一次真实过流——这就是“改参数时烧驱动”
   * 的经典路径。所以 VSENSE 要变时先把电流压到最小档，再切 VSENSE，最后写目标档位。
   * 清零失败说明串口已经不可靠，此时宁可失能不能赌运气。 */
  const bool vsense_changed = ((chopconf_shadow_ & kChopconfVsense) != 0U) != use_low_vfs;
  if (vsense_changed && !writeRegister(kRegIholdIrun, 0U))
  {
    printf("[TMC_SAFE] cannot zero IHOLD_IRUN before VSENSE switch, driver disabled\r\n");
    setEnable(false);
    return false;
  }

  /* VSENSE 在 CHOPCONF 里，必须先写 CHOPCONF 再写电流档位 */
  if (use_low_vfs) chopconf_shadow_ |= kChopconfVsense;
  else chopconf_shadow_ &= ~kChopconfVsense;
  if (!writeRegister(kRegChopconf, chopconf_shadow_)) return false;

  const uint32_t irun = currentToCs(run_current, rsense_total, vfs);
  const uint32_t ihold = config_.enable_hold_current
      ? currentToCs(hold_current, rsense_total, vfs)
      : 0U;

  /* 注意：这块模块回读 IHOLD_IRUN(0x10) 恒为 0，写是否生效只能靠 writeRegister
   * 里的 IFCNT 增量和 DRV_STATUS.CS_ACTUAL 判定，不要用回读值做校验 */
  const uint32_t ihold_irun = ihold | (irun << 8U) | (kIholdDelay << 16U);

  /* 换算结果直接打出来：Rsense 110mΩ + 芯片内阻 20mΩ，VSENSE 决定 Vfs 0.180V / 0.325V */
  const float real_run = (static_cast<float>(irun) + 1.0f) / 32.0f * vfs / (rsense_total * kSqrt2);
  const float real_hold = (static_cast<float>(ihold) + 1.0f) / 32.0f * vfs / (rsense_total * kSqrt2);
  printf("[TMC_CUR] run=%.3fA hold=%.3fA(%s) Vfs=%.3fV IRUN=%lu IHOLD=%lu -> 0x10=0x%08lX (actual run=%.3fA hold=%.3fA)\r\n",
         run_current, hold_current, config_.enable_hold_current ? "on" : "off", vfs,
         static_cast<unsigned long>(irun), static_cast<unsigned long>(ihold),
         static_cast<unsigned long>(ihold_irun), real_run, real_hold);

  const bool ok = writeRegister(kRegIholdIrun, ihold_irun);

  /* 唯一的落地证据：DRV_STATUS 的 CS_ACTUAL 是芯片当前真正在用的档位，
   * OTPW / OT / S2GA / S2GB 才是“到底有没有烧”的判据。一旦报过温或输出短路就直接拉 EN 失能，
   * 宁可不动也不能让芯片持续拉电流。 */
  uint32_t drv_status = 0U;
  if (readRegister(kRegDrvStatus, &drv_status))
  {
    const uint32_t cs_actual = (drv_status >> 16U) & 0x1FU;
    const bool overtemp = (drv_status & 0x03U) != 0U;    /* bit0 OTPW / bit1 OT */
    const bool shorted = (drv_status & 0x0CU) != 0U;     /* bit2 S2GA / bit3 S2GB */
    printf("[TMC_CUR] DRV_STATUS=0x%08lX CS_ACTUAL=%lu OTPW=%u OT=%u S2GA/B=%u/%u\r\n",
           static_cast<unsigned long>(drv_status),
           static_cast<unsigned long>(cs_actual),
           static_cast<unsigned>((drv_status & 0x01U) != 0U),
           static_cast<unsigned>((drv_status & 0x02U) != 0U),
           static_cast<unsigned>((drv_status & 0x04U) != 0U),
           static_cast<unsigned>((drv_status & 0x08U) != 0U));
    if (overtemp || shorted)
    {
      printf("[TMC_SAFE] driver fault (overtemp=%u short=%u), disabling output\r\n",
             static_cast<unsigned>(overtemp), static_cast<unsigned>(shorted));
      setEnable(false);
    }
  }

  return ok;
}

void Tmc2209Driver::delayBit(uint32_t cycles) const
{
  const uint32_t start = DWT->CYCCNT;
  while (static_cast<uint32_t>(DWT->CYCCNT - start) < cycles)
  {
  }
}

/* 发送态：先把输出锁存置高，再切推挽输出，切换瞬间不会拉出一个低电平毛刺 */
void Tmc2209Driver::setUartTransmitMode()
{
  gpio_init_type gpio_init_struct;

  stepper_common::stepper_write_gpio_high(uart_port_, uart_pin_);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = uart_pin_;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(uart_port_, &gpio_init_struct);
}

/* 空闲 / 接收态：引脚作输入并开内部上拉，总线交给驱动器
 * （驱动器内部已有上拉，MCU 直连 PDN_UART 就行，不需要外部电阻） */
void Tmc2209Driver::releaseUartLine()
{
  gpio_init_type gpio_init_struct;

  stepper_common::stepper_write_gpio_high(uart_port_, uart_pin_);    /* input 模式下 ODR 决定上拉还是下拉 */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = uart_pin_;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init(uart_port_, &gpio_init_struct);
}

bool Tmc2209Driver::writeByte(uint8_t value)
{
  const uint32_t bit_cycles = bitCyclesFor(config_.uart_baudrate);

  CriticalSection guard;

  stepper_common::stepper_write_gpio_low(uart_port_, uart_pin_);                 /* 起始位 */
  const uint32_t t0 = DWT->CYCCNT;

  for (uint8_t bit = 0U; bit < 8U; ++bit)                                        /* 数据位，绝对时刻切换 */
  {
    waitUntilCycle(t0 + static_cast<uint32_t>(bit + 1U) * bit_cycles);
    stepper_common::stepper_write_gpio(uart_port_, uart_pin_, (value & (1U << bit)) != 0U);
  }

  waitUntilCycle(t0 + 9U * bit_cycles);
  stepper_common::stepper_write_gpio_high(uart_port_, uart_pin_);                /* 停止位 */
  waitUntilCycle(t0 + 10U * bit_cycles);                                         /* 等停止位彻底发完 */
  return true;
}

bool Tmc2209Driver::readByte(uint8_t *value)
{
  if (value == nullptr) return false;

  const uint32_t bit_cycles = bitCyclesFor(config_.uart_baudrate);

  /* 阶段一：等起始位下降沿。这期间不关中断——应答可能几百 bit 才来，
   * 关中断会把整个系统（含 20kHz 控制环）拖死；万一被 ISR 耽误几 us，采样偏差也不足以跨位，
   * 真错了还有上层重试兜底。 */
  const uint32_t start_timeout = DWT->CYCCNT + kStartBitTimeoutBits * bit_cycles;
  while (lineIsHigh(uart_port_, uart_pin_))
  {
    if (static_cast<int32_t>(DWT->CYCCNT - start_timeout) >= 0)
    {
      return false;
    }
  }

  /* 阶段二：只在真正采样这 9.5bit 内关中断，保证采样点不被推迟 */
  CriticalSection guard;

  const uint32_t t0 = DWT->CYCCNT;
  uint32_t sample_at = t0 + bit_cycles + bit_cycles / 2U;    /* 1.5bit 处采第一位 */
  uint8_t received = 0U;

  for (uint8_t bit = 0U; bit < 8U; ++bit)
  {
    waitUntilCycle(sample_at);
    if (lineIsHigh(uart_port_, uart_pin_))
    {
      received |= static_cast<uint8_t>(1U << bit);
    }
    sample_at += bit_cycles;
  }

  waitUntilCycle(t0 + 9U * bit_cycles + bit_cycles / 2U);    /* 停止位中点 */
  const bool stop_ok = lineIsHigh(uart_port_, uart_pin_);

  *value = received;
  return stop_ok;
}

void Tmc2209Driver::setEnable(bool enable)
{
  enabled_ = enable;
  stepper_common::stepper_write_gpio(EN_OUTPUT_PORT, EN_OUT_PIN, enable ? EN_ACTIVE_LEVEL : !EN_ACTIVE_LEVEL);
}

void Tmc2209Driver::setDirection(bool direction)
{
  direction_ = direction;
  stepper_common::stepper_write_gpio(DIR_OUTPUT_PORT, DIR_OUT_PIN,
                                     direction ? DIR_FORWARD_LEVEL : !DIR_FORWARD_LEVEL);
}

void Tmc2209Driver::setStepState(bool state)
{
  stepper_common::stepper_write_gpio(STEP_OUTPUT_PORT, STEP_OUT_PIN, state);
}

void Tmc2209Driver::sendStepPulse(uint64_t width_ns)
{
  stepper_common::stepper_write_gpio_high(STEP_OUTPUT_PORT, STEP_OUT_PIN);
  delay_ns(width_ns);
  stepper_common::stepper_write_gpio_low(STEP_OUTPUT_PORT, STEP_OUT_PIN);
  delay_ns(width_ns);
}
