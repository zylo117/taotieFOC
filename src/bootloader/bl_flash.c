/**
 * bl_flash.c —— 守卫式 Flash 操作。
 *
 * 这个文件里唯一的"不能出错的东西"是 bl_addr_in_app()：DFU 通路能擦/写到哪里，
 * 完全由它决定。BL 区（0x08000000~0x08001FFF）和 0x0801C000 以上的校准/滞后角/
 * FastCal 数据区都在这条线之外，越界直接 BL_ERR_GUARD，不发任何 Flash 命令。
 */
#include "bl_flash.h"

#include "at32f403a_407.h"

/* ---------------- 交接页 ---------------- */

bl_hsk_t *bl_hsk(void)
{
    return (bl_hsk_t *)BL_HSK_ADDR;
}

uint32_t bl_hsk_crc(const bl_hsk_t *h)
{
    return h->magic ^ h->request ^ h->boot_attempt ^ h->status;
}

int bl_hsk_valid(void)
{
    const bl_hsk_t *h = bl_hsk();
    return (h->magic == BL_HSK_MAGIC) && (h->crc == bl_hsk_crc(h)) ? 1 : 0;
}

void bl_hsk_init_fresh(void)
{
    bl_hsk_t *h = bl_hsk();
    h->magic = BL_HSK_MAGIC;
    h->request = BL_HSK_REQ_NONE;
    h->boot_attempt = 0U;
    h->status = 0U;
    h->crc = bl_hsk_crc(h);
}

void bl_hsk_store(const bl_hsk_t *src)
{
    bl_hsk_t *h = bl_hsk();
    h->magic = BL_HSK_MAGIC;
    h->request = src->request;
    h->boot_attempt = src->boot_attempt;
    h->status = src->status;
    h->crc = bl_hsk_crc(h);
}

/* ---------------- CRC32（IEEE 802.3 反射，多项式 0xEDB88320） ---------------- */
/* 和 tools/dfu_flash.py 里的 zlib.crc32 完全一致，不要改成 CRC16 或换初值。 */

uint32_t bl_crc32(const volatile uint8_t *p, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= p[i];
        for (int b = 0; b < 8; b++)
        {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }
    return ~crc;
}

/* ---------------- 地址守卫 ---------------- */

int bl_addr_in_app(uint32_t addr, uint32_t len)
{
    if ((addr & 3u) != 0u)
    {
        return 0;
    }
    if (addr < APP_REGION_BASE)
    {
        return 0;                       /* BL 自己：DFU 永远不许写 */
    }
    if (len > (DATA_REGION_START - addr))
    {
        return 0;                       /* 越到校准数据区，或加法回绕 */
    }
    return 1;
}

/* 这里原来有一张 bl_sector_size() 扇区表（1K/2K/4K 分段），已删。
   真机证明 AT32F403A 是均匀 2KB 扇区（内置 DFU 自报 128*2Kg），那张表在
   ≥0x0800C000 段按 4KB 跳步会漏擦一半。现在 bl_app_erase_all() 用固定 1KB 步进，
   与真实扇区几何无关，也就不再需要这张表 —— 留着只会诱使下一个人再去"优化"步长。 */

/* ---------------- 镜像校验 ---------------- */
/* 尾签：magic / payload_len / crc32 / flags+version，固定在应用分区最后 16 字节。 */

int bl_app_valid(uint32_t *payload_len, uint32_t *status_out)
{
    const volatile uint32_t *t = (const volatile uint32_t *)APP_TRAILER_ADDR;
    uint32_t status = 0U;
    int ok = 0;

    do
    {
        if (t[0] != APP_TRAILER_MAGIC)
        {
            status = 0x01u;             /* 没尾签：从没刷过，或刷到一半掉电 */
            break;
        }
        const uint32_t len = t[1];
        if (len == 0U || len > APP_PAYLOAD_MAX || (len & 3u) != 0U)
        {
            status = 0x02u;             /* 长度字段不可信 */
            break;
        }
        const uint32_t crc_calc = bl_crc32((const volatile uint8_t *)APP_REGION_BASE, len);
        if (crc_calc != t[2])
        {
            status = 0x03u;             /* CRC 不符：下载中断 / 写坏 */
            break;
        }
        /* 向量表合理性：SP 落在 RAM、Reset_Handler 落在应用区且是 Thumb 位。
           CRC 已经够强，这三条是免费的，专门挡"手搓镜像/地址算错"这类情况。 */
        const uint32_t sp = *(const volatile uint32_t *)APP_REGION_BASE;
        const uint32_t pc = *(const volatile uint32_t *)(APP_REGION_BASE + 4u);
        if (sp < 0x20000000u || sp > 0x20018000u)
        {
            status = 0x04u;
            break;
        }
        if (pc < (APP_REGION_BASE + 8u) || pc >= APP_REGION_END || (pc & 1u) == 0u)
        {
            status = 0x05u;
            break;
        }
        ok = 1;
    } while (0);

    if (payload_len != 0)
    {
        *payload_len = t[1];
    }
    if (status_out != 0)
    {
        *status_out = status;
    }
    return ok;
}

/* ---------------- 擦 / 写 ---------------- */

bl_err_t bl_app_erase_all(void)
{
    /* 哨兵：BL 自己的第一个字和最后一个字。擦完必须分毫不差 —— 万一扇区图和
       芯片实际不符，这里立刻暴露并停机，而不是等下次上电发现 BL 已经没了。 */
    const uint32_t guard_first = *(volatile uint32_t *)BL_REGION_BASE;
    const uint32_t guard_last = *(volatile uint32_t *)(BL_REGION_END - 4u);

    if (!bl_addr_in_app(APP_REGION_BASE, APP_REGION_SIZE))
    {
        return BL_ERR_GUARD;
    }

    /* 擦除步进固定 1KB，不再按"猜的扇区大小"跳。
       真机证据：芯片内置 DFU 自报 `128*2Kg` —— AT32F403A 是**均匀 2KB 扇区**，
       而我原先以为 ≥0x0800C000 是 4KB，于是每 4KB 只擦了一次（硬件只擦地址所在
       那一个 2KB 扇区），每个窗口上半 2KB 从没被擦，整区 0xFF 自检必然失败。
       用最小的 1KB 步进就与真实几何无关了：最坏是同一个扇区被重复擦（幂等，
       多花几毫秒），绝不会漏擦。宁可慢，不可漏 —— 漏擦的后果是镜像写坏。 */
    flash_unlock();
    uint32_t addr = APP_REGION_BASE;
    while (addr < APP_REGION_END)
    {
        if (flash_sector_erase(addr) != FLASH_OPERATE_DONE)
        {
            flash_flag_clear(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
            flash_lock();
            return BL_ERR_ERASE;
        }
        addr += 1024u;
    }
    flash_lock();

    if (*(volatile uint32_t *)BL_REGION_BASE != guard_first ||
        *(volatile uint32_t *)(BL_REGION_END - 4u) != guard_last)
    {
        return BL_ERR_GUARD;
    }

    for (addr = APP_REGION_BASE; addr < APP_REGION_END; addr += 4u)
    {
        if (*(volatile uint32_t *)addr != 0xFFFFFFFFu)
        {
            return BL_ERR_VERIFY;
        }
    }
    return BL_OK;
}

bl_err_t bl_app_write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    if (!bl_addr_in_app(addr, len))
    {
        return BL_ERR_GUARD;
    }

    flash_unlock();
    for (uint32_t i = 0; i + 4u <= len; i += 4u)
    {
        const uint32_t w = (uint32_t)buf[i] |
                           ((uint32_t)buf[i + 1u] << 8) |
                           ((uint32_t)buf[i + 2u] << 16) |
                           ((uint32_t)buf[i + 3u] << 24);
        if (w == 0xFFFFFFFFu)
        {
            continue;                   /* 擦除态不重写：省时间，也避免 program 已 1 的位置 */
        }
        if (flash_word_program(addr + i, w) != FLASH_OPERATE_DONE)
        {
            flash_flag_clear(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
            flash_lock();
            return BL_ERR_PROGRAM;
        }
    }
    flash_lock();

    /* 这里刻意不做逐字回读比对：8KB 预算下它不值这 120 字节。整镜像的一致性由两道
       更硬的关卡保证 —— manifest 时 BL 自己算 CRC32（bl_app_valid），以及主机
       dfu_flash.py 事后用 DFU_UPLOAD 把整片读回来逐字节比对（比 BL 顺手看一眼更强）。
       flash_word_program 的返回值仍然检查：写保护页、时序错这类硬错误当场就会暴。 */
    return BL_OK;
}

/* ---------------- 跳应用 ---------------- */

bl_err_t bl_jump_to_app(void)
{
    const uint32_t sp = *(volatile uint32_t *)APP_REGION_BASE;
    const uint32_t pc = *(volatile uint32_t *)(APP_REGION_BASE + 4u);

    if (sp < 0x20000000u || sp > 0x20018000u)
    {
        return BL_ERR_VERIFY;
    }
    if (pc < (APP_REGION_BASE + 8u) || pc >= APP_REGION_END || (pc & 1u) == 0u)
    {
        return BL_ERR_VERIFY;
    }

    __disable_irq();
    SysTick->CTRL = 0u;
    SysTick->LOAD = 0u;
    SysTick->VAL = 0u;
    for (int i = 0; i < 8; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFFu;
        NVIC->ICPR[i] = 0xFFFFFFFFu;
    }
    /* 先把向量表搬到应用区再跳：应用 main() 里也会再设一次（双保险，
       见 src/main.cpp），因为 SystemInit 会按 VECT_TAB_OFFSET 把它写回 0。 */
    SCB->VTOR = APP_REGION_BASE;
    __DSB();

    __set_MSP(sp);
    ((void (*)(void))pc)();

    return BL_ERR_VERIFY;               /* 走到这里说明应用没接住 */
}

/* ---------------- EPP ---------------- */

bl_err_t bl_epp_protect_bl(void)
{
    /* bit0 = 0x08000000~0x08000FFF，bit1 = 0x08001000~0x08001FFF（每 bit 4KB） */
    uint32_t bits = 0x00000003u;
    const uint32_t now = bl_epp_bits();
    /* epps 原值里 0 = 已保护（flash_epp_set 内部做了按位取反） */
    const uint32_t already = ~now & bits;
    if (already == bits)
    {
        return BL_OK;                   /* 已经全保护，幂等返回 */
    }
    if (flash_epp_set(&bits) != FLASH_OPERATE_DONE)
    {
        return BL_ERR_EPP;
    }
    return BL_OK;
}

uint32_t bl_epp_bits(void)
{
    uint32_t v = 0xFFFFFFFFu;
    flash_epp_status_get(&v);
    return v;
}
