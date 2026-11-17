/**
 * bl_desc.c —— DFU 1.1 设备/配置/字符串描述符。
 *
 * 配置描述符 = configuration(9) + interface(9) + DFU functional(9) = 27 字节，
 * 没有任何非 0 端点：整个 DFU 走 EP0。接口 class/subclass/protocol =
 * 0xFE / 0x01 / 0x02（Application Specific · DFU · **programming mode**：
 * 这个设备只会刷机不会跑应用代码，和 BL 的实际行为一致）。
 */
#include "bl_desc.h"

#include "at32f403a_407.h"
#include "bl_layout.h"      /* DFU_XFER_SIZE：功能描述符里的 wTransferSize 必须和类实现一致 */

#define BL_CONFIG_DESC_SIZE       27U
/* DFU functional 的 bDescriptorType 必须是 0x21。DFU 1.0 草案时代用过 0x04，
   dfu-util 0.11 只认 0x21：给它 0x04，它当"这设备没有功能描述符"，
   于是退化成 DFU 1.0、拿不到 wTransferSize，报 "Transfer size must be specified"，
   一个字节都刷不进去。实测踩过。 */
#define BL_DFU_DESC_SUBTYPE       0x21U
/* bit7 canDnload | bit6 canUpload | bit5 manifestationTolerant | bit4 willDetach */
#define BL_DFU_ATTRIBUTES         0xC0U
#define BL_DFU_TIMEOUT_MS         500U
/* bcdDFU 报 **0x0110（DFU 1.1，不带 a 后缀）**，不要写 0x011A：
   dfu-util 看到 >= 0x011A 就认定这是 ST 的 DfuSe 变体，转而强制要求接口字符串
   是 `@Flash /0x08002000/52*2Kg` 那种内存布局描述，然后 "Failed to parse memory
   layout" 直接退出（实测 exit 74）。我的设备是标准单区 DFU 1.1：地址由设备按
   base + 块号 × wTransferSize 自己算，主机只发块号 —— 语义上本来就不该声明 DfuSe。 */
#define BL_DFU_VERSION            0x0110U

#if defined(__ICCARM__)
#pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_bl_device_desc[USB_DEVICE_DESC_LEN] ALIGNED_TAIL = {
    USB_DEVICE_DESC_LEN,
    USB_DESCIPTOR_TYPE_DEVICE,
    0x00, 0x02,                     /* bcdUSB 2.00 */
    0x00,                           /* bDeviceClass：接口里给 */
    0x00,
    0x00,
    USB_MAX_EP0_SIZE,
    LBYTE(BL_USBD_VENDOR_ID), HBYTE(BL_USBD_VENDOR_ID),
    LBYTE(BL_USBD_PRODUCT_ID), HBYTE(BL_USBD_PRODUCT_ID),
    0x00, 0x01,                     /* bcdDevice 1.00 */
    1, 2, 3,                        /* iManufacturer / iProduct / iSerialNumber */
    1                               /* bNumConfigurations */
};

#if defined(__ICCARM__)
#pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_bl_config_desc[BL_CONFIG_DESC_SIZE] ALIGNED_TAIL = {
    /* configuration */
    USB_DEVICE_CFG_DESC_LEN,
    USB_DESCIPTOR_TYPE_CONFIGURATION,
    LBYTE(BL_CONFIG_DESC_SIZE), HBYTE(BL_CONFIG_DESC_SIZE),
    0x01,                           /* bNumInterfaces */
    0x01,                           /* bConfigurationValue */
    0x00,                           /* iConfiguration */
    0xC0,                           /* bmAttributes: bus powered */
    0x32,                           /* MaxPower 100mA */

    /* interface 0 */
    USB_DEVICE_IF_DESC_LEN,
    USB_DESCIPTOR_TYPE_INTERFACE,
    0x00,                           /* bInterfaceNumber */
    0x00,                           /* bAlternateSetting */
    0x00,                           /* bNumEndpoints：EP0-only */
    0xFE,                           /* bInterfaceClass: Application Specific */
    0x01,                           /* bInterfaceSubClass: DFU */
    0x02,                           /* bInterfaceProtocol: programming mode */
    0x00,                           /* iInterface：不给字符串（见下） */

    /* DFU functional —— 规范里正好 9 字节：
       bLength(1) bDescriptorType(1) bmAttributes(1) bwTimeOut(2) wTransferSize(2) bcdDFUVersion(2)
       bwTimeOut 是 **2 字节**，不是 4。曾经手滑写成 4 字节，数组声明 27 被 GCC 按
       "excess elements in array initializer" 警告截断，结果是 wTransferSize 被主机
       读成 0、bcdDFU 读成 0.64 —— 一个警告换来一整台设备不能工作。描述符数组的
       "excess elements / initializer" 警告必须当错误看。 */
    0x09,
    BL_DFU_DESC_SUBTYPE,
    BL_DFU_ATTRIBUTES,
    LBYTE(BL_DFU_TIMEOUT_MS), HBYTE(BL_DFU_TIMEOUT_MS),
    LBYTE(DFU_XFER_SIZE), HBYTE(DFU_XFER_SIZE),
    LBYTE(BL_DFU_VERSION), HBYTE(BL_DFU_VERSION)
};

static uint8_t g_bl_lang_id[4] = {0x04, 0x03, 0x09, 0x04};   /* EN-US */

/* 字符串描述符直接按 UTF-16 常量摆好：8KB 预算下不值得为它写一个运行时转换函数
   （实测那要占 140 字节 .text，比多存几个字节贵得多）。
   bLength = 2 + 2×字符数。 */
static const uint8_t g_str_mfc[]     = { 0x0E, 0x03, 'A',0, 'r',0, 't',0, 'e',0, 'r',0, 'y',0 };
static const uint8_t g_str_product[] = { 0x16, 0x03, 't',0, 'a',0, 'o',0, 't',0, 'i',0, 'e',0, ' ',0, 'D',0, 'F',0, 'U',0 };
static const uint8_t g_str_serial[]  = { 0x0E, 0x03, 'T',0, 'A',0, 'O',0, 'T',0, 'I',0, 'E',0 };
static const uint8_t g_str_if[]      = { 0x08, 0x03, 'D',0, 'F',0, 'U',0 };

/* usbd_desc_handler 的回调签名是 usbd_desc_t *(*)(void)（返回指针），不是返回值 */
static usbd_desc_t g_desc_device  = { USB_DEVICE_DESC_LEN, g_bl_device_desc };
static usbd_desc_t g_desc_config  = { BL_CONFIG_DESC_SIZE, g_bl_config_desc };
static usbd_desc_t g_desc_lang    = { 4, g_bl_lang_id };
static usbd_desc_t g_desc_mfc     = { sizeof(g_str_mfc), (uint8_t *)g_str_mfc };
static usbd_desc_t g_desc_product = { sizeof(g_str_product), (uint8_t *)g_str_product };
static usbd_desc_t g_desc_serial  = { sizeof(g_str_serial), (uint8_t *)g_str_serial };
static usbd_desc_t g_desc_if      = { sizeof(g_str_if), (uint8_t *)g_str_if };

static usbd_desc_t *bl_get_device_descriptor(void)  { return &g_desc_device; }
static usbd_desc_t *bl_get_configuration_descriptor(void) { return &g_desc_config; }
static usbd_desc_t *bl_get_lang_id(void)            { return &g_desc_lang; }
static usbd_desc_t *bl_get_mfc(void)                { return &g_desc_mfc; }
static usbd_desc_t *bl_get_product(void)            { return &g_desc_product; }
static usbd_desc_t *bl_get_serial(void)             { return &g_desc_serial; }
static usbd_desc_t *bl_get_if(void)                 { return &g_desc_if; }
static usbd_desc_t *bl_get_qualifier(void)          { return &g_desc_device; }
static usbd_desc_t *bl_get_other_speed(void)        { return &g_desc_device; }

usbd_desc_handler dfu_desc_handler = {
    bl_get_device_descriptor,
    bl_get_qualifier,
    bl_get_configuration_descriptor,
    bl_get_other_speed,
    bl_get_lang_id,
    bl_get_mfc,
    bl_get_product,
    bl_get_serial,
    bl_get_if,
    bl_get_configuration_descriptor   /* iConfiguration 没有，回配置描述符无害 */
};
