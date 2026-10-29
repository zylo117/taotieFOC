# AT32F403A 自动刷机和调试脚本

$DFU_UTIL = "C:\Users\Jessi\.platformio\packages\tool-dfuutil\bin\dfu-util.exe"
$FW_FILE = ".pio\build\genericAT32F403ACCT7\firmware.bin"

Write-Host "=== AT32F403A 自动刷机和调试 ===" -ForegroundColor Cyan

# 检查固件文件
if (-not (Test-Path $FW_FILE)) {
    Write-Host "错误：固件文件不存在" -ForegroundColor Red
    exit 1
}

Write-Host "等待设备进入 DFU 模式..." -ForegroundColor Yellow
Write-Host "请确保已按住 BOOT 键或重新上电设备" -ForegroundColor Yellow

# 等待用户准备就绪
Read-Host "按回车键开始检测 DFU 设备..."

# 检测 DFU 设备
Write-Host "`n 扫描 DFU 设备..." -ForegroundColor Green
$dfu_devices = & $DFU_UTIL --list 2>$null | Select-String "AT32|Artery"

if ($dfu_devices) {
    Write-Host "找到 DFU 设备：" -ForegroundColor Green
    Write-Host $dfu_devices
    
    # 刷机
    Write-Host "`n 开始刷机..." -ForegroundColor Yellow
    & $DFU_UTIL -a 0 -s 0x08000000:leave -D $FW_FILE
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host "`n 刷机成功！" -ForegroundColor Green
        Write-Host "设备将重启，请打开串口监视器观察输出..." -ForegroundColor Green
    } else {
        Write-Host "`n 刷机失败！错误码：$LASTEXITCODE" -ForegroundColor Red
    }
} else {
    Write-Host "`n 未找到 AT32 DFU 设备！" -ForegroundColor Red
    Write-Host "请确认：" -ForegroundColor Yellow
    Write-Host "1. 设备已正确连接 USB" -ForegroundColor Yellow
    Write-Host "2. 已按住 BOOT 键或重新上电" -ForegroundColor Yellow
    Write-Host "3. 驱动程序已正确安装" -ForegroundColor Yellow
}
