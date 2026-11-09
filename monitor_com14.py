import serial
import sys
import time

try:
    print("=== 开始监控 COM14 (115200) ===", flush=True)
    print(f"尝试打开 COM14...", flush=True)
    ser = serial.Serial('COM14', 115200, timeout=1)
    print(f"串口已打开：{ser.is_open}", flush=True)
    time.sleep(2)  # 等待设备启动
    
    print("等待设备输出...\n", flush=True)
    
    start_time = time.time()
    while time.time() - start_time < 30:  # 监控 30 秒
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            if data:
                print(data, end='', flush=True)
        time.sleep(0.1)
    
    ser.close()
    print("\n=== 监控结束 ===", flush=True)
    
except serial.SerialException as e:
    print(f"串口错误：{e}", flush=True)
    print("请确认 COM14 是否存在且未被其他程序占用", flush=True)
    sys.exit(1)
except Exception as e:
    print(f"错误：{e}", flush=True)
    sys.exit(1)
