#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
蓝牙数据接收脚本 - Python 2/3 兼容版本
"""

import bluetooth
import sys
from datetime import datetime

def start_bluetooth_server():
    """启动蓝牙服务器接收数据"""
    
    # 创建蓝牙socket
    server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    
    # 绑定到任意可用端口
    port = 1
    server_sock.bind(("", port))
    server_sock.listen(1)
    
    # 注册服务
    bluetooth.advertise_service(
        server_sock,
        "BluetoothReceiver",
        service_id="00001101-0000-1000-8000-00805F9B34FB",
        service_classes=["00001101-0000-1000-8000-00805F9B34FB", bluetooth.SERIAL_PORT_CLASS],
        profiles=[bluetooth.SERIAL_PORT_PROFILE]
    )
    
    print("=" * 60)
    print("蓝牙接收服务已启动")
    print("监听端口: " + str(port))
    print("等待Windows客户端连接...")
    print("=" * 60)
    
    try:
        while True:
            # 等待客户端连接
            client_sock, client_info = server_sock.accept()
            timestamp = datetime.now().strftime('%H:%M:%S')
            print("\n[" + timestamp + "] 客户端已连接: " + str(client_info))
            
            try:
                while True:
                    # 接收数据
                    data = client_sock.recv(1024)
                    if not data:
                        break
                    
                    # 显示接收时间
                    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    
                    # 显示数据
                    print("\n[" + timestamp + "] 接收到 " + str(len(data)) + " 字节:")
                    print("  HEX: " + data.encode('hex'))
                    try:
                        print("  ASCII: " + data.decode('utf-8', errors='ignore'))
                    except:
                        print("  ASCII: [无法解码]")
                    
            except bluetooth.BluetoothError as e:
                print("\n连接错误: " + str(e))
            except Exception as e:
                print("\n接收错误: " + str(e))
            finally:
                client_sock.close()
                timestamp = datetime.now().strftime('%H:%M:%S')
                print("\n[" + timestamp + "] 客户端已断开")
                print("等待新的连接...\n")
                
    except KeyboardInterrupt:
        print("\n\n服务器关闭")
    finally:
        server_sock.close()

if __name__ == "__main__":
    try:
        start_bluetooth_server()
    except Exception as e:
        print("错误: " + str(e))
        print("\n请确保:")
        print("1. 已安装 pybluez: pip install pybluez")
        print("2. 蓝牙服务运行: sudo systemctl start bluetooth")
        print("3. 有蓝牙权限或使用 sudo 运行")
        sys.exit(1)
