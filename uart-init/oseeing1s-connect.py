import serial
import time
import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import ttk
import serial.tools.list_ports
from tkinter import messagebox
import threading

HOST = '0.0.0.0'  # 接收端的IP地址
PORT = 8080

RS485_SET_SERVER_IP	=	0x50
RS485_SET_SOCKET_START=	0x51
global SERIAL_PORT
SERIAL_PORT = "COM5"
BAUDRATE = 9600 # 115200
DeviceID=0xAA

Result = False

def receive_data():
    plt.ion()  # Enable interactive mode
    fig, ax = plt.subplots()
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Listening on {HOST}:{PORT}...")
        
        conn, addr = s.accept()

        with conn:
            print(f"Connected by {addr}")
            img_handle = None
            while True:
                total_bytes = 62 * 80 * 2  # Each RGB565 data point is 2 bytes
                data = b''
                # Receive data until we get the expected amount
                while len(data) < total_bytes:
                    packet = conn.recv(total_bytes - len(data))
                    if not packet:
                        print("Connection closed or data missing")
                        return
                    data += packet

                # Print the actual size of the received data
                #print(f"Received data size: {len(data)} bytes")
                
                disp = struct.unpack('4960H', data)
                disp = np.array(disp, dtype=np.uint16).reshape((62, 80))
                
                # Normalize the temperature data to range [0, 1]
                norm_disp = (disp - disp.min()) / (disp.max() - disp.min())
                
                # Update the plot with the new data
                if img_handle is None:
                    img_handle = ax.imshow(norm_disp, cmap='jet', interpolation='nearest')
                else:
                    img_handle.set_data(norm_disp)
                
                plt.draw()
                plt.pause(0.01)  # Pause to update the plot


def set_socket_enable(device_id, command_id, en) :
    data_packet = bytearray([device_id, command_id, 1, en])
    return  send_and_receive_data(device_id, command_id, data_packet)


def get_local_ip():
    """获取本地IP地址"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    try:
        # 不实际连接到任何服务器，只是获取本地 IP
        s.connect(('10.254.254.254', 1))
        ip_address = s.getsockname()[0]
    except Exception:
        ip_address = '127.0.0.1'  # 如果获取失败，使用 localhost
    finally:
        s.close()
    return ip_address

def send_ip_addr(device_id, command_id):
    """创建要发送的数据包"""
    ip_address = get_local_ip()
    ip_address_bytes = ip_address.encode('utf-8')
    ip_length = len(ip_address_bytes)
    
    # 创建数据包：设备ID + 命令ID + IP地址长度 + IP地址
    data_packet = bytearray([device_id, command_id, ip_length]) + ip_address_bytes
    return send_and_receive_data(device_id, command_id, data_packet)
    #return data_packet

def send_and_receive_data(device_id, command_id, data_packet):
    """通过UART发送数据并确认接收"""
    #data_packet = send_ip_addr(device_id, command_id)
    retry=3
    # 打开串口
    with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
        ser.flushInput()  # 清空接收缓冲区
        ser.flushOutput()  # 清空发送缓冲区
        
        # 发送数据
        ser.write(data_packet)
        print("Data sent:", data_packet)
        
        # 等待接收数据
        while retry:
            received_data = ser.read(len(data_packet))
        
            if received_data == data_packet:
                print("Received data matches sent data:", received_data)
                return True
                #break;
            else:
                print("Received data does not match sent data or no data received.")
                print(received_data)
                retry -= 1
            if retry == 0 :
                print(f"Set command({data_packet}) fail....")
                return False
        time.sleep(1)
            
def select_port(selected_port):
    global SERIAL_PORT
    SERIAL_PORT=selected_port
    #eth.set_serial(selected_port)
    com_port_label.config(text=f"COM port device : {selected_port}")
    print(f"Selece COM port: {SERIAL_PORT}")

def apply_settings():
    print("apply")


def download_start():
    print("download")

import threading

def connect_device():
    global receive_thread

    if connect_button['text'] == "Connect Device":
        if send_ip_addr(DeviceID, RS485_SET_SERVER_IP):
            if set_socket_enable(DeviceID, RS485_SET_SOCKET_START, 1):
                connect_button.config(text="Disconnect")
                # 使用新執行緒啟動資料接收
                receive_thread = threading.Thread(target=receive_data)
                receive_thread.start()
    else:
        if set_socket_enable(DeviceID, RS485_SET_SOCKET_START, 0):
            connect_button.config(text="Connect Device")
            # 停止資料接收
            if receive_thread.is_alive():
                # 在這裡可以添加停止接收資料的邏輯
                print("Stopping data reception...")


def save_settings():
    print("save")

def scan_serial_ports():
    ports = serial.tools.list_ports.comports()  # scan serial port
    return [port.device for port in ports]  #  retrun available device

serial_ports = scan_serial_ports()
print(serial_ports)

# 建立主視窗
root = tk.Tk()
root.title("Settings")
#root.geometry('800x600')
root.geometry('300x200')
root.resizable(True,True)

# 建立 COM Port 標籤和文字輸入欄
menu = tk.Menu(root)
serial_menu = tk.Menu(menu, tearoff=0)
for port in serial_ports:
    serial_menu.add_command(label=port, command=lambda p=port: select_port(p))
    
menu.add_cascade(label="Select Serial Ports Device Name.....", menu=serial_menu)
root.config(menu=menu)
 
com_port_label = tk.Label(root, text="COM Port:")
com_port_label.grid(row=0,column=0,sticky=tk.W)

#apply_button = tk.Button(root, text="Apply Settings", command=apply_settings)
#apply_button.grid(row=14,column=0)

connect_button = tk.Button(root, text="Connect Device", command=connect_device)
connect_button.grid(row=15,column=0)

#download_button = tk.Button(root, text="Download", command=download_start)
#download_button.grid(row=15,column=1)

#save_button = tk.Button(root, text="Save Settings", command=save_settings)
#save_button.grid(row=20,column=0)

# 啟動主迴圈
root.mainloop()

