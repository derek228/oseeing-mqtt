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

RS485_SET_DEVICE_ID	=	0x80
RS485_SET_DEVICE_MODE=	0x81
RS485_SET_SQUARE1	=	0x71
RS485_SET_SQUARE2	=	0x72
RS485_SET_SQUARE3	=	0x73
RS485_SET_SQUARE4	=	0x74
RS485_SET_SQUARE5	=	0x75
RS485_SET_SQUARE6	=	0x76	
RS485_SET_SQUARE7	=	0x77
RS485_SET_SQUARE8	=	0x78
RS485_SET_SQUARE9	=	0x79
RS485_GET_FRAME_STATUS=	0x70
RS485_GET_ALARM_STATUS= 0x82 # return id , alarm, max length 20 bytes
RS485_GET_FRAME_STATUS=	0x84 # return max, min
RS485_GET_SQUARE_STAUTS=0x83 # return max, min
RS485_SET_SERVER_IP	=	0x50
RS485_SET_SOCKET_START=	0x51
RS485_SET_SCCKET_STOP=	0x52

SERIAL_PORT = "COM5"
BAUDRATE = 9600 #115200

def get_square(device_id) :
    data = (RS485_GET_SQUARE_STAUTS + device_id) & 0xff # {ID, 83, 1, checksum}
    print(f"checksum = {hex(data)}")
    data_packet = bytearray([device_id, RS485_GET_SQUARE_STAUTS, 1, data])
    print(f"Get square command : {data_packet}")
    send_and_receive_data( data_packet)   

def get_alarm(device_id) :
    data = (RS485_GET_ALARM_STATUS + device_id) & 0xff # {ID, 83, 1, checksum}
    print(f"checksum = {hex(data)}")
    data_packet = bytearray([device_id, RS485_GET_ALARM_STATUS, 1, data])
    print(f"Get alarm command : {data_packet}")
    send_and_receive_data( data_packet)   


def send_and_receive_data(data_packet):
    """通过UART发送数据并确认接收"""
    #data_packet = send_ip_addr(device_id, command_id)
    retry=3
    # 打开串口
    with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=3) as ser:
        ser.flushInput()  # 清空接收缓冲区
        ser.flushOutput()  # 清空发送缓冲区
        
        # 发送数据
        ser.write(data_packet)
        print("Data sent:", data_packet)
        time.sleep(0.2)
        # 等待接收数据
        while retry:
            #received_data = ser.read(len(data_packet))
            received_data = ser.read_all()
        
            if received_data[:2] == data_packet[:2]:
                print("Received data matches sent data:", received_data)
                print(f"Data length = {received_data[2]}, len = {len(received_data)}")
                print(f"Data = {received_data[3:]}")
                break;
            else:
                print("Received data does not match sent data or no data received.")
                print(received_data)
                retry -= 1
            if retry == 0 :
                print(f"Set command({data_packet}) fail....")
        time.sleep(1)
    print("Close serial port")
            
# 示例：使用设备ID 0x01 和命令ID 0x02
#send_and_receive_data(0xAA, 0x50)

DeviceID = 0xaa
while True:
    print("Get Alarm")
    get_alarm(DeviceID)
    time.sleep(1)
    print("Get Frame ")
    get_square(DeviceID)
    time.sleep(1)

